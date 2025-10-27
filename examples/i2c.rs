#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(proc_macro_hygiene)]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

extern crate alloc;

use esp_backtrace as _;
#[cfg(any(feature = "esp32s3"))]
use esp_hal::gpio::RtcPin as RtcIoWakeupPinType;
#[cfg(any(feature = "esp32c3", feature = "esp32c6"))]
use esp_hal::gpio::RtcPinWithResistors as RtcIoWakeupPinType;
use esp_hal::i2c;
#[cfg(target_arch = "riscv32")]
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::rmt::Rmt;
use esp_hal::rtc_cntl::sleep::{RtcioWakeupSource, TimerWakeupSource, WakeupLevel};
use esp_hal::rtc_cntl::Rtc;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::Async;

use esp_radio::esp_now::{PeerInfo, BROADCAST_ADDRESS};

use defmt_rtt as _;
use esp_backtrace as _;

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Timer};

use core::fmt::Write;

use static_cell::{make_static, StaticCell};

use aht20_async::Aht20;
use bme280_rs;

use sensor_data::{SensorData, SensorValue};

use esp_now_bridge::ds18b20::{check_onewire_crc, Ds18b20};
use esp_now_bridge::esp_hal_rmt_onewire::{OneWire, Search};
use esp_now_bridge::format_mac::format_mac;

pub const RMT_FREQ_MHZ: u32 = 80;
static I2C_BUS: StaticCell<Mutex<NoopRawMutex, i2c::master::I2c<Async>>> = StaticCell::new();

#[esp_hal::ram(unstable(rtc_fast, persistent))]
static mut HUB_ADDRESS: [u8; 6] = [0; 6];

#[esp_hal::ram(unstable(rtc_fast, persistent))]
static mut FIRST_BOOT: u8 = 1;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // Setup RTC & Sleep Timer
    let mut rtc = Rtc::new(peripherals.LPWR);
    let timer = TimerWakeupSource::new(core::time::Duration::from_secs(30));
    let mut pin_1 = peripherals.GPIO1;
    let wakeup_pins: &mut [(&mut dyn RtcIoWakeupPinType, WakeupLevel)] =
        &mut [(&mut pin_1, WakeupLevel::Low)];
    let rtcio = RtcioWakeupSource::new(wakeup_pins);

    defmt::info!(
        "INIT: wakeup={} boot={}",
        esp_hal::rtc_cntl::wakeup_cause(),
        rtc.time_since_boot().as_millis()
    );

    esp_alloc::heap_allocator!(size: 64 * 1024);

    #[cfg(target_arch = "riscv32")]
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(
        timg0.timer0,
        #[cfg(target_arch = "riscv32")]
        sw_int.software_interrupt0,
    );

    let delay = Delay;

    defmt::info!("ESP_RTOS initialized!");

    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(RMT_FREQ_MHZ))
        .expect("Error initialising RMT")
        .into_async();

    #[cfg(feature = "esp32s3")]
    let (tx_chan, rx_chan) = (rmt.channel0, rmt.channel4);
    #[cfg(feature = "esp32c3")]
    let (tx_chan, rx_chan) = (rmt.channel0, rmt.channel2);
    let pin = peripherals.GPIO6;
    let mut ow = OneWire::new(tx_chan, rx_chan, pin).unwrap();
    let mut ds18b20 = heapless::Vec::<Ds18b20, 5>::new();

    // Scan OneWIre bus for DS18B20
    let mut s = Search::new();
    loop {
        match s.next(&mut ow).await {
            Ok(address) => {
                let a = address.0.to_le_bytes();
                if a[0] == 0x28 && check_onewire_crc(&a) {
                    defmt::info!("Found DS18B20 {}", address);
                    ds18b20.push(Ds18b20::new(address.0)).unwrap();
                } else {
                    defmt::info!("Found device {} {}", address, check_onewire_crc(&a));
                }
            }
            Err(_) => {
                defmt::info!("End of search");
                break;
            }
        }
    }

    // Initialise I2C
    let i2c_config = i2c::master::Config::default().with_frequency(Rate::from_khz(100));
    let mut i2c = i2c::master::I2c::new(peripherals.I2C0, i2c_config)
        .expect("Error initailising I2C")
        .with_scl(peripherals.GPIO4)
        .with_sda(peripherals.GPIO5)
        .into_async();

    // I2C Bus Scan
    if unsafe { FIRST_BOOT } == 1 {
        for addr in 0..=127 {
            if let Ok(_) = i2c.write_async(addr, &[0]).await {
                defmt::info!("Found I2C device at address: 0x{:02x}", addr);
            }
        }
        unsafe { FIRST_BOOT = 0 }
    }

    // Create shared I2C bus
    let i2c_bus = I2C_BUS.init(Mutex::new(i2c));

    // Initialise BME280
    let bme280_device = I2cDevice::new(i2c_bus);

    let mut bme280 = bme280_rs::AsyncBme280::new(bme280_device, delay.clone());
    bme280.init().await.expect("Error initialising BME280");

    let configuration = bme280_rs::Configuration::default()
        .with_temperature_oversampling(bme280_rs::Oversampling::Oversample1)
        .with_pressure_oversampling(bme280_rs::Oversampling::Oversample1)
        .with_humidity_oversampling(bme280_rs::Oversampling::Oversample1)
        .with_sensor_mode(bme280_rs::SensorMode::Normal);

    bme280
        .set_sampling_configuration(configuration)
        .await
        .expect("Error serring BME280 configuretion");

    let bme280_id = bme280
        .chip_id()
        .await
        .expect("Error getting BME280 chip_id");

    defmt::info!("BME280: ID={}", bme280_id);

    // Initialise AHT20
    let aht20_device = I2cDevice::new(i2c_bus);
    let mut aht20 = Aht20::new(aht20_device, delay.clone())
        .await
        .expect("Error initialising BME280");
    aht20.calibrate().await.expect("Error calibrating AHT20");

    // Initialise ESP_NOW
    defmt::info!("Initialise ESP_NOW");
    let esp_radio_ctrl = make_static!(esp_radio::init().unwrap());
    let wifi = peripherals.WIFI;
    let (mut controller, interfaces) =
        esp_radio::wifi::new(&esp_radio_ctrl, wifi, Default::default()).unwrap();
    controller.set_mode(esp_radio::wifi::WifiMode::Sta).unwrap();
    controller.start().unwrap();

    let mut esp_now = interfaces.esp_now;
    esp_now.set_channel(11).unwrap();

    defmt::info!("ESP-NOW VERSION: {}", esp_now.version().unwrap());
    defmt::info!(
        "        MAC ADDRESS: {}",
        format_mac(&esp_radio::wifi::sta_mac())
    );

    let mut msg: heapless::String<64> = heapless::String::new();

    let mut hub_address = unsafe {
        let hub_address = &raw const HUB_ADDRESS;
        defmt::info!("HUB_ADDRESS: {}", format_mac(&*hub_address));
        *hub_address.clone()
    };

    if hub_address != [0; 6] {
        // Valid hub address - add peer
        if !esp_now.peer_exists(&hub_address) {
            defmt::info!("ESP-NOW ADD PEER: {}", format_mac(&hub_address));
            esp_now
                .add_peer(PeerInfo {
                    interface: esp_radio::esp_now::EspNowWifiInterface::Sta,
                    peer_address: hub_address,
                    lmk: None,
                    channel: None,
                    encrypt: false,
                })
                .unwrap();
        }
    } else {
        // Wait for HUB broadcast
        loop {
            let r = esp_now.receive_async().await;
            defmt::info!(
                "ESP-NOW RX: [{}]->[{}] >> {} [rssi={}]",
                format_mac(&r.info.src_address),
                format_mac(&r.info.dst_address),
                core::str::from_utf8(r.data()).unwrap_or("UTF8 Error"),
                r.info.rx_control.rssi
            );
            if r.info.dst_address == BROADCAST_ADDRESS {
                if !esp_now.peer_exists(&r.info.src_address) {
                    defmt::info!("ESP-NOW ADD PEER: {}", format_mac(&r.info.src_address));
                    esp_now
                        .add_peer(PeerInfo {
                            interface: esp_radio::esp_now::EspNowWifiInterface::Sta,
                            peer_address: r.info.src_address,
                            lmk: None,
                            channel: None,
                            encrypt: false,
                        })
                        .unwrap();
                }
                defmt::info!("PEER ADDRESS: {}", format_mac(&r.info.src_address));
                // Report RSSI to peer
                msg.clear();
                write!(
                    msg,
                    "{} -> RSSI: {}",
                    format_mac(&esp_radio::wifi::sta_mac()),
                    r.info.rx_control.rssi
                )
                .unwrap();
                let status = esp_now
                    .send_async(&r.info.src_address, msg.as_bytes())
                    .await;
                defmt::info!("REPLY PEER: {:?}", status);
                // Set HUB_ADDRESS
                unsafe {
                    HUB_ADDRESS = r.info.src_address;
                }
                // Set local
                hub_address = r.info.src_address;
                for i in 0..5 {
                    defmt::info!("WAIT [{}]", i);
                    Timer::after_millis(1000).await;
                }
                break;
            }
        }
    }

    loop {
        let mut data = SensorData::new(format_mac(&esp_radio::wifi::sta_mac()));

        for ds in &ds18b20 {
            if let Ok(temp) = ds.read_temp(&mut ow).await {
                let mut addr = heapless::String::<32>::new();
                write!(addr, "{}", ds.address).unwrap();
                defmt::info!("DS18B20: Temp = {}°C [{}]", temp, addr);
                data.push("sensor/ds18b20/temp", SensorValue::Float(temp), true)
                    .unwrap();
            }
            let _ = ds.initiate_conversion(&mut ow).await;
        }
        if let Ok((h, t)) = aht20.read().await {
            defmt::info!("AHT20:   Temp = {}°C / Humidity = {}%", t.celsius(), h.rh());
            data.push("sensor/aht20/temp", SensorValue::Float(t.celsius()), true)
                .unwrap();
            data.push("sensor/aht20/humidity", SensorValue::Float(h.rh()), true)
                .unwrap();
        }
        if let Ok(Some(t)) = bme280.read_temperature().await {
            if let Ok(Some(p)) = bme280.read_pressure().await {
                defmt::info!("BME280:  Temp = {}°C / Pressure = {} hPa", t, p / 100.0);
                data.push("sensor/bme280/temp", SensorValue::Float(t), true)
                    .unwrap();
                data.push(
                    "sensor/bme280/pressure",
                    SensorValue::Float(p / 100.0),
                    true,
                )
                .unwrap();
            }
        }
        defmt::info!("");

        let json = serde_json::to_vec(&data).unwrap();

        if hub_address != [0; 6] {
            // Send to Hub
            let status = esp_now.send_async(&hub_address, &json).await;
            defmt::info!("ESP-NOW TX -> {}: {}", format_mac(&hub_address), status);
        } else {
            // Send Broadcast
            defmt::info!("NO HUB_ADDRESS");
        }

        // Wait for response
        loop {
            let r = esp_now.receive_async().await;
            defmt::info!(
                "ESP-NOW RX: [{}]->[{}] >> {} [rssi={}]",
                format_mac(&r.info.src_address),
                format_mac(&r.info.dst_address),
                core::str::from_utf8(r.data()).unwrap_or("UTF8 Error"),
                r.info.rx_control.rssi
            );
            if r.info.src_address == hub_address {
                defmt::info!("HUB RESPONSE: OK");
                break;
            }
        }
        defmt::info!("UPTIME: {}", rtc.time_since_boot().as_millis());
        Timer::after_millis(200).await;
        rtc.sleep_deep(&[&timer, &rtcio]);
    }
}

mod sensor_data {

    use serde::{Deserialize, Serialize};

    pub const MAX_SENSORS: usize = 10_usize;

    #[derive(Serialize, Deserialize, Debug)]
    #[serde(untagged)]
    pub enum SensorValue {
        Float(f32),
        String(heapless::String<16>),
    }

    #[derive(Serialize, Deserialize, Debug)]
    pub struct SensorReading {
        path: heapless::String<32>,
        value: SensorValue,
        retain: bool,
    }

    #[derive(Serialize, Deserialize, Debug)]
    pub struct SensorData {
        mac: heapless::String<17>,
        data: heapless::Vec<SensorReading, MAX_SENSORS>,
    }

    impl SensorData {
        pub fn new(mac: heapless::String<17>) -> Self {
            let data = heapless::Vec::new();
            Self { mac, data }
        }
        pub fn push(&mut self, path: &str, value: SensorValue, retain: bool) -> anyhow::Result<()> {
            self.data
                .push(SensorReading {
                    path: path.try_into()?,
                    value,
                    retain,
                })
                .map_err(|_| anyhow::anyhow!("Too many SensorValues"))
        }
    }
}
