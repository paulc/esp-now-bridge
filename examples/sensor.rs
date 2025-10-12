#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use esp_hal::clock::CpuClock;
#[cfg(feature = "esp32s3")]
use esp_hal::gpio::RtcPin;
#[cfg(feature = "esp32c3")]
use esp_hal::gpio::RtcPinWithResistors;
use esp_hal::i2c;
#[cfg(target_arch = "riscv32")]
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::rtc_cntl::sleep::{RtcioWakeupSource, TimerWakeupSource, WakeupLevel};
use esp_hal::rtc_cntl::Rtc;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::Async;

use esp_radio::esp_now::PeerInfo;

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Timer};

use defmt_rtt as _;
use esp_backtrace as _;

use bme280_rs;
use core::fmt::Write;
use heapless::String;

use static_cell::{make_static, StaticCell};

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

const HUB: [u8; 6] = [0xa8, 0x46, 0x74, 0x40, 0x0a, 0x3c];

const MAC_FMT_LEN: usize = 17; // "xx:xx:xx:xx:xx:xx" = 17 chars

pub fn format_mac(mac: &[u8; 6]) -> String<MAC_FMT_LEN> {
    let mut s: String<MAC_FMT_LEN> = String::new();
    for (i, &byte) in mac.iter().enumerate() {
        if i > 0 {
            s.push(':').unwrap(); // Won't fail because we know capacity is enough
        }
        // Format byte as two lowercase hex digits
        write!(s, "{:02x}", byte).unwrap(); // `write!` works with heapless::String
    }
    s
}

static I2C_BUS: StaticCell<Mutex<NoopRawMutex, i2c::master::I2c<Async>>> = StaticCell::new();

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::_80MHz));
    esp_alloc::heap_allocator!(size: 64 * 1024);

    defmt::info!("Init");

    esp_alloc::heap_allocator!(size: 64 * 1024);

    #[cfg(target_arch = "riscv32")]
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(
        timg0.timer0,
        #[cfg(target_arch = "riscv32")]
        sw_int.software_interrupt0,
    );

    // Initialise ESP_NOW
    let wifi = peripherals.WIFI;
    let esp_radio_ctrl = make_static!(esp_radio::init().unwrap());
    let (mut controller, interfaces) =
        esp_radio::wifi::new(&esp_radio_ctrl, wifi, Default::default()).unwrap();
    controller.set_mode(esp_radio::wifi::WifiMode::Sta).unwrap();
    controller.start().unwrap();

    let mut esp_now = interfaces.esp_now;
    esp_now.set_channel(11).unwrap();
    // Add peer
    esp_now
        .add_peer(PeerInfo {
            interface: esp_radio::esp_now::EspNowWifiInterface::Sta,
            peer_address: HUB,
            lmk: None,
            channel: None,
            encrypt: false,
        })
        .unwrap();

    defmt::info!("ESP-NOW VERSION: {}", esp_now.version().unwrap());
    defmt::info!(
        "        MAC ADDRESS: {}",
        format_mac(&esp_radio::wifi::sta_mac())
    );
    //
    // Initialise I2C
    let i2c_config = i2c::master::Config::default().with_frequency(Rate::from_khz(100));
    let mut i2c = i2c::master::I2c::new(peripherals.I2C0, i2c_config)
        .expect("Error initailising I2C")
        .with_scl(peripherals.GPIO4)
        .with_sda(peripherals.GPIO5)
        .into_async();

    // I2C Bus Scan
    defmt::info!("Scanning I2C bus");
    for addr in 0..=127 {
        if let Ok(_) = i2c.write_async(addr, &[0]).await {
            defmt::info!("Found I2C device at address: 0x{:02x}", addr);
        }
    }

    // Create shared I2C bus
    let i2c_bus = I2C_BUS.init(Mutex::new(i2c));
    let bme280_device = I2cDevice::new(i2c_bus);

    let delay = Delay;
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

    // Configure sleep
    let mut rtc = Rtc::new(peripherals.LPWR);
    let timer = TimerWakeupSource::new(core::time::Duration::from_secs(60));
    let mut pin5 = peripherals.GPIO3;
    #[cfg(feature = "esp32c3")]
    let rtcio_pins: &mut [(&mut dyn RtcPinWithResistors, WakeupLevel)] =
        &mut [(&mut pin5, WakeupLevel::Low)];
    #[cfg(feature = "esp32s3")]
    let rtcio_pins: &mut [(&mut dyn RtcPin, WakeupLevel)] = &mut [(&mut pin5, WakeupLevel::Low)];
    let rtcio = RtcioWakeupSource::new(rtcio_pins);

    let mut temp: heapless::String<16> = heapless::String::try_from("<ERROR>").unwrap();
    let mut pressure: heapless::String<16> = heapless::String::try_from("<ERROR>").unwrap();

    defmt::info!("TEMP >> {}", bme280.read_temperature().await);
    defmt::info!("PRESSURE >> {}", bme280.read_pressure().await);

    defmt::info!("TEMP >> {}", bme280.read_temperature().await);
    defmt::info!("PRESSURE >> {}", bme280.read_pressure().await);

    if let Ok(Some(t)) = bme280.read_temperature().await {
        temp.clear();
        write!(temp, "{:.2}°C", t).unwrap();
    }
    if let Ok(Some(p)) = bme280.read_pressure().await {
        pressure.clear();
        write!(pressure, "{:.2} hPa", p).unwrap();
    }

    let mut msg: heapless::String<128> = heapless::String::try_from("<ERROR>").unwrap();
    write!(
        msg,
        "{} -> BME280: Temp = {} Pressure = {}",
        format_mac(&esp_radio::wifi::sta_mac()),
        temp,
        pressure
    )
    .unwrap();
    let status = esp_now.send(&HUB, msg.as_bytes());
    defmt::info!("SEND MSG: {:?}", status.is_ok());

    Timer::after_millis(10000).await;
    defmt::info!("SLEEP");
    rtc.sleep_deep(&[&timer, &rtcio]);
}
