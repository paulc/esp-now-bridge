#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use esp_backtrace as _;
use esp_hal::i2c;
#[cfg(target_arch = "riscv32")]
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::rmt::Rmt;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::Async;

use esp_radio::esp_now::BROADCAST_ADDRESS;

use defmt_rtt as _;
use esp_backtrace as _;

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Duration, Timer};

use core::fmt::Write;

use static_cell::{make_static, StaticCell};

use aht20_async::Aht20;
use bme280_rs;

use esp_now_bridge::ds18b20::{check_onewire_crc, Ds18b20};
use esp_now_bridge::esp_hal_rmt_onewire::{OneWire, Search};
use esp_now_bridge::format_mac::format_mac;

pub const RMT_FREQ_MHZ: u32 = 80;
static I2C_BUS: StaticCell<Mutex<NoopRawMutex, i2c::master::I2c<Async>>> = StaticCell::new();

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
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
    for addr in 0..=127 {
        if let Ok(_) = i2c.write_async(addr, &[0]).await {
            defmt::info!("Found I2C device at address: 0x{:02x}", addr);
        }
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

    defmt::info!("");

    loop {
        for ds in &ds18b20 {
            if let Ok(temp) = ds.read_temp(&mut ow).await {
                let mut addr = heapless::String::<32>::new();
                write!(addr, "{}", ds.address).unwrap();
                defmt::info!("DS18B20: Temp = {}°C [{}]", temp, addr);
            }
            let _ = ds.initiate_conversion(&mut ow).await;
        }
        if let Ok((h, t)) = aht20.read().await {
            defmt::info!("AHT20:   Temp = {}°C / Humidity = {}%", t.celsius(), h.rh());
        }
        if let Ok(Some(t)) = bme280.read_temperature().await {
            if let Ok(Some(p)) = bme280.read_pressure().await {
                defmt::info!("BME280:  Temp = {}°C / Pressure = {} hPa", t, p / 100.0);
            }
        }
        defmt::info!("");

        let status = esp_now.send_async(&BROADCAST_ADDRESS, b"BROADCAST").await;
        defmt::info!("ESP-NOW  BROADCAST: {}", status);

        Timer::after(Duration::from_secs(5)).await;
    }
}
