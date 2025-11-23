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
use esp_hal::i2c;
#[cfg(target_arch = "riscv32")]
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::Async;

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;

use defmt_rtt as _;
use static_cell::StaticCell;

use esp_backtrace as _;

pub const APB_CLOCK_MHZ: u32 = 80;

static I2C_BUS: StaticCell<Mutex<NoopRawMutex, i2c::master::I2c<Async>>> = StaticCell::new();

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let rtc = esp_hal::rtc_cntl::Rtc::new(peripherals.LPWR);

    let boot_time = rtc.time_since_boot().as_millis();
    let wakeup_cause = esp_hal::rtc_cntl::wakeup_cause();

    defmt::info!("INIT: boot_time={} wakeup={}", boot_time, wakeup_cause);

    esp_alloc::heap_allocator!(size: 64 * 1024);

    #[cfg(target_arch = "riscv32")]
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(
        timg0.timer0,
        #[cfg(target_arch = "riscv32")]
        sw_int.software_interrupt0,
    );

    defmt::info!("ESP_RTOS initialized!");

    // Initialise I2C
    let i2c_config = i2c::master::Config::default().with_frequency(Rate::from_khz(100));
    let mut i2c = i2c::master::I2c::new(peripherals.I2C0, i2c_config)
        .expect("Error initailising I2C")
        .with_scl(peripherals.GPIO4)
        .with_sda(peripherals.GPIO5)
        .into_async();

    defmt::info!("Scan I2C bus: START");
    for addr in 0..=127 {
        if let Ok(_) = i2c.write_async(addr, &[0]).await {
            defmt::info!("Found I2C device at address: 0x{:02x}", addr);
        }
    }
    defmt::info!("Scan I2C bus: DONE");

    // Create shared I2C bus
    let i2c_bus = I2C_BUS.init(Mutex::new(i2c));

    let aht20_device = I2cDevice::new(i2c_bus);
    spawner.spawn(aht20_task(aht20_device)).unwrap();

    let bmp280_device = I2cDevice::new(i2c_bus);
    spawner.spawn(bmp280_task(bmp280_device)).unwrap();

    loop {
        Timer::after_millis(5000).await;
        let now = rtc.time_since_boot().as_millis();
        defmt::info!("Tick: [{}]", now - boot_time);
    }
}

#[embassy_executor::task]
async fn aht20_task(
    i2c: I2cDevice<'static, NoopRawMutex, esp_hal::i2c::master::I2c<'static, Async>>,
) {
    let mut aht20 = aht20::Aht20::new(i2c, 0x38);
    defmt::info!("AHT20 INIT: {}", aht20.init().await.unwrap());
    loop {
        let (rh, t) = aht20.read().await.unwrap();
        defmt::info!("AHT20  TEMP: {}°C", t);
        defmt::info!("       HUMIDITY: {}%", rh);
        Timer::after_millis(5000).await;
    }
}

#[embassy_executor::task]
async fn bmp280_task(
    i2c: I2cDevice<'static, NoopRawMutex, esp_hal::i2c::master::I2c<'static, Async>>,
) {
    let mut bmp280 = bmp280::Bmp280::new(i2c, 0x77);
    defmt::info!("BMP280 RESET: {}", bmp280.reset().await.unwrap());
    defmt::info!("BMP280 ID: 0x{:x}", bmp280.id().await.unwrap());
    defmt::info!("BMP280 INIT: {}", bmp280.init_default().await.unwrap());
    defmt::info!("BMP280 WAIT: {}", bmp280.wait().await.unwrap());
    loop {
        defmt::info!("BMP280 TEMP: {}°C", bmp280.temp().await.unwrap());
        defmt::info!("       PRESSURE: {}hPa", bmp280.pressure().await.unwrap());
        Timer::after_millis(5000).await;
    }
}

mod aht20 {

    #![allow(unused)]

    use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
    use embassy_sync::blocking_mutex::raw::RawMutex;
    use embedded_hal_async::i2c::I2c;

    #[derive(Debug)]
    pub enum Aht20Error {
        I2cError,
        CrcError,
    }

    pub struct Aht20<'a, M, BUS>
    where
        M: RawMutex,
        BUS: I2c,
    {
        i2c: I2cDevice<'a, M, BUS>,
        address: u8,
    }

    impl<'a, M, BUS> Aht20<'a, M, BUS>
    where
        M: RawMutex,
        BUS: I2c,
    {
        pub fn new(i2c: I2cDevice<'a, M, BUS>, address: u8) -> Self {
            Self { i2c, address }
        }
        pub async fn init(&mut self) -> Result<(), Aht20Error> {
            self.i2c
                .write(self.address, &[0xBA])
                .await
                .map_err(|_| Aht20Error::I2cError)?;
            // Wait 40ms (reset time)
            embassy_time::Timer::after_millis(40).await;

            // Initialize (normal mode, enable heater off, enable CRC)
            self.i2c
                .write(self.address, &[0xBE, 0x08, 0x00])
                .await
                .map_err(|_| Aht20Error::I2cError)?;
            // Wait for calibration
            embassy_time::Timer::after_millis(10).await;
            Ok(())
        }
        pub async fn read(&mut self) -> Result<(f32, f32), Aht20Error> {
            self.i2c
                .write(self.address, &[0xAC, 0x33, 0x00])
                .await
                .map_err(|_| Aht20Error::I2cError)?;

            let mut buf = [0u8; 7];
            // Poll device for ready instead of waiting 80ms
            loop {
                self.i2c
                    .read(self.address, &mut buf[..1])
                    .await
                    .map_err(|_| Aht20Error::I2cError)?;
                if buf[0] & 0x80 == 0 {
                    // Bit 7 = 0 means not busy
                    break;
                }
                embassy_time::Timer::after_millis(10).await;
            }

            // Read result
            self.i2c
                .read(self.address, &mut buf)
                .await
                .map_err(|_| Aht20Error::I2cError)?;

            // Check CRC
            if crc8(&buf[..6]) != buf[6] {
                return Err(Aht20Error::CrcError);
            }

            let rh =
                (((buf[1] as u32) << 12) | ((buf[2] as u32) << 4) | ((buf[3] as u32) >> 4)) as f32;
            let temp = ((((buf[3] as u32) & 0x0F) << 16) | ((buf[4] as u32) << 8) | (buf[5] as u32))
                as f32;

            Ok((rh * 100.0 / 1_048_576.0, temp * 200.0 / 1_048_576.0 - 50.0))
        }
    }

    fn crc8(data: &[u8]) -> u8 {
        let mut crc = 0xFFu8;
        for &b in data {
            crc ^= b;
            for _ in 0..8 {
                crc = if crc & 0x80 != 0 {
                    (crc << 1) ^ 0x31
                } else {
                    crc << 1
                };
            }
        }
        crc
    }
}

mod bmp280 {

    #![allow(unused)]

    use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
    use embassy_sync::blocking_mutex::raw::RawMutex;
    use embedded_hal_async::i2c::I2c;

    const ID: u8 = 0xD0;
    const RESET: u8 = 0xE0;
    const STATUS: u8 = 0xF3;
    const CTRL_MEAS: u8 = 0xF4;
    const CONFIG: u8 = 0xF5;
    const PRESSURE: u8 = 0xF7;
    const TEMP: u8 = 0xFA;
    const CALIB: u8 = 0x88;

    #[derive(Debug)]
    pub enum Bme280Error {
        I2cError,
        NoCalibrationData,
    }

    #[derive(Debug)]
    pub enum Mode {
        Sleep = 0b00,
        Forced = 0b01,
        Normal = 0b11,
    }

    #[derive(Debug)]
    pub enum Oversample {
        Skip = 0b000,
        X1 = 0b001,
        X2 = 0b010,
        X4 = 0b011,
        X8 = 0b100,
        X16 = 0b101,
    }

    #[derive(Debug)]
    pub enum Filter {
        Off = 0x000,
        X2 = 0b001,
        X4 = 0b010,
        X8 = 0b011,
        X16 = 0b100,
    }

    #[derive(Debug)]
    pub enum Standby {
        T0_5 = 0b000,
        T62_5 = 0b001,
        T125 = 0b010,
        T250 = 0b011,
        T500 = 0b100,
        T1000 = 0b101,
        T2000 = 0b110,
        T4000 = 0b111,
    }

    pub struct Bmp280<'a, M, BUS>
    where
        M: RawMutex,
        BUS: I2c,
    {
        i2c: I2cDevice<'a, M, BUS>,
        address: u8,
        t_calib: Option<[f32; 3]>,
        p_calib: Option<[f32; 9]>,
        t_fine: f32,
    }

    impl<'a, M, BUS> Bmp280<'a, M, BUS>
    where
        M: RawMutex,
        BUS: I2c,
    {
        pub fn new(i2c: I2cDevice<'a, M, BUS>, address: u8) -> Self {
            Self {
                i2c,
                address,
                t_calib: None,
                p_calib: None,
                t_fine: 0.0,
            }
        }
        pub async fn init(
            &mut self,
            mode: Mode,
            os_t: Oversample,
            os_p: Oversample,
            t_sb: Standby,
            filter: Filter,
        ) -> Result<(), Bme280Error> {
            self.calibrate().await?;
            self.set_config(t_sb, filter).await?;
            self.set_ctrl_meas(os_t, os_p, mode).await?;
            Ok(())
        }
        pub async fn init_default(&mut self) -> Result<(), Bme280Error> {
            self.calibrate().await?;
            // Default: normal mode, X1 oversampling, 16x pressure filter, 500ms standby
            self.set_config(Standby::T500, Filter::X16).await?;
            self.set_ctrl_meas(Oversample::X1, Oversample::X16, Mode::Normal)
                .await?;
            Ok(())
        }
        pub async fn init_low_power(&mut self) -> Result<(), Bme280Error> {
            self.calibrate().await?;
            // Low power: sleep mode, minimal oversampling, no filter
            self.set_config(Standby::T0_5, Filter::Off).await?;
            self.set_ctrl_meas(Oversample::Skip, Oversample::X1, Mode::Sleep)
                .await?;
            Ok(())
        }
        pub async fn wait(&mut self) -> Result<(), Bme280Error> {
            // Wait for completion
            loop {
                let (measuring, _) = self.status().await?;
                if !measuring {
                    break Ok(());
                }
                embassy_time::Timer::after_millis(10).await;
            }
        }
        pub async fn force_measurement(&mut self) -> Result<(f32, f32), Bme280Error> {
            self.set_ctrl_meas(Oversample::X2, Oversample::X16, Mode::Forced)
                .await?;
            // Wait for measurement to compete
            self.wait().await?;
            let temp = self.temp().await?;
            let pressure = self.pressure().await?;

            // Return to sleep to save power
            self.set_ctrl_meas(Oversample::X2, Oversample::X16, Mode::Sleep)
                .await?;

            Ok((temp, pressure))
        }
        pub async fn id(&mut self) -> Result<u8, Bme280Error> {
            let mut buf = [0_u8; 1];
            self.read_register(ID, &mut buf).await?;
            Ok(buf[0])
        }
        pub async fn reset(&mut self) -> Result<(), Bme280Error> {
            let buf: [u8; 2] = [RESET, 0xB6];
            self.i2c
                .write(self.address, &buf)
                .await
                .map_err(|_| Bme280Error::I2cError)
        }
        pub async fn status(&mut self) -> Result<(bool, bool), Bme280Error> {
            let mut buf = [0_u8; 1];
            self.read_register(STATUS, &mut buf).await?;
            // (measuring,im_update)
            Ok((buf[0] & 0b0000_1000 != 0, buf[0] & 0b0000_0001 != 0))
        }
        pub async fn set_ctrl_meas(
            &mut self,
            os_t: Oversample,
            os_p: Oversample,
            mode: Mode,
        ) -> Result<(), Bme280Error> {
            let buf: [u8; 2] = [
                CTRL_MEAS,
                ((os_t as u8) << 5) | ((os_p as u8) << 2) | mode as u8,
            ];
            self.i2c
                .write(self.address, &buf)
                .await
                .map_err(|_| Bme280Error::I2cError)
        }
        pub async fn ctrl_meas(&mut self) -> Result<u8, Bme280Error> {
            let mut buf = [0_u8; 1];
            self.read_register(CTRL_MEAS, &mut buf).await?;
            Ok(buf[0])
        }
        pub async fn set_config(
            &mut self,
            t_sb: Standby,
            filter: Filter,
        ) -> Result<(), Bme280Error> {
            let buf: [u8; 2] = [CONFIG, ((t_sb as u8) << 5) | ((filter as u8) << 2)];
            self.i2c
                .write(self.address, &buf)
                .await
                .map_err(|_| Bme280Error::I2cError)
        }
        pub async fn config(&mut self) -> Result<u8, Bme280Error> {
            let mut buf = [0_u8; 1];
            self.read_register(CONFIG, &mut buf).await?;
            Ok(buf[0])
        }
        pub async fn temp(&mut self) -> Result<f32, Bme280Error> {
            if self.t_calib.is_none() {
                return Err(Bme280Error::NoCalibrationData);
            }
            let mut buf = [0_u8; 3];
            self.read_register(TEMP, &mut buf).await?;
            let t_raw =
                (((buf[0] as u32) << 12) + ((buf[1] as u32) << 4) + ((buf[2] as u32) & 0xf)) as f32;
            self.compensate_t(t_raw)
        }
        pub async fn pressure(&mut self) -> Result<f32, Bme280Error> {
            if self.p_calib.is_none() {
                return Err(Bme280Error::NoCalibrationData);
            }
            let mut buf = [0_u8; 3];
            self.read_register(PRESSURE, &mut buf).await?;
            let p_raw =
                (((buf[0] as u32) << 12) + ((buf[1] as u32) << 4) + ((buf[2] as u32) & 0xf)) as f32;
            self.compensate_p(p_raw)
        }
        pub async fn calibrate(&mut self) -> Result<(), Bme280Error> {
            let mut buf = [0u8; 24];
            self.read_register(CALIB, &mut buf).await?;
            self.t_calib = Some([
                u16::from_le_bytes([buf[0], buf[1]]) as f32,
                i16::from_le_bytes([buf[2], buf[3]]) as f32,
                i16::from_le_bytes([buf[4], buf[5]]) as f32,
            ]);
            self.p_calib = Some([
                u16::from_le_bytes([buf[6], buf[7]]) as f32,
                i16::from_le_bytes([buf[8], buf[9]]) as f32,
                i16::from_le_bytes([buf[10], buf[11]]) as f32,
                i16::from_le_bytes([buf[12], buf[13]]) as f32,
                i16::from_le_bytes([buf[14], buf[15]]) as f32,
                i16::from_le_bytes([buf[16], buf[17]]) as f32,
                i16::from_le_bytes([buf[18], buf[19]]) as f32,
                i16::from_le_bytes([buf[20], buf[21]]) as f32,
                i16::from_le_bytes([buf[22], buf[23]]) as f32,
            ]);
            Ok(())
        }
        pub async fn read_register(
            &mut self,
            register: u8,
            buf: &mut [u8],
        ) -> Result<(), Bme280Error> {
            self.i2c
                .write(self.address, &[register])
                .await
                .map_err(|_| Bme280Error::I2cError)?;
            self.i2c
                .read(self.address, buf)
                .await
                .map_err(|_| Bme280Error::I2cError)
        }
        fn compensate_t(&mut self, adc_t: f32) -> Result<f32, Bme280Error> {
            match self.t_calib {
                Some([t1, t2, t3]) => {
                    let var1 = (adc_t / 16384.0 - t1 / 1024.0) * t2;
                    let var2 =
                        ((adc_t / 131072.0 - t1 / 8192.0) * (adc_t / 131072.0 - t1 / 8192.0)) * t3;
                    self.t_fine = var1 + var2;
                    Ok((var1 + var2) / 5120.0)
                }
                None => Err(Bme280Error::NoCalibrationData),
            }
        }
        fn compensate_p(&self, adc_p: f32) -> Result<f32, Bme280Error> {
            match self.p_calib {
                Some([p1, p2, p3, p4, p5, p6, p7, p8, p9]) => {
                    let mut var1: f32;
                    let mut var2: f32;
                    let mut p: f32;
                    var1 = (self.t_fine / 2.0) - 64000.0;
                    var2 = (var1 * var1 * p6) / 32768.0;
                    var2 = var2 + var1 * p5 * 2.0;
                    var2 = (var2 / 4.0) + (p4 * 65536.0);
                    var1 = (p3 * var1 * var1 / 524288.0 + p2 * var1) / 524288.0;
                    var1 = (1.0 + var1 / 32768.0) * p1;
                    if var1 == 0.0 {
                        Ok(0.0)
                    } else {
                        p = 1048576.0 - adc_p;
                        p = (p - (var2 / 4096.0)) * 6250.0 / var1;
                        var1 = p9 * p * p / 2147483648.0;
                        var2 = p * p8 / 32768.0;
                        p = p + (var1 + var2 + p7) / 16.0;
                        // Convert from Pa -> HPa
                        Ok(p / 100.0)
                    }
                }
                None => Err(Bme280Error::NoCalibrationData),
            }
        }
    }
}
