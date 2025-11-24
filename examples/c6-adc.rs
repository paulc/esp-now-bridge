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
use esp_hal::analog::adc::{Adc, AdcCalBasic, AdcConfig, Attenuation};
#[cfg(target_arch = "riscv32")]
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::timer::timg::TimerGroup;

use embassy_executor::Spawner;

use embassy_time::Timer;

use defmt_rtt as _;

use esp_backtrace as _;

// This creates a default app-descriptor required by the esp-idf bootloader.
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
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

    // ADC Task
    let adc_pin = peripherals.GPIO0;
    let mut adc_config = AdcConfig::new();
    let mut pin = adc_config.enable_pin_with_cal::<_, AdcCalBasic<_>>(adc_pin, Attenuation::_11dB);
    let mut adc = Adc::new(peripherals.ADC1, adc_config).into_async();

    let v_ref = 1.1; // ADC refreence voltage
    let k = 3.981; // 11dB scaling factor

    loop {
        let adc_raw = adc.read_oneshot(&mut pin).await;
        let adc_v = v_ref * k * (adc_raw as f32 / 4095.0);
        defmt::info!("ADC Value: {}V [{}]", adc_v, adc_raw);
        Timer::after_millis(1000).await;
    }
}
