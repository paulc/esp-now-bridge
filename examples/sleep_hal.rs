#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Level, Output, OutputConfig, RtcPinWithResistors};
use esp_hal::main;
use esp_hal::rtc_cntl::sleep::{RtcioWakeupSource, TimerWakeupSource, WakeupLevel};
use esp_hal::rtc_cntl::Rtc;

use defmt_rtt as _;
use esp_backtrace as _;

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::_80MHz));
    esp_alloc::heap_allocator!(size: 64 * 1024);

    defmt::info!("Init");

    let mut led_pin = Output::new(peripherals.GPIO6, Level::High, OutputConfig::default());
    let delay = esp_hal::delay::Delay::new();

    for _ in 0..2 {
        led_pin.set_low();
        delay.delay_millis(250);
        led_pin.set_high();
        delay.delay_millis(250);
    }

    let mut rtc = Rtc::new(peripherals.LPWR);
    let timer = TimerWakeupSource::new(core::time::Duration::from_secs(55));
    let mut pin5 = peripherals.GPIO5;
    let mut rtcio_pins: &mut [(&mut dyn RtcPinWithResistors, WakeupLevel)] =
        &mut [(&mut pin5, WakeupLevel::Low)];
    let rtcio = RtcioWakeupSource::new(&mut rtcio_pins);

    let mut count = 0_u32;
    loop {
        count += 1;
        if count.is_multiple_of(5) {
            defmt::info!("SLEEP");
            rtc.sleep_deep(&[&timer, &rtcio]);
        }
        defmt::info!("Tick [{}]", count);
        //Timer::after_millis(1000).await
        delay.delay_millis(1000);
    }
}
