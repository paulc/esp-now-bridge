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
use esp_hal::main;
#[cfg(feature = "esp32c6")]
use esp_hal::rtc_cntl::sleep::{Ext1WakeupSource, TimerWakeupSource, WakeupLevel};
#[cfg(any(feature = "esp32c3", feature = "esp32s3"))]
use esp_hal::rtc_cntl::sleep::{RtcioWakeupSource, TimerWakeupSource, WakeupLevel};
use esp_hal::rtc_cntl::Rtc;
use esp_hal::system::SleepSource;

use defmt_rtt as _;
use esp_backtrace as _;

#[esp_hal::ram(unstable(rtc_fast, persistent))]
static mut COUNTER: u32 = 0;

// This creates a default app-descriptor required by the esp-idf bootloader.
esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    esp_alloc::heap_allocator!(size: 64 * 1024);
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // Setup RTC & Sleep Timer
    let mut rtc = Rtc::new(peripherals.LPWR);
    let timer = TimerWakeupSource::new(core::time::Duration::from_secs(10));
    let mut pin_1 = peripherals.GPIO1;

    {
        // XXX This doesnt seem to work reliably across devices - probably need to use external pull-up
        use esp_hal::gpio::RtcPinWithResistors;
        pin_1.rtcio_pullup(true);
    }

    let wakeup_pins: &mut [(&mut dyn RtcIoWakeupPinType, WakeupLevel)] =
        &mut [(&mut pin_1, WakeupLevel::Low)];
    #[cfg(not(feature = "esp32c6"))]
    let rtcio = RtcioWakeupSource::new(wakeup_pins);
    #[cfg(feature = "esp32c6")]
    let rtcio = Ext1WakeupSource::new(wakeup_pins);

    let delay = esp_hal::delay::Delay::new();

    let boot_time = rtc.time_since_boot().as_millis();
    let wakeup_cause = esp_hal::rtc_cntl::wakeup_cause();

    defmt::info!("INIT: wakeup={} boot={}", wakeup_cause, boot_time);

    defmt::info!("COUNTER: {}", unsafe { COUNTER });
    match wakeup_cause {
        SleepSource::Undefined => {
            for i in 0..5 {
                defmt::info!("WAIT [{}]", i);
                delay.delay_millis(1000);
            }
        }
        _ => {}
    }
    unsafe { COUNTER += 1 };

    defmt::info!("SLEEPING:");
    delay.delay_millis(2000);
    rtc.sleep_deep(&[&timer, &rtcio]);
}
