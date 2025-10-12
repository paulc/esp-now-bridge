#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use esp_hal::gpio::{Level, RtcPinWithResistors};
#[cfg(target_arch = "riscv32")]
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::rmt::{Rmt, TxChannelConfig, TxChannelCreator};
use esp_hal::rtc_cntl::sleep::{RtcioWakeupSource, TimerWakeupSource, WakeupLevel};
use esp_hal::rtc_cntl::Rtc;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;

use defmt_rtt as _;
use esp_backtrace as _;

use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Ticker, Timer};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

use static_cell::make_static;

use esp_now_bridge::rgb::{Rgb, RgbLayout};
use esp_now_bridge::ws2812::{Ws2812Single, RMT_CLK_DIVIDER, RMT_FREQ_MHZ};

extern crate alloc;

static LED_SIGNAL: Signal<CriticalSectionRawMutex, bool> = Signal::new();

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    esp_alloc::heap_allocator!(size: 64 * 1024);

    defmt::info!("Init");

    #[cfg(target_arch = "riscv32")]
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(
        timg0.timer0,
        #[cfg(target_arch = "riscv32")]
        sw_int.software_interrupt0,
    );

    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(RMT_FREQ_MHZ))
        .expect("Error initialising RMT")
        .into_async();

    let tx_config = TxChannelConfig::default()
        .with_clk_divider(RMT_CLK_DIVIDER)
        .with_idle_output_level(Level::Low)
        .with_carrier_modulation(false);

    let gpio = peripherals.GPIO10;
    let channel = rmt.channel0.configure_tx(gpio, tx_config).unwrap();
    let ws2812 = Ws2812Single::new(channel, RgbLayout::Rgb);

    // LED task
    spawner
        .spawn(led_task(make_static!(ws2812)))
        .expect("Error spawning led task");

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
            LED_SIGNAL.signal(true);
            Timer::after_millis(100).await;
            rtc.sleep_deep(&[&timer, &rtcio]);
        }
        defmt::info!("Tick [{}]", count);
        Timer::after_millis(1000).await
    }
}

#[embassy_executor::task]
async fn led_task(ws2812: &'static mut Ws2812Single<'static>) {
    let mut ticker = Ticker::every(Duration::from_millis(500));
    loop {
        for c in [Rgb::new(20, 0, 0), Rgb::new(0, 20, 0), Rgb::new(0, 0, 20)] {
            match select(ticker.next(), LED_SIGNAL.wait()).await {
                Either::First(_) => ws2812.set(c).await.unwrap(), // Ticker
                Either::Second(_) => {
                    defmt::info!("LED OFF");
                    ws2812.set(Rgb::new(0, 0, 0)).await.unwrap(); // Signal
                    Timer::after_millis(10).await;
                    ws2812.set(Rgb::new(0, 0, 0)).await.unwrap(); // Signal
                }
            }
        }
    }
}
