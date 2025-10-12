#![no_std]
#![no_main]
#![feature(extend_one)]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use esp_hal::timer::timg::TimerGroup;

use esp_hal::usb_serial_jtag::{UsbSerialJtag, UsbSerialJtagRx, UsbSerialJtagTx};
use esp_hal::Async;

use defmt_rtt as _;
use esp_backtrace as _;

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use embassy_time::{Duration, Ticker};

use core::fmt::Write;
use static_cell::StaticCell;

extern crate alloc;

const BUFFER_SIZE: usize = 256;
const USB_BUFFER_SIZE: usize = 64;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    defmt::info!("Init!");

    #[cfg(target_arch = "riscv32")]
    let sw_int =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(
        timg0.timer0,
        #[cfg(target_arch = "riscv32")]
        sw_int.software_interrupt0,
    );

    let (rx, tx) = UsbSerialJtag::new(peripherals.USB_DEVICE)
        .into_async()
        .split();

    static SIGNAL: StaticCell<Signal<NoopRawMutex, heapless::String<BUFFER_SIZE>>> =
        StaticCell::new();
    let signal = &*SIGNAL.init(Signal::new());

    spawner.spawn(reader(rx, &signal)).unwrap();
    spawner.spawn(writer(tx, &signal)).unwrap();

    let mut ticker = Ticker::every(Duration::from_millis(5_000));

    loop {
        defmt::info!("Bing!");
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn writer(
    mut tx: UsbSerialJtagTx<'static, Async>,
    signal: &'static Signal<NoopRawMutex, heapless::String<BUFFER_SIZE>>,
) {
    embedded_io_async::Write::write_all(
        &mut tx,
        b"Hello async USB Serial JTAG. Type something.\r\n",
    )
    .await
    .unwrap();
    loop {
        let message = signal.wait().await;
        signal.reset();
        write!(&mut tx, "-- received '{}' --\r\n", message).unwrap();
        embedded_io_async::Write::flush(&mut tx).await.unwrap();
    }
}

#[embassy_executor::task]
async fn reader(
    mut rx: UsbSerialJtagRx<'static, Async>,
    signal: &'static Signal<NoopRawMutex, heapless::String<BUFFER_SIZE>>,
) {
    let mut rbuf = [0u8; USB_BUFFER_SIZE];
    let mut string_buffer: heapless::Vec<u8, BUFFER_SIZE> = heapless::Vec::new();
    loop {
        let r = embedded_io_async::Read::read(&mut rx, &mut rbuf).await;
        match r {
            Ok(len) => {
                defmt::info!("Serial RX: {}", len);
                rbuf.iter()
                    .take(len)
                    // .inspect(|c| defmt::info!(">> {}", c))
                    .for_each(|c| match c {
                        b'\r' => {
                            defmt::info!("Found CR");
                            if let Ok(line) = heapless::String::from_utf8(string_buffer.clone()) {
                                defmt::info!("Line: {}", line.as_str());
                                signal.signal(line);
                            } else {
                                signal.signal(heapless::format!("Invalid UTF8 string").unwrap());
                            }
                            string_buffer.clear();
                        }
                        b'\n' => {
                            defmt::info!("Found NL");
                        }
                        &c => {
                            if string_buffer.is_full() {
                                let line = heapless::String::from_utf8(string_buffer.clone())
                                    .expect("UTF8 error");
                                defmt::info!("Buffer Full: {}", line.as_str());
                                signal.signal(line);
                                string_buffer.clear();
                            }
                            string_buffer.extend_one(c);
                        }
                    });
            }
            #[allow(unreachable_patterns)]
            Err(e) => defmt::error!("RX Error: {:?}", e),
        }
    }
}
