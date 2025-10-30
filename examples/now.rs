#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(extend_one)]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use esp_hal::timer::timg::TimerGroup;

use esp_hal::usb_serial_jtag::{UsbSerialJtag, UsbSerialJtagRx, UsbSerialJtagTx};
use esp_hal::Async;

#[cfg(target_arch = "riscv32")]
use esp_hal::interrupt::software::SoftwareInterruptControl;

use esp_radio::esp_now::{PeerInfo, BROADCAST_ADDRESS};

use defmt_rtt as _;
use esp_backtrace as _;

use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::{Channel, Receiver};
use embassy_time::{Duration, Ticker};

use core::fmt::Write;

use static_cell::make_static;

use esp_now_bridge::format_mac::format_mac;

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    defmt::info!("Init!");

    esp_alloc::heap_allocator!(size: 64 * 1024);

    // Start RTOS
    #[cfg(target_arch = "riscv32")]
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(
        timg0.timer0,
        #[cfg(target_arch = "riscv32")]
        sw_int.software_interrupt0,
    );

    // Start ESP-NOW
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

    // Start serial
    let (rx, tx) = UsbSerialJtag::new(peripherals.USB_DEVICE)
        .into_async()
        .split();

    let now_channel = make_static!(Channel::<NoopRawMutex, heapless::Vec<u8, 255>, 2>::new());

    spawner.spawn(writer(tx, now_channel.receiver())).unwrap();
    spawner.spawn(reader(rx)).unwrap();

    // Main Loop
    let mut ticker = Ticker::every(Duration::from_secs(10));
    loop {
        let res = select(ticker.next(), async {
            // Wait for ESP-NOW message
            let r = esp_now.receive_async().await;
            if r.info.dst_address != BROADCAST_ADDRESS {
                let mut data: heapless::Vec<u8, 255> = heapless::Vec::new();
                data.extend_from_slice(r.data()).unwrap();
                defmt::info!(
                    "RX: [{}]->[{}] >> {} [rssi={}]",
                    format_mac(&r.info.src_address),
                    format_mac(&r.info.dst_address),
                    data,
                    r.info.rx_control.rssi
                );

                // Test Postcard data decoding
                // match postcard::from_bytes::<sensor_message::SensorMessage>(&data) {
                //     Ok(msg) => defmt::info!(">> Postcard Data: {:?}", msg.mac),
                //     Err(_) => defmt::error!(">> Postcard Error"),
                // }

                now_channel.send(data).await;
                // Add peer is necessary
                if !esp_now.peer_exists(&r.info.src_address) {
                    defmt::info!("ADD PEER: {}", format_mac(&r.info.src_address));
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
                // Respond to peer
                let mut msg: heapless::String<64> = heapless::String::new();
                write!(
                    msg,
                    "{} -> OK: RSSI={}",
                    format_mac(&esp_radio::wifi::sta_mac()),
                    r.info.rx_control.rssi
                )
                .unwrap();
                let status = esp_now
                    .send_async(&r.info.src_address, msg.as_bytes())
                    .await;
                defmt::info!("REPLY PEER: {:?}", status);
            } else {
                defmt::info!(
                    "RX BROADCAST: [{}]->[{}] >> {} [rssi={}]",
                    format_mac(&r.info.src_address),
                    format_mac(&r.info.dst_address),
                    core::str::from_utf8(r.data()).unwrap_or("<UTF8 Error>"),
                    r.info.rx_control.rssi
                );
            }
        })
        .await;

        match res {
            Either::First(_) => {
                let status = esp_now.send_async(&BROADCAST_ADDRESS, b"BROADCAST").await;
                defmt::info!("SENDING BROADCAST: {}", status);
            }
            Either::Second(_) => (),
        }
    }
}

#[embassy_executor::task]
async fn writer(
    mut tx: UsbSerialJtagTx<'static, Async>,
    channel_rx: Receiver<'static, NoopRawMutex, heapless::Vec<u8, 255>, 2>,
) {
    embedded_io_async::Write::write_all(&mut tx, b"ESP-NOW BRIDGE\r\n")
        .await
        .unwrap();
    loop {
        embedded_io_async::Write::write_all(&mut tx, b"XXXYYYZZZ")
            .await
            .unwrap();
        let message = channel_rx.receive().await;
        let len = (message.len() as u16).to_le_bytes();
        embedded_io_async::Write::write_all(&mut tx, &[0x00, 0xff, 0x00, 0xff])
            .await
            .unwrap();
        embedded_io_async::Write::write_all(&mut tx, &len)
            .await
            .unwrap();
        embedded_io_async::Write::write_all(&mut tx, &message)
            .await
            .unwrap();
        embedded_io_async::Write::flush(&mut tx).await.unwrap();
    }
}

const BUFFER_SIZE: usize = 256;
const USB_BUFFER_SIZE: usize = 64;

#[embassy_executor::task]
async fn reader(mut rx: UsbSerialJtagRx<'static, Async>) {
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
