#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use esp_hal::timer::timg::TimerGroup;

#[cfg(target_arch = "riscv32")]
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_radio::esp_now::{PeerInfo, BROADCAST_ADDRESS};

use defmt_rtt as _;
use esp_backtrace as _;

use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Ticker};

use core::fmt::Write;

use static_cell::make_static;

use esp_now_bridge::format_mac::format_mac;

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    defmt::info!("Init!");

    esp_alloc::heap_allocator!(size: 64 * 1024);

    #[cfg(target_arch = "riscv32")]
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(
        timg0.timer0,
        #[cfg(target_arch = "riscv32")]
        sw_int.software_interrupt0,
    );

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
    let mut ticker = Ticker::every(Duration::from_secs(10));
    loop {
        let res = select(ticker.next(), async {
            let r = esp_now.receive_async().await;
            defmt::info!(
                "RX: [{}]->[{}] >> {} [rssi={}]",
                format_mac(&r.info.src_address),
                format_mac(&r.info.dst_address),
                core::str::from_utf8(r.data()).unwrap_or("UTF8 Error"),
                r.info.rx_control.rssi
            );
            // if r.info.dst_address == BROADCAST_ADDRESS {}
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
            // Respond to peer
            msg.clear();
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
            defmt::info!("REPLY: {:?}", status);
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
