use esp_hal::gpio::Level;
use esp_hal::rmt::{Channel, PulseCode, Tx};
use esp_hal::Async;

use crate::rgb::{Rgb, RgbLayout};

use anyhow;

pub const RMT_FREQ_MHZ: u32 = 80;
pub const RMT_CLK_DIVIDER: u8 = 2;

// WS2812 timings: 1us = RMT_FREQ / RMT_CLK_DIVIDER
const RMT_CHAN_FREQ: u16 = RMT_FREQ_MHZ as u16 / RMT_CLK_DIVIDER as u16;
const T0H: u16 = RMT_CHAN_FREQ * 400 / 1000; // 0.4us
const T0L: u16 = RMT_CHAN_FREQ * 850 / 1000; // 0.85us
const T1H: u16 = RMT_CHAN_FREQ * 800 / 1000; // 0.8us
const T1L: u16 = RMT_CHAN_FREQ * 450 / 1000; // 0.45us
const RESET_H: u16 = RMT_CHAN_FREQ * 50; // 50us

const T0: PulseCode = PulseCode::new(Level::High, T0H, Level::Low, T0L);
const T1: PulseCode = PulseCode::new(Level::High, T1H, Level::Low, T1L);
const RESET: PulseCode = PulseCode::new(Level::Low, RESET_H, Level::High, 0);

/*
    Ws2812Single
    ============

    // Create RMT Device
    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(ws2812_single::RMT_FREQ_MHZ))
        .expect("Error initialising RMT")
        .into_async();

    // Set TX config
    let tx_config = TxChannelConfig::default()
        .with_clk_divider(ws2812_single::RMT_CLK_DIVIDER)
        .with_idle_output_level(Level::Low)
        .with_carrier_modulation(false);

    // Create channel
    let gpio = peripherals.GPIO10;
    let channel = rmt.channel0.configure_tx(gpio, tx_config).unwrap();

    // Pass to WS2812 driver
    let ws2812 = ws2812::Ws2812Single::new(channel, RgbLayout::Rgb);
*/
pub struct Ws2812Single<'a> {
    channel: Channel<'a, Async, Tx>,
    rgb_layout: RgbLayout,
}

impl<'a> Ws2812Single<'a> {
    pub fn new(channel: Channel<'a, Async, Tx>, rgb_layout: RgbLayout) -> Self {
        Self {
            channel,
            rgb_layout,
        }
    }

    pub async fn set(&mut self, colour: Rgb) -> anyhow::Result<()> {
        let c = colour.to_u32(self.rgb_layout);

        // Generate pulses
        let mut pulses = [PulseCode::default(); 25];
        #[allow(clippy::needless_range_loop)]
        for i in 0..24 {
            // Send MSB first
            let bit = (c >> (23 - i)) & 1;
            pulses[i] = if bit == 0 { T0 } else { T1 };
        }
        pulses[24] = RESET;

        // Transmit the data
        self.channel
            .transmit(&pulses)
            .await
            .map_err(|e| anyhow::anyhow!("RMT Error: {e}"))
    }
}
