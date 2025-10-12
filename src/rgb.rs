#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RgbLayout {
    Rgb,
    Grb,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub struct Rgb {
    r: u8,
    g: u8,
    b: u8,
}
pub mod colour {
    use super::Rgb;
    pub const OFF: Rgb = Rgb { r: 0, g: 0, b: 0 };
    pub const RED: Rgb = Rgb { r: 255, g: 0, b: 0 };
    pub const GREEN: Rgb = Rgb { r: 0, g: 255, b: 0 };
    pub const BLUE: Rgb = Rgb { r: 0, g: 0, b: 255 };
    pub const WHITE: Rgb = Rgb {
        r: 255,
        g: 255,
        b: 255,
    };
}

impl Rgb {
    pub fn new(r: u8, g: u8, b: u8) -> Self {
        Self { r, g, b }
    }
    #[inline]
    pub fn to_u32(&self, format: RgbLayout) -> u32 {
        match format {
            RgbLayout::Rgb => ((self.r as u32) << 16) | ((self.g as u32) << 8) | self.b as u32,
            RgbLayout::Grb => ((self.g as u32) << 16) | ((self.r as u32) << 8) | self.b as u32,
        }
    }
}

impl Default for Rgb {
    fn default() -> Self {
        colour::OFF
    }
}
