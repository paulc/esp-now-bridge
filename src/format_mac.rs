use core::fmt::Write;

const MAC_FMT_LEN: usize = 17; // "xx:xx:xx:xx:xx:xx" = 17 chars

pub fn format_mac(mac: &[u8; 6]) -> heapless::String<MAC_FMT_LEN> {
    let mut s: heapless::String<MAC_FMT_LEN> = heapless::String::new();
    for (i, &byte) in mac.iter().enumerate() {
        if i > 0 {
            s.push(':').unwrap(); // Won't fail because we know capacity is enough
        }
        // Format byte as two lowercase hex digits
        write!(s, "{:02x}", byte).unwrap(); // `write!` works with heapless::String
    }
    s
}
