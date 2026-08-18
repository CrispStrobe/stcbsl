// SPDX-License-Identifier: MIT
// rxprobe v2 — bench instrument against a live 9600-baud chatterer:
// exercise stcbsl's OWN SerialWire (open, set_baud incl. the macOS
// IOSSIOSPEED path) and report byte counts AND legibility per rate.
// A mismatched rate must still yield garbage BYTES (python at 115200
// read 203B of the 9600 stream); zero bytes at any rate = the layer
// where the flasher goes deaf, reproduced without the bootloader.
use std::time::{Duration, Instant};

use stcbsl::driver::Wire;
use stcbsl::transport::SerialWire;

fn count(w: &mut SerialWire, secs: u64, label: &str) {
    let mut n = 0usize;
    let mut sample = Vec::new();
    let mut buf = [0u8; 256];
    let end = Instant::now() + Duration::from_secs(secs);
    while Instant::now() < end {
        match Wire::read(w, &mut buf) {
            Ok(k) => {
                n += k;
                if sample.len() < 40 {
                    sample.extend_from_slice(&buf[..k.min(40)]);
                }
            }
            Err(e) => {
                println!("{label}: read error {e}");
                return;
            }
        }
    }
    let legible = sample.windows(5).any(|w| w == b"hello");
    println!("{label}: {n} bytes in {secs}s, legible={legible}");
}

fn main() {
    let dev = std::env::args().nth(1).expect("usage: rxprobe <device>");
    let mut w = SerialWire::open(&dev, 2400, stcbsl::transport::WIRE_PARITY)
        .expect("open");
    println!("opened at 2400 via SerialWire");
    count(&mut w, 3, "phase 1 @2400 (mismatch: expect garbage bytes)");
    w.set_baud(9600).expect("set 9600");
    count(&mut w, 4, "phase 2 @9600 (match: expect legible)");
    // phase 2b: the REAL session's chain — write, tcdrain, switch, read
    Wire::write_all(&mut w, &[0x46, 0xb9, 0x6a, 0x00, 0x0c, 0x8f, 0xff,
                              0xfd, 0x00, 0x06, 0xa0, 0x81, 0x28, 0x16])
        .expect("write");
    Wire::drain(&mut w).expect("drain");
    w.set_baud(9600).expect("re-set 9600");
    count(&mut w, 4, "phase 2b write+drain+switch @9600 (expect legible)");
    w.set_baud(115200).expect("set 115200");
    count(&mut w, 4, "phase 3 @115200 (mismatch: expect garbage bytes)");
    w.set_baud(9600).expect("set 9600 again");
    count(&mut w, 4, "phase 4 @9600 again (match: expect legible)");
}
