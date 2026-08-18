// SPDX-License-Identifier: MIT
// rxprobe — bench instrument: does RX survive a mid-session set_baud on
// this adapter? Open at a wrong rate, count bytes, switch to the rate a
// chattering firmware transmits at (04-hello89: 9600, one line/second),
// count again. If the second count is zero while the firmware
// demonstrably prints, serialport's set_baud kills receive on this
// CH340/macOS — the last candidate for the stcbsl 115200 silence,
// reproduced with no bootloader and no power cycles.
use std::io::Read;
use std::time::{Duration, Instant};

fn count(port: &mut dyn serialport::SerialPort, secs: u64, label: &str) {
    let mut n = 0usize;
    let mut buf = [0u8; 256];
    let end = Instant::now() + Duration::from_secs(secs);
    while Instant::now() < end {
        match port.read(&mut buf) {
            Ok(k) => n += k,
            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(e) => {
                println!("{label}: read error {e}");
                return;
            }
        }
    }
    println!("{label}: {n} bytes in {secs}s");
}

fn main() {
    let dev = std::env::args().nth(1).expect("usage: rxprobe <device>");
    let mut port = serialport::new(&dev, 2400)
        .timeout(Duration::from_millis(100))
        .open()
        .expect("open");
    println!("open at 2400");
    count(&mut *port, 3, "phase 1 (2400, expect garbage/none)");
    port.set_baud_rate(9600).expect("set_baud");
    println!("set_baud 9600 done");
    count(&mut *port, 6, "phase 2 (9600, expect ~26 B/s if RX alive)");
    // control: a fresh open directly at 9600 must always hear it
    drop(port);
    let mut port2 = serialport::new(&dev, 9600)
        .timeout(Duration::from_millis(100))
        .open()
        .expect("reopen");
    count(&mut *port2, 6, "phase 3 (fresh open 9600, control)");
}
