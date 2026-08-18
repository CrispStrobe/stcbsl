// SPDX-License-Identifier: MIT
// Copyright (c) 2026 CrispStrobe
//
//! A [`Wire`] over a real serial port.
//!
//! The only file in the crate that touches hardware, and the only one the
//! replay tests do not exercise.
//!
//! Two things the spec insists on and this module implements:
//!
//! * **Retune in place.** §3.4: the baud switch "must happen on the open
//!   port"; `docs/BENCH-FLASHING.md` records that a close-and-reopen loses
//!   bytes across the gap. `serialport`'s `set_baud_rate` reconfigures the
//!   existing handle.
//! * **Parity is one constant in one place.** §3.2 leaves parity open
//!   (`[NEEDS-BENCH]` B-1): `[NCRMNT]` claims even parity for its generation,
//!   our host-tool logs cannot show it. The spec warns that a wrong choice
//!   "will present as 'the chip never answers' rather than as an error", so
//!   it lives in [`WIRE_PARITY`] with a CLI override next to it.

use std::io::{Read, Write};
use std::time::Duration;

use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};

use crate::driver::Wire;

/// §3.2, `[NEEDS-BENCH]` B-1. 8N1 is this crate's working assumption; run
/// `stcbsl` with `--parity even` if a bench session ever shows otherwise.
pub const WIRE_PARITY: Parity = Parity::None;
pub const WIRE_DATA_BITS: DataBits = DataBits::Eight;
pub const WIRE_STOP_BITS: StopBits = StopBits::One;

/// How long a single read may block. Short, because the driver polls: the
/// real per-command deadline is the step's own timeout.
const READ_POLL: Duration = Duration::from_millis(20);

pub struct SerialWire {
    port: Box<dyn SerialPort>,
    baud: u32,
    /// The raw fd, kept for a real `tcdrain(2)` — the trait object cannot
    /// hand it back. Unix only; drain() falls back to flush elsewhere.
    #[cfg(unix)]
    fd: std::os::unix::io::RawFd,
}

impl SerialWire {
    pub fn open(path: &str, baud: u32, parity: Parity) -> serialport::Result<SerialWire> {
        let builder = serialport::new(path, baud)
            .data_bits(WIRE_DATA_BITS)
            .parity(parity)
            .stop_bits(WIRE_STOP_BITS)
            .flow_control(FlowControl::None)
            .timeout(READ_POLL);
        // Open the platform-native handle so its fd is reachable for
        // tcdrain, then keep it behind the trait object as before.
        #[cfg(unix)]
        let (port, fd): (Box<dyn SerialPort>, std::os::unix::io::RawFd) = {
            use std::os::unix::io::AsRawFd;
            let native = builder.open_native()?;
            let fd = native.as_raw_fd();
            (Box::new(native), fd)
        };
        #[cfg(not(unix))]
        let port = builder.open()?;
        // Deliberately NOT touching DTR/RTS. §3.3: DTR reaches nothing that
        // helps here — stcgal flashed this chip with both lines forced LOW
        // (owner-watched, 2026-08-18), so their state is irrelevant — and any
        // pulse would be a warm boot (§2.2.5), straight to the application.
        let wire = SerialWire {
            port,
            baud,
            #[cfg(unix)]
            fd,
        };
        // The initial rate went in via bare tcsetattr (serialport::new); on
        // macOS that is nondeterministic for USB adapters, so force it.
        #[cfg(target_os = "macos")]
        wire.force_baud_iossiospeed(baud)?;
        Ok(wire)
    }

    /// macOS: make a baud change actually take.
    ///
    /// The macOS CH340 driver **silently ignores** bare-termios (`tcsetattr`)
    /// rate changes depending on call history — proven on the live chip: a
    /// fresh open requesting 115200 stayed at 9600 and read the running
    /// firmware's UART perfectly (stc-e1, 2026-08-18). pyserial's darwin path
    /// always follows `tcsetattr` with `ioctl(IOSSIOSPEED)`, which is why
    /// stcgal is reliable and every bare-termios stcbsl round was physically
    /// stuck at 2400 while the chip echoed at 115200. We do the same ioctl.
    #[cfg(target_os = "macos")]
    fn force_baud_iossiospeed(&self, baud: u32) -> std::io::Result<()> {
        // IOSSIOSPEED = _IOW('T', 2, speed_t); speed_t is a 4-byte int here,
        // so the request encodes size 4 → 0x80045402. Arg: pointer to the
        // speed. Matches pyserial's `array('i', [baud])`.
        const IOSSIOSPEED: libc::c_ulong = 0x8004_5402;
        let speed: libc::c_uint = baud;
        let rc = unsafe { libc::ioctl(self.fd, IOSSIOSPEED, &speed as *const libc::c_uint) };
        if rc != 0 {
            return Err(std::io::Error::last_os_error());
        }
        Ok(())
    }

    pub fn baud(&self) -> u32 {
        self.baud
    }

    /// Enumerate candidate ports, newest-looking first. Convenience for the
    /// CLI's `ports` subcommand; `tools/find-port.sh` does the same job from
    /// the shell.
    pub fn list() -> Vec<String> {
        serialport::available_ports()
            .map(|ps| ps.into_iter().map(|p| p.port_name).collect())
            .unwrap_or_default()
    }
}

impl Wire for SerialWire {
    fn write_all(&mut self, bytes: &[u8]) -> std::io::Result<()> {
        Write::write_all(&mut self.port, bytes)
    }

    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        match Read::read(&mut self.port, buf) {
            Ok(n) => Ok(n),
            // A poll that saw nothing is not an error at this layer.
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => Ok(0),
            Err(e) => Err(e),
        }
    }

    fn set_baud(&mut self, baud: u32) -> std::io::Result<()> {
        // No same-baud early return: on macOS the ioctl must run every time,
        // because whether a bare tcsetattr "takes" depends on the call
        // history (proven: 2400→115200 was ignored, but 2400→9600→115200
        // took). Forcing IOSSIOSPEED on each call is what makes it
        // deterministic.
        self.port
            .set_baud_rate(baud)
            .map_err(|e| std::io::Error::other(e.to_string()))?;
        #[cfg(target_os = "macos")]
        self.force_baud_iossiospeed(baud)?;
        self.baud = baud;
        Ok(())
    }

    fn flush(&mut self) -> std::io::Result<()> {
        Write::flush(&mut self.port)
    }

    fn baud(&self) -> u32 {
        self.baud
    }

    /// A real `tcdrain(2)`: block until the UART has physically shifted out
    /// every queued byte. `flush()` did not guarantee this on CH340/macOS
    /// (silicon, 2026-08-18), which is the whole reason this exists.
    #[cfg(unix)]
    fn drain(&mut self) -> std::io::Result<()> {
        Write::flush(&mut self.port)?;
        // SAFETY: self.fd is the live descriptor of self.port, owned for the
        // lifetime of this SerialWire; tcdrain only reads it.
        let rc = unsafe { libc::tcdrain(self.fd) };
        if rc != 0 {
            return Err(std::io::Error::last_os_error());
        }
        Ok(())
    }
}
