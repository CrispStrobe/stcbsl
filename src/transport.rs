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
}

impl SerialWire {
    pub fn open(path: &str, baud: u32, parity: Parity) -> serialport::Result<SerialWire> {
        let port = serialport::new(path, baud)
            .data_bits(WIRE_DATA_BITS)
            .parity(parity)
            .stop_bits(WIRE_STOP_BITS)
            .flow_control(FlowControl::None)
            .timeout(READ_POLL)
            .open()?;
        // Deliberately NOT touching DTR/RTS. §3.3: on this board DTR reaches
        // nothing useful (`00-autoreset-attempt.log`), and if it reached RST
        // that would be a *warm* boot, which `[DS89]` §2.2.5 sends straight to
        // the application. There is no wiring of DTR that produces a cold
        // boot on its own, so an autoreset mode here would be a lie.
        Ok(SerialWire { port, baud })
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
        if baud == self.baud {
            return Ok(());
        }
        self.port
            .set_baud_rate(baud)
            .map_err(|e| std::io::Error::other(e.to_string()))?;
        self.baud = baud;
        Ok(())
    }

    fn flush(&mut self) -> std::io::Result<()> {
        Write::flush(&mut self.port)
    }

    fn baud(&self) -> u32 {
        self.baud
    }
}
