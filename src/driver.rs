// SPDX-License-Identifier: MIT
// Copyright (c) 2026 CrispStrobe
//
//! The driver: the only place where the protocol meets a clock and a wire.
//!
//! It has no protocol knowledge of its own — it walks the [`Session`]'s
//! actions and hands bytes to a [`Wire`]. Everything it does that looks like
//! a decision is a spec requirement:
//!
//! * it hunts for `46 B9` on every read, in every phase (§5.1.2);
//! * it retunes the baud **in place**, never closing the port (§3.4);
//! * it does not wait for a reply to `0x82` (§5.8);
//! * on any abort after the first write block it reports the flash as
//!   indeterminate (§6).

use std::io;
use std::time::{Duration, Instant};

use crate::frame::{Frame, FrameError, Receiver};
use crate::protocol::{ProtocolError, ProtocolFamily, SessionOptions, TargetInfo};
use crate::session::{Action, Session, SessionError};

/// A byte pipe whose baud rate can be changed under it.
pub trait Wire {
    fn write_all(&mut self, bytes: &[u8]) -> io::Result<()>;
    /// Read whatever is available, blocking for at most the wire's own read
    /// timeout. Returning `Ok(0)` means "nothing yet", not EOF.
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize>;
    /// Change the line rate without closing the port.
    fn set_baud(&mut self, baud: u32) -> io::Result<()>;
    fn flush(&mut self) -> io::Result<()>;
    /// The line rate currently configured — used to compute how long the
    /// frame just written needs on the wire before a retune is safe.
    fn baud(&self) -> u32;
}

/// Milliseconds a frame of `n` bytes needs on the wire at `baud`, at 8N1
/// (10 bit-times per byte), rounded up, plus a fixed margin for the
/// USB-serial adapter's own latency. `set_baud` reconfigures the UART the
/// instant it is called; if the frame has not physically drained, its tail
/// goes out at the new rate and the chip never sees a valid frame — which
/// is exactly why the 0x8F probe got no echo until this wait existed
/// (CH340/macOS, silicon 2026-08-18: `flush()`/tcdrain did not guarantee
/// the wire was empty). Deterministic and portable — no reliance on the OS
/// draining on our behalf.
fn wire_drain_ms(n: usize, baud: u32) -> u64 {
    const MARGIN_MS: u64 = 10;
    if baud == 0 {
        return MARGIN_MS;
    }
    (n as u64 * 10 * 1000).div_ceil(baud as u64) + MARGIN_MS
}

/// Progress reporting, so the CLI's chattiness is not baked into the driver.
pub trait Log {
    fn step(&mut self, _label: &str, _bytes: &[u8]) {}
    fn reply(&mut self, _frame: &Frame) {}
    fn note(&mut self, _msg: &str) {}
    fn discarded(&mut self, _bytes: &[u8]) {}
}

/// A `Log` that says nothing.
pub struct Quiet;
impl Log for Quiet {}

#[derive(Debug)]
pub enum Error {
    Io(io::Error),
    Protocol(ProtocolError),
    Session(SessionError),
    Frame(FrameError),
    /// No reply within the step's timeout.
    Timeout { label: String, ms: u64 },
    /// The BSL never appeared. Per §3.3 the retry unit is a **power cycle**,
    /// not a resend.
    NoBootloader { waited_ms: u64, noise_bytes: usize },
    /// Aborted after at least one write block went out (§6).
    IndeterminateFlash(Box<Error>),
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::Io(e) => write!(f, "I/O: {e}"),
            Error::Protocol(e) => write!(f, "{e}"),
            Error::Session(e) => write!(f, "{e}"),
            Error::Frame(e) => write!(f, "{e}"),
            Error::Timeout { label, ms } => write!(f, "{label}: no reply within {ms} ms"),
            Error::NoBootloader { waited_ms, noise_bytes } => write!(
                f,
                "no bootloader after {waited_ms} ms ({noise_bytes} bytes of non-protocol noise \
                 seen). The BSL only runs after a COLD power-on and only listens for a few \
                 hundred milliseconds: switch the board off, start this command, then switch \
                 it on. A reset button will not do it."
            ),
            Error::IndeterminateFlash(inner) => write!(
                f,
                "{inner}\n  flash is in an INDETERMINATE state — part of the image was \
                 programmed. There is no read-back on this part, so the only safe move is to \
                 power-cycle and flash again."
            ),
        }
    }
}

impl std::error::Error for Error {}
impl From<io::Error> for Error {
    fn from(e: io::Error) -> Self {
        Error::Io(e)
    }
}
impl From<ProtocolError> for Error {
    fn from(e: ProtocolError) -> Self {
        Error::Protocol(e)
    }
}

/// The sync byte. `0b01111111`: LSB-first that is a start bit, seven ones and
/// a zero — one clean isolated edge pair per byte, which is what makes it a
/// bit-time reference (§5.1.1). The value and the cadence come from public
/// sources, not from our captures (a host-tool log cannot show the host's own
/// TX); spec item B-2.
pub const SYNC_BYTE: u8 = 0x7F;

/// Pulse `0x7F` at the handshake baud and wait for the unsolicited status
/// packet, discarding everything that is not part of a frame.
///
/// §3.3: this is a **race, not a handshake** — the BSL listens for "tens of
/// milliseconds to several hundred milliseconds" after a cold power-on and
/// then jumps to the application, so the host must already be transmitting
/// before power is applied.
pub fn handshake<W: Wire, F: ProtocolFamily, L: Log>(
    wire: &mut W,
    family: &F,
    opts: &SessionOptions,
    wait: Duration,
    log: &mut L,
) -> Result<TargetInfo, Error> {
    wire.set_baud(opts.handshake_baud)?;
    let mut rx = Receiver::new();
    let mut buf = [0u8; 256];
    let started = Instant::now();
    let mut noise = 0usize;

    while started.elapsed() < wait {
        wire.write_all(&[SYNC_BYTE])?;
        wire.flush()?;
        let n = wire.read(&mut buf)?;
        if n > 0 {
            rx.feed(&buf[..n]);
        }
        while let Some(result) = rx.next_frame() {
            let d = rx.take_discarded();
            if !d.is_empty() {
                noise += d.len();
                log.discarded(&d);
            }
            match result {
                Ok(frame) => {
                    if family.is_status_frame(&frame) {
                        log.reply(&frame);
                        return Ok(family.parse_status(&frame, opts.handshake_baud)?);
                    }
                    log.note(&format!("ignoring a frame with cmd 0x{:02x} during sync", frame.cmd));
                }
                Err(e) => log.note(&format!("discarding a malformed frame during sync: {e}")),
            }
        }
        let d = rx.take_discarded();
        if !d.is_empty() {
            noise += d.len();
            log.discarded(&d);
        }
    }
    Err(Error::NoBootloader {
        waited_ms: wait.as_millis() as u64,
        noise_bytes: noise,
    })
}

/// Run a planned session to completion.
pub fn run<W: Wire, L: Log>(
    wire: &mut W,
    session: &mut Session,
    log: &mut L,
) -> Result<(), Error> {
    let mut rx = Receiver::new();
    let mut buf = [0u8; 512];
    loop {
        match session.next_action() {
            Action::Finished => return Ok(()),
            Action::AwaitingReply => unreachable!("driver never leaves a reply outstanding"),
            Action::SetBaud(baud) => {
                log.note(&format!("retuning the link to {baud} baud (port stays open)"));
                wire.set_baud(baud).map_err(|e| wrap(session, Error::Io(e)))?;
                // Whatever was in flight across the rate change is garbage by
                // definition; the only thing to resynchronise on is the next
                // 46 B9 (§3.4), which the Receiver already does.
                rx = Receiver::new();
            }
            Action::Send { label, bytes, expect_reply, timeout_ms, retune_after_send, .. } => {
                log.step(&label, &bytes);
                wire.write_all(&bytes).map_err(|e| wrap(session, Error::Io(e)))?;
                wire.flush().map_err(|e| wrap(session, Error::Io(e)))?;
                // The chip switched rate on receiving this frame (the 0x8F
                // probe) and its echo comes back at the new baud — retune
                // between the write and the read, or the reply times out at
                // the old rate (silicon, 2026-08-18). But the frame must
                // FULLY LEAVE THE WIRE first: set_baud reconfigures the UART
                // immediately, and if the frame's tail is still queued it
                // goes out at the new rate and the chip sees garbage (the
                // probe got no echo at ANY rate until this wait existed).
                // flush() alone did not guarantee it on CH340/macOS, so we
                // sleep the frame's computed wire time at the OLD baud.
                if let Some(baud) = retune_after_send {
                    let drain = wire_drain_ms(bytes.len(), wire.baud());
                    log.note(&format!(
                        "draining {} bytes (~{drain} ms) then retuning to {baud} baud before the reply",
                        bytes.len()
                    ));
                    std::thread::sleep(Duration::from_millis(drain));
                    wire.set_baud(baud).map_err(|e| wrap(session, Error::Io(e)))?;
                    rx = Receiver::new();
                }
                if !expect_reply {
                    continue;
                }
                let deadline = Instant::now() + Duration::from_millis(timeout_ms);
                let frame = loop {
                    if let Some(result) = rx.next_frame() {
                        let d = rx.take_discarded();
                        if !d.is_empty() {
                            log.discarded(&d);
                        }
                        match result {
                            Ok(f) => break f,
                            Err(e) => {
                                // §5.1.2: re-hunt rather than abort. One
                                // stray 0x00 ended 03-flash-blink-run2 at 25%
                                // written; a resynchronising receiver would
                                // have read the ack that followed it.
                                log.note(&format!("re-hunting after a bad frame: {e}"));
                                continue;
                            }
                        }
                    }
                    if Instant::now() >= deadline {
                        return Err(wrap(
                            session,
                            Error::Timeout { label: label.clone(), ms: timeout_ms },
                        ));
                    }
                    let n = wire.read(&mut buf).map_err(|e| wrap(session, Error::Io(e)))?;
                    if n > 0 {
                        rx.feed(&buf[..n]);
                    }
                };
                let d = rx.take_discarded();
                if !d.is_empty() {
                    log.discarded(&d);
                }
                log.reply(&frame);
                if let Err(e) = session.on_reply(&frame) {
                    return Err(wrap(session, Error::Session(e)));
                }
            }
        }
    }
}

fn wrap(session: &Session, e: Error) -> Error {
    if session.flash_is_indeterminate() {
        Error::IndeterminateFlash(Box::new(e))
    } else {
        e
    }
}

#[cfg(test)]
mod tests {
    use super::wire_drain_ms;

    // The 0x8F probe is a 14-byte frame. At the 2400 handshake baud it needs
    // ~58 ms on the wire (14 × 10 / 2400 = 58.3 ms) — the exact figure the
    // bench measured — plus the 10 ms adapter margin. Retuning before this
    // elapses truncated the frame and the chip never echoed (silicon fix,
    // 2026-08-18).
    #[test]
    fn probe_frame_wire_time_at_handshake_baud() {
        assert_eq!(wire_drain_ms(14, 2400), 59 + 10); // ceil(58.33) + margin
    }

    // At the transfer baud the same frame is ~1 ms, so the margin dominates —
    // still safe, never harmful.
    #[test]
    fn fast_baud_is_margin_bound() {
        assert_eq!(wire_drain_ms(14, 115200), 2 + 10); // ceil(1.22) + margin
    }

    // A zero baud (never reached in practice) must not divide by zero.
    #[test]
    fn zero_baud_is_margin_only() {
        assert_eq!(wire_drain_ms(14, 0), 10);
    }
}
