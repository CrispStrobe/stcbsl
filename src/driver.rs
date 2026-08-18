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
    /// Block until every byte written has physically left the wire (a real
    /// `tcdrain(2)`, not `flush()`). This is the exact wire-end, the point
    /// stcgal retunes at; the default here is `flush()` for wires that
    /// cannot do better (the tests never call it on real hardware).
    fn drain(&mut self) -> io::Result<()> {
        self.flush()
    }
}

/// How the driver handles each `0x8F`/`0x8E` baud switch: a frame is sent at
/// the old rate, then the wire is drained and the rate changed before the
/// echo is read. Draining is essential (a switch on a half-sent frame
/// corrupts its tail); the settle margin is a tunable pause after the switch,
/// before the read, for the adapter/chip to settle.
#[derive(Clone, Copy, Debug)]
pub struct DrainConfig {
    pub mode: DrainMode,
    /// Milliseconds to wait AFTER the drain and BEFORE the baud switch — a
    /// fixed idle-at-old-baud window (stcgal's `time.sleep(0.1)`). Negative
    /// is clamped to zero.
    pub margin_ms: i64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DrainMode {
    /// `tcdrain(2)` the fd before the switch — block until the frame has
    /// physically left the wire. What stcgal (via pyserial's flush) does.
    TcDrain,
    /// Skip the tcdrain (rely on `flush()` alone). A fallback for adapters
    /// where tcdrain misbehaves; unlikely to be needed.
    ComputeWireTime,
}

impl Default for DrainConfig {
    fn default() -> Self {
        // 100 ms matches stcgal EXACTLY: its handshake does write_packet
        // (which flushes/tcdrains) then `time.sleep(0.1)` then switches baud.
        // We tcdrain, then wait this settle, then switch. Giving the chip a
        // fixed idle-at-old-baud window after the 0x8F/0x8E frame before any
        // line change is the last timing delta we found against the proven
        // stcgal sequence (stcgal is MIT; its source is the reference now).
        DrainConfig { mode: DrainMode::TcDrain, margin_ms: 100 }
    }
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
    // Pulse 0x7F on a fixed ~30 ms cadence (stcgal's timer), and HOLD pulses
    // only while a real frame is mid-flight — a `46 B9` prefix is accumulating
    // and still advancing. That is the exact discriminator we need:
    //   * Pulsing every loop iteration barraged the chip: across the status
    //     packet's ~237 ms the fast partial reads fired 20-40 strays, which a
    //     BSL reads as a fresh sync and drops out of command mode.
    //   * A latch on the first byte was worse: a chip left chattering at the
    //     wrong baud (or any running app) makes us stop pulsing BEFORE the
    //     cold cycle, so the fresh BSL — which auto-bauds off our pulses —
    //     hears silence and never appears (stc-e1, 2026-08-18).
    // Noise almost never contains `46 B9` (the whole corpus's 248 noise bytes
    // have none), so `pending() > 0` after draining means a genuine frame is
    // arriving; hold then. If that partial stalls (a fluke magic in noise
    // that never completes), resume pulsing after 400 ms.
    const PULSE_EVERY: Duration = Duration::from_millis(30);
    const PARTIAL_STALL: Duration = Duration::from_millis(400);
    let mut last_pulse = started - PULSE_EVERY; // force an immediate first pulse
    let mut last_byte = started;
    while started.elapsed() < wait {
        let mid_frame = rx.pending() > 0 && last_byte.elapsed() < PARTIAL_STALL;
        if !mid_frame && last_pulse.elapsed() >= PULSE_EVERY {
            wire.write_all(&[SYNC_BYTE])?;
            wire.flush()?;
            last_pulse = Instant::now();
        }
        let n = wire.read(&mut buf)?;
        if n > 0 {
            last_byte = Instant::now();
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
    drain: DrainConfig,
    log: &mut L,
) -> Result<(), Error> {
    let mut rx = Receiver::new();
    let mut buf = [0u8; 512];
    loop {
        match session.next_action() {
            Action::Finished => return Ok(()),
            Action::AwaitingReply => unreachable!("driver never leaves a reply outstanding"),
            // Every baud change now rides a Send (retune_before_send /
            // retune_after_send), so a standalone SetBaud is never produced.
            Action::SetBaud(baud) => {
                wire.set_baud(baud).map_err(|e| wrap(session, Error::Io(e)))?;
                rx = Receiver::new();
            }
            Action::Send {
                label, bytes, expect_reply, timeout_ms,
                retune_before_send, retune_after_send, ..
            } => {
                // Drop the link to the send baud first (the 0x8E commit is
                // sent at the handshake baud after the 0x8F echo left us at
                // the transfer baud).
                if let Some(b) = retune_before_send {
                    wire.set_baud(b).map_err(|e| wrap(session, Error::Io(e)))?;
                    rx = Receiver::new();
                }
                log.step(&label, &bytes);
                wire.write_all(&bytes).map_err(|e| wrap(session, Error::Io(e)))?;
                wire.flush().map_err(|e| wrap(session, Error::Io(e)))?;
                // The chip retimes on RECEIVING this frame and echoes at the
                // NEW baud (after an ~940 ms internal trial): drain the frame
                // fully off the wire, then switch, then read. Draining before
                // the switch is essential — a set_baud on a half-sent frame
                // sends its tail at the new rate and the chip sees garbage.
                if let Some(b) = retune_after_send {
                    if drain.mode == DrainMode::TcDrain {
                        wire.drain().map_err(|e| wrap(session, Error::Io(e)))?;
                    }
                    let settle = drain.margin_ms.max(0) as u64;
                    if settle > 0 {
                        std::thread::sleep(Duration::from_millis(settle));
                    }
                    wire.set_baud(b).map_err(|e| wrap(session, Error::Io(e)))?;
                    rx = Receiver::new();
                    log.note(&format!(
                        "drained, settled {settle} ms at the old baud, switched to {b}; reading echo there"
                    ));
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

