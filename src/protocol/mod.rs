// SPDX-License-Identifier: MIT
// Copyright (c) 2026 CrispStrobe
//
//! Protocol families.
//!
//! `docs/STC-ISP-PROTOCOL.md` §1 is emphatic that STC12 (§10) and STC15 (§11)
//! are *different protocol generations, not dialects*, and that "nothing in
//! §4–§9 may be assumed to carry over" — §10 specifically warns that the
//! checksum width must be re-derived rather than assumed, because an
//! "obvious" port that is wrong there corrupts flash silently.
//!
//! Hence a trait rather than a family enum with shared helpers: adding
//! [`stc89`]'s siblings later means writing a new implementation against new
//! captures, not adding branches to this one. v1 ships exactly one family.

use crate::frame::Frame;
use crate::session::Step;

pub mod stc89;

/// What the caller wants done in this session.
#[derive(Clone, Debug)]
pub enum Job {
    /// Handshake, report, release the chip to its application. No baud
    /// switch: §3.4, an info-only session never switches.
    Identify,
    /// Erase code flash. `blocks` in the family's erase-block unit; `None`
    /// means "the whole code flash", which requires a known chip.
    Erase { blocks: Option<u8> },
    /// Erase exactly as much as the image needs, then program it.
    Flash {
        image: Vec<u8>,
        /// Echo the option byte back after programming, as the captured
        /// sessions do. See [`TargetInfo::option_bytes`] for why this can
        /// only ever be an echo.
        write_options: bool,
    },
}

/// Knobs that are host policy rather than protocol.
#[derive(Clone, Debug)]
pub struct SessionOptions {
    pub handshake_baud: u32,
    pub transfer_baud: u32,
    /// Per-command reply timeout. §3.5 has no real timings
    /// (`[NEEDS-BENCH]` N-1), so the spec's advice is followed literally:
    /// generous, order 1 s.
    pub reply_timeout_ms: u64,
    /// Erase gets its own, much longer, timeout (§3.5).
    pub erase_timeout_ms: u64,
}

impl Default for SessionOptions {
    fn default() -> Self {
        SessionOptions {
            handshake_baud: 2400,
            transfer_baud: 115200,
            reply_timeout_ms: 2000,
            erase_timeout_ms: 20_000,
        }
    }
}

/// Everything the host learned from the unsolicited status packet.
///
/// Note what is *not* here: any interpretation of the option byte's bits, and
/// anything from the opaque region of the status packet. Both are refusals
/// the spec demands, not omissions — see [`TargetInfo::option_bytes`] and
/// §5.2.
#[derive(Clone, Debug)]
pub struct TargetInfo {
    pub family: &'static str,
    /// Chip identity bytes, e.g. `F0 02` (§5.2). Echoed back by the link test
    /// and by the erase reply.
    pub chip_id: Vec<u8>,
    /// Model name, if this chip id is one we have actually captured.
    pub model: Option<&'static str>,
    pub code_flash: Option<usize>,
    pub eeprom_flash: Option<usize>,
    /// e.g. "6.6C" — BCD major.minor plus an ASCII letter (§5.2).
    pub bsl_version: String,
    /// The frequency the **BSL measured during this handshake** (§7.1). It is
    /// a property of the handshake, not of the board: the same crystal
    /// reported 10.973 MHz in three sessions and 11.030 MHz in five. Never
    /// report it as "the crystal frequency" without saying it was measured.
    pub measured_hz: u64,
    /// The raw frequency words, all eight of them, as received.
    pub freq_words: Vec<u16>,
    /// The option byte(s) exactly as read from the wire.
    ///
    /// §5.7: the bit map is `[NEEDS-BENCH]` (item O-1, the highest-value open
    /// item in the spec), and one of this part's seven options — P1.0/P1.1
    /// download protection — can make a board unflashable without a wiring
    /// change. Until the map exists, "`stcbsl` **must refuse to write options
    /// at all** except by echoing back the byte it read". That refusal is
    /// enforced structurally: there is no API anywhere in this crate that
    /// takes an option byte as an argument. The only thing that can be sent
    /// is this field, which only [`ProtocolFamily::parse_status`] can fill.
    pub option_bytes: Vec<u8>,
    /// The status frame's payload verbatim, for `--dump` and for anyone who
    /// wants to re-derive something later. Offsets 28…56 of the frame are
    /// opaque and session-varying: §5.2 says an implementation must not
    /// validate, fingerprint, or identify on them, and this crate does not.
    pub raw_status: Vec<u8>,
}

#[derive(Clone, Debug)]
pub enum ProtocolError {
    NotAStatusFrame { cmd: u8 },
    ShortStatus { got: usize, want: usize },
    /// The eight frequency words disagree with each other — §7.1 lists this
    /// as a sanity check worth implementing ("all eight words should be
    /// equal (they always were)").
    InconsistentFrequency(Vec<u16>),
    /// The derived frequency is outside anything this part could be running
    /// at, so the reload we would compute from it is not trustworthy (§7.1).
    ImplausibleFrequency(u64),
    /// The transfer baud does not yield a usable 8-bit timer reload (§7.2).
    UnreachableBaud { baud: u32, reload: i64 },
    ImageTooLarge { image: usize, capacity: usize },
    EmptyImage,
    /// `Job::Erase { blocks: None }` against a chip whose flash size we have
    /// never captured.
    UnknownChipSize { chip_id: Vec<u8> },
    EraseTooLarge { blocks: usize },
}

impl core::fmt::Display for ProtocolError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        use crate::frame::hex;
        match self {
            ProtocolError::NotAStatusFrame { cmd } => {
                write!(f, "expected the status packet, got command 0x{cmd:02x}")
            }
            ProtocolError::ShortStatus { got, want } => {
                write!(f, "status payload is {got} bytes, expected {want}")
            }
            ProtocolError::InconsistentFrequency(w) => write!(
                f,
                "the eight frequency words disagree ({w:?}) — the handshake measurement is unreliable"
            ),
            ProtocolError::ImplausibleFrequency(hz) => {
                write!(f, "measured frequency {hz} Hz is out of range for this part")
            }
            ProtocolError::UnreachableBaud { baud, reload } => write!(
                f,
                "transfer baud {baud} needs timer reload {reload}, which is not an 8-bit value"
            ),
            ProtocolError::ImageTooLarge { image, capacity } => {
                write!(f, "image is {image} bytes; this part has {capacity} bytes of code flash")
            }
            ProtocolError::EmptyImage => write!(f, "image contains no data"),
            ProtocolError::UnknownChipSize { chip_id } => write!(
                f,
                "chip id {} is not in our (single-capture) table, so the flash size is unknown — \
                 pass an explicit block count",
                hex(chip_id)
            ),
            ProtocolError::EraseTooLarge { blocks } => {
                write!(f, "erase block count {blocks} does not fit in one byte")
            }
        }
    }
}

/// One generation of the STC serial bootloader protocol.
pub trait ProtocolFamily {
    fn name(&self) -> &'static str;

    /// The baud the handshake runs at before any negotiation (§3.2).
    fn default_handshake_baud(&self) -> u32;

    /// The transfer baud used when there is work to do (§3.2).
    fn default_transfer_baud(&self) -> u32;

    /// Does this frame look like the unsolicited status packet that opens
    /// every successful session (§5.2)?
    fn is_status_frame(&self, frame: &Frame) -> bool;

    fn parse_status(
        &self,
        frame: &Frame,
        handshake_baud: u32,
    ) -> Result<TargetInfo, ProtocolError>;

    /// Compute the entire session up front — §8: the host is a pure function
    /// of (image, status packet).
    fn plan(
        &self,
        info: &TargetInfo,
        job: &Job,
        opts: &SessionOptions,
    ) -> Result<Vec<Step>, ProtocolError>;
}
