// SPDX-License-Identifier: MIT
// Copyright (c) 2026 CrispStrobe
//
//! Frame codec — `docs/STC-ISP-PROTOCOL.md` §4.
//!
//! ```text
//!   +------+------+-----+-----------+-----+---------------+------+------+
//!   | 0x46 | 0xB9 | DIR | LEN (BE16)| CMD |    PAYLOAD    | CKSM | 0x16 |
//!   +------+------+-----+-----------+-----+---------------+------+------+
//!      0      1      2      3   4      5     6 … n-3        n-2    n-1
//! ```
//!
//! Two rules from the spec that this module exists to enforce:
//!
//! * `LEN = n − 2` — it counts every byte after the magic, including itself,
//!   the checksum and the terminator (§4). There is **no escaping**, so a
//!   receiver must use `LEN` to find the frame end and must never scan for
//!   the `0x16` terminator (§4, "Two consequences").
//! * `CKSM = (Σ frame[2 … n−3]) mod 256` — an 8-bit sum from `DIR` up to and
//!   including the last payload byte (§4). Verified 149/149 on the bench
//!   corpus.
//!
//! Nothing here is target-specific; §10 warns that the checksum *width* must
//! be re-derived for STC12/STC15 rather than assumed, so a future family that
//! disagrees gets its own codec rather than a flag on this one.

use core::fmt;

/// Frame preamble. Also STC's `IAP_TRIG` arming pattern — see spec §2.6.
pub const MAGIC: [u8; 2] = [0x46, 0xB9];
/// `DIR` byte in frames the host sends. 149/149 (§4).
pub const DIR_HOST_TO_MCU: u8 = 0x6A;
/// `DIR` byte in frames the MCU sends. 149/149 (§4).
pub const DIR_MCU_TO_HOST: u8 = 0x68;
/// Frame terminator. Redundant with `LEN`; never used for framing (§4).
pub const TERMINATOR: u8 = 0x16;

/// Bytes of overhead around a payload: magic(2) + DIR(1) + LEN(2) + CMD(1)
/// + CKSM(1) + terminator(1).
pub const FRAME_OVERHEAD: usize = 8;

/// Smallest legal `LEN`: an empty-payload frame is 8 bytes long, so `LEN = 6`
/// (`46 B9 6A 00 06 82 F2 16`, the shortest frame in the corpus — §5.8).
pub const MIN_LEN_FIELD: u16 = 6;

/// Largest `LEN` this codec will accept while hunting. The largest observed
/// frame is `n = 142` (a 128-byte write block, §5.6); §4 flags the real
/// maximum as `[NEEDS-BENCH]` (C-6), so the cap is deliberately loose — it is
/// a resynchronisation sanity bound, not a protocol claim.
pub const MAX_LEN_FIELD: u16 = 1024;

/// Direction of a frame, as carried by the `DIR` byte.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Dir {
    HostToMcu,
    McuToHost,
}

impl Dir {
    pub fn byte(self) -> u8 {
        match self {
            Dir::HostToMcu => DIR_HOST_TO_MCU,
            Dir::McuToHost => DIR_MCU_TO_HOST,
        }
    }

    pub fn from_byte(b: u8) -> Option<Dir> {
        match b {
            DIR_HOST_TO_MCU => Some(Dir::HostToMcu),
            DIR_MCU_TO_HOST => Some(Dir::McuToHost),
            _ => None,
        }
    }
}

/// One decoded `46 B9 … 16` message.
///
/// `cmd` is formally the first payload byte (§5); it is split out because
/// every frame has one and every dispatch decision is made on it.
#[derive(Clone, PartialEq, Eq)]
pub struct Frame {
    pub dir: Dir,
    pub cmd: u8,
    pub payload: Vec<u8>,
}

impl Frame {
    /// A frame the host will send.
    pub fn host(cmd: u8, payload: Vec<u8>) -> Frame {
        Frame { dir: Dir::HostToMcu, cmd, payload }
    }

    /// A frame the MCU sent (used by tests and by the replay harness).
    pub fn mcu(cmd: u8, payload: Vec<u8>) -> Frame {
        Frame { dir: Dir::McuToHost, cmd, payload }
    }

    /// Total wire length `n`.
    pub fn wire_len(&self) -> usize {
        self.payload.len() + FRAME_OVERHEAD
    }

    /// The `LEN` field value: `n − 2`.
    pub fn len_field(&self) -> u16 {
        (self.wire_len() - 2) as u16
    }

    /// Serialise to the exact bytes that go on the wire.
    pub fn encode(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(self.wire_len());
        out.extend_from_slice(&MAGIC);
        out.push(self.dir.byte());
        let len = self.len_field();
        out.push((len >> 8) as u8);
        out.push((len & 0xFF) as u8);
        out.push(self.cmd);
        out.extend_from_slice(&self.payload);
        // CKSM covers DIR .. last payload byte, i.e. everything from index 2
        // of what we have written so far.
        out.push(checksum(&out[2..]));
        out.push(TERMINATOR);
        out
    }

    /// Parse one complete frame from exactly its own bytes.
    ///
    /// Callers that read from a wire should use [`Receiver`] instead: it does
    /// the preamble hunting the spec requires (§5.1.2).
    pub fn decode(bytes: &[u8]) -> Result<Frame, FrameError> {
        if bytes.len() < FRAME_OVERHEAD {
            return Err(FrameError::TooShort(bytes.len()));
        }
        if bytes[0] != MAGIC[0] || bytes[1] != MAGIC[1] {
            return Err(FrameError::BadMagic([bytes[0], bytes[1]]));
        }
        let declared = u16::from_be_bytes([bytes[3], bytes[4]]);
        let actual = (bytes.len() - 2) as u16;
        if declared != actual {
            return Err(FrameError::BadLength { declared, actual });
        }
        if *bytes.last().unwrap() != TERMINATOR {
            return Err(FrameError::BadTerminator(*bytes.last().unwrap()));
        }
        let n = bytes.len();
        let got = bytes[n - 2];
        let computed = checksum(&bytes[2..n - 2]);
        if got != computed {
            return Err(FrameError::BadChecksum { computed, got });
        }
        let dir = Dir::from_byte(bytes[2]).ok_or(FrameError::BadDir(bytes[2]))?;
        Ok(Frame { dir, cmd: bytes[5], payload: bytes[6..n - 2].to_vec() })
    }

    /// `true` if this frame is a byte-for-byte echo of `req` with only `DIR`
    /// swapped — the MCU's answer to `0x8F`, `0x8E` and `0x8D` (§5.3, §5.7).
    ///
    /// Spec note worth keeping in mind while reading a log by hand: such a
    /// pair's checksums differ by exactly 2, because only `DIR` changed.
    pub fn is_echo_of(&self, req: &Frame) -> bool {
        self.dir == Dir::McuToHost
            && req.dir == Dir::HostToMcu
            && self.cmd == req.cmd
            && self.payload == req.payload
    }
}

impl fmt::Debug for Frame {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Frame {{ {} cmd=0x{:02x} payload={} }}",
            match self.dir {
                Dir::HostToMcu => "host->mcu",
                Dir::McuToHost => "mcu->host",
            },
            self.cmd,
            hex(&self.payload)
        )
    }
}

/// `CKSM = (Σ body) mod 256`, where `body` is `DIR .. last payload byte`.
pub fn checksum(body: &[u8]) -> u8 {
    body.iter().fold(0u8, |a, b| a.wrapping_add(*b))
}

/// Lower-case, space-separated hex — the same shape the capture logs and the
/// `frames/*.jsonl` fixtures use, so diffs line up by eye.
pub fn hex(bytes: &[u8]) -> String {
    let mut s = String::with_capacity(bytes.len() * 3);
    for (i, b) in bytes.iter().enumerate() {
        if i > 0 {
            s.push(' ');
        }
        s.push_str(&format!("{:02x}", b));
    }
    s
}

/// Parse a `"46 b9 …"` hex string (the fixtures' `bytes_hex` field).
pub fn unhex(s: &str) -> Result<Vec<u8>, String> {
    s.split_whitespace()
        .map(|t| u8::from_str_radix(t, 16).map_err(|_| format!("bad hex byte {t:?}")))
        .collect()
}

#[derive(Clone, PartialEq, Eq, Debug)]
pub enum FrameError {
    TooShort(usize),
    BadMagic([u8; 2]),
    BadLength { declared: u16, actual: u16 },
    BadTerminator(u8),
    BadChecksum { computed: u8, got: u8 },
    BadDir(u8),
    /// `LEN` outside [`MIN_LEN_FIELD`, `MAX_LEN_FIELD`] — treated as a
    /// mis-sync rather than as a frame (see [`Receiver`]).
    ImplausibleLength(u16),
}

impl fmt::Display for FrameError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            FrameError::TooShort(n) => write!(f, "frame too short ({n} bytes)"),
            FrameError::BadMagic(m) => write!(f, "bad magic {:02x} {:02x}", m[0], m[1]),
            FrameError::BadLength { declared, actual } => {
                write!(f, "bad length: declared {declared}, actual {actual}")
            }
            FrameError::BadTerminator(b) => write!(f, "bad terminator 0x{b:02x} (want 0x16)"),
            FrameError::BadChecksum { computed, got } => {
                write!(f, "bad checksum: computed 0x{computed:02x}, got 0x{got:02x}")
            }
            FrameError::BadDir(b) => write!(f, "unknown direction byte 0x{b:02x}"),
            FrameError::ImplausibleLength(l) => write!(f, "implausible LEN {l}"),
        }
    }
}

/// A resynchronising byte-stream decoder.
///
/// Spec §5.1.2 makes this a hard requirement, not a nicety: on every board
/// this lab owns the application prints on the ISP UART, so the host waits
/// for a bootloader while an application's 115200-baud output arrives
/// misframed at 2400. `05-timeout-nocycle` is 152 such bytes in 38 distinct
/// values. None of the corpus's 248 noise bytes happens to be `0x46` or
/// `0xB9` — the spec says explicitly that this is luck and must not be
/// depended on, so a failed length/checksum check re-hunts from the *next*
/// byte rather than giving up or trusting the header.
#[derive(Default)]
pub struct Receiver {
    buf: Vec<u8>,
    discarded: Vec<u8>,
}

impl Receiver {
    pub fn new() -> Receiver {
        Receiver::default()
    }

    pub fn feed(&mut self, bytes: &[u8]) {
        self.buf.extend_from_slice(bytes);
    }

    /// Bytes thrown away since the last [`Receiver::take_discarded`], in
    /// order. The CLI reports the count; the replay tests assert on it.
    pub fn take_discarded(&mut self) -> Vec<u8> {
        core::mem::take(&mut self.discarded)
    }

    pub fn discarded_len(&self) -> usize {
        self.discarded.len()
    }

    /// Bytes buffered but not yet forming a complete frame.
    pub fn pending(&self) -> usize {
        self.buf.len()
    }

    /// Pull the next complete, valid frame, discarding anything that is not
    /// one. Returns `None` when more bytes are needed.
    ///
    /// A structurally-plausible frame that fails validation is reported once
    /// as `Some(Err(..))` and the stream is advanced by a single byte, so the
    /// caller learns about corruption without losing the ability to resync.
    pub fn next_frame(&mut self) -> Option<Result<Frame, FrameError>> {
        loop {
            match self.hunt() {
                Hunt::NeedMore => return None,
                Hunt::Advanced => continue,
                Hunt::Frame(n) => {
                    let bytes: Vec<u8> = self.buf.drain(..n).collect();
                    match Frame::decode(&bytes) {
                        Ok(f) => return Some(Ok(f)),
                        Err(e) => {
                            // Put everything but the leading 0x46 back and
                            // keep hunting from inside the bad run.
                            let mut rest = bytes;
                            self.discarded.push(rest.remove(0));
                            rest.extend_from_slice(&self.buf);
                            self.buf = rest;
                            return Some(Err(e));
                        }
                    }
                }
            }
        }
    }

    fn hunt(&mut self) -> Hunt {
        loop {
            if self.buf.is_empty() {
                return Hunt::NeedMore;
            }
            if self.buf[0] != MAGIC[0] {
                self.discarded.push(self.buf.remove(0));
                continue;
            }
            if self.buf.len() < 2 {
                return Hunt::NeedMore;
            }
            if self.buf[1] != MAGIC[1] {
                // A lone 0x46 in the noise. Drop just it — the next byte may
                // itself be the real 0x46.
                self.discarded.push(self.buf.remove(0));
                continue;
            }
            if self.buf.len() < 5 {
                return Hunt::NeedMore;
            }
            let declared = u16::from_be_bytes([self.buf[3], self.buf[4]]);
            if !(MIN_LEN_FIELD..=MAX_LEN_FIELD).contains(&declared) {
                self.discarded.push(self.buf.remove(0));
                return Hunt::Advanced;
            }
            let n = declared as usize + 2;
            if self.buf.len() < n {
                return Hunt::NeedMore;
            }
            return Hunt::Frame(n);
        }
    }
}

enum Hunt {
    NeedMore,
    Advanced,
    Frame(usize),
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Spec §4's worked example, the shortest frame in the corpus.
    #[test]
    fn worked_example_from_spec() {
        let f = Frame::host(0x82, vec![]);
        assert_eq!(f.len_field(), 6);
        assert_eq!(f.encode(), unhex("46 b9 6a 00 06 82 f2 16").unwrap());
        assert_eq!(checksum(&[0x6A, 0x00, 0x06, 0x82]), 0xF2);
    }

    /// §4: request and echo differ in checksum by exactly 2.
    #[test]
    fn echo_checksum_differs_by_two() {
        for (cmd, payload) in [
            (0x8Fu8, "ff fd 00 06 a0 81"),
            (0x8E, "ff fd 00 06 a0"),
            (0x8D, "fd ff ff ff"),
        ] {
            let req = Frame::host(cmd, unhex(payload).unwrap());
            let rep = Frame::mcu(cmd, unhex(payload).unwrap());
            let (a, b) = (req.encode(), rep.encode());
            let (ca, cb) = (a[a.len() - 2], b[b.len() - 2]);
            assert_eq!(ca.wrapping_sub(cb), 2, "cmd 0x{cmd:02x}");
            assert!(rep.is_echo_of(&req));
        }
    }

    #[test]
    fn roundtrip() {
        let f = Frame::host(0x00, vec![0xAA; 134]);
        assert_eq!(f.wire_len(), 142);
        assert_eq!(Frame::decode(&f.encode()).unwrap(), f);
    }

    #[test]
    fn receiver_discards_noise_and_finds_the_frame() {
        let good = Frame::mcu(0x80, vec![0xEE]).encode();
        let mut rx = Receiver::new();
        rx.feed(&unhex("a0 b3 e1 25 fc 4d 54 46 00 b9").unwrap());
        assert!(rx.next_frame().is_none());
        rx.feed(&good);
        let f = rx.next_frame().unwrap().unwrap();
        assert_eq!(f.cmd, 0x80);
        assert_eq!(rx.take_discarded().len(), 10);
    }

    #[test]
    fn receiver_rehunts_after_a_bad_checksum() {
        let mut bad = Frame::mcu(0x80, vec![0xEE]).encode();
        let n = bad.len();
        bad[n - 2] ^= 0xFF;
        let good = Frame::mcu(0x80, vec![0x3C]).encode();
        let mut rx = Receiver::new();
        rx.feed(&bad);
        rx.feed(&good);
        assert!(matches!(rx.next_frame(), Some(Err(FrameError::BadChecksum { .. }))));
        let f = rx.next_frame().unwrap().unwrap();
        assert_eq!(f.payload, vec![0x3C]);
    }

    /// §4: no escaping. A payload containing the magic must survive, and the
    /// receiver must find the frame end from LEN rather than from 0x16.
    /// (Real silicon confirmation is spec item C-2, `[NEEDS-BENCH]`.)
    #[test]
    fn payload_may_contain_magic_and_terminator() {
        let f = Frame::host(0x00, vec![0x46, 0xB9, 0x16, 0x16, 0x46, 0xB9]);
        let mut rx = Receiver::new();
        rx.feed(&f.encode());
        assert_eq!(rx.next_frame().unwrap().unwrap(), f);
        assert_eq!(rx.take_discarded().len(), 0);
    }
}
