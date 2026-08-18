// SPDX-License-Identifier: MIT
// Copyright (c) 2026 CrispStrobe
//
//! The **STC89** protocol family — `docs/STC-ISP-PROTOCOL.md` §5–§9.
//!
//! Derived entirely from the nine bench captures in
//! `docs/isp-captures/stc89c52rc/` (149 frames, all satisfying §4's grammar
//! with zero exceptions) as written up in the spec. Every constant below
//! carries the section it comes from, and every one that never varied across
//! the corpus says so, because "never varied in six sessions that all used
//! the same two baud rates" is a weaker claim than "constant".

use crate::frame::Frame;
use crate::protocol::{Job, ProtocolError, ProtocolFamily, SessionOptions, TargetInfo};
use crate::session::{Expect, Phase, Step};

// ---------------------------------------------------------------- commands

/// Unsolicited status/identify packet, MCU→host (§5.2). Also the write-block
/// command, host→MCU (§5.6) — the direction byte disambiguates.
pub const CMD_STATUS: u8 = 0x00;
/// Write one 128-byte block (§5.6).
pub const CMD_WRITE: u8 = 0x00;
/// Baud probe (§5.3 step 1). Echoed.
pub const CMD_BAUD_PROBE: u8 = 0x8F;
/// Baud commit (§5.3 step 2). Echoed. Both ends retune afterwards.
pub const CMD_BAUD_COMMIT: u8 = 0x8E;
/// Link test at the new baud (§5.4). Sent four times.
pub const CMD_LINK_TEST: u8 = 0x80;
/// Erase `NN` blocks of 256 bytes (§5.5).
pub const CMD_ERASE: u8 = 0x84;
/// Write the option byte (§5.7). Echoed. Skippable.
pub const CMD_OPTIONS: u8 = 0x8D;
/// Leave the BSL and run the application (§5.8). **Never answered.**
pub const CMD_RUN: u8 = 0x82;
/// The MCU's generic acknowledgement command byte (§5).
pub const REPLY_ACK: u8 = 0x80;

// --------------------------------------------------------------- constants

/// Status frame payload length: a 60-byte frame minus 8 bytes of framing.
pub const STATUS_PAYLOAD_LEN: usize = 52;
/// Payload offset of the eight frequency words (frame offsets 6…21, §5.2).
const OFF_FREQ: usize = 0;
const FREQ_WORD_COUNT: usize = 8;
/// Payload offset of the BSL version (frame offsets 22…23).
const OFF_VERSION: usize = 16;
/// Payload offset of the option byte (frame offset 24).
const OFF_OPTION: usize = 18;
/// Payload offset of the chip identity (frame offsets 25…26).
const OFF_CHIP_ID: usize = 19;
const CHIP_ID_LEN: usize = 2;

/// The BSL measures the host's bit time across a `0x7F` byte and reports it
/// as `word`; `f_osc = word × baud_handshake × 12 / 7` (§7.1). The 12 is the
/// classic 8051 machine cycle, the 7 the number of bit times measured. The
/// relation is *exact*, and it only closes if the handshake baud is 2400 —
/// which is how §3.2's first row is confirmed.
const FREQ_NUM: u64 = 12;
const FREQ_DEN: u64 = 7;

/// Sanity band for the measured frequency (§7.1: "the derived frequency
/// should be inside the part's rated range before it is used for anything").
/// `[DS89]` §1.9 rates this family to 40 MHz; the lower bound is arbitrary
/// and only catches a garbled measurement.
const FREQ_MIN_HZ: u64 = 1_000_000;
const FREQ_MAX_HZ: u64 = 48_000_000;

/// Bytes 2…4 of the `0x8F`/`0x8E` payload. Constant across all six sessions
/// that switch, but every capture used the same pair of baud rates, so
/// "constant" here is `[NEEDS-BENCH]` N-4.
const BAUD_CONSTS: [u8; 3] = [0x00, 0x06, 0xA0];
/// Trailing byte present only in the probe (§5.3). `[NEEDS-BENCH]` N-4.
const BAUD_PROBE_TAIL: u8 = 0x81;

/// First four bytes of the link-test payload; the chip id follows (§5.4).
/// Never varied. `[NEEDS-BENCH]` N-5.
const LINK_TEST_PREFIX: [u8; 4] = [0x00, 0x00, 0x36, 0x01];
/// §5.4: "An implementation should send all four and require all four acks:
/// they are the only proof that both ends actually landed on the new baud."
const LINK_TEST_REPEATS: usize = 4;
/// The MCU's link-test reply: `0x80`, empty payload.
const LINK_TEST_ACK_PAYLOAD: [u8; 0] = [];

/// Erase payload filler after the block count (§5.5).
const ERASE_FILL: [u8; 6] = [0x33; 6];
/// The erase reply's payload is 7 bytes: `F0 02 C4 2B 01 83 74`, constant in
/// all six sessions. `F0 02` is the chip id; the other five are unexplained
/// (`[NEEDS-BENCH]` N-6), so we assert the length and the chip id, not the
/// tail.
const ERASE_REPLY_LEN: usize = 7;

/// The erase unit (§1's glossary, §5.5). *Not* the write unit.
pub const ERASE_BLOCK: usize = 256;
/// §5.5: both image cases fit `NN = 2 × ceil(size / 512)`, i.e. erase
/// granularity is 512 bytes expressed as an even number of 256-byte blocks,
/// and the whole-chip case fits it too.
///
/// The spec logs this as `[NEEDS-BENCH]` C-1 — at the time it was written,
/// only the hello capture separated this rule from `NN = ceil(size/256)`, by
/// a single sample. **A second discriminating sample has since landed on the
/// bench:** `docs/isp-captures/stc89c52rc/06-write-frame-error.log` flashes a
/// **714-byte** image and reports "Erasing 4 blocks". The 512-byte rule
/// predicts 4; the 256-byte rule predicts 3. So this rule is now confirmed
/// twice over, and C-1 can be closed.
pub const ERASE_GRANULE: usize = 512;
/// The write unit: 128 data bytes per frame, `n = 142`, in every write frame
/// in the corpus (§5.6). Whether the BSL's buffer could take more is
/// `[NEEDS-BENCH]` C-6.
pub const WRITE_BLOCK: usize = 128;

/// Three trailing `0xFF`s after the option byte; never varied (§5.7).
const OPTIONS_TAIL: [u8; 3] = [0xFF; 3];

/// Chip identities we have actually seen on a bench, with what the session
/// reported about them.
///
/// One entry, one capture. §5.2 flags the encoding as `[NEEDS-BENCH]` (C-5):
/// `[DS89]` §1.9's naming rules make "52" mean 8 KB of program space, which
/// is *consistent* with a low byte that indexes the program-space code point,
/// but the datasheet has no device-ID table and a single sample cannot
/// establish a rule. So this is a lookup table of observations, deliberately
/// not an algorithm.
const KNOWN_CHIPS: &[(&[u8], &str, usize, usize)] = &[
    (&[0xF0, 0x02], "STC89C52RC/LE52RC", 8 * 1024, 6 * 1024),
];

/// The STC89 family, v1 of this crate's only implementation.
pub struct Stc89;

impl Stc89 {
    /// `f_osc = word × baud × 12 / 7` (§7.1). Exact integer arithmetic:
    /// 2667 × 2400 × 12 / 7 = 10 972 800 with no remainder.
    pub fn frequency_hz(word: u16, handshake_baud: u32) -> u64 {
        (word as u64 * handshake_baud as u64 * FREQ_NUM) / FREQ_DEN
    }

    /// `reload = round(256 − f_osc / (baud × 32))`, as an 8-bit timer reload
    /// (§7.2).
    ///
    /// Both measured frequencies (10 972 800 and 11 030 400) round to `0xFD`
    /// at 115200, so the captures **confirm the formula is consistent but do
    /// not uniquely determine it**. Spec item B-3 — one capture at a transfer
    /// baud of 19200 — pins it; this formula predicts `0xEE` there.
    pub fn timer_reload(f_osc: u64, transfer_baud: u32) -> Result<u8, ProtocolError> {
        let denom = transfer_baud as u64 * 32;
        if denom == 0 {
            return Err(ProtocolError::UnreachableBaud { baud: transfer_baud, reload: 0 });
        }
        // round(f / denom) in integer arithmetic
        let ticks = (f_osc + denom / 2) / denom;
        let reload = 256i64 - ticks as i64;
        if !(1..=255).contains(&reload) {
            return Err(ProtocolError::UnreachableBaud { baud: transfer_baud, reload });
        }
        Ok(reload as u8)
    }

    /// The 16-bit big-endian two's-complement form the `0x8F`/`0x8E` payloads
    /// carry: an 8-bit reload of `0xFD` appears on the wire as `FF FD`, i.e.
    /// −3 (§7.2).
    pub fn reload_field(reload: u8) -> [u8; 2] {
        let v = (0x1_0000i32 - (256 - reload as i32)) as u16;
        v.to_be_bytes()
    }

    /// §5.5's rule: `NN = 2 × ceil(size / 512)`.
    pub fn erase_blocks_for(image_len: usize) -> usize {
        2 * image_len.div_ceil(ERASE_GRANULE)
    }

    pub fn baud_probe_frame(reload: u8) -> Frame {
        let r = Self::reload_field(reload);
        let mut p = vec![r[0], r[1]];
        p.extend_from_slice(&BAUD_CONSTS);
        p.push(BAUD_PROBE_TAIL);
        Frame::host(CMD_BAUD_PROBE, p)
    }

    pub fn baud_commit_frame(reload: u8) -> Frame {
        let r = Self::reload_field(reload);
        let mut p = vec![r[0], r[1]];
        p.extend_from_slice(&BAUD_CONSTS);
        Frame::host(CMD_BAUD_COMMIT, p)
    }

    pub fn link_test_frame(chip_id: &[u8]) -> Frame {
        let mut p = LINK_TEST_PREFIX.to_vec();
        p.extend_from_slice(chip_id);
        Frame::host(CMD_LINK_TEST, p)
    }

    pub fn erase_frame(blocks: u8) -> Frame {
        let mut p = vec![blocks];
        p.extend_from_slice(&ERASE_FILL);
        Frame::host(CMD_ERASE, p)
    }

    /// One write block. `data` must be exactly [`WRITE_BLOCK`] bytes.
    pub fn write_frame(addr: u32, data: &[u8]) -> Frame {
        debug_assert_eq!(data.len(), WRITE_BLOCK);
        let mut p = Vec::with_capacity(6 + data.len());
        p.extend_from_slice(&addr.to_be_bytes());
        p.extend_from_slice(&(data.len() as u16).to_be_bytes());
        p.extend_from_slice(data);
        Frame::host(CMD_WRITE, p)
    }

    /// The per-block ack byte: the low byte of the sum of the 128 data bytes
    /// (§5.6). An all-`0xFF` block sums to `0x7F80` and always acks `0x80`.
    pub fn block_ack(data: &[u8]) -> u8 {
        data.iter().fold(0u8, |a, b| a.wrapping_add(*b))
    }

    /// The options frame. Takes the [`TargetInfo`] and not a byte, on
    /// purpose — see [`TargetInfo::option_bytes`]. There is no way to ask
    /// this crate to write an option value it did not first read off the
    /// wire.
    pub fn options_frame(info: &TargetInfo) -> Frame {
        let mut p = info.option_bytes.clone();
        p.extend_from_slice(&OPTIONS_TAIL);
        Frame::host(CMD_OPTIONS, p)
    }

    pub fn run_frame() -> Frame {
        Frame::host(CMD_RUN, vec![])
    }
}

impl ProtocolFamily for Stc89 {
    fn name(&self) -> &'static str {
        "stc89"
    }

    fn default_handshake_baud(&self) -> u32 {
        2400
    }

    fn default_transfer_baud(&self) -> u32 {
        115200
    }

    fn is_status_frame(&self, frame: &Frame) -> bool {
        frame.dir == crate::frame::Dir::McuToHost
            && frame.cmd == CMD_STATUS
            && frame.payload.len() >= OFF_CHIP_ID + CHIP_ID_LEN
    }

    fn parse_status(
        &self,
        frame: &Frame,
        handshake_baud: u32,
    ) -> Result<TargetInfo, ProtocolError> {
        if frame.cmd != CMD_STATUS || frame.dir != crate::frame::Dir::McuToHost {
            return Err(ProtocolError::NotAStatusFrame { cmd: frame.cmd });
        }
        let p = &frame.payload;
        if p.len() < STATUS_PAYLOAD_LEN {
            return Err(ProtocolError::ShortStatus {
                got: p.len(),
                want: STATUS_PAYLOAD_LEN,
            });
        }

        let mut words = Vec::with_capacity(FREQ_WORD_COUNT);
        for i in 0..FREQ_WORD_COUNT {
            let o = OFF_FREQ + i * 2;
            words.push(u16::from_be_bytes([p[o], p[o + 1]]));
        }
        if words.iter().any(|w| *w != words[0]) {
            return Err(ProtocolError::InconsistentFrequency(words));
        }
        let hz = Self::frequency_hz(words[0], handshake_baud);
        if !(FREQ_MIN_HZ..=FREQ_MAX_HZ).contains(&hz) {
            return Err(ProtocolError::ImplausibleFrequency(hz));
        }

        // §5.2: 0x66 is BCD "6.6", 0x43 is ASCII 'C' — together "6.6C".
        let vmaj = p[OFF_VERSION] >> 4;
        let vmin = p[OFF_VERSION] & 0x0F;
        let vletter = p[OFF_VERSION + 1] as char;
        let bsl_version = format!("{vmaj}.{vmin}{vletter}");

        let chip_id = p[OFF_CHIP_ID..OFF_CHIP_ID + CHIP_ID_LEN].to_vec();
        let known = KNOWN_CHIPS.iter().find(|(id, ..)| *id == chip_id.as_slice());

        Ok(TargetInfo {
            family: "stc89",
            chip_id,
            model: known.map(|(_, n, ..)| *n),
            code_flash: known.map(|(_, _, c, _)| *c),
            eeprom_flash: known.map(|(.., e)| *e),
            bsl_version,
            measured_hz: hz,
            freq_words: words,
            // The one byte the host is allowed to write back, and only
            // verbatim (§5.7).
            option_bytes: vec![p[OFF_OPTION]],
            raw_status: p.clone(),
        })
    }

    fn plan(
        &self,
        info: &TargetInfo,
        job: &Job,
        opts: &SessionOptions,
    ) -> Result<Vec<Step>, ProtocolError> {
        let mut steps: Vec<Step> = Vec::new();

        // §3.4: an info-only session never switches baud — 01-info-run1 is
        // two frames long, status in and 0x82 out, both at 2400. The switch
        // belongs to the work, not to the session.
        if matches!(job, Job::Identify) {
            steps.push(run_step(opts));
            return Ok(steps);
        }

        // --- §5.3 baud negotiation, from the MEASURED frequency (§7.1 rule 2)
        let reload = Stc89::timer_reload(info.measured_hz, opts.transfer_baud)?;
        // §5.3 baud negotiation — the DEFINITIVE sequence, from a pyserial
        // trace of a real, successful stcgal flash of this chip (2026-08-18;
        // an earlier pty replay misled us because a fake chip answered
        // instantly). BOTH 0x8F and 0x8E are SENT at the handshake baud, then
        // the host drains, switches to the transfer baud, and READS the echo
        // there — the chip retimes on receiving the frame and answers at the
        // new rate after an ~940 ms internal trial. 0x8E additionally drops
        // BACK to the handshake baud to be sent (0x8F's echo left us at the
        // transfer baud). After 0x8E's echo the link stays at the transfer
        // baud through the 0x80 tests and the rest of the session.
        steps.push(Step {
            phase: Phase::BaudProbe,
            label: format!("baud probe -> {} baud", opts.transfer_baud),
            frame: Stc89::baud_probe_frame(reload),
            expect: Expect::Echo,
            timeout_ms: opts.reply_timeout_ms,
            retune_before_send: None, // already at the handshake baud
            retune_after_send: Some(opts.transfer_baud),
            write_addr: None,
        });
        steps.push(Step {
            phase: Phase::BaudCommit,
            label: format!("baud commit -> {} baud", opts.transfer_baud),
            frame: Stc89::baud_commit_frame(reload),
            expect: Expect::Echo,
            timeout_ms: opts.reply_timeout_ms,
            retune_before_send: Some(opts.handshake_baud), // drop back to send
            retune_after_send: Some(opts.transfer_baud),
            write_addr: None,
        });

        // --- §5.4 link test ×4
        for i in 0..LINK_TEST_REPEATS {
            steps.push(Step {
                phase: Phase::LinkTest,
                label: format!("link test {}/{}", i + 1, LINK_TEST_REPEATS),
                frame: Stc89::link_test_frame(&info.chip_id),
                expect: Expect::Ack {
                    cmd: REPLY_ACK,
                    payload: LINK_TEST_ACK_PAYLOAD.to_vec(),
                },
                timeout_ms: opts.reply_timeout_ms,
                retune_before_send: None,
                retune_after_send: None,
                write_addr: None,
            });
        }

        // --- §5.5 erase
        let blocks = match job {
            Job::Identify => unreachable!(),
            Job::Erase { blocks: Some(n) } => *n as usize,
            Job::Erase { blocks: None } => {
                let cap = info
                    .code_flash
                    .ok_or_else(|| ProtocolError::UnknownChipSize {
                        chip_id: info.chip_id.clone(),
                    })?;
                cap / ERASE_BLOCK
            }
            Job::Flash { image, .. } => {
                if image.is_empty() {
                    return Err(ProtocolError::EmptyImage);
                }
                if let Some(cap) = info.code_flash {
                    if image.len() > cap {
                        return Err(ProtocolError::ImageTooLarge {
                            image: image.len(),
                            capacity: cap,
                        });
                    }
                }
                Stc89::erase_blocks_for(image.len())
            }
        };
        if blocks == 0 || blocks > 0xFF {
            return Err(ProtocolError::EraseTooLarge { blocks });
        }
        steps.push(Step {
            phase: Phase::Erase,
            label: format!("erase {} blocks ({} bytes)", blocks, blocks * ERASE_BLOCK),
            frame: Stc89::erase_frame(blocks as u8),
            expect: Expect::AckLen { cmd: REPLY_ACK, payload_len: ERASE_REPLY_LEN },
            timeout_ms: opts.erase_timeout_ms,
            retune_before_send: None,
            retune_after_send: None,
            write_addr: None,
        });

        // --- §5.6 write
        if let Job::Flash { image, write_options } = job {
            // §5.5: "the host's rule is: erase a region, then fill the whole
            // erased region, padding with 0xFF". The block count exactly
            // determines how much is subsequently written: NN × 256 bytes,
            // every time.
            let region = blocks * ERASE_BLOCK;
            let mut padded = image.clone();
            padded.resize(region, 0xFF);
            let total_blocks = region / WRITE_BLOCK;
            for (i, chunk) in padded.chunks(WRITE_BLOCK).enumerate() {
                let addr = (i * WRITE_BLOCK) as u32;
                steps.push(Step {
                    phase: Phase::Write,
                    label: format!(
                        "write block {}/{} at 0x{:04x}",
                        i + 1,
                        total_blocks,
                        addr
                    ),
                    frame: Stc89::write_frame(addr, chunk),
                    expect: Expect::BlockAck { sum: Stc89::block_ack(chunk) },
                    timeout_ms: opts.reply_timeout_ms,
                    retune_before_send: None,
                    retune_after_send: None,
                    write_addr: Some(addr),
                });
            }

            // --- §5.7 options: sent only in the flashing sessions; the
            // erase-only sessions go straight from erase to 0x82.
            if *write_options {
                steps.push(Step {
                    phase: Phase::Options,
                    label: "options (echoing the byte we read)".to_string(),
                    frame: Stc89::options_frame(info),
                    expect: Expect::Echo,
                    timeout_ms: opts.reply_timeout_ms,
                    retune_before_send: None,
                    retune_after_send: None,
                    write_addr: None,
                });
            }
        }

        steps.push(run_step(opts));
        Ok(steps)
    }
}

fn run_step(opts: &SessionOptions) -> Step {
    Step {
        phase: Phase::Run,
        label: "run application".to_string(),
        frame: Stc89::run_frame(),
        // §5.8: the MCU never answers this. Waiting here reports a false
        // failure on a perfectly successful flash.
        expect: Expect::Nothing,
        timeout_ms: opts.reply_timeout_ms,
        retune_before_send: None,
        retune_after_send: None,
        write_addr: None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// §7.1's table, both rows.
    #[test]
    fn frequency_table() {
        assert_eq!(Stc89::frequency_hz(0x0A6B, 2400), 10_972_800);
        assert_eq!(Stc89::frequency_hz(0x0A79, 2400), 11_030_400);
    }

    /// §7.2's table, including the two predictions.
    #[test]
    fn reload_table() {
        assert_eq!(Stc89::timer_reload(10_972_800, 115200).unwrap(), 0xFD);
        assert_eq!(Stc89::timer_reload(11_030_400, 115200).unwrap(), 0xFD);
        // Predictions, unverified on silicon — spec item B-3.
        assert_eq!(Stc89::timer_reload(11_030_400, 19200).unwrap(), 0xEE);
        assert_eq!(Stc89::timer_reload(11_030_400, 9600).unwrap(), 0xDC);
        assert_eq!(Stc89::reload_field(0xFD), [0xFF, 0xFD]);
        assert_eq!(Stc89::reload_field(0xEE), [0xFF, 0xEE]);
    }

    /// §5.5's table, plus the sample that settles `[NEEDS-BENCH]` C-1.
    #[test]
    fn erase_block_counts() {
        assert_eq!(Stc89::erase_blocks_for(299), 2); // 03-flash-blink
        assert_eq!(Stc89::erase_blocks_for(608), 4); // 04-flash-hello
        assert_eq!(Stc89::erase_blocks_for(8192), 32); // whole 8 KB
        // 06-write-frame-error.log: a 714-byte image, "Erasing 4 blocks".
        // The 256-byte rule would have said 3 — so the 512-byte granularity
        // is confirmed by a second, discriminating sample.
        assert_eq!(Stc89::erase_blocks_for(714), 4);
    }

    /// §5.6: an all-0xFF block sums to 0x7F80 and always acks 0x80.
    #[test]
    fn all_ff_block_acks_0x80() {
        assert_eq!(Stc89::block_ack(&[0xFF; WRITE_BLOCK]), 0x80);
    }

    /// §5.3's byte-exact frames, from [02-erase-run1:38-41].
    #[test]
    fn negotiation_frames_match_the_capture() {
        use crate::frame::unhex;
        assert_eq!(
            Stc89::baud_probe_frame(0xFD).encode(),
            unhex("46 b9 6a 00 0c 8f ff fd 00 06 a0 81 28 16").unwrap()
        );
        assert_eq!(
            Stc89::baud_commit_frame(0xFD).encode(),
            unhex("46 b9 6a 00 0b 8e ff fd 00 06 a0 a5 16").unwrap()
        );
        assert_eq!(
            Stc89::link_test_frame(&[0xF0, 0x02]).encode(),
            unhex("46 b9 6a 00 0c 80 00 00 36 01 f0 02 1f 16").unwrap()
        );
        assert_eq!(
            Stc89::erase_frame(0x20).encode(),
            unhex("46 b9 6a 00 0d 84 20 33 33 33 33 33 33 4d 16").unwrap()
        );
        assert_eq!(
            Stc89::run_frame().encode(),
            unhex("46 b9 6a 00 06 82 f2 16").unwrap()
        );
    }
}
