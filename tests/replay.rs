// SPDX-License-Identifier: MIT
// Copyright (c) 2026 CrispStrobe
//
//! Replay tests: drive the pure protocol layer with the committed bench
//! captures and require byte-identical host output.
//!
//! These rest on the property `docs/STC-ISP-PROTOCOL.md` §8 establishes:
//! **every host frame is byte-identical between runs** — the host is a pure
//! function of (image, status packet). §8 also spells out how a replay test
//! must treat the two sides:
//!
//! > A replay test must therefore treat the status packet as an **input**
//! > (feed it from the fixture) and every host frame as an **expected
//! > output** (compare byte-for-byte).
//!
//! That is exactly what [`replay`] does. No hardware, no I/O, no clock.

mod support;

use stcbsl::frame::{Dir, Frame, Receiver};
use stcbsl::protocol::stc89::Stc89;
use stcbsl::protocol::{Job, ProtocolFamily, SessionOptions, TargetInfo};
use stcbsl::session::{Action, Phase, Session};
use support::{load, Record, ALL_CAPTURES};

// ---------------------------------------------------------------------------
// §4 — the frame grammar, over the whole corpus
// ---------------------------------------------------------------------------

/// §4: "Derived from 149 frames across nine sessions; all 149 satisfy every
/// rule below with zero exceptions."
#[test]
fn every_captured_frame_satisfies_the_grammar() {
    let mut frames = 0usize;
    let mut noise = 0usize;
    for name in ALL_CAPTURES {
        for r in load(name) {
            if r.is_noise() {
                noise += r.bytes.len();
                continue;
            }
            let f = Frame::decode(&r.bytes)
                .unwrap_or_else(|e| panic!("{}:{} — {e}", r.src, r.seq));
            // Re-encoding must reproduce the captured bytes exactly: that is
            // the checksum, the LEN field and the terminator all at once.
            assert_eq!(f.encode(), r.bytes, "{}:{} re-encode", r.src, r.seq);
            match r.dir.as_str() {
                "host->mcu" => assert_eq!(f.dir, Dir::HostToMcu),
                "mcu->host" => assert_eq!(f.dir, Dir::McuToHost),
                other => panic!("unknown direction {other:?}"),
            }
            frames += 1;
        }
    }
    assert_eq!(frames, 149, "the corpus is 149 frames (spec §4)");
    // §5.1.2 counts 248 noise bytes across the whole corpus.
    assert_eq!(noise, 248, "and 248 bytes of non-protocol noise (spec §5.1.2)");
}

/// §4: "none of the 248 noise bytes across the whole corpus is `0x46` or
/// `0xB9`. That is luck, not design, and an implementation must not depend on
/// it." The test records the luck; `noise_before_a_frame_is_discarded` and
/// `receiver_survives_magic_in_the_noise` are what make it not matter.
#[test]
fn the_recorded_noise_happens_to_contain_no_magic() {
    for name in ALL_CAPTURES {
        for r in load(name).iter().filter(|r| r.is_noise()) {
            for b in &r.bytes {
                assert!(*b != 0x46 && *b != 0xB9, "{}:{}", r.src, r.seq);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// §5.2 / §7 — the status packet and the frequency decode
// ---------------------------------------------------------------------------

fn status_frames() -> Vec<(String, Frame)> {
    let mut out = Vec::new();
    for name in ALL_CAPTURES {
        for r in load(name) {
            if r.is_noise() || !r.is_mcu() {
                continue;
            }
            let f = Frame::decode(&r.bytes).unwrap();
            if Stc89.is_status_frame(&f) {
                out.push((format!("{}:{}", r.src, r.seq), f));
            }
        }
    }
    out
}

/// §5.2: "Identical layout in all eight sessions that got that far."
/// §7.1: the two measured frequencies, both exact.
#[test]
fn status_packets_decode() {
    let statuses = status_frames();
    assert_eq!(statuses.len(), 8, "eight sessions reach the status packet");
    for (whence, f) in statuses {
        let info = Stc89.parse_status(&f, 2400).unwrap_or_else(|e| panic!("{whence}: {e}"));
        assert_eq!(info.model, Some("STC89C52RC/LE52RC"), "{whence}");
        assert_eq!(info.chip_id, vec![0xF0, 0x02], "{whence}");
        assert_eq!(info.bsl_version, "6.6C", "{whence}");
        assert_eq!(info.code_flash, Some(8 * 1024), "{whence}");
        assert_eq!(info.eeprom_flash, Some(6 * 1024), "{whence}");
        // §5.2: the option byte is 0xFD in all eight, and equals the byte the
        // host writes back in the 0x8D frame.
        assert_eq!(info.option_bytes, vec![0xFD], "{whence}");
        // §7.1: the same board reported 10.973 MHz in three sessions and
        // 11.030 MHz in five. Both are legitimate.
        assert!(
            info.measured_hz == 10_972_800 || info.measured_hz == 11_030_400,
            "{whence}: measured {} Hz",
            info.measured_hz
        );
        // §7.1's sanity check: all eight words equal.
        assert!(info.freq_words.iter().all(|w| *w == info.freq_words[0]), "{whence}");
        // §7.2: both frequencies round to the same reload byte at 115200,
        // which is why the corpus confirms the formula without pinning it.
        assert_eq!(Stc89::timer_reload(info.measured_hz, 115200).unwrap(), 0xFD, "{whence}");
    }
}

/// §8: "The status packet is the only non-deterministic frame." Two runs of
/// the same session differ in the opaque region — and this crate must not
/// care, because §5.2 forbids validating on offsets 28…56.
#[test]
fn opaque_region_varies_and_is_ignored() {
    let a = load("01-info-run1");
    let b = load("01-info-run2");
    let fa = Frame::decode(&a.iter().find(|r| !r.is_noise()).unwrap().bytes).unwrap();
    let fb = Frame::decode(&b.iter().find(|r| !r.is_noise()).unwrap().bytes).unwrap();
    // Payload offsets 22…50 are frame offsets 28…56.
    let differing = (22..=50).filter(|i| fa.payload[*i] != fb.payload[*i]).count();
    assert!(differing > 0, "the opaque region should differ between runs");
    // Same chip, same everything we actually use.
    let ia = Stc89.parse_status(&fa, 2400).unwrap();
    let ib = Stc89.parse_status(&fb, 2400).unwrap();
    assert_eq!(ia.chip_id, ib.chip_id);
    assert_eq!(ia.option_bytes, ib.option_bytes);
    assert_eq!(ia.bsl_version, ib.bsl_version);
}

// ---------------------------------------------------------------------------
// §5.6 — the per-block ack
// ---------------------------------------------------------------------------

/// §5.6: "Verified on 21/21 write/ack pairs across four sessions."
#[test]
fn every_captured_block_ack_is_the_data_sum() {
    let mut pairs = 0usize;
    let mut all_ff_acks = 0usize;
    for name in ALL_CAPTURES {
        let recs = load(name);
        let mut pending: Option<Vec<u8>> = None;
        for r in recs {
            if r.is_noise() {
                continue;
            }
            let f = Frame::decode(&r.bytes).unwrap();
            if f.dir == Dir::HostToMcu && f.cmd == 0x00 && f.payload.len() == 134 {
                // addr(4) + count(2) + 128 data bytes
                assert_eq!(&f.payload[4..6], &[0x00, 0x80], "count field is always 128");
                pending = Some(f.payload[6..].to_vec());
            } else if f.dir == Dir::McuToHost && f.cmd == 0x80 && f.payload.len() == 1 {
                let data = pending.take().expect("an ack with no write before it");
                assert_eq!(
                    Stc89::block_ack(&data),
                    f.payload[0],
                    "{}:{} block ack",
                    r.src,
                    r.seq
                );
                if data.iter().all(|b| *b == 0xFF) {
                    assert_eq!(f.payload[0], 0x80);
                    all_ff_acks += 1;
                }
                pairs += 1;
            }
        }
    }
    assert_eq!(pairs, 21, "21 write/ack pairs in the corpus (spec §5.6)");
    assert!(all_ff_acks >= 2, "the all-0xFF blocks appear in both images (spec §8)");
}

// ---------------------------------------------------------------------------
// §5.1.2 — noise rejection
// ---------------------------------------------------------------------------

/// The `05-timeout-nocycle` fixture: 30 seconds of an application's
/// 115200-baud output misframed at 2400 while the host waits for a
/// bootloader. 152 single-byte receptions, and not one frame.
#[test]
fn timeout_capture_is_all_noise_and_yields_nothing() {
    let recs = load("05-timeout-nocycle");
    assert_eq!(recs.len(), 152, "§5.1.2: 152 single-byte receptions");
    assert!(recs.iter().all(|r| r.is_noise()));

    // Spec §5.1.2 reads "152 single-byte receptions, 38 distinct values".
    // The 152 is this capture; the 38 is the whole corpus's 248 noise bytes
    // (this file alone has 24 distinct). Both counts are asserted here so the
    // difference is recorded rather than argued about — it changes nothing
    // about the requirement, which is that all of them get discarded.
    let distinct: std::collections::BTreeSet<u8> =
        recs.iter().flat_map(|r| r.bytes.clone()).collect();
    assert_eq!(distinct.len(), 24, "distinct values in this capture");
    let corpus_distinct: std::collections::BTreeSet<u8> = ALL_CAPTURES
        .iter()
        .flat_map(|n| load(n))
        .filter(|r| r.is_noise())
        .flat_map(|r| r.bytes)
        .collect();
    assert_eq!(corpus_distinct.len(), 38, "distinct values across the corpus");

    let mut rx = Receiver::new();
    for r in &recs {
        rx.feed(&r.bytes);
        assert!(rx.next_frame().is_none(), "a frame appeared out of pure noise");
    }
    assert_eq!(rx.take_discarded().len(), 152, "every noise byte must be discarded");
    assert_eq!(rx.pending(), 0);
}

/// The requirement §5.1.2 actually states: the receiver must **resync**.
/// Feed the real noise, then a real status packet, and the frame must come
/// out intact with the noise accounted for.
#[test]
fn resyncs_on_the_preamble_after_the_timeout_capture_noise() {
    let noise: Vec<u8> = load("05-timeout-nocycle")
        .iter()
        .flat_map(|r| r.bytes.clone())
        .collect();
    assert_eq!(noise.len(), 152);
    let status = load("04-flash-hello-run1")
        .into_iter()
        .find(|r| r.is_mcu() && !r.is_noise())
        .unwrap()
        .bytes;

    let mut rx = Receiver::new();
    // Worst case for a byte-at-a-time decoder: noise and frame arrive in one
    // read, split across arbitrary boundaries.
    let mut stream = noise.clone();
    stream.extend_from_slice(&status);
    for chunk in stream.chunks(7) {
        rx.feed(chunk);
    }
    let f = rx.next_frame().expect("frame not found").expect("frame invalid");
    assert!(Stc89.is_status_frame(&f));
    assert_eq!(f.encode(), status);
    assert_eq!(rx.take_discarded(), noise, "the application bytes, all of them");
    assert!(rx.next_frame().is_none());
}

/// §5.1.2's other half: `03-flash-blink-run2` died because one stray `0x00`
/// arrived where a frame header was expected. "A resynchronizing receiver
/// would have discarded that byte and read the ack that followed it."
#[test]
fn a_stray_byte_between_frames_does_not_lose_the_ack() {
    let recs = load("03-flash-blink-run2");
    let stray = recs
        .iter()
        .find(|r| r.is_noise() && r.phase == "write")
        .expect("the mid-programming stray byte");
    assert_eq!(stray.bytes, vec![0x00]);

    // The ack that a resynchronising receiver would have gone on to read —
    // taken from run1, which is byte-identical there (§8).
    let ack = load("03-flash-blink-run1")
        .into_iter()
        .filter(|r| r.is_mcu() && !r.is_noise() && r.phase == "write")
        .nth(1)
        .unwrap()
        .bytes;

    let mut rx = Receiver::new();
    rx.feed(&stray.bytes);
    rx.feed(&ack);
    let f = rx.next_frame().unwrap().unwrap();
    assert_eq!(f.encode(), ack);
    assert_eq!(rx.take_discarded(), vec![0x00]);
}

// ---------------------------------------------------------------------------
// The session replays
// ---------------------------------------------------------------------------

struct Outcome {
    host_frames_matched: usize,
    mcu_replies_accepted: usize,
    retunes: Vec<(usize, u32)>,
    /// Set when the capture stops feeding a reply the plan is waiting for —
    /// i.e. the aborted session.
    stopped_awaiting_reply: bool,
    session: Session,
    info: TargetInfo,
}

/// Feed a capture to a freshly planned session and require that every host
/// frame we would send matches the one that was actually sent, in order.
fn replay(name: &str, job: Job) -> Outcome {
    let recs: Vec<Record> = load(name);
    let opts = SessionOptions::default();

    // The status packet is an INPUT (§8).
    let status_rec = recs
        .iter()
        .find(|r| !r.is_noise() && r.is_mcu())
        .unwrap_or_else(|| panic!("{name}: no MCU frame"));
    let status = Frame::decode(&status_rec.bytes).unwrap();
    assert!(Stc89.is_status_frame(&status), "{name}: first MCU frame is the status packet");
    let info = Stc89.parse_status(&status, opts.handshake_baud).unwrap();

    let steps = Stc89.plan(&info, &job, &opts).unwrap();
    let mut session = Session::new(steps);

    let mut out = Outcome {
        host_frames_matched: 0,
        mcu_replies_accepted: 0,
        retunes: Vec::new(),
        stopped_awaiting_reply: false,
        session: Session::new(vec![]),
        info: info.clone(),
    };

    let mut seen_status = false;
    for r in &recs {
        if r.is_noise() {
            // The driver's Receiver drops these; nothing reaches the session.
            continue;
        }
        let f = Frame::decode(&r.bytes).unwrap();
        if !seen_status {
            assert_eq!(r.seq, status_rec.seq);
            seen_status = true;
            continue;
        }
        if r.is_mcu() {
            session
                .on_reply(&f)
                .unwrap_or_else(|e| panic!("{name} {}:{}: {e}", r.src, r.seq));
            out.mcu_replies_accepted += 1;
            continue;
        }
        // A host frame: whatever we would send next must equal it exactly.
        let bytes = loop {
            match session.next_action() {
                Action::SetBaud(b) => out.retunes.push((out.host_frames_matched, b)),
                Action::Send { bytes, .. } => break Some(bytes),
                Action::AwaitingReply => {
                    // The capture moved on without giving us the reply the
                    // plan is waiting for: the aborted session.
                    break None;
                }
                Action::Finished => panic!("{name}: plan ran out at {}:{}", r.src, r.seq),
            }
        };
        let Some(bytes) = bytes else {
            out.stopped_awaiting_reply = true;
            break;
        };
        assert_eq!(
            stcbsl::frame::hex(&bytes),
            stcbsl::frame::hex(&r.bytes),
            "{name} {}:{} ({} phase)",
            r.src,
            r.seq,
            r.phase
        );
        out.host_frames_matched += 1;
    }
    out.session = session;
    out
}

fn blink_image() -> Vec<u8> {
    stcbsl::ihex::parse(&support::hex_fixture("01-blink.hex.txt")).unwrap().bytes
}

fn hello_image() -> Vec<u8> {
    stcbsl::ihex::parse(&support::hex_fixture("04-hello89.hex.txt")).unwrap().bytes
}

/// §3.4: "An info-only session **never switches**: `01-info-run1.log` is two
/// frames long — status in, `0x82` out — both at 2400."
#[test]
fn replay_info_sessions() {
    for name in ["01-info-run1", "01-info-run2"] {
        let out = replay(name, Job::Identify);
        assert_eq!(out.host_frames_matched, 1, "{name}: only the 0x82 frame");
        assert_eq!(out.mcu_replies_accepted, 0, "{name}: 0x82 is never answered");
        assert!(out.retunes.is_empty(), "{name}: no baud switch without work");
        assert!(out.session.is_finished(), "{name}");
        assert!(!out.session.flash_is_indeterminate());
    }
}

/// §5.5: the whole-chip erase, `NN = 0x20 = 32` blocks = 8192 B.
#[test]
fn replay_erase_sessions() {
    for name in ["02-erase-run1", "02-erase-run2"] {
        let out = replay(name, Job::Erase { blocks: None });
        // 8F, 8E, 4x link test, 84, 82
        assert_eq!(out.host_frames_matched, 8, "{name}");
        assert_eq!(out.mcu_replies_accepted, 7, "{name}: everything but 0x82");
        assert!(out.session.is_finished(), "{name}");
        // §3.4/§5.3: the retune happens after the 0x8E exchange, i.e. once
        // two host frames have gone out.
        assert_eq!(out.retunes, vec![(2, 115200)], "{name}");
        assert!(!out.session.flash_is_indeterminate());
    }
}

/// The full flash sessions, end to end from the Intel HEX file that was
/// actually flashed. This is the strongest claim the crate makes without
/// silicon: parse the image, plan the session from the captured status
/// packet, and every one of the host's frames comes out byte-identical.
#[test]
fn replay_flash_sessions() {
    for (name, image, blocks) in [
        ("03-flash-blink-run1", blink_image(), 4usize),
        ("04-flash-hello-run1", hello_image(), 8),
        ("04-flash-hello-run2", hello_image(), 8),
    ] {
        let out = replay(name, Job::Flash { image, write_options: true });
        // 8F, 8E, 4x link, 84 (=7), N writes, 8D, 82
        assert_eq!(out.host_frames_matched, 9 + blocks, "{name}");
        // everything but 0x82 is answered
        assert_eq!(out.mcu_replies_accepted, 8 + blocks, "{name}");
        assert_eq!(out.retunes, vec![(2, 115200)], "{name}");
        assert!(out.session.is_finished(), "{name}");
        assert!(!out.session.flash_is_indeterminate(), "{name}");
    }
}

/// §8: `04-flash-hello` run1 measured `0A6B` and run2 `0A79` — "a genuine
/// two-frequency test" of the reload path, since both must still produce the
/// same `FF FD` on the wire.
#[test]
fn the_two_hello_runs_measured_different_frequencies() {
    let a = replay("04-flash-hello-run1", Job::Flash { image: hello_image(), write_options: true });
    let b = replay("04-flash-hello-run2", Job::Flash { image: hello_image(), write_options: true });
    assert_ne!(a.info.measured_hz, b.info.measured_hz);
    assert_eq!(a.info.freq_words[0], 0x0A6B);
    assert_eq!(b.info.freq_words[0], 0x0A79);
}

/// §6 and §5.1.2: the aborted run. The host frames match right up to the
/// point where the stray byte hit, and the session is then — correctly —
/// stuck waiting for an ack it will never get, with the flash in an
/// indeterminate state.
#[test]
fn replay_the_aborted_session_and_report_indeterminate_flash() {
    let out = replay("03-flash-blink-run2", Job::Flash { image: blink_image(), write_options: true });
    assert!(out.stopped_awaiting_reply, "the capture stops mid-write");
    // 8F, 8E, 4x link, 84 (=7), then two of the four write blocks.
    assert_eq!(out.host_frames_matched, 9);
    // …of which only the first write was acknowledged: 7 + 1.
    assert_eq!(out.mcu_replies_accepted, 8);
    assert!(
        out.session.flash_is_indeterminate(),
        "§6: a half-written flash is not a recoverable state"
    );
    let (at, total) = out.session.progress();
    assert!(at < total);
}

/// §5.7: options are sent only in the flashing sessions; the erase-only ones
/// go straight from erase to `0x82`. And the byte can only ever be an echo.
#[test]
fn options_are_only_ever_an_echo_of_what_was_read() {
    let status = status_frames().into_iter().next().unwrap().1;
    let info = Stc89.parse_status(&status, 2400).unwrap();
    let opts = SessionOptions::default();

    let flash = Stc89
        .plan(&info, &Job::Flash { image: blink_image(), write_options: true }, &opts)
        .unwrap();
    let opt_step = flash.iter().find(|s| s.phase == Phase::Options).unwrap();
    assert_eq!(opt_step.frame.payload, vec![0xFD, 0xFF, 0xFF, 0xFF]);
    assert_eq!(opt_step.frame.payload[0], info.option_bytes[0]);
    assert_eq!(
        opt_step.frame.encode(),
        stcbsl::frame::unhex("46 b9 6a 00 0a 8d fd ff ff ff fb 16").unwrap()
    );

    let erase = Stc89.plan(&info, &Job::Erase { blocks: None }, &opts).unwrap();
    assert!(erase.iter().all(|s| s.phase != Phase::Options));

    let skipped = Stc89
        .plan(&info, &Job::Flash { image: blink_image(), write_options: false }, &opts)
        .unwrap();
    assert!(skipped.iter().all(|s| s.phase != Phase::Options));
}

/// §5.8: the run frame is never answered, so the plan must not wait for one.
#[test]
fn the_run_frame_expects_no_reply() {
    let status = status_frames().into_iter().next().unwrap().1;
    let info = Stc89.parse_status(&status, 2400).unwrap();
    let steps = Stc89.plan(&info, &Job::Identify, &SessionOptions::default()).unwrap();
    assert_eq!(steps.len(), 1);
    assert_eq!(steps[0].phase, Phase::Run);
    assert_eq!(steps[0].expect, stcbsl::session::Expect::Nothing);

    let mut s = Session::new(steps);
    assert!(matches!(s.next_action(), Action::Send { expect_reply: false, .. }));
    assert!(matches!(s.next_action(), Action::Finished));
}

/// The image the Intel HEX reader produces must be the image that was
/// programmed — including the zero-filled gap at 0x0003…0x0007 that the
/// linker left in `01-blink.hex` (see `ihex.rs`).
#[test]
fn hex_images_match_the_captured_write_payloads() {
    for (name, image) in [("03-flash-blink-run1", blink_image()), ("04-flash-hello-run1", hello_image())] {
        let mut programmed = Vec::new();
        for r in load(name) {
            if r.is_noise() || !r.is_host() {
                continue;
            }
            let f = Frame::decode(&r.bytes).unwrap();
            if f.cmd == 0x00 && f.payload.len() == 134 {
                let addr = u32::from_be_bytes(f.payload[0..4].try_into().unwrap());
                assert_eq!(addr as usize, programmed.len(), "{name}: blocks are contiguous");
                programmed.extend_from_slice(&f.payload[6..]);
            }
        }
        assert_eq!(&programmed[..image.len()], &image[..], "{name}");
        assert!(
            programmed[image.len()..].iter().all(|b| *b == 0xFF),
            "{name}: the tail is 0xFF padding (spec §5.5)"
        );
    }
    // And the gap is real, not an artefact of the reader.
    let img = stcbsl::ihex::parse(&support::hex_fixture("01-blink.hex.txt")).unwrap();
    assert_eq!(img.gap_bytes, 5);
    assert_eq!(&img.bytes[3..8], &[0, 0, 0, 0, 0]);
    assert_eq!(img.bytes.len(), 299);
    assert_eq!(hello_image().len(), 608);
}

/// `00-autoreset-attempt` is evidence of absence: DTR was toggled and not one
/// byte ever arrived (§3.3). The fixture is empty, and it must stay that way,
/// because it is the reason this crate has no autoreset mode.
#[test]
fn the_autoreset_capture_contains_nothing() {
    assert!(load("00-autoreset-attempt").is_empty());
}
