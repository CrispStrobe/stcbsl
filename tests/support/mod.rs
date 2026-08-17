// SPDX-License-Identifier: MIT
// Copyright (c) 2026 CrispStrobe
//
//! Fixture loading for the replay tests.
//!
//! Reads `docs/isp-captures/*/frames/*.jsonl` — the normalized frame tables
//! described in `tools/isp-capture/README.md`. The schema is flat and
//! string-valued, so this is a 60-line reader rather than a serde dependency:
//! the crate's whole point is that the pure layer builds and tests with no
//! dependencies at all.

#![allow(dead_code)]

use std::path::PathBuf;

pub const CAPTURE_DIR: &str = "docs/isp-captures/stc89c52rc";

/// One record from a `frames/*.jsonl` file.
#[derive(Clone, Debug)]
pub struct Record {
    pub phase: String,
    /// `"host->mcu"` or `"mcu->host"` — the **sender** is named first.
    pub dir: String,
    pub bytes: Vec<u8>,
    pub note: String,
    pub seq: usize,
    pub src: String,
}

impl Record {
    pub fn is_host(&self) -> bool {
        self.dir == "host->mcu"
    }
    pub fn is_mcu(&self) -> bool {
        self.dir == "mcu->host"
    }
    /// A byte run the normalizer could not read as a frame — the application
    /// noise of spec §5.1.2.
    pub fn is_noise(&self) -> bool {
        self.note.contains("not-a-frame")
    }
    pub fn is_frame_ok(&self) -> bool {
        self.note.contains("frame-ok")
    }
}

pub fn repo_root() -> PathBuf {
    // CARGO_MANIFEST_DIR is <repo>/tools/stcbsl
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("..")
}

pub fn fixture_path(name: &str) -> PathBuf {
    repo_root().join(CAPTURE_DIR).join("frames").join(format!("{name}.jsonl"))
}

pub fn load(name: &str) -> Vec<Record> {
    let path = fixture_path(name);
    let text = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("fixture {}: {e}", path.display()));
    text.lines()
        .filter(|l| !l.trim().is_empty())
        .map(|l| parse_record(l, &path.display().to_string()))
        .collect()
}

/// Every capture, in the order the spec lists them.
pub const ALL_CAPTURES: &[&str] = &[
    "00-autoreset-attempt",
    "01-info-run1",
    "01-info-run2",
    "02-erase-run1",
    "02-erase-run2",
    "03-flash-blink-run1",
    "03-flash-blink-run2",
    "04-flash-hello-run1",
    "04-flash-hello-run2",
    "05-timeout-nocycle",
];

pub fn hex_fixture(name: &str) -> String {
    let p = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures")
        .join(name);
    std::fs::read_to_string(&p).unwrap_or_else(|e| panic!("fixture {}: {e}", p.display()))
}

fn parse_record(line: &str, whence: &str) -> Record {
    let bytes_hex = string_field(line, "bytes_hex")
        .unwrap_or_else(|| panic!("{whence}: record without bytes_hex: {line}"));
    Record {
        phase: string_field(line, "phase").unwrap_or_default(),
        dir: string_field(line, "dir").unwrap_or_default(),
        bytes: stcbsl::frame::unhex(&bytes_hex).expect("bad hex in fixture"),
        note: string_field(line, "note").unwrap_or_default(),
        seq: number_field(line, "seq").unwrap_or(0),
        src: string_field(line, "src").unwrap_or_default(),
    }
}

/// Pull `"key": "value"` out of a flat JSON object, honouring the escapes
/// JSON actually uses in these files (the `note` field embeds the capture
/// tool's progress text, complete with `\r` and `█`).
fn string_field(line: &str, key: &str) -> Option<String> {
    let needle = format!("\"{key}\":");
    let start = line.find(&needle)? + needle.len();
    let rest = line[start..].trim_start();
    let mut chars = rest.chars();
    if chars.next()? != '"' {
        return None;
    }
    let mut out = String::new();
    while let Some(c) = chars.next() {
        match c {
            '"' => return Some(out),
            '\\' => match chars.next()? {
                'n' => out.push('\n'),
                'r' => out.push('\r'),
                't' => out.push('\t'),
                'b' => out.push('\u{8}'),
                'f' => out.push('\u{c}'),
                'u' => {
                    let hex: String = (0..4).filter_map(|_| chars.next()).collect();
                    let cp = u32::from_str_radix(&hex, 16).ok()?;
                    out.push(char::from_u32(cp).unwrap_or('\u{fffd}'));
                }
                other => out.push(other),
            },
            other => out.push(other),
        }
    }
    None
}

fn number_field(line: &str, key: &str) -> Option<usize> {
    let needle = format!("\"{key}\":");
    let start = line.find(&needle)? + needle.len();
    let rest = line[start..].trim_start();
    let end = rest.find(|c: char| !c.is_ascii_digit())?;
    rest[..end].parse().ok()
}
