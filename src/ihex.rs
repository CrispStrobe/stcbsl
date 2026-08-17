// SPDX-License-Identifier: MIT
// Copyright (c) 2026 CrispStrobe
//
//! Intel HEX reader.
//!
//! Not part of the STC protocol — it is the format this repo's Makefile
//! emits (`build/stc89c52rc/*/*.hex`), so it is the format `stcbsl flash`
//! takes.
//!
//! One behaviour here is *not* from the format's definition but from the
//! capture, and it matters:
//!
//! **Gaps inside the image are filled with `0x00`; padding past the end of
//! the image is `0xFF`.** `01-blink.hex` has no record covering 0x0003…0x0007
//! and the captured write block carries `00 00 00 00 00` there, while
//! everything past the image's last byte (0x012B onward) is `0xFF`. The two
//! fills are different because they mean different things: a gap is part of
//! the image the linker simply had nothing to say about, and the tail is
//! erased flash being written back as erased (spec §5.5: "fill the whole
//! erased region, padding with 0xFF"). Reproducing both is what makes the
//! replay tests byte-identical to the capture.
//!
//! The tail fill lives in the protocol layer, where the erase region size is
//! known; only the gap fill is this module's business.

use core::fmt;

/// A parsed image: contiguous bytes starting at address 0.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Image {
    pub bytes: Vec<u8>,
    /// Lowest address any record touched — non-zero means the image does not
    /// start at the reset vector, which is almost always a mistake here.
    pub lowest_addr: u32,
    /// Number of addresses below `bytes.len()` that no record covered.
    pub gap_bytes: usize,
}

#[derive(Clone, Debug)]
pub enum HexError {
    Empty,
    NoData,
    BadRecord { line: usize, why: String },
    BadChecksum { line: usize, computed: u8, got: u8 },
    /// §5.6 `[NEEDS-BENCH]` C-4: the write frame's address field is four
    /// bytes big-endian, but only the low 16 bits were ever non-zero in the
    /// corpus and this part has 8 KB. Refuse rather than guess.
    AddressTooHigh { line: usize, addr: u32 },
}

impl fmt::Display for HexError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            HexError::Empty => write!(f, "file is empty"),
            HexError::NoData => write!(f, "file contains no data records"),
            HexError::BadRecord { line, why } => write!(f, "line {line}: {why}"),
            HexError::BadChecksum { line, computed, got } => write!(
                f,
                "line {line}: record checksum is 0x{got:02x}, computed 0x{computed:02x}"
            ),
            HexError::AddressTooHigh { line, addr } => write!(
                f,
                "line {line}: address 0x{addr:08x} is above 64 KB — unsupported, \
                 the protocol's behaviour there is untested (spec item C-4)"
            ),
        }
    }
}

/// Parse Intel HEX text.
pub fn parse(text: &str) -> Result<Image, HexError> {
    if text.trim().is_empty() {
        return Err(HexError::Empty);
    }
    let mut mem: Vec<Option<u8>> = Vec::new();
    let mut base: u32 = 0;
    let mut lowest: u32 = u32::MAX;
    let mut saw_data = false;

    for (idx, raw) in text.lines().enumerate() {
        let line = idx + 1;
        let s = raw.trim();
        if s.is_empty() {
            continue;
        }
        let Some(body) = s.strip_prefix(':') else {
            return Err(HexError::BadRecord { line, why: format!("no ':' start code: {s:?}") });
        };
        if body.len() < 10 || body.len() % 2 != 0 {
            return Err(HexError::BadRecord { line, why: "truncated record".into() });
        }
        let bytes = decode_hex(body).map_err(|why| HexError::BadRecord { line, why })?;
        let count = bytes[0] as usize;
        if bytes.len() != count + 5 {
            return Err(HexError::BadRecord {
                line,
                why: format!("byte count {count} disagrees with record length {}", bytes.len()),
            });
        }
        // Intel HEX checksum: two's complement of the sum of all bytes
        // before it, so the sum of the whole record is 0 mod 256.
        let sum = bytes.iter().fold(0u8, |a, b| a.wrapping_add(*b));
        if sum != 0 {
            let got = *bytes.last().unwrap();
            let computed = got.wrapping_sub(sum);
            return Err(HexError::BadChecksum { line, computed, got });
        }
        let offset = u16::from_be_bytes([bytes[1], bytes[2]]) as u32;
        let rtype = bytes[3];
        let data = &bytes[4..4 + count];

        match rtype {
            0x00 => {
                let addr = base + offset;
                let end = addr as usize + count;
                if end > 0x1_0000 {
                    return Err(HexError::AddressTooHigh { line, addr });
                }
                if mem.len() < end {
                    mem.resize(end, None);
                }
                for (i, b) in data.iter().enumerate() {
                    mem[addr as usize + i] = Some(*b);
                }
                lowest = lowest.min(addr);
                saw_data = true;
            }
            0x01 => break,
            0x02 => {
                // Extended segment address: paragraph number << 4.
                if count != 2 {
                    return Err(HexError::BadRecord { line, why: "bad type-02 length".into() });
                }
                base = (u16::from_be_bytes([data[0], data[1]]) as u32) << 4;
            }
            0x04 => {
                // Extended linear address: upper 16 bits.
                if count != 2 {
                    return Err(HexError::BadRecord { line, why: "bad type-04 length".into() });
                }
                let upper = u16::from_be_bytes([data[0], data[1]]) as u32;
                if upper != 0 {
                    return Err(HexError::AddressTooHigh { line, addr: upper << 16 });
                }
                base = 0;
            }
            // Start-address records carry an entry point for a host debugger;
            // irrelevant to an 8051 that always starts at 0.
            0x03 | 0x05 => {}
            other => {
                return Err(HexError::BadRecord {
                    line,
                    why: format!("unsupported record type 0x{other:02x}"),
                })
            }
        }
    }

    if !saw_data || mem.is_empty() {
        return Err(HexError::NoData);
    }
    let gap_bytes = mem.iter().filter(|b| b.is_none()).count();
    Ok(Image {
        bytes: mem.into_iter().map(|b| b.unwrap_or(0x00)).collect(),
        lowest_addr: lowest,
        gap_bytes,
    })
}

fn decode_hex(s: &str) -> Result<Vec<u8>, String> {
    let b = s.as_bytes();
    let mut out = Vec::with_capacity(b.len() / 2);
    for pair in b.chunks(2) {
        let hi = nibble(pair[0])?;
        let lo = nibble(pair[1])?;
        out.push(hi << 4 | lo);
    }
    Ok(out)
}

fn nibble(c: u8) -> Result<u8, String> {
    match c {
        b'0'..=b'9' => Ok(c - b'0'),
        b'a'..=b'f' => Ok(c - b'a' + 10),
        b'A'..=b'F' => Ok(c - b'A' + 10),
        _ => Err(format!("non-hex character {:?}", c as char)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn simple_image() {
        let img = parse(":03000000010203F7\n:00000001FF\n").unwrap();
        assert_eq!(img.bytes, vec![1, 2, 3]);
        assert_eq!(img.gap_bytes, 0);
    }

    #[test]
    fn gaps_fill_with_zero() {
        let img = parse(":01000000AA55\n:0100050055A5\n:00000001FF\n").unwrap();
        assert_eq!(img.bytes, vec![0xAA, 0, 0, 0, 0, 0x55]);
        assert_eq!(img.gap_bytes, 4);
    }

    #[test]
    fn rejects_a_bad_checksum() {
        assert!(matches!(
            parse(":03000000010203F8\n:00000001FF\n"),
            Err(HexError::BadChecksum { .. })
        ));
    }

    #[test]
    fn rejects_high_addresses() {
        assert!(matches!(
            parse(":02000004FFFFFC\n:00000001FF\n"),
            Err(HexError::AddressTooHigh { .. })
        ));
    }
}
