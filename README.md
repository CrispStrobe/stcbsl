<!-- SPDX-License-Identifier: MIT -->
# `stcbsl` — a clean-room STC serial bootloader flasher

A Rust implementation of the **STC serial ISP protocol**: handshake with the
bootloader in an STC MCU over a plain USB-TTL adapter, read what the chip says
about itself, erase its code flash, and program an Intel HEX image into it.

**MIT.** Written from a specification of protocol facts and from
byte-exact captures of bench sessions against silicon.

**Status: silicon-verified at full speed.** Flashes a real STC89C52RC end to
end at 115200 baud — handshake, baud switch, erase, per-block program with
checksum acks, options, release — and the programmed firmware runs.
`cargo test` proves the crate reproduces every byte a real successful
session put on the wire; it does not prove a chip will accept them from *this*
program, because the pulse train that opens the session is the one part of the
protocol our captures could not show (see [Scope](#scope-and-what-is-still-open)).

---

## What it is

```
tools/stcbsl/
  src/frame.rs           frame codec + resynchronising receiver     no I/O
  src/session.rs         the session as data: Vec<Step>, Session    no I/O
  src/protocol/mod.rs    the ProtocolFamily trait, Job, TargetInfo  no I/O
  src/protocol/stc89.rs  the STC89 family — v1's only member        no I/O
  src/ihex.rs            Intel HEX reader                           no I/O
  ------------------------------------------------------------------------
  src/driver.rs          walks a Session over a `Wire`              std + clock
  src/transport.rs       `Wire` over a real serial port             feature "serial"
  src/bin/stcbsl.rs      the CLI
  tests/replay.rs        the capture replays
```

Everything above the line is dependency-free and pure: no sockets, no clock,
no ports. That is what the replay tests drive, and it is why they run headless
in about 20 ms.

The design rests on one measured property of the protocol
(`docs/STC-ISP-PROTOCOL.md` §8): **every host frame is byte-identical between
runs — the host is a pure function of (image, status packet).** So a session is
not a procedure, it is a `Vec<Step>` computed up front from those two inputs.
Running it is a loop with no protocol knowledge in it; replaying it is the same
`Vec<Step>` compared against a capture.

### Layers, and why the split is where it is

- **`frame`** owns the `46 B9 … 16` grammar and the checksum. It also owns the
  *receiver*, which hunts for the preamble on every read and re-hunts after any
  malformed frame. That is a hard requirement, not defensive coding: on every
  board this lab owns, the application prints on the ISP UART, so the host waits
  for a bootloader while an application's 115200-baud output arrives misframed
  at 2400 (spec §5.1.2).
- **`session`** knows what a step is and what a reply must look like. It knows
  nothing about STC.
- **`protocol::stc89`** turns a status packet plus a job into steps. Every
  constant in it cites the spec section it came from, and says whether the
  corpus establishes it or merely never contradicted it.
- **`driver`** is the only place with a clock and a wire.

## Usage

```bash
cd tools/stcbsl
cargo build --release          # -> target/release/stcbsl
cargo test                     # 32 tests, no hardware needed
```

```
stcbsl [OPTIONS] <COMMAND>

  identify                  handshake, report what the chip says, let it run
  erase [--blocks N]        erase code flash (whole chip unless --blocks)
  write <FILE.hex>          erase what the image needs, program it, let it run
  flash <FILE.hex>          alias for `write`
  explain <FILE.hex>        offline: show the block plan for an image; no port
  ports                     list serial ports on this machine

  -p, --port <DEV>          serial device, e.g. /dev/cu.usbserial-1110
      --baud <N>            transfer baud                 [default 115200]
      --handshake-baud <N>  handshake baud                [default 2400]
      --parity <none|even>  wire parity                   [default none]
      --wait <SECONDS>      how long to pulse for the BSL [default 30]
      --skip-options        do not send the option frame at all
  -q, --quiet               only print the outcome
```

The happy path:

```bash
stcbsl ports
stcbsl --port /dev/cu.usbserial-1110 identify
stcbsl --port /dev/cu.usbserial-1110 flash ../../build/stc89c52rc/01-blink/01-blink.hex
```

`explain` needs no chip and no port, and is the cheap way to check an image
before a bench session — it prints exactly which 128-byte blocks will be
written where and what each one should be acknowledged with:

```
$ stcbsl explain build/stc89c52rc/01-blink/01-blink.hex
image: … — 299 bytes
  note: 5 bytes are gaps in the hex, filled with 0x00
erase: 2 blocks x 256 B = 512 B  (rule: NN = 2 x ceil(size/512))
write: 4 blocks x 128 B
  0x0000  ack 0x3c
  0x0080  ack 0xf7
  0x0100  ack 0xf0
  0x0180  ack 0x80   (padding)
```

Those four ack bytes are the ones the chip actually sent in
`03-flash-blink-run1.log`.

### The one thing to know before touching the bench

The bootloader runs **only after a cold power-on** and listens for a few
hundred milliseconds before handing over to the application. So the order is:
board **off**, start `stcbsl`, board **on**. A reset button will not do it, and
neither will DTR: `00-autoreset-attempt.log` is the experiment, and the
datasheet explains why no wiring of DTR could work (a reset-pin reset is a
*warm* boot, which goes straight to the application).

`stcbsl` therefore has **no autoreset mode**. Offering one that silently does
nothing would be worse than not offering one.

**Every retry is a power cycle.** Once the BSL has handed over, it is gone.

### Two refusals built into the API

- **It will never invent an option byte.** The option-byte bit map is unknown
  (spec item O-1), and one of this part's seven options — P1.0/P1.1 download
  protection — can make a board unflashable without a wiring change. So the
  only value that can appear in a `0x8D` frame is the byte read from the status
  packet moments earlier. This is enforced structurally, not by discipline:
  `options_frame` takes the parsed `TargetInfo`, and no function in the crate
  accepts an option byte as an argument.
- **It will never claim to have verified a flash.** There is no read-back
  command anywhere in the corpus and STC protects code from being read out, so
  a `--verify` flag would be a lie. What `stcbsl` can and does check is the
  per-block acknowledgement: the MCU independently checksums each 128-byte
  block and reports the low byte of the sum, and a mismatch aborts. If an abort
  happens after the first block has gone out, the report says *indeterminate,
  power-cycle and reflash*, because that is what it is.

## Testing

```
$ cargo test
   15 unit tests   (frame codec, Intel HEX, the STC89 arithmetic)
   17 replay tests (the nine captured sessions)
```

The replay tests read `docs/isp-captures/stc89c52rc/frames/*.jsonl` — the
normalized frame tables committed beside the raw logs — and treat the status
packet as an **input** and every host frame as an **expected output**, which is
the split spec §8 prescribes. What they establish:

| Test | Claim |
|---|---|
| `every_captured_frame_satisfies_the_grammar` | all 149 frames encode and decode to their captured bytes exactly — checksum, `LEN` and terminator at once |
| `replay_info_sessions` | an info-only session is two frames and no baud switch |
| `replay_erase_sessions` | whole-chip erase, `NN = 0x20`, retune after the `0x8E` echo |
| `replay_flash_sessions` | Intel HEX in, byte-identical session out, for both images and both runs of the larger one |
| `replay_the_aborted_session_…` | the run that died mid-write matches up to the stray byte, and the flash is then correctly reported indeterminate |
| `status_packets_decode` | all 8 status packets: chip id, BSL 6.6C, option byte, and `f_osc = word × baud × 12/7` exact on both measured values |
| `the_two_hello_runs_measured_different_frequencies` | two different measurements, same reload byte — a genuine two-frequency test of the baud path |
| `every_captured_block_ack_is_the_data_sum` | 21/21 write/ack pairs |
| `timeout_capture_is_all_noise_…` | 152 bytes of application output produce no frame and are all discarded |
| `resyncs_on_the_preamble_after_…` | the same noise followed by a real status packet: the frame comes out intact |
| `a_stray_byte_between_frames_…` | the exact byte that killed `03-flash-blink-run2` no longer loses the ack behind it |
| `opaque_region_varies_and_is_ignored` | the status packet's varying region differs between runs and is not used for anything |
| `options_are_only_ever_an_echo_…` | the `0x8D` payload equals the byte that was read |

Everything runs with `cargo test --no-default-features` too, i.e. with the
serial-port dependency compiled out entirely.

## Clean-room provenance

Governed by [`docs/STC-ISP-CLEANROOM.md`](../../docs/STC-ISP-CLEANROOM.md).
This crate is the **implementation role**, which may read the specification and
the captures and nothing else. Byte layouts, handshakes, checksums and timings are facts.

### Inputs actually used

| Input | What it gave |
|---|---|
| [`docs/STC-ISP-PROTOCOL.md`](../../docs/STC-ISP-PROTOCOL.md) | every protocol fact in this crate; each constant cites its section |
| `docs/isp-captures/stc89c52rc/*.log` + `frames/*.jsonl` | the replay fixtures — nine sessions, 149 frames, against this lab's own STC89C52RC |
| `docs/isp-captures/stc89c52rc/NOTES.md` | bench conditions, chip identity, image checksums |
| `tools/isp-capture/README.md` | the JSONL schema the tests parse |
| `build/stc89c52rc/{01-blink,04-hello89}/*.hex` | the two images that were actually flashed; copied into `tests/fixtures/` (see the README there) |
| Rust and `serialport` crate documentation | the language and the one dependency |

### Sources refused

Not consulted, in any form — not source, not documentation, not a search-result
snippet, not a fork or mirror:

- **stcgal** (MIT) — repository, forks, mirrors, readthedocs, PyPI page.
  The bench role ran the stcgal *binary* to produce the capture logs, which
  creates no derivative work; no stcgal artefact was opened by this role at any
  point.
- **stcflash** and any other GPL-licensed ISP tool, in any language.
- **`stc8prog`** — MIT label, unaudited ancestry, explicitly not approved by
  the contract. The `rgm3/ledcube444` lesson applies: an upstream with no
  licence to give cannot be laundered by a later label.
- **`github.com/van9ogh/stc-isp`**, **`codeberg.org/azman/my1stcflash`** —
  unaudited ancestry.
- **The CSDN 《STC单片机的下载协议》 write-up** — declares itself GPL v3 and
  embeds tool source.
- **`../stc-compiler`'s STC12 web flasher, `stc12-session.json`, and
  `BENCH-FLASHING.md`** — our own sibling repo.

Where the spec was silent, the answer was a `[NEEDS-BENCH]` note or a question
back — never a peek. The CLI surface is this crate's own design; it is not a
flag-for-flag copy of anything.

### One thing derived here rather than in the spec

The spec does not say how a host fills a **gap inside** an Intel HEX image, as
opposed to padding past its end. `01-blink.hex` has no record covering
0x0003…0x0007 and the captured write block carries five `0x00` bytes there,
while everything past the image's last byte is `0xFF`. `ihex.rs` reproduces
both, and `hex_images_match_the_captured_write_payloads` pins it. Derived from
our own capture by arithmetic, like everything else here.

## Scope, and what is still open

**STC89 only.** The spec's STC12 (§10) and STC15 (§11) chapters are stubs, and
§10 is explicit that these are *different protocol generations, not dialects* —
the checksum width in particular must be re-derived rather than assumed,
because an "obvious" port that is wrong there corrupts flash silently. The
`ProtocolFamily` trait exists so those families can be added from their own
captures, not so this one can be stretched over them. This matters for this
repo specifically: its original target is an **STC12C5A60S2**, and `stcbsl`
cannot flash one today.

Carried forward from the spec's `[NEEDS-BENCH]` register, in the order they
would bite:

- **B-1 — parity and stop bits.** A host-tool log cannot show wire framing.
  8N1 is the assumption; a wrong choice presents as "the chip never answers"
  rather than as an error, which is why it is a single constant
  (`transport::WIRE_PARITY`) with a `--parity` override beside it.
- **B-2 — the `0x7F` pulse train.** Value, cadence and count come from public
  write-ups, not from our captures, because the capturing tool does not dump
  its own sync bytes. This is the **one part of a session `stcbsl` has no
  fixture for**, and therefore the likeliest thing to be wrong on first
  contact with silicon.
- **B-3 — the reload formula.** `reload = round(256 − f_osc / (baud × 32))`
  fits both measured frequencies, but both round to `0xFD` at 115200, so the
  corpus confirms the formula without pinning it. One capture at 19200 settles
  it; this crate predicts `FF EE` there, and the prediction is a test.
- **C-1 — erase granularity: now settled, and in this rule's favour.**
  `NN = 2 × ceil(size/512)` fits all three block counts the spec had. While
  this crate was being written a fourth landed on the bench:
  `06-write-frame-error.log` flashes a **714-byte** image and reports
  "Erasing 4 blocks", where the competing `NN = ceil(size/256)` rule predicts
  3. That log is also a second independent sighting of the mid-write
  "incorrect frame start" desync that killed `03-flash-blink-run2` — which is
  the failure mode this crate's resynchronising receiver exists to survive.
- **N-1 — real timings.** The captures carry no timestamps, so the timeouts are
  the spec's advice taken literally: generous per command, much longer for
  erase.
- **O-1 — the option-byte bit map.** Unknown, and the reason for the
  echo-only rule above.

Nothing here has run on real silicon. The acceptance test is the same one the
rest of this repo uses: `stcbsl flash` puts `01-blink` onto the bench's
STC89C52RC and the owner watches it blink.
