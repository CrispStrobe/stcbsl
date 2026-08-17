<!-- SPDX-License-Identifier: MIT -->
# Replay fixtures

Two of the three fixture sets live outside this directory, because the
originals are the evidence and a copy would be a second source of truth:

- **Frame tables** — `docs/isp-captures/stc89c52rc/frames/*.jsonl`, read
  directly by `tests/replay.rs` via `CARGO_MANIFEST_DIR`. Schema:
  `tools/isp-capture/README.md`. Regenerate with
  `tools/isp-capture/normalize.py`; the `.log` files beside them are the
  primary evidence and win any disagreement.
- **Bench notes** — `docs/isp-captures/stc89c52rc/NOTES.md`.

What *is* copied here, and why:

| file | is | why a copy |
|---|---|---|
| `01-blink.hex.txt` | `build/stc89c52rc/01-blink/01-blink.hex` | the exact image flashed in `03-flash-blink-run1/run2` |
| `04-hello89.hex.txt` | `build/stc89c52rc/04-hello89/04-hello89.hex` | the exact image flashed in `04-flash-hello-run1/run2` |

`build/` is git-ignored and its contents are rebuilt by SDCC, so a test that
read them there would pass or fail depending on whether someone had run
`make` — and on *which* `make`. These copies are frozen: the replay test
asserts that parsing them and planning a session reproduces the captured
write frames byte for byte, which is only a meaningful claim if the bytes
are the ones that were actually on the wire.

The `.txt` suffix is not decoration. This repo's `.gitignore` excludes
`*.hex` everywhere except `examples/`, so a fixture named `.hex` would not be
committed and the tests would fail on a fresh clone.

Identities, matching `NOTES.md`'s "Flashed image identities" section:

```
9984fa68119f32822196639a8b60dfccab5b0e945ee22c862e078252b63a9aeb  01-blink.hex.txt
464ee03699c7d8175fa6d2d37a2517b2971c8db21d1321aec1d152f08fa0ffcf  04-hello89.hex.txt
```

Both are this repo's own MIT-licensed code, which is why their bytes may
appear inside the captured write packets and here.
