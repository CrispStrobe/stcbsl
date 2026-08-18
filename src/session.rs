// SPDX-License-Identifier: MIT
// Copyright (c) 2026 CrispStrobe
//
//! The session state machine, expressed as **data**.
//!
//! `docs/STC-ISP-PROTOCOL.md` §8 establishes the property this design rests
//! on: *every host frame is byte-identical between runs* — the host is a pure
//! function of (image, status packet). So a session is a `Vec<Step>` computed
//! up front from those two inputs, and running it is a loop with no protocol
//! knowledge in it at all.
//!
//! That is also what makes the replay tests possible: the same `Vec<Step>` is
//! either handed to a serial port or compared byte-for-byte against a capture.
//! No I/O appears anywhere in this module.

use crate::frame::{Dir, Frame};

/// Session phase, using the spec's own vocabulary (§5, and the `phase` field
/// of the `frames/*.jsonl` fixtures).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Phase {
    Sync,
    Status,
    BaudProbe,
    BaudCommit,
    LinkTest,
    Erase,
    Write,
    Options,
    Run,
}

impl Phase {
    pub fn as_str(self) -> &'static str {
        match self {
            Phase::Sync => "sync",
            Phase::Status => "status",
            Phase::BaudProbe => "baud_probe",
            Phase::BaudCommit => "baud_commit",
            Phase::LinkTest => "link_test",
            Phase::Erase => "erase",
            Phase::Write => "write",
            Phase::Options => "options",
            Phase::Run => "run",
        }
    }
}

/// What the MCU is expected to answer with.
#[derive(Clone, PartialEq, Eq, Debug)]
pub enum Expect {
    /// No reply, ever. Only `0x82` (§5.8): "a host that waits for an ack here
    /// will report a false failure on a perfectly successful flash".
    Nothing,
    /// The request echoed back with `DIR` swapped — `0x8F`, `0x8E`, `0x8D`
    /// (§5.3, §5.7).
    Echo,
    /// An `0x80` reply whose payload must match exactly. Used where the
    /// corpus shows one constant answer in all six sessions.
    Ack { cmd: u8, payload: Vec<u8> },
    /// An `0x80` reply whose payload length is fixed but whose content is
    /// not asserted.
    AckLen { cmd: u8, payload_len: usize },
    /// The per-block ack of §5.6: one payload byte equal to the low byte of
    /// the sum of the 128 data bytes just sent. Verified 21/21 in the corpus,
    /// and the only per-block integrity signal the protocol offers.
    BlockAck { sum: u8 },
}

/// One request in the session.
#[derive(Clone, Debug)]
pub struct Step {
    pub phase: Phase,
    pub label: String,
    pub frame: Frame,
    pub expect: Expect,
    pub timeout_ms: u64,
    /// Retune the link to this baud once this step's reply has been accepted,
    /// before the next frame goes out. Set on the `0x8E` commit: the proven
    /// stcgal sequence (2026-08-18) reads both the `0x8F` and `0x8E` echoes at
    /// the handshake baud and switches only at the first `0x80` link test —
    /// so the SetBaud lands here, between the commit's echo and that test.
    /// In place, on the open port, resynchronising on the `46 B9` preamble
    /// (§3.4).
    pub retune_to: Option<u32>,
    /// Byte offset this step programs, for error reporting.
    pub write_addr: Option<u32>,
}

/// What the driver should do next.
#[derive(Clone, Debug)]
pub enum Action {
    Send {
        index: usize,
        label: String,
        phase: Phase,
        bytes: Vec<u8>,
        expect_reply: bool,
        timeout_ms: u64,
    },
    /// Change the port's baud rate in place — do **not** close and reopen:
    /// `docs/BENCH-FLASHING.md` records that a close/reopen loses bytes
    /// across the gap (§3.4).
    SetBaud(u32),
    /// A reply to the previous `Send` is outstanding.
    AwaitingReply,
    Finished,
}

#[derive(Clone, Debug)]
pub enum SessionError {
    NotAwaiting,
    WrongCommand { step: String, want: u8, got: u8 },
    WrongDirection { step: String, got: Dir },
    NotAnEcho { step: String, got: Frame },
    WrongPayload { step: String, want: Vec<u8>, got: Vec<u8> },
    WrongPayloadLen { step: String, want: usize, got: usize },
    /// The MCU's own checksum of the block disagrees with ours (§5.6).
    BlockAckMismatch { addr: u32, want: u8, got: u8 },
}

impl core::fmt::Display for SessionError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        use crate::frame::hex;
        match self {
            SessionError::NotAwaiting => write!(f, "a reply arrived when none was expected"),
            SessionError::WrongCommand { step, want, got } => write!(
                f,
                "{step}: expected reply command 0x{want:02x}, got 0x{got:02x}"
            ),
            SessionError::WrongDirection { step, got } => {
                write!(f, "{step}: reply carries direction {got:?}")
            }
            SessionError::NotAnEcho { step, got } => {
                write!(f, "{step}: reply is not an echo of the request ({got:?})")
            }
            SessionError::WrongPayload { step, want, got } => write!(
                f,
                "{step}: expected payload [{}], got [{}]",
                hex(want),
                hex(got)
            ),
            SessionError::WrongPayloadLen { step, want, got } => {
                write!(f, "{step}: expected {want} payload bytes, got {got}")
            }
            SessionError::BlockAckMismatch { addr, want, got } => write!(
                f,
                "block at 0x{addr:04x}: MCU checksummed 0x{got:02x}, we sent 0x{want:02x} \
                 — flash is now in an INDETERMINATE state; power-cycle and reflash"
            ),
        }
    }
}

/// A planned session being executed.
///
/// Contract: call [`Session::next_action`]; if it returns `Send` with
/// `expect_reply`, feed the next valid frame to [`Session::on_reply`] before
/// asking for another action.
pub struct Session {
    steps: Vec<Step>,
    at: usize,
    awaiting: bool,
    retune_pending: Option<u32>,
    writes_started: bool,
    writes_finished: bool,
}

impl Session {
    pub fn new(steps: Vec<Step>) -> Session {
        Session {
            steps,
            at: 0,
            awaiting: false,
            retune_pending: None,
            writes_started: false,
            writes_finished: false,
        }
    }

    pub fn steps(&self) -> &[Step] {
        &self.steps
    }

    pub fn next_action(&mut self) -> Action {
        if self.awaiting {
            return Action::AwaitingReply;
        }
        if let Some(baud) = self.retune_pending.take() {
            return Action::SetBaud(baud);
        }
        if self.at >= self.steps.len() {
            return Action::Finished;
        }
        let step = &self.steps[self.at];
        let expect_reply = step.expect != Expect::Nothing;
        if step.phase == Phase::Write {
            self.writes_started = true;
        }
        let action = Action::Send {
            index: self.at,
            label: step.label.clone(),
            phase: step.phase,
            bytes: step.frame.encode(),
            expect_reply,
            timeout_ms: step.timeout_ms,
        };
        if expect_reply {
            self.awaiting = true;
        } else {
            self.advance();
        }
        action
    }

    fn advance(&mut self) {
        let step = &self.steps[self.at];
        self.retune_pending = step.retune_to;
        if step.phase == Phase::Write {
            let last_write = self
                .steps
                .iter()
                .rposition(|s| s.phase == Phase::Write)
                .unwrap_or(self.at);
            if self.at == last_write {
                self.writes_finished = true;
            }
        }
        self.at += 1;
        self.awaiting = false;
    }

    pub fn on_reply(&mut self, reply: &Frame) -> Result<(), SessionError> {
        if !self.awaiting {
            return Err(SessionError::NotAwaiting);
        }
        let step = self.steps[self.at].clone();
        if reply.dir != Dir::McuToHost {
            return Err(SessionError::WrongDirection { step: step.label, got: reply.dir });
        }
        match &step.expect {
            Expect::Nothing => return Err(SessionError::NotAwaiting),
            Expect::Echo => {
                if !reply.is_echo_of(&step.frame) {
                    return Err(SessionError::NotAnEcho {
                        step: step.label,
                        got: reply.clone(),
                    });
                }
            }
            Expect::Ack { cmd, payload } => {
                if reply.cmd != *cmd {
                    return Err(SessionError::WrongCommand {
                        step: step.label,
                        want: *cmd,
                        got: reply.cmd,
                    });
                }
                if &reply.payload != payload {
                    return Err(SessionError::WrongPayload {
                        step: step.label,
                        want: payload.clone(),
                        got: reply.payload.clone(),
                    });
                }
            }
            Expect::AckLen { cmd, payload_len } => {
                if reply.cmd != *cmd {
                    return Err(SessionError::WrongCommand {
                        step: step.label,
                        want: *cmd,
                        got: reply.cmd,
                    });
                }
                if reply.payload.len() != *payload_len {
                    return Err(SessionError::WrongPayloadLen {
                        step: step.label,
                        want: *payload_len,
                        got: reply.payload.len(),
                    });
                }
            }
            Expect::BlockAck { sum } => {
                if reply.cmd != crate::protocol::stc89::REPLY_ACK {
                    return Err(SessionError::WrongCommand {
                        step: step.label,
                        want: crate::protocol::stc89::REPLY_ACK,
                        got: reply.cmd,
                    });
                }
                if reply.payload.len() != 1 {
                    return Err(SessionError::WrongPayloadLen {
                        step: step.label,
                        want: 1,
                        got: reply.payload.len(),
                    });
                }
                if reply.payload[0] != *sum {
                    return Err(SessionError::BlockAckMismatch {
                        addr: step.write_addr.unwrap_or(0),
                        want: *sum,
                        got: reply.payload[0],
                    });
                }
            }
        }
        self.advance();
        Ok(())
    }

    /// True once at least one write block has gone out and the last one has
    /// not been acknowledged.
    ///
    /// §6: "A half-written flash is not a recoverable state." There is no
    /// read-back (§5.6, `[NEEDS-BENCH]` — assume verification is impossible),
    /// so any abort in this window can only honestly be reported as
    /// indeterminate. `03-flash-blink-run2` is the captured instance: one
    /// stray `0x00` ended the session with 128 of 512 bytes written.
    pub fn flash_is_indeterminate(&self) -> bool {
        self.writes_started && !self.writes_finished
    }

    pub fn is_finished(&self) -> bool {
        self.at >= self.steps.len() && !self.awaiting
    }

    pub fn progress(&self) -> (usize, usize) {
        (self.at, self.steps.len())
    }
}
