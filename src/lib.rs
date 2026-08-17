// SPDX-License-Identifier: MIT
// Copyright (c) 2026 CrispStrobe
//
//! `stcbsl` — a clean-room implementation of the STC serial bootloader (ISP)
//! protocol.
//!
//! Written from `docs/STC-ISP-PROTOCOL.md` and the byte-exact bench captures
//! in `docs/isp-captures/`, under the contract in
//! `docs/STC-ISP-CLEANROOM.md`. No existing ISP tool's source or
//! documentation was consulted — see the provenance section of this crate's
//! README, which lists both what was used and what was refused.
//!
//! # Layering
//!
//! ```text
//!   frame      frame codec + resynchronising receiver      no I/O
//!   session    the session as data: Vec<Step>, Session     no I/O
//!   protocol   families; stc89 is the only one in v1       no I/O
//!   ihex       Intel HEX reader                            no I/O
//!   ------------------------------------------------------------
//!   driver     walks a Session over a `Wire`               std::io + clock
//!   transport  `Wire` over a real serial port              feature "serial"
//! ```
//!
//! Everything above the line is what the replay tests in `tests/` exercise
//! against the committed captures; it is deliberately dependency-free.
//!
//! # Scope
//!
//! **STC89 only.** The spec's STC12 (§10) and STC15 (§11) chapters are stubs,
//! and §10 is explicit that these are different protocol generations rather
//! than dialects — an "obvious" port that guesses the checksum width would
//! corrupt flash silently. [`protocol::ProtocolFamily`] exists so that the
//! others can be added from their own captures later, not so that this one
//! can be stretched to cover them.

pub mod driver;
pub mod frame;
pub mod ihex;
pub mod protocol;
pub mod session;

#[cfg(feature = "serial")]
pub mod transport;

pub use driver::{Error, Wire};
pub use frame::{Frame, Receiver};
pub use protocol::stc89::Stc89;
pub use protocol::{Job, ProtocolFamily, SessionOptions, TargetInfo};
pub use session::{Action, Session, Step};
