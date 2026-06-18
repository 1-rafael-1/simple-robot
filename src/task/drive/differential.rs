//! Differential drive speed passthrough.
//!
//! Called by the dispatch for `DriveAction::Differential` commands. Speeds are
//! clamped and passed through unchanged — drift compensation is deferred to the
//! higher-level intents (distance, rotation) that have proper encoder sampling.
//!
//! Unlike other control modules, this is not an intent — commands complete
//! instantly and never block the queue.

/// Passthrough: clamp and return the commanded speeds unchanged.
///
/// Returns (`left`, `right`) clamped to [-100, 100].
pub(super) fn set_speeds(left: i8, right: i8) -> (i8, i8) {
    (left.clamp(-100, 100), right.clamp(-100, 100))
}
