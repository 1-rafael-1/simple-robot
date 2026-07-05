//! Ultrasonic cone correction for HC-SR04 sweep data.
//!
//! The HC-SR04 has a 30° detection cone (15° half-angle). During a sweep,
//! an obstacle appears 30° wider than reality because the cone edges detect
//! it before and after the sensor faces it directly. This module shrinks
//! obstacle runs by 15° on each edge to compensate.
//!
//! # Expected behaviors
//!
//! - **Single uniform-distance run:** Shrinks by 15° on each side.
//!   A run from 40°–100° with consistent distance becomes 55°–85°.
//! - **Point obstacle (raw span < 3°):** Skips discontinuity scanning
//!   and collapses to a 3° minimum width centered on the midpoint.
//!   A single-angle reading at 50° becomes 49°–51°.
//! - **Merged obstacles (>40% distance jump):** When two obstacles are
//!   merged by cone overlap, the >40% relative distance discontinuity
//!   splits them into independent sub-runs, each corrected independently.
//! - **Boundary-left run (starts at 0°):** The left edge is not
//!   corrected; only the right edge shrinks by 15°.
//! - **Boundary-right run (ends at 160°):** The right edge is not
//!   corrected; only the left edge shrinks by 15°.
//! - **Raw span < 30° (but ≥ 3°):** After edge correction the span
//!   would be negative, so the run collapses to the midpoint with a
//!   3° minimum width.

use heapless::Vec;

/// Maximum number of sub-runs extractable from a single obstacle run.
/// Worst-case is one sub-run per 2° (≈81), but 32 is ample in practice
/// and avoids unbounded stack growth.
const MAX_SUB_RUNS: usize = 32;

/// Metadata for a sub-run extracted from a contiguous obstacle sweep.
struct SubRun {
    /// Start angle (inclusive) of the sub-run within the sweep buffer.
    start: u8,
    /// End angle (inclusive) of the sub-run within the sweep buffer.
    end: u8,
    /// Minimum distance (mm) observed across the sub-run's raw readings.
    min_distance: u16,
}

/// Corrects ultrasonic sweep data for the 30° detection cone of the HC-SR04.
///
/// Each obstacle run is shrunk by 15° on each edge. If the run touches
/// the sweep boundary (0° or 160°), that edge is not corrected.
///
/// Distance discontinuities >40% within a run indicate separate obstacles
/// merged by cone overlap; these are split into sub-runs and corrected
/// independently.
///
/// # Arguments
///
/// * `buffer` - The sweep buffer: `[Option<u16>; 161]` where index = angle
///   (0–160°) and `Some(distance_mm)` is a valid reading, `None` is
///   timeout/error.
/// * `run_start` - Start angle (inclusive) of the obstacle run.
/// * `run_end` - End angle (inclusive) of the obstacle run.
pub fn correct_contiguous_run(buffer: &mut [Option<u16>; 161], run_start: u8, run_end: u8) {
    // Defensive: ignore invalid ranges
    if run_start > run_end || run_end > 160 {
        return;
    }

    let run_start_us = usize::from(run_start);
    let run_end_us = usize::from(run_end);

    // Collect sub-runs before clearing the buffer
    let mut sub_runs: Vec<SubRun, MAX_SUB_RUNS> = Vec::new();

    let span = run_end - run_start;
    if span < 2 {
        // Raw span < 3°: treat as a single sub-run, no discontinuity scanning
        let min_dist = min_distance_in_range(buffer, run_start_us, run_end_us);
        let _ = sub_runs.push(SubRun {
            start: run_start,
            end: run_end,
            min_distance: min_dist,
        });
    } else {
        // Scan for >40% distance discontinuities between consecutive angles
        let mut current_start = run_start;
        let mut i = run_start;
        while i < run_end {
            let idx_i = usize::from(i);
            let idx_next = usize::from(i + 1);

            if should_split(buffer, idx_i, idx_next) {
                let min_d = min_distance_in_range(buffer, usize::from(current_start), idx_i);
                let _ = sub_runs.push(SubRun {
                    start: current_start,
                    end: i,
                    min_distance: min_d,
                });
                current_start = i + 1;
            }
            i += 1;
        }
        // Final sub-run from last split point (or original start) to run_end
        let min_d = min_distance_in_range(buffer, usize::from(current_start), run_end_us);
        let _ = sub_runs.push(SubRun {
            start: current_start,
            end: run_end,
            min_distance: min_d,
        });
    }

    // Step 2: Clear the raw obstacle run from the buffer
    buffer[run_start_us..=run_end_us].fill(None);

    // Step 3: Write corrected obstacle points for each sub-run
    for sub in &sub_runs {
        write_corrected_sub_run(buffer, sub.start, sub.end, sub.min_distance);
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Returns `true` if the relative distance jump between two consecutive
/// angles exceeds 40%: `|d2 - d1| / max(d1, d2) > 0.4`.
///
/// If either reading is `None` or zero the pair is not split (zero-distance
/// is treated as a sensor artefact, not a real obstacle edge).
fn should_split(buffer: &[Option<u16>; 161], i: usize, j: usize) -> bool {
    match (buffer[i], buffer[j]) {
        (Some(d1), Some(d2)) if d1 > 0 && d2 > 0 => {
            let diff = d1.abs_diff(d2);
            let max_d = f32::from(d1.max(d2));
            f32::from(diff) / max_d > 0.4
        }
        _ => false,
    }
}

/// Scans the inclusive range `start..=end` and returns the minimum distance
/// found, or `u16::MAX` if no valid reading exists in the range.
fn min_distance_in_range(buffer: &[Option<u16>; 161], start: usize, end: usize) -> u16 {
    buffer[start..=end]
        .iter()
        .filter_map(|opt| *opt)
        .fold(u16::MAX, u16::min)
}

/// Writes the cone-corrected obstacle points for a single sub-run into the
/// sweep buffer.
fn write_corrected_sub_run(buffer: &mut [Option<u16>; 161], sub_start: u8, sub_end: u8, min_distance: u16) {
    // Edge correction: shrink by 15° unless at a sweep boundary
    let corrected_start = if sub_start == 0 {
        0u8
    } else {
        sub_start.saturating_add(15)
    };

    let corrected_end = if sub_end == 160 {
        160u8
    } else {
        sub_end.saturating_sub(15)
    };

    let (final_start, final_end) = if corrected_start > corrected_end {
        // Raw span < 30° (edge correction inverted the range):
        // collapse to midpoint with 3° minimum width
        let midpoint = u16::from(sub_start).midpoint(u16::from(sub_end));
        #[allow(clippy::cast_possible_truncation)]
        let mid = midpoint as u8; // midpoint of two 0..=160 values fits in u8
        (mid.saturating_sub(1), (mid + 1).min(160))
    } else {
        (corrected_start, corrected_end)
    };

    // Fallback: if no distance readings exist (shouldn't happen), use u16::MAX
    let dist = min_distance;

    for angle in final_start..=final_end {
        buffer[usize::from(angle)] = Some(dist);
    }
}
