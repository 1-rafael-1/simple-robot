//! Gap analysis for ultrasonic sweep buffer.
//!
//! Pure, stateless algorithm that analyzes a 161-entry ultrasonic sweep buffer
//! (indexed by servo angle 0–160°) and returns the best gap to drive through.

use core::f32::consts::PI;

use crate::task::{drive::types::RotationDirection, sensors::ultrasonic::ULTRASONIC_MAX_DISTANCE_CM};

// ── Constants ───────────────────────────────────────────────────────────────

/// Maximum sensing distance in mm, derived from the ultrasonic module.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
const MAX_SENSING_DISTANCE_MM: u16 = (ULTRASONIC_MAX_DISTANCE_CM * 10.0) as u16;
/// Minimum width for a valid gap in mm (30 cm).
const MIN_GAP_WIDTH_MM: u16 = 300;
/// If no constriction exceeds this, the path is blocked (10 cm).
const NO_PATH_THRESHOLD_MM: u16 = 100;
/// Safety margin subtracted from constriction depth (cm).
const SAFETY_MARGIN_CM: f32 = 10.0;
/// Center (straight ahead) servo angle in degrees.
const CENTER_ANGLE_DEG: f32 = 80.0;
/// Maximum heading offset from center (±90°).
const MAX_HEADING_OFFSET_DEG: f32 = 90.0;

// ── Types ───────────────────────────────────────────────────────────────────

/// Result of gap analysis: either a chosen gap or blocked.
#[derive(Debug, Clone, Copy)]
pub struct GapDecision {
    /// Servo midpoint angle of the chosen gap (degrees, 0–160).
    pub servo_midpoint_deg: f32,
    /// Rotation needed from center (80°) to face the gap midpoint.
    pub rotation_degrees: f32,
    /// Direction to rotate.
    pub rotation_direction: RotationDirection,
    /// Distance to drive through this gap (cm), capped at remaining target.
    pub drive_distance_cm: f32,
}

/// Internal representation of a contiguous clear arc.
#[derive(Debug, Clone, Copy)]
struct ClearArc {
    /// First angle in the arc (inclusive, 0–160).
    start_angle: u8,
    /// Last angle in the arc (inclusive, 0–160).
    end_angle: u8,
    /// Minimum distance reading within the arc (mm).
    constriction_depth_mm: u16,
    /// Distance of the obstacle that starts the arc (left flank), or `u16::MAX`
    /// if the arc starts at angle 0 (no left flank obstacle).
    left_flank_distance_mm: u16,
    /// Distance of the obstacle that ends the arc (right flank), or `u16::MAX`
    /// if the arc ends at angle 160 (no right flank obstacle).
    right_flank_distance_mm: u16,
}

impl ClearArc {
    /// Midpoint angle of the arc in degrees.
    fn midpoint_deg(self) -> f32 {
        f32::midpoint(f32::from(self.start_angle), f32::from(self.end_angle))
    }

    /// Angular span of the arc in degrees.
    fn span_deg(self) -> f32 {
        f32::from(self.end_angle) - f32::from(self.start_angle)
    }

    /// Whether the arc touches the boundary (angle 0 or 160).
    const fn is_boundary(self) -> bool {
        self.start_angle == 0 || self.end_angle == 160
    }

    /// Estimated lateral width in mm at the constriction point.
    ///
    /// The constriction depth is the distance to the **nearest flanking obstacle**
    /// (left or right), not the minimum reading inside the arc (which is
    /// `u16::MAX` for timeout-only arcs).  This correctly captures the geometry:
    /// the gap narrows as the robot approaches the nearer of the two obstacles
    /// that define the gap.
    fn lateral_width_mm(self) -> f32 {
        let depth = f32::from(self.effective_constriction_mm());
        let half_span_rad = (self.span_deg() * PI) / 360.0;
        2.0 * depth * libm::sinf(half_span_rad)
    }

    /// Effective constriction depth: the minimum distance among the flanking
    /// obstacles, capped at the maximum sensing distance.
    ///
    /// If both flanks are `u16::MAX` (boundary arc), falls back to the interior
    /// minimum (also capped).  The cap ensures the robot never plans a leg
    /// longer than what the ultrasonic sensor can actually see (200 cm).
    fn effective_constriction_mm(self) -> u16 {
        let raw = if self.left_flank_distance_mm == u16::MAX && self.right_flank_distance_mm == u16::MAX {
            // Arc touches both boundaries — no flanking obstacles; use interior min.
            self.constriction_depth_mm
        } else {
            self.left_flank_distance_mm.min(self.right_flank_distance_mm)
        };
        raw.min(MAX_SENSING_DISTANCE_MM)
    }

    /// Whether the arc midpoint is within the heading limit.
    fn within_heading_limit(self) -> bool {
        (self.midpoint_deg() - CENTER_ANGLE_DEG).abs() <= MAX_HEADING_OFFSET_DEG
    }
}

// ── Main algorithm ──────────────────────────────────────────────────────────

/// Analyze the sweep buffer and return the best gap decision, or `None` if no path.
///
/// # Arguments
/// * `buffer` — 161-entry array indexed by servo angle (0–160°), `Some(mm)` for
///   valid reading, `None` for timeout/error.
/// * `correction_angle_deg` — the desired correction angle in servo space
///   (derived from accumulated drift: `80 - drift` where drift is signed,
///   positive=right).
/// * `remaining_target_cm` — remaining distance to target in cm.
///
/// # Returns
/// * `Some(GapDecision)` if a valid gap or fallback path exists.
/// * `None` if no forward progress is possible (no constriction >10 cm).
#[must_use]
pub fn analyze_gaps(
    buffer: &[Option<u16>; 161],
    correction_angle_deg: f32,
    remaining_target_cm: f32,
) -> Option<GapDecision> {
    // ── Step 1 & 2: Classify each angle and extract contiguous clear arcs ──
    //
    // A reading is "clear" only when `None` (timeout — no echo).
    // Any `Some(d)` means an object was detected — an obstacle to navigate around.

    // Maximum number of arcs: alternating clear/obstacle across 161 entries
    // gives at most 81 arcs.
    let mut arcs_buf = [ClearArc {
        start_angle: 0,
        end_angle: 0,
        constriction_depth_mm: 0,
        left_flank_distance_mm: 0,
        right_flank_distance_mm: 0,
    }; 81];
    let mut arc_count: usize = 0;

    let mut i: usize = 0;
    while i < 161 {
        // Find the start of a clear run.
        if !is_clear(buffer[i]) {
            i += 1;
            continue;
        }
        // SAFETY: i is bounded to [0, 160], which fits in u8.
        #[allow(clippy::cast_possible_truncation)]
        let start = i as u8;
        let mut min_distance: u16 = u16::MAX;

        // Look back to find the left-flank obstacle distance.
        let left_flank_distance_mm = if start == 0 {
            u16::MAX
        } else {
            // SAFETY: start > 0, so start-1 is in [0, 159].
            // The angle immediately before the clear arc is an obstacle angle.
            // The buffer entry at that angle holds the obstacle's distance.
            buffer[(start - 1) as usize].unwrap_or(u16::MAX)
        };

        // Walk the clear run.
        while i < 161 && is_clear(buffer[i]) {
            if let Some(d) = buffer[i] {
                min_distance = min_distance.min(d);
            }
            i += 1;
        }
        // SAFETY: i-1 is bounded to [0, 160], which fits in u8.
        #[allow(clippy::cast_possible_truncation)]
        let end = (i - 1) as u8;

        // Look ahead to find the right-flank obstacle distance.
        // SAFETY: i is the index of the first obstacle after the clear run.
        let right_flank_distance_mm = if i >= 161 {
            u16::MAX
        } else {
            buffer[i].unwrap_or(u16::MAX)
        };

        arcs_buf[arc_count] = ClearArc {
            start_angle: start,
            end_angle: end,
            constriction_depth_mm: min_distance,
            left_flank_distance_mm,
            right_flank_distance_mm,
        };
        arc_count += 1;
    }

    if arc_count == 0 {
        return None;
    }

    let arcs = &arcs_buf[..arc_count];

    // ── Step 3–6: Filter to valid gaps ─────────────────────────────────────
    //
    // Valid gaps: not boundary, within heading limit, lateral width ≥ MIN_GAP_WIDTH_MM.

    let mut best_idx: Option<usize> = None;
    let mut best_score: f32 = 0.0;

    for (idx, arc) in arcs.iter().enumerate() {
        if arc.is_boundary() {
            continue;
        }
        if !arc.within_heading_limit() {
            continue;
        }
        if arc.lateral_width_mm() < f32::from(MIN_GAP_WIDTH_MM) {
            continue;
        }

        let score = (arc.midpoint_deg() - correction_angle_deg).abs();

        let replace = best_idx.is_none_or(|prev_idx| {
            if score < best_score {
                true
            } else if (score - best_score).abs() < f32::EPSILON {
                // Tie-break: gap with midpoint closest to CENTER_ANGLE_DEG.
                let current_tb = (arc.midpoint_deg() - CENTER_ANGLE_DEG).abs();
                let best_tb = (arcs[prev_idx].midpoint_deg() - CENTER_ANGLE_DEG).abs();
                current_tb < best_tb
            } else {
                false
            }
        });

        if replace {
            best_idx = Some(idx);
            best_score = score;
        }
    }

    // ── Step 7–8: Compute decision for the chosen gap ──────────────────────

    if let Some(idx) = best_idx {
        return Some(build_decision(arcs[idx], remaining_target_cm));
    }

    // ── Step 9: Fallback — longest constriction among all arcs ─────────────

    // `arc_count > 0` was already checked above, so `max_by_key` on a
    // non-empty slice always returns `Some`; `?` is safe here.
    let best_fallback = arcs.iter().max_by_key(|arc| arc.effective_constriction_mm())?;

    if best_fallback.effective_constriction_mm() > NO_PATH_THRESHOLD_MM {
        return Some(build_decision(*best_fallback, remaining_target_cm));
    }

    // ── Step 10: No path ──────────────────────────────────────────────────

    None
}

// ── Helpers ─────────────────────────────────────────────────────────────────

/// A reading is "clear" when the sensor detected no echo return (`None`).
const fn is_clear(reading: Option<u16>) -> bool {
    reading.is_none()
}

/// Build a `GapDecision` from a chosen `ClearArc`.
fn build_decision(arc: ClearArc, remaining_target_cm: f32) -> GapDecision {
    let servo_midpoint_deg = arc.midpoint_deg();
    let rotation_degrees = (servo_midpoint_deg - CENTER_ANGLE_DEG).abs();
    // Gap on the left (angle > 80°) → turn left (CounterClockwise).
    // Gap on the right (angle < 80°) → turn right (Clockwise).
    // Servo coordinate system: 0° = right, 80° = center, 160° = left.
    let rotation_direction = if servo_midpoint_deg > CENTER_ANGLE_DEG {
        RotationDirection::CounterClockwise
    } else {
        RotationDirection::Clockwise
    };

    // Effective constriction — already capped at MAX_SENSING_DISTANCE_MM.
    let effective_depth_mm = arc.effective_constriction_mm();
    let depth_cm = f32::from(effective_depth_mm) / 10.0;
    let drive_distance_cm = (depth_cm - SAFETY_MARGIN_CM).min(remaining_target_cm).max(0.0);

    GapDecision {
        servo_midpoint_deg,
        rotation_degrees,
        rotation_direction,
        drive_distance_cm,
    }
}
