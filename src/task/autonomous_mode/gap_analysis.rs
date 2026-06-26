//! Gap analysis for ultrasonic sweep buffer.
//!
//! Pure, stateless algorithm that analyzes a 161-entry ultrasonic sweep buffer
//! (indexed by servo angle 0–160°) and returns the best gap to drive through.

use core::f32::consts::PI;

use crate::task::drive::types::RotationDirection;

// ── Constants ───────────────────────────────────────────────────────────────

/// Obstacle threshold in mm (15 cm — matches `ULTRASONIC_OBSTACLE_THRESHOLD_CM`).
const OBSTACLE_THRESHOLD_MM: u16 = 150;
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

    /// Estimated lateral width in mm.
    fn lateral_width_mm(self) -> f32 {
        let depth = f32::from(self.constriction_depth_mm);
        let half_span_rad = (self.span_deg() * PI) / 360.0;
        2.0 * depth * libm::sinf(half_span_rad)
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
    // A reading is "clear" when Some(d) with d > OBSTACLE_THRESHOLD_MM.
    // None (timeout/error) is treated as obstacle for safety.

    // Maximum number of arcs: alternating clear/obstacle across 161 entries
    // gives at most 81 arcs.
    let mut arcs_buf = [ClearArc {
        start_angle: 0,
        end_angle: 0,
        constriction_depth_mm: 0,
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

        arcs_buf[arc_count] = ClearArc {
            start_angle: start,
            end_angle: end,
            constriction_depth_mm: min_distance,
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
    let best_fallback = arcs.iter().max_by_key(|arc| arc.constriction_depth_mm)?;

    if best_fallback.constriction_depth_mm > NO_PATH_THRESHOLD_MM {
        return Some(build_decision(*best_fallback, remaining_target_cm));
    }

    // ── Step 10: No path ──────────────────────────────────────────────────

    None
}

// ── Helpers ─────────────────────────────────────────────────────────────────

/// A reading is "clear" when `Some(d)` with `d > OBSTACLE_THRESHOLD_MM`.
/// `None` (timeout/error) is treated as obstacle for safety.
const fn is_clear(reading: Option<u16>) -> bool {
    matches!(reading, Some(d) if d > OBSTACLE_THRESHOLD_MM)
}

/// Build a `GapDecision` from a chosen `ClearArc`.
fn build_decision(arc: ClearArc, remaining_target_cm: f32) -> GapDecision {
    let servo_midpoint_deg = arc.midpoint_deg();
    let rotation_degrees = (servo_midpoint_deg - CENTER_ANGLE_DEG).abs();
    let rotation_direction = if servo_midpoint_deg > CENTER_ANGLE_DEG {
        RotationDirection::CounterClockwise
    } else {
        RotationDirection::Clockwise
    };

    let depth_cm = f32::from(arc.constriction_depth_mm) / 10.0;
    let drive_distance_cm = (depth_cm - SAFETY_MARGIN_CM).min(remaining_target_cm).max(0.0);

    GapDecision {
        servo_midpoint_deg,
        rotation_degrees,
        rotation_direction,
        drive_distance_cm,
    }
}
