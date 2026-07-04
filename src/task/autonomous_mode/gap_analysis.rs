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
/// Minimum width for a valid gap in mm (30 cm) — Pass 1.
const MIN_GAP_WIDTH_MM: u16 = 300;
/// If no constriction exceeds this, the path is blocked (10 cm).
const NO_PATH_THRESHOLD_MM: u16 = 100;
/// Safety margin subtracted from constriction depth (cm).
const SAFETY_MARGIN_CM: f32 = 10.0;
/// Center (straight ahead) servo angle in degrees.
const CENTER_ANGLE_DEG: f32 = 80.0;
/// Maximum heading offset from center (±90°).
const MAX_HEADING_OFFSET_DEG: f32 = 90.0;
/// Hysteresis threshold in degrees (±15° from previous gap midpoint).
const HYSTERESIS_THRESHOLD_DEG: f32 = 15.0;

// ── Scoring weights ─────────────────────────────────────────────────────────

/// Weight for constriction depth (primary: make forward progress).
const DEPTH_WEIGHT: f32 = 0.35;
/// Weight for heading alignment (secondary: maintain direction).
const HEADING_WEIGHT: f32 = 0.30;
/// Weight for gap width (safety margin).
const WIDTH_WEIGHT: f32 = 0.15;
/// Weight for hysteresis (prevent oscillation).
const HYST_WEIGHT: f32 = 0.10;
/// Weight for alignment within the gap (centering reduces scrape risk).
const ALIGN_WEIGHT: f32 = 0.10;
/// Penalty weight for boundary arcs (unknown space beyond sweep edge).
const BOUNDARY_WEIGHT: f32 = 0.05;
/// Relaxed minimum width in mm (25 cm) — Passes 2–3.
const RELAXED_MIN_WIDTH_MM: f32 = 250.0;

// ── Types ───────────────────────────────────────────────────────────────────

/// Result of gap analysis: either a chosen gap or blocked.
#[derive(Debug, Clone, Copy, defmt::Format)]
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

// ── Scoring helpers ─────────────────────────────────────────────────────────

/// Cosine falloff: 1.0 at 0° offset, ~0.0 at 90° offset.
fn heading_score(midpoint_deg: f32, desired_heading_deg: f32) -> f32 {
    libm::cosf((midpoint_deg - desired_heading_deg).to_radians())
}

/// Linear depth score, capped at 1.0.
fn depth_score(arc: ClearArc) -> f32 {
    let depth = f32::from(arc.effective_constriction_mm());
    (depth / f32::from(MAX_SENSING_DISTANCE_MM)).min(1.0)
}

/// Linear width score: 0.0 at `min_width`, 1.0 at `2 × min_width`, saturates above.
fn width_score(arc: ClearArc, min_width_mm: f32) -> f32 {
    if min_width_mm <= 0.0 {
        return 1.0;
    }
    ((arc.lateral_width_mm() - min_width_mm) / min_width_mm).clamp(0.0, 1.0)
}

/// Binary hysteresis bonus: 1.0 if within ±15° of the previous gap midpoint.
fn hysteresis_score(midpoint_deg: f32, prev_midpoint_deg: Option<f32>) -> f32 {
    if let Some(prev) = prev_midpoint_deg
        && (midpoint_deg - prev).abs() <= HYSTERESIS_THRESHOLD_DEG
    {
        return 1.0;
    }
    0.0
}

/// Alignment within the gap: 1.0 when centered on the desired heading,
/// decreasing as the midpoint diverges from desired relative to the arc span.
fn alignment_score(midpoint_deg: f32, desired_heading_deg: f32, span_deg: f32) -> f32 {
    let span = span_deg.max(1.0);
    (1.0 - (midpoint_deg - desired_heading_deg).abs() / span).clamp(0.0, 1.0)
}

// ── Pass helper ─────────────────────────────────────────────────────────────

/// Run one pass of the fallback cascade: filter arcs by width, boundary policy,
/// and heading limit; score survivors with all factors; return the best arc.
///
/// Returns the winning `ClearArc`, or `None` if no valid arc survived filtering.
fn scored_pass(
    arcs: &[ClearArc],
    desired_heading_deg: f32,
    prev_gap_midpoint_deg: Option<f32>,
    min_width_mm: f32,
    allow_boundary: bool,
) -> Option<ClearArc> {
    let mut best_arc: Option<ClearArc> = None;
    let mut best_score = f32::NEG_INFINITY;

    for &arc in arcs {
        // ── Filtering ──
        if !allow_boundary && arc.is_boundary() {
            continue;
        }
        if !arc.within_heading_limit() {
            continue;
        }
        if arc.lateral_width_mm() < min_width_mm {
            continue;
        }

        // ── Scoring ──
        let midpoint = arc.midpoint_deg();
        let score = DEPTH_WEIGHT * depth_score(arc)
            + HEADING_WEIGHT * heading_score(midpoint, desired_heading_deg)
            + WIDTH_WEIGHT * width_score(arc, min_width_mm)
            + HYST_WEIGHT * hysteresis_score(midpoint, prev_gap_midpoint_deg)
            + ALIGN_WEIGHT * alignment_score(midpoint, desired_heading_deg, arc.span_deg());

        let penalty = if allow_boundary && arc.is_boundary() {
            BOUNDARY_WEIGHT
        } else {
            0.0
        };
        let final_score = score - penalty;

        if final_score > best_score {
            best_score = final_score;
            best_arc = Some(arc);
        }
    }

    best_arc
}

// ── Main algorithm ──────────────────────────────────────────────────────────

/// Analyze the sweep buffer and return the best gap decision, or `None` if no path.
///
/// # Arguments
/// * `buffer` — 161-entry array indexed by servo angle (0–160°), `Some(mm)` for
///   valid reading, `None` for timeout/error.
/// * `robot_x_cm`, `robot_y_cm` — current estimated position in world frame.
/// * `target_x_cm` — target X coordinate (always `(target_cm, 0)` in world frame).
/// * `robot_heading_deg` — current estimated heading in world-frame degrees.
/// * `prev_gap_midpoint_deg` — previous gap midpoint for hysteresis (None on first sweep).
/// * `remaining_target_cm` — remaining distance to target in cm.
///
/// # Returns
/// * `Some(GapDecision)` if a valid gap or fallback path exists.
/// * `None` if no forward progress is possible.
#[must_use]
#[allow(clippy::similar_names)]
pub fn analyze_gaps(
    buffer: &[Option<u16>; 161],
    robot_x_cm: f32,
    robot_y_cm: f32,
    target_x_cm: f32,
    _robot_heading_deg: f32,
    prev_gap_midpoint_deg: Option<f32>,
    remaining_target_cm: f32,
) -> Option<GapDecision> {
    // ── Compute desired heading from odometry ──────────────────────────────
    //
    // World frame: target is at (target_x_cm, 0).  Desired world heading is
    // atan2 from current position toward the target.  Then map to servo space:
    //   servo center (80°) = world heading 0°
    //   servo > 80° = left (CCW, negative world heading)
    //   servo < 80° = right (CW, positive world heading)

    let desired_heading_world_rad = libm::atan2f(-robot_y_cm, target_x_cm - robot_x_cm);
    let desired_heading_deg = (80.0 - desired_heading_world_rad.to_degrees()).clamp(0.0, 160.0);

    // ── Step 1 & 2: Classify each angle and extract contiguous clear arcs ──
    //
    // A reading is "clear" only when `None` (timeout — no echo).
    // Any `Some(d)` means an object was detected — an obstacle to navigate around.

    // After cone correction, obstacles have at least 3° width, so the
    // worst-case alternating pattern yields ~40 clear arcs.  A capacity
    // of 64 is safe with margin while keeping the stack allocation small.
    let mut arcs_buf = [ClearArc {
        start_angle: 0,
        end_angle: 0,
        constriction_depth_mm: 0,
        left_flank_distance_mm: 0,
        right_flank_distance_mm: 0,
    }; 64];
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

    // ── Serially-constrained fallback cascade ──────────────────────────────
    //
    // Each pass relaxes exactly one constraint.  If a pass finds a gap, stop.

    // Pass 1 — Primary: full width (300 mm), no boundary arcs.
    if let Some(gap) = scored_pass(
        arcs,
        desired_heading_deg,
        prev_gap_midpoint_deg,
        f32::from(MIN_GAP_WIDTH_MM),
        false,
    ) {
        return Some(build_decision(gap, remaining_target_cm));
    }

    // Pass 2 — Relax width: 250 mm, no boundary arcs.
    if let Some(gap) = scored_pass(
        arcs,
        desired_heading_deg,
        prev_gap_midpoint_deg,
        RELAXED_MIN_WIDTH_MM,
        false,
    ) {
        return Some(build_decision(gap, remaining_target_cm));
    }

    // Pass 3 — Allow boundary arcs: 250 mm, boundary arcs incur BOUNDARY_WEIGHT penalty.
    if let Some(gap) = scored_pass(
        arcs,
        desired_heading_deg,
        prev_gap_midpoint_deg,
        RELAXED_MIN_WIDTH_MM,
        true,
    ) {
        return Some(build_decision(gap, remaining_target_cm));
    }

    // Pass 4 — Last resort: any arc with constriction > 100 mm (no scoring, just depth).
    // `arc_count > 0` was already checked above, so `max_by_key` always returns `Some`.
    let best_fallback = arcs.iter().max_by_key(|arc| arc.effective_constriction_mm())?;
    if best_fallback.effective_constriction_mm() > NO_PATH_THRESHOLD_MM {
        return Some(build_decision(*best_fallback, remaining_target_cm));
    }

    None
}

// ── Odometry ────────────────────────────────────────────────────────────────

/// Pure per-leg dead-reckoning update.
///
/// Rotation is applied **first** (heading changes), then forward drive happens
/// along the new heading.
///
/// # Arguments
/// * `x_cm`, `y_cm` — current position in cm (world frame, target is at `(target_cm, 0)`).
/// * `heading_deg` — current heading in world-frame degrees.
/// * `rotation_deg` — rotation amount in degrees (always positive).
/// * `direction` — rotation direction (`Clockwise` → heading increases).
/// * `distance_cm` — forward distance driven in cm.
#[must_use]
pub fn update_odometry(
    x_cm: f32,
    y_cm: f32,
    heading_deg: f32,
    rotation_deg: f32,
    direction: RotationDirection,
    distance_cm: f32,
) -> (f32, f32, f32) {
    // Apply rotation to heading first.
    let new_heading_deg = match direction {
        RotationDirection::Clockwise => heading_deg + rotation_deg,
        RotationDirection::CounterClockwise => heading_deg - rotation_deg,
    };

    // Forward drive along the new heading.
    let heading_rad = new_heading_deg.to_radians();
    let new_x = x_cm + distance_cm * libm::cosf(heading_rad);
    let new_y = y_cm + distance_cm * libm::sinf(heading_rad);

    (new_x, new_y, new_heading_deg)
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
