# Attempt Straight Line — buffered sweep, gap analysis, and drift correction

The robot needed an autonomous mode that travels toward a user-set target distance while avoiding obstacles via sensor-guided steering. The design combines a buffered ultrasonic sweep with gap-analysis geometry and accumulated drift correction to approximate the endpoint an unobstructed straight line would have reached.

## Considered Options

**Sweep data collection**: streaming each reading through `perception::set_ultrasonic_reading` (loose coupling, reuses existing pipeline) vs. buffering a full 0–160° pass into a shared `Mutex<[Option<u16>; 161]>` and raising a lean `UltrasonicSweepCompleted` event. **Chose buffering** — gap analysis needs consistent, synchronous access to the whole angular picture, and a 2093-byte buffer on the event channel would be rejected by Clippy. The mm representation (`u16`) is dense enough for a 161-entry heapless buffer while staying memory-friendly.

**Gap selection priority**: minimize divergence from current heading vs. minimize accumulated drift. **Chose drift-first** — the mode's purpose is to reach the user's target waypoint, so correcting past deviations matters more than staying on the instantaneous heading. Ties break toward 0° (straight ahead).

**Speed-boundary obstacle gating**: a system-wide sensor gate vs. gating in the drive mode task itself. **Chose in-task gating** — the mode only arms obstacle detection during drive legs (via `start_ultrasonic_fixed` with `obstacle_detect: true`), and leaves it disarmed during sweeps where the robot is stationary. Deferred system-wide motion-state gating until it is needed across multiple modes.

**Travel distance entry**: embedding range/step constants in the UI controller vs. exposing them from the drive mode module. **Chose encapsulation in the drive mode module** — a new `UiMode` variant handles the UX, but constants (100–1000 cm, 10cm steps, 300cm preset) live in the mode's module, surfaced through accessors. The confirmed distance passes to the mode task via `AutonomousCommand::AttemptStraightLine { target_distance_cm: u16 }`, flowing through the existing autonomous mode controller channel.

## Consequences

- The event system gains `UltrasonicSweepCompleted`, routed by the orchestrator to the mode's handler.
- The sweep task gains a `StartBufferedSweep` command that does one 0–160° pass into the shared buffer.
- Gap analysis is stateless: walk the buffer once, classify clear/obstacle/unknown per angle, extract contiguous arcs, compute constriction depth and width, filter to valid gaps (≥30 cm width, flanked by obstacles ≤15 cm, within ±90° of center), then score by drift correction.
- Partial progress on emergency-braked legs is tracked via `cancellation_telemetry()` from the intent subsystem, so the mode always knows true remaining distance.
- Future spatial-awareness modes (e.g., full room scan, follow-me) can reuse the buffered sweep and gap-analysis primitives.
