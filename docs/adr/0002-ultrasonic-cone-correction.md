# Ultrasonic cone correction — in-place buffer mutation with heuristic obstacle splitting

The HC-SR04 ultrasonic sensor's 30° detection cone inflates every obstacle's apparent width by 30° in the sweep buffer, making obstacles appear wider than they are and degrading gap analysis. We apply a geometry-based correction that shrinks each contiguous obstacle run by 15° on each edge, mutating `SWEEP_BUFFER` in-place so all consumers (display, gap analysis) see corrected data with no extra steps.

## Considered Options

**Raw + corrected dual buffer vs. in-place mutation.** Keeping raw data in a separate buffer would preserve it for debugging but doubles the 322-byte footprint and forces every consumer to know which buffer to read. **Chose in-place mutation** — the raw data has no consumer that needs it after correction, and a single buffer simplifies the display and gap-analysis paths.

**Full deconvolution vs. heuristic splitting.** A full deconvolution could infer each obstacle's true angular profile from the cone's overlap pattern, but requires assumptions about obstacle geometry (flat walls, circular posts) that don't generalise. **Chose heuristic splitting** — a >40% distance jump within a contiguous run signals a separate obstacle, and each sub-run is independently edge-corrected. Simple, handles the common case (two obstacles at different distances), and degrades gracefully for angled walls.

**On-the-fly vs. post-sweep correction.** Correcting each run as it ends during the sweep enables real-time corrected display updates mid-sweep and keeps the same code path for continuous and buffered sweeps. **Chose on-the-fly** — the sensor task tracks run boundaries and calls correction at each `Some → None` transition, so the buffer is always fully corrected.

## Consequences

- The display path consolidates from two `ShowSweep` callers with separate point-accumulation logic to a single `ShowSweepFromBuffer` variant that renders corrected points from the buffer. `PointsBuffer`, direction tracking, and point retention logic are removed from the display task.
- Gap analysis (`attempt_straight_line`) receives corrected obstacle widths without any change — it already reads from `SWEEP_BUFFER`.
- Obstacle runs touching the sweep boundary (0° or 160°) receive one-sided correction only, so boundary obstacles are treated as wider than they may actually be. This is conservative for gap analysis.
- Minimum obstacle width of 3° (corrected) prevents thin objects like chair legs from collapsing to points while keeping the correction honest for genuinely narrow obstacles.
- Corrected obstacle interiors are filled with the minimum distance in the run (not left as `None` between edges). This is conservative for gap analysis — the constriction depth may be slightly underestimated, but it prevents false gaps from appearing inside corrected obstacles.
- A dedicated `ultrasonic_correction` module keeps the correction logic testable in isolation with no hardware or async dependencies.
