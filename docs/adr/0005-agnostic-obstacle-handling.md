# Mode-agnostic obstacle handling — unconditional EmergencyBrake

The obstacle behavior handler (`handle_obstacle_detected`) directly checked `coast_obstacle_avoid::is_active()`, `coast_obstacle_avoid::is_forward_phase()`, and `attempt_straight_line::is_active()` to decide whether to send an `EmergencyBrake` interrupt. This coupled the handler to knowledge of which autonomous modes existed and what phases they were in. Adding a third autonomous mode would require editing the obstacle handler.

## Considered options

**Mode self-filtering via atomics.** Each autonomous mode task reads perception atomics in its own control loop and self-interrupts. **Rejected** — adds worst-case 20ms latency (control tick period) vs. the current direct `Signal` wake. Obstacle avoidance needs immediate braking.

**Registration mechanism.** Modes register a wake signal with the handler. **Rejected** — one adapter means a hypothetical seam. Adds complexity without a second consumer.

**Unconditional EmergencyBrake.** `handle_obstacle_detected` sends an `EmergencyBrake` interrupt on any combined obstacle detection, regardless of mode. `handle_interrupt` in dispatch brakes motors, bumps the command epoch, and drains queued commands — even when no intent is active. This is the intended safety behaviour: IR sensors are armed in all modes (RC, autonomous, testing), and obstacle braking is a system-wide safety invariant. The epoch bump + queue drain are side-effects that always apply; non-autonomous callers should not queue commands they cannot afford to lose. **Chosen** — simplest change, preserves immediate wake, mode-agnostic handler, zero coupling, consistent safety response across all modes.

## Consequences

- `handle_obstacle_detected` loses all imports of `coast_obstacle_avoid` and `attempt_straight_line`.
- The handler's logic reduces to: update perception atomics → update LED indicator → send unconditional `EmergencyBrake` if combined obstacle detected.
- New autonomous modes require no changes to the obstacle handler.
- IR obstacle sensors are armed in all modes (RC, autonomous, testing). Obstacle braking is a system-wide safety invariant: any obstacle detection immediately brakes motors, bumps the command epoch, and drains queued drive commands.
- Callers that queue drive commands during RC mode or testing must expect them to be dropped on obstacle detection.
