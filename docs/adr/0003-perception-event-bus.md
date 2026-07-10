# Ultrasonic perception — event-bus-only path and pure data store

The ultrasonic sensor task wrote obstacle state through two parallel paths: directly via `set_ultrasonic_reading` (which updated atomics) and indirectly via `Events::ObstacleDetected` → `handle_obstacle_detected` → `set_ultrasonic_obstacle` (which updated the same atomics). This dual-write path created a race condition and made the `ChangeDetected` return from `set_ultrasonic_obstacle` unreliable. Additionally, `set_ultrasonic_reading` acquired the perception mutex twice, creating a TOCTOU window between threshold read and reading storage.

## Considered options

**Keep the dual-write path and fix TOCTOU separately.** Would have merged the two mutex acquires in `set_ultrasonic_reading` without addressing the race with the event-bus path. **Rejected** — the dual-write race remained, just with a narrower TOCTOU window.

**Remove the event bus path, make `set_ultrasonic_reading` the sole atomic writer.** Would have made ultrasonic asymmetric with IR (which still uses the event bus). **Rejected** — asymmetry is its own friction; both obstacle sensors should use the same communication pattern.

**Event bus only, `set_ultrasonic_reading` becomes pure data store.** The ultrasonic task classifies obstacles locally (as it already does) and raises `Events::ObstacleDetected`. `handle_obstacle_detected` remains the single writer of both `IR_DETECTED` and `ULTRASONIC_DETECTED` atomics. `set_ultrasonic_reading` stores only the reading + angle in the mutex — no atomics, no threshold read. **Chosen** — symmetric with IR, one writer per atomic, TOCTOU vanishes naturally.

## Consequences

- `set_ultrasonic_reading` loses its threshold-aware obstacle classification and atomic updates. It becomes a pure data store with a single lock scope.
- `set_ultrasonic_obstacle` remains the sole writer of `ULTRASONIC_DETECTED`, called only from the event-bus path.
- Both IR and ultrasonic obstacle state follow the same flow: sensor task → event bus → behavior handler → perception atomics.
- The dual-write race is eliminated.
- The TOCTOU window is eliminated.
