# Delete IntentSetup — intent init owns sensor lifecycle

The `IntentSetup` enum declared what sensor streams each intent needed during setup. Control modules returned a descriptor from `init()`; the `dispatch` executed it via `execute_intent_setup`. In practice, only `EncoderSettle` (brake/coast) was used; `RotationImu` was dead (rotation's init started IMU itself) and `DistanceImuAndEncoder` was partially bypassed (dispatch did most setup inline in `handle_drive_distance` before the descriptor was executed).

After making `handle_drive_distance` consistent with other intents and giving `DistanceDriveState` an async `init()`, all three control modules own their full setup internally. `execute_intent_setup` has no real work. The seam had one real adapter and two dead ones — a hypothetical seam, not a real one.

## Considered options

**Make the descriptor system real.** Move sensor start from `init()` into `execute_intent_setup` for all three intents. Dispatch owns sensor lifecycle. **Rejected** — adds indirection without leverage. The dispatch gains no new capability (it can't substitute sensors, can't test them independently), and the one-caller-per-descriptor ratio means the seam is hypothetical. Complexity moves from init functions to a central match statement.

**Delete IntentSetup, keep IntentTeardown.** `IntentTeardown` is real — all three variants are executed by `execute_intent_teardown` on completion or interrupt, and the teardown logic (stop encoder, stop IMU, zero motors) benefits from centralisation since it runs from both the completion and interrupt paths. **Chosen** — setup is init's responsibility; teardown is dispatch's. Teardown remains a real seam with two callers (completion and interrupt).

## Consequences

- `IntentSetup` enum and `execute_intent_setup` are deleted.
- `IntentTeardown` and `execute_intent_teardown` remain unchanged.
- `BrakeCoastState::init()` becomes async and calls `start_encoder_sampling()` internally.
- `DistanceDriveState::init()` becomes async and owns sensor start, IMU stabilise, reference capture, and initial motor command.
- `RotationState::init()` unchanged — already async, already owns setup.
- `handle_drive_distance` in dispatch shrinks to a thin router matching `handle_rotate_exact`.
- The dispatch is now genuinely thin: route commands, execute teardown on completion/interrupt, manage epoch.
