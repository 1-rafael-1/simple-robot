# Domain Glossary

Shared vocabulary for the simple-robot project. Use these terms when discussing architecture, design, or implementation.

## Hardware

**Microcontroller**
The Raspberry Pi Pico 2 (RP2350), the main processor running the firmware. One core used at 150 MHz.

**IMU** (Inertial Measurement Unit)
The ICM-20948 9-axis sensor (accelerometer, gyroscope, magnetometer). Provides orientation data for navigation and stabilization. Currently the magnetometer is unreliable (motor/wire interference), so effectively 6-axis in practice.

**Ultrasonic Sensor**
The HC-SR04 distance sensor mounted on a servo. Provides distance readings (cm) at varying angles. Used for obstacle detection and spatial awareness.

**IR Sensors** (Infrared)
Two IR obstacle detection sensors (left-front, right-front). Edge-triggered: report obstacle present/absent. Less precise than ultrasonic but reliable at close range.

**Encoders**
Four motor encoders (left-front, left-rear, right-front, right-rear). Report pulse counts per motor revolution. Used for speed measurement, distance tracking, and drift compensation.

**Motor Driver**
Two TB6612FNG motor driver carriers (one per track). Controlled via PWM for speed and direction.

**OLED**
SSD1306 I2C display (128×64 pixels). Primary user interface for menus, sensor readouts, and status. The top 16 px show text (4 lines × 20 characters); the bottom 48 px render a radar sweep visualization showing obstacle positions from the sweep buffer.

**Rotary Encoder**
EC11 rotary encoder with push button. The main user input for menu navigation and selection. A/B quadrature signals on Pico GPIO 22/23 via PIO; push button on port expander Port 1 bit 7.

**RC Receiver**
Remote control receiver with 4 buttons (A/B/C/D). Used for teleoperation and mode switching.

**LED** (RGB LED)
Status indicator — shows battery level and obstacle detection state via color.

**Port Expander**
PCA9555 I2C GPIO expander. Outputs: motor direction pins (Port 0, 8 bits) and motor driver chip enables (Port 1, bits 4-5). Inputs: RC receiver buttons A-D (Port 1, bits 0-3), IR obstacle detect (Port 1, bit 6), rotary encoder push button (Port 1, bit 7). The rotary encoder's A/B quadrature signals go directly to Pico GPIO 22/23 (PIO); the servo uses Pio on a separate Pico pin.

**Battery**
LiPo battery pack, stepped down via LM2596 buck converter. Voltage monitoring provides charge level (0–100%).

**Vision Module**
Grove Vision AI v2 camera module (future use). Intended for follow-me and object-finding modes.

## Driving

**Track**
One side of the robot's drivetrain (left or right). Each track has two motors driving a continuous belt.

**Coast**
Freewheel — motors are unpowered, the robot rolls to a stop. Slower deceleration than braking.

**Brake**
Active electrical braking — motors are shorted or driven in opposition for rapid stop.

**Drive Distance**
A `DriveAction` that commands the robot to travel a specified distance (straight or curved arc) using encoder feedback with optional IMU correction. Completes when the target distance is reached or an interrupt preempts it.

**Rotate Exact**
A `DriveAction` that commands an in-place rotation to a target angle using IMU feedback. Completes when the angle is reached within tolerance.

**Drift Compensation**
Encoder-based correction that equalises left/right track speeds to prevent veering. Currently handled by higher-level intents (distance, rotation) via IMU heading correction and encoder feedback — the inline per-command path was removed from `Differential` (now a passthrough). The pure math lives in `distance.rs` as helper functions.

**Ramp-Down**
Progressive speed reduction as a distance or rotation command approaches its target. Prevents overshoot.

**Sweep**
A full 0–160° pass of the ultrasonic sensor on its servo, producing one distance reading per 1° of angle. Obstacle positions in the sweep buffer are corrected for the sensor's 30° detection cone (see Sweep Correction). Used to build a spatial picture for gap analysis.
_Avoid_: Scan, radar pass

**Sweep Correction**
Algorithm applied to each contiguous obstacle run in a sweep that compensates for the ultrasonic sensor's 30° detection cone. Subtracts 15° from each edge of the run and splits runs into separate obstacles when a >40% distance jump is detected. Corrected positions overwrite the raw readings in the sweep buffer.
_Avoid_: Cone compensation, de-widening

**Obstacle Run**
A contiguous sequence of Some(distance) readings within a sweep, bounded by Timeout/Error readings or sweep boundaries (0° or 160°). Each run is independently corrected by Sweep Correction.
_Avoid_: Detection block, contiguous hit

**Distance Discontinuity**
A >40% jump in reported distance between consecutive angles within an obstacle run. Signals that two physically separate obstacles at different distances are merging into one run due to the 30° cone overlap. Causes the run to be split into sub-runs before correction.
_Avoid_: Range jump, depth break

**Sweep Buffer**
A shared `[Option<u16>; 161]` array (`SWEEP_BUFFER`) indexed by servo angle (0–160°). Each entry holds a distance reading in millimeters (`Some`) or a timeout/error marker (`None`). Written by the ultrasonic task during sweeps after cone correction; read by gap analysis and the display for rendering.
_Avoid_: Radar buffer, scan buffer

**ObstaclePoint**
A data struct carrying a single corrected obstacle detection: `angle_deg: f32` (servo angle, 0–160°) and `distance_cm: f64` (distance in centimeters). Extracted from the sweep buffer by `sweep_buffer_points()` for consumers that need point-level data (e.g., display rendering).
_Avoid_: SweepPoint, detection point

**SweepPoints**
A fixed-capacity list of `ObstaclePoint` values (max 32). Returned by `sweep_buffer_points()` — the canonical conversion from the raw sweep buffer to consumer-friendly obstacle coordinates. Capacity is bounded because a 161° sweep produces at most ~54 corrected obstacles (3° minimum width), and 32 points saturate a 128×64 display.
_Avoid_: Point list, obstacle list

**Gap**
A contiguous angular arc within a corrected sweep where the ultrasonic sensor reads clear (no obstacle within 15 cm). A valid gap has at least 30 cm lateral width at its constriction depth, is flanked by obstacles on both sides, and lies within ±90° of the robot's forward center. The robot drives through a gap by rotating to its midpoint angle and driving straight.
_Avoid_: Opening, corridor, path

**Constriction Depth**
The minimum distance reading across the angles that make up a gap. Determines how far the robot can safely travel through that gap before it must re-sweep.
_Avoid_: Minimum range, bottleneck distance

**Leg**
One rotation-then-straight-drive maneuver through a selected gap. A leg's length is the constriction depth (minus 10 cm safety margin), or the remaining target distance if shorter. The robot's accumulated drift and total forward progress are updated after each leg.
_Avoid_: Step, segment, hop

**Accumulated Drift**
The signed sum of all deviation angles taken so far during an Attempt Straight Line run. Positive = right, negative = left. The correction target for the next leg is `-drift`, so the robot strives to return toward the ideal unobstructed course.
_Avoid_: Heading error, offset sum

**Correction Angle**
The ideal gap midpoint angle that would cancel the accumulated drift (= `-drift`). The gap closest to this angle is preferred when choosing the next leg.
_Avoid_: Recovery angle, compensation heading

## Modes

**RC Mode** (Remote Control)
Direct teleoperation via remote control buttons. The operator steers.

**Autonomous Mode**
The robot drives itself. Two variants: coast-and-avoid and attempt-straight-line.

**Coast-and-Avoid**
An autonomous mode: drive forward until an obstacle is detected (via IR or ultrasonic), then back up and turn a random angle before resuming. Has two phases:
- **Forward phase**: driving forward at constant speed, obstacle detection is armed.
- **Avoidance phase**: backing up and turning after an obstacle interrupt.

**Attempt Straight Line**
An autonomous mode: travel toward a user-set target distance (100–1000 cm) while avoiding obstacles to approximate the endpoint an unobstructed straight line would have reached. The robot sweeps the ultrasonic sensor 0–160°, finds gaps through obstacles, drives a leg toward the best gap, and repeats until the target distance is reached or no forward progress is possible. Accumulated drift is tracked across legs so later legs correct past deviations.

**Test Mode**
Interactive hardware tests accessible from the menu: motor test, encoder test, IMU display (6-axis and 9-axis), ultrasonic sweep, IR+ultrasonic live display, turn accuracy test, straight drive test, arc drive test.

**Calibration**
Procedures that measure and store correction factors: motor speed calibration, IMU calibration (magnetometer), distance factor calibration. Results are persisted to flash storage.

## Architecture

**Orchestrator**
The central `orchestrate` task. Runs a single event loop: waits for events from the system event channel and dispatches them to behavior handlers or UI handlers. Pure routing — no domain logic.

**Event System**
A multi-producer, single-consumer channel (`Events` enum, capacity 64). Sensor tasks and input tasks raise events; the orchestrator consumes them. The seam between producers and consumers.

**State Modules**
Four domain-specific state modules under `system/state/`, each with its own `Mutex`:
1. **Power** — battery level and voltage.
2. **Calibration** — calibration data load status, flags, and the distance calibration factor.
3. **Perception** — obstacle detection state (IR, ultrasonic, combined) and ultrasonic readings. Deepened: all access goes through accessor functions; internal atomics provide lock-free reads.
4. **Motion** — track speed state, also exposed via lock-free atomics for the IMU hot path.

**Lock Order**
When multiple state mutexes must be held: Power → Calibration → Perception → Motion. Prevents deadlocks.

**Drive Subsystem**
The `drive` module tree. Owns the drive task, command queue, interrupt signal, control algorithms (rotation, distance, brake/coast, differential), sensor feedback channels, and calibration procedures. Commands flow through a thin `dispatch` that routes to control modules. Control modules own sensor setup internally; teardown is declared via `IntentTeardown` descriptors and executed by the dispatch on completion or interrupt. Exposes two public entry points: `send_drive_command` and `send_drive_interrupt`.

**Drive Queue**
A builder (`DriveQueueBuilder`) that accumulates `DriveCommand` steps and submits them for sequential execution. A single `drive_queue_executor` task runs one queue at a time and emits a single queue-level completion.

**Intent**
A state machine that owns a specific motion behaviour — rotation, distance drive, brake/coast settle, or idle. Each intent is an `ActiveIntent` variant carrying its controller state, completion flag, and lifecycle descriptors. Commands that complete instantly (e.g. `Differential`) are not intents; they are fire-and-forget.

**IntentTeardown**
An enum declaring what sensor streams must be stopped when an intent completes or is interrupted. Each `ActiveIntent` variant returns its teardown descriptor via `teardown()`; the `dispatch` executes it. This keeps the dispatch thin — it knows to stop sensors but not which specific sensors each intent required. Sensor setup is owned by each intent's async `init()` function. See [ADR-0004](docs/adr/0004-intent-setup-removal.md).

**Dispatch**
The `dispatch` module — the thin seam between the drive command queue and the control modules. Routes incoming `DriveCommand` envelopes, handles standby wake-up, and executes `IntentTeardown` descriptors on completion or interrupt. Owns no per-intent knowledge.

**Behavior Handlers**
Functions under `task/behavior/` that react to specific events. Domain logic lives here — obstacle fusion, sensor forwarding, battery updates. Called by the orchestrator.

**UI Subsystem**
The `task/ui/` module tree. Owns the display rendering, menu navigation, and UI state (`UiMode` enum). Receives rotary encoder events from the orchestrator.

**Sensor Tasks**
Embassy tasks that interface with hardware sensors (`task/sensors/`): IMU read loop, encoder read loop, IR obstacle polling, ultrasonic sweep/servo control. Measurement data flows point-to-point via direct channels into the drive subsystem or perception state (bypassing the event bus). Semantic events (obstacle detection, button presses) still flow through the system event channel via `raise_event`.

**IO Modules**
Hardware abstraction tasks under `task/io/`: display driver, flash storage (calibration persistence), port expander driver. Use channel-based command APIs.

**Perception**
The deepened state module that owns obstacle detection. Obsolete: direct `PERCEPTION_STATE` field access. Current: accessor functions for lock-free boolean reads (`is_obstacle_detected`, `is_ir_obstacle_detected`, `is_ultrasonic_obstacle_detected`) and mutex-guarded reading/angle storage (`set_ultrasonic_reading`, a pure data store). Obstacle classification flows through the event bus: sensor tasks raise `Events::ObstacleDetected`, the behavior handler calls `set_ir_obstacle` / `set_ultrasonic_obstacle` to update atomics. See [ADR-0003](docs/adr/0003-perception-event-bus.md).

**Obstacle Threshold**
The distance cutoff (default 20 cm, matching the ultrasonic sensor task's `ULTRASONIC_OBSTACLE_THRESHOLD_CM`) used by the ultrasonic sensor task to classify readings as obstacles. Reading distance ≤ threshold → `Events::ObstacleDetected` raised on the event bus. The perception module no longer owns threshold-based classification (see [ADR-0003](docs/adr/0003-perception-event-bus.md)).

**ObstacleSource**
An enum (`Ir` | `Ultrasonic`) carried by `Events::ObstacleDetected` that identifies which sensor triggered the detection. Used by the obstacle behavior handler to route updates to the correct perception atomic.

**InterruptKind**
An enum (`EmergencyBrake`, `Stop`, `CancelCurrent`) that specifies how the drive subsystem preempts the active intent. See also [`Dispatch`](#dispatch).

**EmergencyBrake**
An `InterruptKind::EmergencyBrake` interrupt sent to the drive subsystem when a combined obstacle is detected. Causes immediate active motor braking, cancels the active intent (if any), bumps the command epoch, and drains queued commands. Mode-agnostic since [ADR-0005](docs/adr/0005-agnostic-obstacle-handling.md) — dispatched unconditionally on any obstacle detection across all modes.

**Change Detected**
An enum (`ChangeDetected`) returned by perception setters: `NoChange`, `ChangedToDetected`, `ChangedToCleared`. Lets callers react to obstacle state transitions without re-reading the combined flag.
