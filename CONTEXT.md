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
SSD1306 I2C display (4 lines × 20 characters). Primary user interface for menus, sensor readouts, and status.

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

## Modes

**RC Mode** (Remote Control)
Direct teleoperation via remote control buttons. The operator steers.

**Autonomous Mode**
The robot drives itself. Currently one variant: coast-and-avoid.

**Coast-and-Avoid**
An autonomous mode: drive forward until an obstacle is detected (via IR or ultrasonic), then back up and turn a random angle before resuming. Has two phases:
- **Forward phase**: driving forward at constant speed, obstacle detection is armed.
- **Avoidance phase**: backing up and turning after an obstacle interrupt.

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
The `drive` module tree. Owns the drive task, command queue, interrupt signal, control algorithms (rotation, distance, brake/coast, differential), sensor feedback channels, and calibration procedures. Commands flow through a thin `dispatch` that routes to control modules via `IntentSetup` / `IntentTeardown` descriptors — sensor lifecycle is declared by each module, not hardcoded in the dispatch. Exposes two public entry points: `send_drive_command` and `send_drive_interrupt`.

**Drive Queue**
A builder (`DriveQueueBuilder`) that accumulates `DriveCommand` steps and submits them for sequential execution. A single `drive_queue_executor` task runs one queue at a time and emits a single queue-level completion.

**Intent**
A state machine that owns a specific motion behaviour — rotation, distance drive, brake/coast settle, or idle. Each intent is an `ActiveIntent` variant carrying its controller state, completion flag, and lifecycle descriptors. Commands that complete instantly (e.g. `Differential`) are not intents; they are fire-and-forget.

**IntentSetup / IntentTeardown**
Enums declaring what sensor streams an intent needs during setup and teardown. Control modules return these descriptors from `init()` and `teardown()`; the `dispatch` executes them. This keeps the dispatch thin — it knows to start or stop sensors but not which specific sensors each intent requires.

**Dispatch**
The `dispatch` module — the thin seam between the drive command queue and the control modules. Routes incoming `DriveCommand` envelopes, handles standby wake-up, and executes `IntentSetup` / `IntentTeardown` descriptors. Owns no per-intent knowledge.

**Behavior Handlers**
Functions under `task/behavior/` that react to specific events. Domain logic lives here — obstacle fusion, sensor forwarding, battery updates. Called by the orchestrator.

**UI Subsystem**
The `task/ui/` module tree. Owns the display rendering, menu navigation, and UI state (`UiMode` enum). Receives rotary encoder events from the orchestrator.

**Sensor Tasks**
Embassy tasks that interface with hardware sensors (`task/sensors/`): IMU read loop, encoder read loop, IR obstacle polling, ultrasonic sweep/servo control. Measurement data flows point-to-point via direct channels into the drive subsystem or perception state (bypassing the event bus). Semantic events (obstacle detection, button presses) still flow through the system event channel via `raise_event`.

**IO Modules**
Hardware abstraction tasks under `task/io/`: display driver, flash storage (calibration persistence), port expander driver. Use channel-based command APIs.

**Perception**
The deepened state module that owns obstacle detection. Obsolete: direct `PERCEPTION_STATE` field access. Current: 10 accessor functions including lock-free boolean reads and an obstacle distance threshold that auto-detects obstacles from ultrasonic readings.

**Obstacle Threshold**
The distance cutoff (default 15 cm, matching the ultrasonic sensor task) used to classify ultrasonic readings as obstacles. Reading distance ≤ threshold → obstacle detected. Overridable via `set_obstacle_threshold`.

**Change Detected**
An enum (`ChangeDetected`) returned by perception setters: `NoChange`, `ChangedToDetected`, `ChangedToCleared`. Lets callers react to obstacle state transitions without re-reading the combined flag.
