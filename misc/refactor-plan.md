# Refactoring Plan: Orchestrator Decomposition & Task Tree Restructure

## Goals
- Keep `task/orchestrate.rs` as a thin event router.
- Move feature ownership into focused modules (UI, initialization/calibration, etc.).
- Restructure the `task` tree for clarity-first organization as the system grows.
- Preserve behavior; only move code and update wiring/imports.
- Improve discoverability by grouping multi-file domains into submodules.

## Scope
- Decompose `task/orchestrate.rs` into UI and initialization domains.
- Introduce new module subtrees where a domain has **two or more files**.
- Keep functional behavior identical (no logic changes).
- Update all module wiring, imports, and references.

---

## 1) New Module Layout (Target)

```
src/
  task/
    orchestrate.rs          # Thin router only
    ui/
      mod.rs                # UI public API
      state.rs              # UiState + UI_STATE
      render.rs             # rendering helpers
      menu.rs               # menu mapping + navigation helpers
    initialization/
      mod.rs                # public API
      calibration.rs        # flash/IMU calibration handling
      status.rs             # calibration status display (optional split)
    behavior/
      mod.rs
      battery.rs
      input.rs
      inactivity.rs
      obstacle.rs
      operation_mode.rs
      motion.rs
      sensor_events.rs
    drive/
      ...                   # existing (keep, possibly expand)
    sensors/
      mod.rs
      imu.rs
      encoders.rs
      ultrasonic.rs
      ir_obstacle.rs
    io/
      mod.rs
      display.rs
      flash_storage.rs
      port_expander.rs
    control/
      mod.rs
      rc_control.rs
      rotary_encoder.rs
      track_inactivity.rs
    indicators/
      mod.rs
      rgb_led_indicate.rs
    testing/
      mod.rs
      testing.rs
```

### Why this structure
- Every domain with 2+ files gets a subtree.
- UI and initialization are now cohesive, testable, and easier to maintain.
- Runtime behavior/event handling is isolated under `task/behavior`.
- Sensors and IO are grouped by concern.
- Input control tasks are grouped together.
- Indicators and testing are clearer as their own domains.

---

## 2) Detailed Orchestrator Decomposition

### 2.1 Move UI ownership
Move the following from `task/orchestrate.rs` into `task/ui/*`:

**State + UI flow**
- `UiState`
- `UI_STATE`
- `ui_initialized`
- `handle_ui_back`
- `show_main_menu`
- `handle_rotary_turned`
- `handle_rotary_button_pressed`
- `handle_rotary_button_hold_start`
- `handle_rotary_button_hold_end`
- `handle_testing_completed`

**Rendering and helpers**
- `render_current_ui`
- `render_test_running`
- `render_calibrating`
- `show_line`
- `build_system_info_data`
- `menu_selection_from_index`
- `calibration_selection_from_index`
- `calibration_label`
- `next_menu_index`
- `max_system_info_scroll`

#### Suggested UI module placement
- `task/ui/state.rs`: `UiState`, `UI_STATE`
- `task/ui/menu.rs`: menu index/selection helpers + scrolling helpers
- `task/ui/render.rs`: render helpers + `build_system_info_data`
- `task/ui/mod.rs`: public entrypoints: `handle_*` + `show_main_menu`

#### Orchestrator after UI move
`handle_event` calls `ui::handle_*` functions only.

---

### 2.2 Move initialization/calibration ownership
Move the following from `task/orchestrate.rs` into `task/initialization/*`:

**Initialization flow**
- `handle_initialize`
- `check_initialization_complete`

**Calibration handling**
- `handle_calibration_data_loaded`
- `handle_imu_calibration_flags_loaded`
- `handle_calibration_status`

#### Suggested initialization module placement
- `task/initialization/mod.rs`: public API
- `task/initialization/calibration.rs`: flash + IMU flags handling
- `task/initialization/status.rs` (optional): only if calibration display logic grows

#### Orchestrator after initialization move
`handle_event` calls `initialization::handle_*`.

---

### 2.3 Move remaining handlers into behavior domains
Move the remaining event handlers out of `task/orchestrate.rs` into `task/behavior/*`, so `orchestrate.rs` only dispatches.

**Behavior modules and ownership**
- `task/behavior/battery.rs`: `handle_battery_measured`
- `task/behavior/operation_mode.rs`: `handle_operation_mode_set`
- `task/behavior/obstacle.rs`: `handle_obstacle_detected`, `handle_obstacle_avoidance_attempted`
- `task/behavior/input.rs`: `handle_button_pressed`, `handle_button_hold_start`, `handle_button_hold_end`
- `task/behavior/inactivity.rs`: `handle_inactivity_timeout`
- `task/behavior/sensor_events.rs`: `handle_encoder_measurement`, `handle_ultrasonic_sweep_reading`, `handle_imu_measurement`
- `task/behavior/motion.rs`: `handle_rotation_completed`, `handle_start_stop_motion_data`, `handle_start_stop_ultrasonic_sweep`

**Orchestrator after behavior move**
`handle_event` calls `behavior::handle_*` functions only.

---

## 3) Task Tree Restructure Plan

### 3.0 Behavior subtree
Create `task/behavior/` to house the non-UI, non-initialization event handlers:

- `battery.rs` (battery + LED indicator updates)
- `operation_mode.rs`
- `obstacle.rs`
- `input.rs` (RC button press/hold actions)
- `inactivity.rs`
- `sensor_events.rs` (forwarding sensor readings to other tasks)
- `motion.rs` (rotation + start/stop control)

Add `task/behavior/mod.rs` with re-exports.

### 3.1 Sensors subtree
If `task/` contains multiple sensor tasks, move into `task/sensors/`:

- `task/imu_read.rs` → `task/sensors/imu.rs`
- `task/encoder_read.rs` → `task/sensors/encoders.rs`
- `task/sweep_ultrasonic.rs` → `task/sensors/ultrasonic.rs`
- `task/ir_obstacle_detect.rs` → `task/sensors/ir_obstacle.rs`

Update `task/sensors/mod.rs`:
- `pub mod imu;`
- `pub mod encoders;`
- `pub mod ultrasonic;`
- `pub mod ir_obstacle;`

Update references in `main.rs` and `orchestrate.rs`.

---

### 3.2 IO subtree
Move IO-related tasks into `task/io/`:

- `task/display.rs` → `task/io/display.rs`
- `task/flash_storage.rs` → `task/io/flash_storage.rs`
- `task/port_expander.rs` → `task/io/port_expander.rs`

Add `task/io/mod.rs` re-exports.

---

### 3.3 Control subtree
Move control input tasks into `task/control/`:

- `task/rc_control.rs` → `task/control/rc_control.rs`
- `task/rotary_encoder.rs` → `task/control/rotary_encoder.rs`
- `task/track_inactivity.rs` → `task/control/track_inactivity.rs`

Add `task/control/mod.rs` re-exports.

---

### 3.4 Indicators subtree
Move LEDs into `task/indicators/`:

- `task/rgb_led_indicate.rs` → `task/indicators/rgb_led_indicate.rs`

---

### 3.5 Testing subtree
Move testing into `task/testing/`:

- `task/testing.rs` → `task/testing/testing.rs`
- `task/testing/mod.rs` for exports

---

### 3.6 Keep drive subtree
`task/drive/` already exists. Expand it if more drive-specific helpers appear.

---

## 4) Update `task/mod.rs`

Replace the flat exports with grouped module exports:

```
pub mod orchestrate;
pub mod ui;
pub mod initialization;
pub mod behavior;
pub mod drive;
pub mod sensors;
pub mod io;
pub mod control;
pub mod indicators;
pub mod testing;
```

---

## 5) Update Imports and References

### 5.1 `main.rs`
Update task paths:
- `task::display` → `task::io::display`
- `task::imu_read` → `task::sensors::imu`
- `task::encoder_read` → `task::sensors::encoders`
- `task::sweep_ultrasonic` → `task::sensors::ultrasonic`
- `task::ir_obstacle_detect` → `task::sensors::ir_obstacle`
- `task::flash_storage` → `task::io::flash_storage`
- `task::port_expander` → `task::io::port_expander`
- `task::rc_control` → `task::control::rc_control`
- `task::rotary_encoder` → `task::control::rotary_encoder`
- `task::track_inactivity` → `task::control::track_inactivity`
- `task::rgb_led_indicate` → `task::indicators::rgb_led_indicate`
- `task::testing` → `task::testing::testing`

### 5.2 `system/event.rs`
Adjust imports for types moved:
- `task::encoder_read::EncoderMeasurement` → `task::sensors::encoders::EncoderMeasurement`
- `task::imu_read::ImuMeasurement` → `task::sensors::imu::ImuMeasurement`
- `task::flash_storage` → `task::io::flash_storage`

### 5.3 `orchestrate.rs` (after split)
Import from `task::ui`, `task::initialization`, and `task::behavior` as required.

---

## 6) Step-by-Step Execution Plan (No Code Changes Yet)

### Phase 1: UI module split
1. Create `task/ui/` subtree (`mod.rs`, `state.rs`, `menu.rs`, `render.rs`).
2. Move UI-related code into proper files.
3. Update references in `orchestrate.rs`.
4. Ensure module exports in `task/mod.rs`.

### Phase 2: Initialization split
1. Create `task/initialization/` subtree.
2. Move initialization + calibration logic.
3. Update `orchestrate.rs` calls.
4. Update imports.

### Phase 3: Behavior handler split
1. Create `task/behavior/` subtree and move remaining handlers into the files listed in Section 3.0.
2. Update `orchestrate.rs` to dispatch into `behavior::*`.

### Phase 4: Task tree restructure
1. Create `task/sensors/`, `task/io/`, `task/control/`, `task/indicators/`, `task/testing/`.
2. Move corresponding files.
3. Update `task/mod.rs`.
4. Update `main.rs`, `system/event.rs`, and any other references.

### Phase 5: Build + check
1. Ensure module paths resolve.
2. Run compilation to verify no missing paths or imports.

---

## 7) Notes / Guidelines
- Keep behavior unchanged; only move code and update paths.
- Ensure all `pub` functions used by orchestrator remain exported from the new module roots.
- Favor `mod.rs` as the public API surface for each subtree.
- Keep `orchestrate.rs` thin and readable.

---

## 8) Optional Future Enhancements
- Add `task/ui/actions.rs` if UI flow expands further.
- Add `task/initialization/state.rs` if initialization becomes stateful.
- Add `system/ui_state.rs` if you want UI state under `system/` rather than `task/`.

---

If you want, I can next produce a checklist with concrete `mv` operations and path changes, or proceed with the actual refactor.