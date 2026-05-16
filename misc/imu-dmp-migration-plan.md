# IMU DMP Migration Plan (Behavior-Preserving)

## Goal
Migrate IMU pipeline from host-side Madgwick (`ahrs`) fusion to `icm20948-rs` DMP-based fusion while preserving existing runtime behavior and decision paths as much as viable:
- Keep mode decision surface (`Axis6` vs `Axis9`)
- Keep default behavior (`Axis6`)
- Keep fallback behavior (degrade to 6-axis if 9-axis path is unavailable)
- Keep event/state interfaces stable for downstream modules

## Current status
- [x] Create and commit migration plan document
- [x] Investigate current IMU + calibration + flash code in repository
- [x] Investigate `icm20948-rs` DMP APIs and calibration injection points
- [x] Implement new `src/task/sensors/imu.rs` (DMP polling, mode switching)
- [x] Update `Cargo.toml` (add `dmp` feature, remove `ahrs`)
- [x] Update flash storage (`ImuCalibration` — remove gyro/accel fields; `ImuCalibrationFlags` — keep `mag` only)
- [x] Update calibration state (`CalibrationState` — remove `accel`/`gyro` status fields)
- [x] Update initialization handler
- [x] Remove gyro/accel calibration procedures from `drive/calibration/imu.rs`
- [x] Trim `ImuCalibrationKind` and `CalibrationSelection` enums
- [x] Update UI menu, handler, render, screens
- [x] Run diagnostics/build checks and fix regressions — **clean: 0 errors, 0 warnings**
- [x] Final review complete

## Work log
### Step 1: Create plan file
Created this plan file as requested.

### Step 2: Investigation complete
- Confirmed `DEFAULT_FUSION_MODE = Axis6`. Axis9 has fallback to Axis6 if mag unavailable.
- DMP API confirmed: `try_new` → `init` → `dmp_init(delay)` → `dmp_init_magnetometer(delay)` → `dmp_configure` → `dmp_enable(true)` → `reset_fifo`. No INT pin required for polling.
- DMP calibration: gyro/accel handled internally by DMP engines. Mag requires host-side hard/soft-iron + motor interference correction (injected into `LATEST_CALIBRATED_MAG` from raw_mag packet field). No bias injection API needed.
- Flash schema: Remove `gyro_x/y/z_bias` and `accel_x/y/z_bias` from `ImuCalibration`. Remove `gyro`/`accel` from `ImuCalibrationFlags`. New `ImuCalibration` = 24 f32 (96 bytes, mag fields only).
- UI: `SystemInfoData` in `screens.rs` and `render.rs` also carry accel/gyro status — those are removed too.
- Test modes (`imu_6axis`, `imu_9axis`): no changes needed — all consumed public APIs are preserved.
- INT pin: not wired. Using timer-based FIFO polling at 10 ms interval (DMP at 100 Hz).

### Step 3: Implementation in progress
- Writing new `imu.rs`, `Cargo.toml`, and spawning agents for remaining files.

## Design constraints
1. No backward compatibility required for flash schema.
2. Prefer direct implementation over extra abstraction layers unless needed for compile-time sanity.
3. Preserve existing decision points and behavior where practical.
4. Keep code comprehensible and close to current project style.

## Open questions — RESOLVED
1. DMP calibration: DMP handles gyro/accel internally via calibration engines (enabled automatically in six_axis/nine_axis modes). Mag requires host software correction. No flash injection of gyro/accel biases needed.
2. Mode switches: handled by `dmp_enable(false)` → `dmp_configure(new_config)` → `dmp_enable(true)` → `reset_fifo()`. Firmware (`dmp_init`) is NOT reloaded on mode switches — only on cold start.
3. Meaningful calibration artifacts: only mag hard/soft-iron + motor interference. Gyro/accel bias fields removed from flash schema.

## Final file touch list
- `Cargo.toml`: add `dmp` feature to `icm20948-rs`; remove `ahrs` dep
- `src/task/sensors/imu.rs`: full rewrite (DMP polling loop, mode switching)
- `src/task/io/flash_storage.rs`: strip gyro/accel fields from `ImuCalibration`; simplify `ImuCalibrationFlags` to `mag` only
- `src/system/state/calibration.rs`: remove `accel_calibration_status`/`gyro_calibration_status`
- `src/task/initialization/mod.rs`: update flag handler; remove accel/gyro status writes
- `src/task/drive/calibration/imu.rs`: remove gyro/accel calibration procedures
- `src/task/drive/types.rs`: remove `Gyro`/`Accel` from `ImuCalibrationKind`
- `src/system/state.rs`: remove `Accel`/`Gyro` from `CalibrationSelection`
- `src/task/ui/menu.rs`: trim calibration menu to Motor + Mag
- `src/task/ui/mod.rs`: remove Accel/Gyro arms from calibration press handler
- `src/task/ui/render.rs`: remove accel/gyro fields from `SystemInfoData` build
- `src/task/ui/screens.rs`: remove accel/gyro from `SystemInfoData` struct + display
- `src/task/testmode/imu_6axis.rs`: NO changes needed
- `src/task/testmode/imu_9axis.rs`: NO changes needed

## Risk register
- DMP mode switch latency/reinit causing temporary data gaps
- Magnetometer quality gating differences vs host-side checks
- Downstream heading expectations if quaternion source semantics differ

## Progress updates
- Initial document created.
- Investigation complete: DMP API, calibration model, all touched files identified.
- Implementation started: `Cargo.toml` updated, new `imu.rs` written, agents spawned for remaining files.
