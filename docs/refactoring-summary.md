# Encoder Refactoring - Quick Reference

**STATUS: Phases 1-4 COMPLETED ✅**

## What We're Doing

1. Moving encoder reading from `motor_driver` task to a new dedicated `encoder_read` task
2. Moving motor calibration from `motor_driver` task to `drive` task (or dedicated calibration task)

## Why

1. **Consistency**: Match the IMU architecture (dedicated sensor task)
2. **Separation**: Motor driver should only do actuation, not sensing or procedures
3. **Proper Layering**: Calibration is a control procedure, not a low-level motor function
4. **Flexibility**: Multiple tasks need encoder data (drive, odometry, diagnostics)
5. **Delta Tracking**: Clean reset mechanism for measuring encoder deltas during maneuvers

## Current vs. Proposed

### Current (Problematic)
```
motor_driver
├── Owns encoders         ← Mixed concerns!
├── Owns motor PWM
├── Does calibration      ← Wrong layer!
└── ??? How does drive task get encoder data?
```

### Proposed (Clean)
```
encoder_read          motor_driver          drive
├── Owns encoders     ├── Owns motor PWM    ├── Calibration procedure
├── Sends events  →   ├── Loads calib       ├── Rotation control
├── Reset control     └── Pure actuation    ├── Straight driving
└── 20-50Hz                                 └── Coordinates sensors + motors
        ↓                                           ↑
        └───────────────────────────────────────────┘
                    Encoder events
```

## Key Changes

### New Files
- `src/task/encoder_read.rs` - New encoder task with reset and auto-reset features

### Modified Files
- `src/task/motor_driver.rs` - Remove encoders, remove calibration procedure, just actuation
- `src/task/drive.rs` - Add calibration procedure, receive encoder events
- `src/system/event.rs` - Add `EncoderMeasurementTaken` event
- `src/main.rs` - Spawn encoder_read task, pass encoders to it (not motor_driver)

### New Event
```rust
Events::EncoderMeasurementTaken(EncoderMeasurement {
    left_front: u16,
    left_rear: u16,
    right_front: u16,
    right_rear: u16,
    timestamp_ms: u32,  // Always included
})
```

### New Commands
```rust
EncoderCommand::Start { interval_ms }        // Start sampling
EncoderCommand::Stop                         // Stop sampling (power save)
EncoderCommand::Reset                        // Reset counters to zero
EncoderCommand::AutoResetOnMotorStop(bool)   // Auto-reset when motors stop
```

### Calibration Moves
```rust
// OLD: In motor_driver
MotorCommand::RunCalibration  // Handled in motor_driver.rs

// NEW: In drive task
DriveCommand::RunMotorCalibration  // Handled in drive.rs
```

## Delta Tracking Solution

### The Problem
- Need to measure "motor turned 500 pulses during this 2-second test"
- Continuous counting requires storing start values and calculating deltas
- Multiple consumers need independent tracking

### The Solution: Hybrid Approach
```rust
// 1. Reset before maneuver
encoder_read::send_command(EncoderCommand::Reset).await;

// 2. Do the maneuver
motor_driver::send_command(SetSpeed { speed: 50 }).await;
Timer::after(2_seconds).await;

// 3. Read absolute counts (which ARE the delta since reset)
let measurement = wait_for_encoder_event().await;
let pulses = measurement.left_front;  // This IS the delta!

// OR: Use auto-reset for stop-and-go driving
encoder_read::send_command(EncoderCommand::AutoResetOnMotorStop(true)).await;
// Now encoders reset automatically each time motors stop
```

## Benefits at a Glance

✅ Consistent architecture (matches IMU pattern)  
✅ Clean separation of sensing vs. control vs. actuation  
✅ Calibration is a procedure (where it belongs)  
✅ Motor driver simplified to pure actuation  
✅ Multiple consumers can use encoder data  
✅ Power management (start/stop sampling)  
✅ Flexible sampling rates  
✅ Clean delta tracking (reset + read = delta)  
✅ Auto-reset for convenient stop-and-go driving  
✅ Timestamps always included  
✅ Future-ready for odometry, diagnostics, etc.

## Migration Phases

1. **✅ DONE - Create** encoder_read task (non-breaking)
2. **✅ DONE - Move** calibration to drive task
3. **✅ DONE - Update** motor_driver (remove encoders, remove calibration)
4. **✅ DONE - Enable** encoder task and test
5. **TODO - Implement** drive features with encoder feedback (simple straight driving command)
6. **TODO - Cleanup** and documentation

## Calibration Example

### Before (in motor_driver)
```rust
// Awkward: calibration buried in motor_driver
async fn motor_driver(pwm, encoders) {
    loop {
        match command {
            RunCalibration => {
                // Complex procedure code here
                run_motor_calibration(&pwm, &encoders).await;
            }
            _ => process_command(...)
        }
    }
}
```

### After (in drive task)
```rust
// Clean: calibration is a control procedure
async fn drive() {
    loop {
        match command {
            RunMotorCalibration => {
                // Coordinate: encoders + motors + flash
                encoder_read::send_command(Reset).await;
                motor_driver::send_command(SetSpeed { ... }).await;
                let measurement = wait_for_encoder_event().await;
                let factor = calculate_factor(measurement);
                flash_storage::save(factor).await;
                motor_driver::send_command(LoadCalibration(factor)).await;
            }
            _ => // ... other driving
        }
    }
}
```

## When to Do This

**Ready when**: You're implementing rotation commands or other encoder-dependent features.

**Estimated effort**: 6-8 hours total across 6 phases

**Risk level**: Low (phased approach with validation at each step)

## Key Decisions Made

1. **Hybrid encoder tracking**: Manual reset + auto-reset option (not just continuous counting)
2. **Calibration location**: Drive task (not motor_driver)
3. **Timestamps**: Always included in encoder measurements
4. **Sampling control**: Start/stop commands for power management

## See Also

- [Full Refactoring Document](./refactoring-encoder-task.md) - Detailed architecture and implementation
- [Architecture Comparison](./architecture-comparison.md) - Visual diagrams and decision analysis
- `src/task/imu_read.rs` - Reference implementation for sensor task pattern
- `src/task/drive.rs` - Will be the main consumer of encoder events and home for calibration