//! Core drive loop state and intent handling.
//!
//! This module keeps the main drive loop state (`DriveLoop`) and the currently
//! executing intent (`ActiveIntent`). It also contains the intent handling logic
//! used by the drive task.

use core::sync::atomic::Ordering;

use defmt::info;
use embassy_time::{Duration, Instant, Timer};
use micromath::F32Ext;

use super::control::RotationState;
use crate::{
    system::state::SYSTEM_STATE,
    task::{
        drive::{
            api::{self, DriveCommandEnvelope},
            distance::DistanceDriveState,
            lifecycle::{self, start_encoder_sampling},
            types::{self, CompletionStatus, DriveAction, DriveCommand, DriveCompletion},
        },
        motor_driver::{self, MotorCommand},
    },
};

/// Drift compensation state (encoder-based straight-line correction)
#[derive(Clone, Copy)]
pub(super) struct DriftCompensationState {
    /// Whether drift compensation is currently active
    pub(super) enabled: bool,
    /// Base speeds as commanded (before compensation)
    pub(super) base_left: i8,
    /// Base speeds as commanded (before compensation)
    pub(super) base_right: i8,
    /// Currently applied left speed (after compensation)
    pub(super) adjusted_left: i8,
    /// Currently applied right speed (after compensation)
    pub(super) adjusted_right: i8,
    /// Timestamp of the last applied compensation update (ms)
    pub(super) last_applied_ms: u64,
    /// Timestamp of the last processed encoder measurement (ms)
    pub(super) last_encoder_timestamp_ms: u64,
    /// Previous encoder measurement for computing per-sample deltas.
    /// Encoder hardware counters are cumulative since last reset, so we must
    /// subtract the previous reading to get pulses for the current window.
    pub(super) last_encoder_measurement: Option<crate::task::sensors::encoders::EncoderMeasurement>,
}

impl DriftCompensationState {
    /// Create a new `DriftCompensationState` with default values (disabled, zero speeds, no timestamps)
    pub(super) const fn new() -> Self {
        Self {
            enabled: false,
            base_left: 0,
            base_right: 0,
            adjusted_left: 0,
            adjusted_right: 0,
            last_applied_ms: 0,
            last_encoder_timestamp_ms: 0,
            last_encoder_measurement: None,
        }
    }
}

/// Active intent being executed by the drive task.
pub(super) enum ActiveIntent {
    /// Active in-place rotation intent with state and optional completion sender.
    RotateExact {
        /// Rotation controller state for the in-progress turn.
        state: RotationState,
        /// Optional completion sender for the issued command.
        completion: Option<types::CompletionSender>,
        /// Start time (ms) used for duration telemetry.
        started_at_ms: u64,
    },
    /// Active distance drive intent with state and optional completion sender.
    DriveDistance {
        /// Distance controller state for the in-progress drive.
        state: DistanceDriveState,
        /// Optional completion sender for the issued command.
        completion: Option<types::CompletionSender>,
    },
}

/// State for the main drive control loop
pub(super) struct DriveLoop {
    /// Whether the system is currently in standby mode (motors disabled)
    pub(super) standby_enabled: bool,
    /// Active intent, if any
    pub(super) active_intent: Option<ActiveIntent>,
    /// State for encoder-based drift compensation (straight-line correction)
    pub(super) drift: DriftCompensationState,
}

impl DriveLoop {
    /// Create a new `DriveLoop` with default state (standby enabled, no active intent, drift compensation disabled)
    pub(super) const fn new() -> Self {
        Self {
            standby_enabled: true,
            active_intent: None,
            drift: DriftCompensationState::new(),
        }
    }

    /// Handle a dequeued command envelope, honoring epoch cancellation.
    pub(super) async fn handle_envelope(&mut self, envelope: DriveCommandEnvelope) {
        let current_epoch = api::CURRENT_EPOCH.load(Ordering::Relaxed);
        if envelope.epoch != current_epoch {
            api::send_completion(
                envelope.completion,
                DriveCompletion {
                    status: CompletionStatus::Cancelled,
                    telemetry: types::CompletionTelemetry::None,
                },
            )
            .await;
            return;
        }

        let completion = envelope.completion;
        match envelope.command {
            DriveCommand::Drive(action) => {
                self.handle_drive_action(action, completion).await;
            }
            DriveCommand::RunMotorCalibration => {
                info!("Starting motor calibration procedure");
                crate::task::drive::calibration::run_motor_calibration().await;
                info!("Motor calibration procedure completed");
                api::send_completion(
                    completion,
                    DriveCompletion {
                        status: CompletionStatus::Success,
                        telemetry: types::CompletionTelemetry::None,
                    },
                )
                .await;
            }
            DriveCommand::RunImuCalibration(kind) => {
                info!("Starting IMU calibration procedure");
                crate::task::drive::calibration::run_imu_calibration(kind).await;
                info!("IMU calibration procedure completed");
                api::send_completion(
                    completion,
                    DriveCompletion {
                        status: CompletionStatus::Success,
                        telemetry: types::CompletionTelemetry::None,
                    },
                )
                .await;
            }
        }
    }

    /// Handle a `DriveAction` command by executing the corresponding motor commands and state updates.
    pub(super) async fn handle_drive_action(
        &mut self,
        action: DriveAction,
        completion: Option<types::CompletionSender>,
    ) {
        // Wake from standby if movement requested
        if self.standby_enabled {
            match &action {
                DriveAction::SetSpeed { .. } | DriveAction::RotateExact { .. } | DriveAction::DriveDistance { .. } => {
                    motor_driver::send_motor_command(MotorCommand::SetAllDriversEnable { enabled: true }).await;
                    self.standby_enabled = false;
                    Timer::after(Duration::from_millis(100)).await;
                }
                _ => {}
            }
        }

        match action {
            DriveAction::SetSpeed { left, right } => {
                self.handle_set_speed(left, right).await;
                api::send_completion(
                    completion,
                    DriveCompletion {
                        status: CompletionStatus::Success,
                        telemetry: types::CompletionTelemetry::None,
                    },
                )
                .await;
            }
            DriveAction::RotateExact {
                degrees,
                direction,
                motion,
            } => {
                self.handle_rotate_exact(degrees, direction, motion, completion).await;
            }
            DriveAction::DriveDistance { kind, direction, speed } => {
                self.handle_drive_distance(kind, direction, speed, completion).await;
            }
            DriveAction::Coast => {
                self.handle_coast().await;
                api::send_completion(
                    completion,
                    DriveCompletion {
                        status: CompletionStatus::Success,
                        telemetry: types::CompletionTelemetry::None,
                    },
                )
                .await;
            }
            DriveAction::Brake => {
                self.handle_brake().await;
                api::send_completion(
                    completion,
                    DriveCompletion {
                        status: CompletionStatus::Success,
                        telemetry: types::CompletionTelemetry::None,
                    },
                )
                .await;
            }
            DriveAction::Standby => {
                self.handle_standby().await;
                api::send_completion(
                    completion,
                    DriveCompletion {
                        status: CompletionStatus::Success,
                        telemetry: types::CompletionTelemetry::None,
                    },
                )
                .await;
            }
        }
    }

    /// Handle a `SetSpeed` command by applying drift compensation logic and sending motor commands.
    async fn handle_set_speed(&mut self, left: i8, right: i8) {
        // Clamp speeds to valid range
        let left_clamped = left.clamp(-100, 100);
        let right_clamped = right.clamp(-100, 100);

        // Detect straight-line driving (speeds equal or very close)
        // Enable drift compensation only in that mode.
        if (left_clamped - right_clamped).abs() <= 2 && left_clamped != 0 {
            self.drift.enabled = true;
            self.drift.base_left = left_clamped;
            self.drift.base_right = right_clamped;
            self.drift.adjusted_left = left_clamped;
            self.drift.adjusted_right = right_clamped;

            // Start encoder sampling at configured interval and reset counters.
            self.drift.last_encoder_measurement = None;
            start_encoder_sampling(types::DRIFT_COMPENSATION_INTERVAL_MS, false).await;
        } else {
            // Turning/differential or stop: disable compensation and stop encoder sampling
            self.drift.enabled = false;
            lifecycle::stop_encoder_sampling().await;
        }

        // Apply command (initially base, then adjusted as feedback arrives)
        motor_driver::send_motor_command(MotorCommand::SetTracks {
            left_speed: self.drift.adjusted_left,
            right_speed: self.drift.adjusted_right,
        })
        .await;

        // Update system state directly
        let mut state = SYSTEM_STATE.lock().await;
        state.left_track_speed = self.drift.adjusted_left;
        state.right_track_speed = self.drift.adjusted_right;
    }

    /// Handle a `RotateExact` command by initializing rotation state and starting IMU-based control.
    async fn handle_rotate_exact(
        &mut self,
        degrees: f32,
        direction: types::RotationDirection,
        motion: types::RotationMotion,
        completion: Option<types::CompletionSender>,
    ) {
        lifecycle::start_rotation_imu().await;

        let rotation_state = RotationState::new(degrees, direction, motion);
        let started_at_ms = Instant::now().as_millis();

        // Apply initial motor speeds for rotation
        let (left_speed, right_speed) = rotation_state.calculate_motor_speeds();
        motor_driver::send_motor_command(MotorCommand::SetTracks {
            left_speed,
            right_speed,
        })
        .await;

        // Update system state
        let mut state = SYSTEM_STATE.lock().await;
        state.left_track_speed = left_speed;
        state.right_track_speed = right_speed;
        drop(state);

        self.active_intent = Some(ActiveIntent::RotateExact {
            state: rotation_state,
            completion,
            started_at_ms,
        });
    }

    /// Handle a `DriveDistance` command by initializing encoder-based control.
    #[allow(clippy::cast_possible_wrap, clippy::cast_possible_truncation)]
    async fn handle_drive_distance(
        &mut self,
        kind: types::DriveDistanceKind,
        direction: types::DriveDirection,
        speed: u8,
        completion: Option<types::CompletionSender>,
    ) {
        // Disable drift loop; distance control applies compensation internally.
        self.drift.enabled = false;

        let (target_left_revs, target_right_revs, target_inner_revs) = match kind {
            types::DriveDistanceKind::Straight { distance_cm } => {
                let revolutions = distance_cm / types::SPROCKET_CIRCUMFERENCE_CM;
                (revolutions, revolutions, revolutions)
            }
            types::DriveDistanceKind::CurveArc {
                radius_cm,
                arc_length_cm,
                direction,
            } => {
                let half_width = types::TRACK_WIDTH_CM * 0.5;
                let inner_radius = (radius_cm - half_width).max(0.0);
                let outer_radius = radius_cm + half_width;
                let (left_radius, right_radius, inner_left) = match direction {
                    types::TurnDirection::Left => (inner_radius, outer_radius, true),
                    types::TurnDirection::Right => (outer_radius, inner_radius, false),
                };
                let safe_radius = radius_cm.max(0.001);
                let left_arc_cm = arc_length_cm * (left_radius / safe_radius);
                let right_arc_cm = arc_length_cm * (right_radius / safe_radius);
                let left_revs = left_arc_cm / types::SPROCKET_CIRCUMFERENCE_CM;
                let right_revs = right_arc_cm / types::SPROCKET_CIRCUMFERENCE_CM;
                let inner_revs = if inner_left { left_revs } else { right_revs };
                (left_revs, right_revs, inner_revs)
            }
        };

        if target_inner_revs <= types::DISTANCE_TOLERANCE_REVS {
            api::send_completion(
                completion,
                DriveCompletion {
                    status: CompletionStatus::Success,
                    telemetry: types::CompletionTelemetry::DriveDistance {
                        achieved_left_revs: 0.0,
                        achieved_right_revs: 0.0,
                        target_left_revs,
                        target_right_revs,
                        duration_ms: 0,
                    },
                },
            )
            .await;
            return;
        }

        lifecycle::start_curve_imu(&kind).await;
        start_encoder_sampling(types::DISTANCE_CONTROL_INTERVAL_MS, true).await;

        let mut state = DistanceDriveState::new(kind, direction, speed);

        let base_speed = speed.min(types::DISTANCE_MAX_SPEED);
        let signed_base = match direction {
            types::DriveDirection::Forward => base_speed as i8,
            types::DriveDirection::Backward => -(base_speed as i8),
        };

        let left_speed = (f32::from(signed_base) * state.left_ratio).round() as i8;
        let right_speed = (f32::from(signed_base) * state.right_ratio).round() as i8;

        state.last_left_speed = left_speed;
        state.last_right_speed = right_speed;

        motor_driver::send_motor_command(MotorCommand::SetTracks {
            left_speed,
            right_speed,
        })
        .await;

        let mut sys = SYSTEM_STATE.lock().await;
        sys.left_track_speed = left_speed;
        sys.right_track_speed = right_speed;
        drop(sys);

        self.active_intent = Some(ActiveIntent::DriveDistance { state, completion });
    }

    /// Handle a `Coast` command by disabling compensation, stopping encoder sampling, and sending coast command.
    async fn handle_coast(&mut self) {
        info!("coast");

        // Disable compensation on stop/coast and stop encoder sampling
        self.drift.enabled = false;
        lifecycle::stop_encoder_sampling().await;

        motor_driver::send_motor_command(MotorCommand::CoastAll).await;

        // Update system state
        let mut state = SYSTEM_STATE.lock().await;
        state.left_track_speed = 0;
        state.right_track_speed = 0;
    }

    /// Handle a `Brake` command by disabling compensation, stopping encoder sampling, and sending brake command.
    async fn handle_brake(&mut self) {
        info!("brake");

        // Disable compensation on brake and stop encoder sampling
        self.drift.enabled = false;
        lifecycle::stop_encoder_sampling().await;

        motor_driver::send_motor_command(MotorCommand::BrakeAll).await;

        // Update system state
        let mut state = SYSTEM_STATE.lock().await;
        state.left_track_speed = 0;
        state.right_track_speed = 0;
    }

    /// Handle a `Standby` command by disabling compensation, stopping encoder sampling, and disabling motors.
    async fn handle_standby(&mut self) {
        // Disable compensation in standby and stop encoder sampling
        self.drift.enabled = false;
        lifecycle::stop_encoder_sampling().await;

        if !self.standby_enabled {
            motor_driver::send_motor_command(MotorCommand::BrakeAll).await;
            Timer::after(Duration::from_millis(100)).await;
            motor_driver::send_motor_command(MotorCommand::CoastAll).await;
            Timer::after(Duration::from_millis(100)).await;
            motor_driver::send_motor_command(MotorCommand::SetAllDriversEnable { enabled: false }).await;
            self.standby_enabled = true;

            // Update system state
            let mut state = SYSTEM_STATE.lock().await;
            state.left_track_speed = 0;
            state.right_track_speed = 0;
        }
    }

    /// Handle an interrupt by applying motor action, cancelling the active intent only,
    /// and bumping the epoch to invalidate queued commands.
    /// Only the active intent’s completion is resolved here; queued commands are cancelled
    /// when dequeued due to epoch mismatch.
    pub(super) async fn handle_interrupt(&mut self, kind: types::InterruptKind) {
        match kind {
            types::InterruptKind::EmergencyBrake => {
                motor_driver::send_motor_command(MotorCommand::BrakeAll).await;
            }
            types::InterruptKind::Stop | types::InterruptKind::CancelCurrent => {
                motor_driver::send_motor_command(MotorCommand::CoastAll).await;
            }
        }

        // Disable compensation and stop encoder sampling
        self.drift.enabled = false;
        lifecycle::stop_encoder_sampling().await;

        // Cancel active intent
        if let Some(intent) = self.active_intent.take() {
            match intent {
                ActiveIntent::RotateExact {
                    state,
                    completion,
                    started_at_ms,
                } => {
                    let accumulated = state.accumulated_angle.abs();
                    let target = state.target_angle.abs();
                    let last_yaw_deg = state.last_yaw.unwrap_or(0.0);
                    let duration_ms = Instant::now().as_millis() - started_at_ms;

                    lifecycle::stop_rotation_imu().await;
                    api::send_completion(
                        completion,
                        DriveCompletion {
                            status: CompletionStatus::Cancelled,
                            telemetry: types::CompletionTelemetry::RotateExact {
                                final_yaw_deg: last_yaw_deg,
                                angle_error_deg: accumulated - target,
                                duration_ms,
                            },
                        },
                    )
                    .await;
                }
                ActiveIntent::DriveDistance { state, completion } => {
                    let duration_ms = Instant::now().as_millis() - state.started_at_ms;

                    lifecycle::stop_curve_imu(&state.kind).await;

                    api::send_completion(
                        completion,
                        DriveCompletion {
                            status: CompletionStatus::Cancelled,
                            telemetry: types::CompletionTelemetry::DriveDistance {
                                achieved_left_revs: state.accumulated_left_revs,
                                achieved_right_revs: state.accumulated_right_revs,
                                target_left_revs: state.target_left_revs,
                                target_right_revs: state.target_right_revs,
                                duration_ms,
                            },
                        },
                    )
                    .await;
                }
            }
        }

        // Invalidate queued commands
        api::CURRENT_EPOCH.fetch_add(1, Ordering::Relaxed);

        let mut state = SYSTEM_STATE.lock().await;
        state.left_track_speed = 0;
        state.right_track_speed = 0;
    }
}
