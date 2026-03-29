//! Drive command handlers for the main drive loop.
//!
//! This module adds all command-handling methods to [`DriveLoop`] as an `impl`
//! block. It is the *behaviour* counterpart to `state.rs`, which only holds
//! the data structures.
//!
//! # Responsibilities
//!
//! - Dispatch incoming [`DriveCommand`] envelopes (epoch-checked).
//! - Dispatch [`DriveAction`] variants to the appropriate handler.
//! - Start and stop sensor streams (via [`sensors::control`]) as each intent begins.
//! - Initialise [`ActiveIntent`] state and place it into [`DriveLoop`].
//! - Handle interrupts: apply the motor action, cancel the active intent, and
//!   bump the epoch to invalidate queued commands.
//!
//! # Relationship to other modules
//!
//! - [`state`]: defines the structs this `impl` block operates on.
//! - [`intent`]: polls the active intent each tick and calls back here for interrupts.
//! - [`rotation`] / [`distance`]: own the state machines; handlers only initialise them.
//! - [`sensors::control`]: all sensor start/stop calls are delegated here.

use core::sync::atomic::Ordering;

use defmt::{info, warn};
use embassy_time::{Duration, Instant, Timer};
use micromath::F32Ext;

use crate::{
    system::state::motion,
    task::{
        drive::{
            api::{self, DriveCommandEnvelope},
            brake_coast::BrakeCoastState,
            clear_imu_measurements,
            distance::DistanceDriveState,
            rotation::RotationState,
            sensors::control::{self as lifecycle, start_encoder_sampling},
            state::{ActiveIntent, DriveLoop},
            types::{self, CompletionStatus, DriveAction, DriveCommand, DriveCompletion},
        },
        motor_driver::{self, MotorCommand},
    },
};

impl DriveLoop {
    /// Handle a dequeued command envelope, honouring epoch cancellation.
    ///
    /// If the envelope's epoch is stale (an interrupt fired after this command
    /// was enqueued), the completion is resolved as `Cancelled` immediately and
    /// the command is discarded.
    pub(super) async fn handle_envelope(&mut self, envelope: DriveCommandEnvelope) {
        let current_epoch = api::CURRENT_EPOCH.load(Ordering::Relaxed);
        if envelope.epoch != current_epoch {
            api::send_completion(
                envelope.completion_requested,
                DriveCompletion {
                    status: CompletionStatus::Cancelled,
                    telemetry: types::CompletionTelemetry::None,
                },
            )
            .await;
            return;
        }

        let completion_requested = envelope.completion_requested;
        match envelope.command {
            DriveCommand::Drive(action) => {
                self.handle_drive_action(action, completion_requested).await;
            }
            DriveCommand::RunMotorCalibration => {
                info!("Starting motor calibration procedure");
                crate::task::drive::calibration::run_motor_calibration().await;
                info!("Motor calibration procedure completed");
                api::send_completion(
                    completion_requested,
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
                    completion_requested,
                    DriveCompletion {
                        status: CompletionStatus::Success,
                        telemetry: types::CompletionTelemetry::None,
                    },
                )
                .await;
            }
        }
    }

    /// Dispatch a [`DriveAction`] to the appropriate handler.
    ///
    /// Wakes the motor drivers from standby if a movement action is requested
    /// while the system is in standby mode.
    pub(super) async fn handle_drive_action(&mut self, action: DriveAction, completion_requested: bool) {
        // Wake from standby if a movement command arrives.
        if self.standby_enabled {
            match &action {
                DriveAction::Differential { .. }
                | DriveAction::RotateExact { .. }
                | DriveAction::DriveDistance { .. } => {
                    motor_driver::send_motor_command(MotorCommand::SetAllDriversEnable { enabled: true }).await;
                    self.standby_enabled = false;
                    Timer::after(Duration::from_millis(100)).await;
                }
                _ => {}
            }
        }

        match action {
            DriveAction::Differential { left, right } => {
                self.handle_set_speed(left, right).await;
                api::send_completion(
                    completion_requested,
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
                self.handle_rotate_exact(degrees, direction, motion, completion_requested)
                    .await;
            }
            DriveAction::DriveDistance { kind, direction, speed } => {
                self.handle_drive_distance(kind, direction, speed, completion_requested)
                    .await;
            }
            DriveAction::Coast => {
                self.handle_coast(completion_requested).await;
            }
            DriveAction::Brake => {
                self.handle_brake(completion_requested).await;
            }
            DriveAction::Idle { duration_ms } => {
                self.handle_idle(duration_ms, completion_requested).await;
            }
            DriveAction::Standby => {
                self.handle_standby().await;
                api::send_completion(
                    completion_requested,
                    DriveCompletion {
                        status: CompletionStatus::Success,
                        telemetry: types::CompletionTelemetry::None,
                    },
                )
                .await;
            }
        }
    }

    /// Handle a `Differential` command.
    ///
    /// Enables drift compensation when both tracks are commanded at equal (or
    /// near-equal) speed and at least one track is non-zero. Disables it for
    /// differential / turning commands.
    async fn handle_set_speed(&mut self, left: i8, right: i8) {
        let left_clamped = left.clamp(-100, 100);
        let right_clamped = right.clamp(-100, 100);

        // Enable drift compensation only for symmetric (straight-line) commands.
        if (left_clamped - right_clamped).abs() <= 2 && left_clamped != 0 {
            self.drift.enabled = true;
            self.drift.base_left = left_clamped;
            self.drift.base_right = right_clamped;
            self.drift.adjusted_left = left_clamped;
            self.drift.adjusted_right = right_clamped;
            self.drift.last_encoder_measurement = None;
            start_encoder_sampling(types::DRIFT_COMPENSATION_INTERVAL_MS, false).await;
        } else {
            // Differential / turning / stop: disable compensation.
            self.drift.enabled = false;
            lifecycle::stop_encoder_sampling().await;
        }

        motor_driver::send_motor_command(MotorCommand::SetTracks {
            left_speed: self.drift.adjusted_left,
            right_speed: self.drift.adjusted_right,
        })
        .await;

        motion::set_track_speeds(self.drift.adjusted_left, self.drift.adjusted_right).await;
    }

    /// Handle a `RotateExact` command.
    ///
    /// Starts IMU streaming, initialises [`RotationState`], applies the initial
    /// motor speeds, and stores the intent as the active intent so the poll loop
    /// can drive it to completion.
    async fn handle_rotate_exact(
        &mut self,
        degrees: f32,
        direction: types::RotationDirection,
        motion: types::RotationMotion,
        completion_requested: bool,
    ) {
        lifecycle::start_rotation_imu().await;
        clear_imu_measurements();

        motor_driver::send_motor_command(MotorCommand::SetTracks {
            left_speed: 0,
            right_speed: 0,
        })
        .await;
        motion::set_track_speeds(0, 0).await;

        let rotation_state = RotationState::new(degrees, direction, motion);
        let started_at_ms = Instant::now().as_millis();

        self.active_intent = Some(ActiveIntent::RotateExact {
            state: rotation_state,
            completion_requested,
            started_at_ms,
        });
    }

    /// Handle a `DriveDistance` command.
    ///
    /// Disables the drift compensation loop (distance control applies its own
    /// internal compensation), starts sensor streams, initialises
    /// [`DistanceDriveState`], applies initial motor speeds, and stores the intent.
    #[allow(clippy::cast_possible_wrap, clippy::cast_possible_truncation)]
    async fn handle_drive_distance(
        &mut self,
        kind: types::DriveDistanceKind,
        direction: types::DriveDirection,
        speed: u8,
        completion_requested: bool,
    ) {
        // Distance control manages its own per-track compensation.
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

        // Resolve trivially-short distances immediately without starting sensors.
        if target_inner_revs <= types::DISTANCE_TOLERANCE_REVS {
            api::send_completion(
                completion_requested,
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

        motion::set_track_speeds(left_speed, right_speed).await;

        self.active_intent = Some(ActiveIntent::DriveDistance {
            state,
            completion_requested,
        });
    }

    /// Handle a `Coast` command.
    ///
    /// Disables drift compensation, starts encoder sampling for settle detection,
    /// and sends a coast (freewheel) command to all motor drivers. Completion is
    /// reported only after encoders settle or the timeout expires.
    async fn handle_coast(&mut self, completion_requested: bool) {
        info!("coast");
        self.drift.enabled = false;
        start_encoder_sampling(types::BRAKE_COAST_SETTLE_INTERVAL_MS, true).await;
        motor_driver::send_motor_command(MotorCommand::CoastAll).await;

        motion::set_track_speeds(0, 0).await;

        self.active_intent = Some(ActiveIntent::BrakeCoast {
            state: BrakeCoastState::new(),
            completion_requested,
        });
    }

    /// Handle a `Brake` command.
    ///
    /// Disables drift compensation, starts encoder sampling for settle detection,
    /// and applies active electrical braking to all motor drivers. Completion is
    /// reported only after encoders settle or the timeout expires.
    async fn handle_brake(&mut self, completion_requested: bool) {
        info!("brake");
        self.drift.enabled = false;
        start_encoder_sampling(types::BRAKE_COAST_SETTLE_INTERVAL_MS, true).await;
        motor_driver::send_motor_command(MotorCommand::BrakeAll).await;

        motion::set_track_speeds(0, 0).await;

        self.active_intent = Some(ActiveIntent::BrakeCoast {
            state: BrakeCoastState::new(),
            completion_requested,
        });
    }

    /// Handle an `Idle` command.
    ///
    /// Disables drift compensation and stops encoder sampling, then registers
    /// an idle intent while coasting in place.
    async fn handle_idle(&mut self, duration_ms: u64, completion_requested: bool) {
        self.drift.enabled = false;
        lifecycle::stop_encoder_sampling().await;

        motor_driver::send_motor_command(MotorCommand::CoastAll).await;
        motion::set_track_speeds(0, 0).await;

        if duration_ms == 0 {
            api::send_completion(
                completion_requested,
                DriveCompletion {
                    status: CompletionStatus::Success,
                    telemetry: types::CompletionTelemetry::None,
                },
            )
            .await;
            return;
        }

        let started_at_ms = Instant::now().as_millis();
        self.active_intent = Some(ActiveIntent::Idle {
            duration_ms,
            started_at_ms,
            completion_requested,
        });
    }

    /// Handle a `Standby` command.
    ///
    /// Disables drift compensation, stops encoder sampling, and powers down the
    /// motor drivers. A brief brake → coast sequence is issued first to ensure
    /// a controlled stop before the drivers are disabled.
    async fn handle_standby(&mut self) {
        self.drift.enabled = false;
        lifecycle::stop_encoder_sampling().await;

        if !self.standby_enabled {
            motor_driver::send_motor_command(MotorCommand::BrakeAll).await;
            Timer::after(Duration::from_millis(100)).await;
            motor_driver::send_motor_command(MotorCommand::CoastAll).await;
            Timer::after(Duration::from_millis(100)).await;
            motor_driver::send_motor_command(MotorCommand::SetAllDriversEnable { enabled: false }).await;
            self.standby_enabled = true;

            motion::set_track_speeds(0, 0).await;
        }
    }

    /// Handle an interrupt by applying the requested motor action, cancelling
    /// the active intent (resolving its completion as `Cancelled`), and bumping
    /// the epoch counter to invalidate any queued commands stamped before the
    /// interrupt.
    ///
    /// Queued commands are drained here. Any queued command that requested
    /// completion is resolved as `Cancelled`. If an active completion was already
    /// emitted, additional queued completion requests are dropped with a warning.
    /// This assumes a single governing producer that does not enqueue new commands
    /// during interrupt handling.
    #[allow(clippy::too_many_lines)]
    pub(super) async fn handle_interrupt(&mut self, kind: types::InterruptKind) {
        match kind {
            types::InterruptKind::EmergencyBrake => {
                motor_driver::send_motor_command(MotorCommand::BrakeAll).await;
            }
            types::InterruptKind::Stop | types::InterruptKind::CancelCurrent => {
                motor_driver::send_motor_command(MotorCommand::CoastAll).await;
            }
        }

        self.drift.enabled = false;
        lifecycle::stop_encoder_sampling().await;

        let mut completion_sent = false;

        // Cancel and resolve the active intent.
        if let Some(intent) = self.active_intent.take() {
            match intent {
                ActiveIntent::RotateExact {
                    state,
                    completion_requested,
                    started_at_ms,
                } => {
                    let accumulated = state.accumulated_angle.abs();
                    let target = state.target_angle.abs();
                    let last_yaw_deg = state.last_yaw.unwrap_or(0.0);
                    let duration_ms = Instant::now().as_millis() - started_at_ms;

                    lifecycle::stop_rotation_imu().await;
                    api::send_completion(
                        completion_requested,
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

                    if completion_requested {
                        completion_sent = true;
                    }
                }
                ActiveIntent::DriveDistance {
                    state,
                    completion_requested,
                } => {
                    let duration_ms = Instant::now().as_millis() - state.started_at_ms;

                    lifecycle::stop_curve_imu(&state.kind).await;
                    api::send_completion(
                        completion_requested,
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

                    if completion_requested {
                        completion_sent = true;
                    }
                }
                ActiveIntent::Idle {
                    completion_requested, ..
                }
                | ActiveIntent::BrakeCoast {
                    completion_requested, ..
                } => {
                    api::send_completion(
                        completion_requested,
                        DriveCompletion {
                            status: CompletionStatus::Cancelled,
                            telemetry: types::CompletionTelemetry::None,
                        },
                    )
                    .await;

                    if completion_requested {
                        completion_sent = true;
                    }
                }
            }
        }

        // Bump epoch to invalidate queued commands.
        api::CURRENT_EPOCH.fetch_add(1, Ordering::Relaxed);

        // Drain queued commands on interrupt (queue is explicitly cleared).
        while let Ok(envelope) = api::DRIVE_QUEUE.receiver().try_receive() {
            if envelope.completion_requested {
                if completion_sent {
                    warn!("interrupt: additional completion-requested command dropped");
                } else {
                    api::send_completion(
                        true,
                        DriveCompletion {
                            status: CompletionStatus::Cancelled,
                            telemetry: types::CompletionTelemetry::None,
                        },
                    )
                    .await;
                    completion_sent = true;
                }
            }
        }

        motion::set_track_speeds(0, 0).await;
    }
}
