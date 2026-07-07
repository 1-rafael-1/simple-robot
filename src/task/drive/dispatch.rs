//! Drive command dispatch — envelope routing, standby wake-up, and `IntentTeardown` execution.
//!
//! This module is the thin seam between the drive queue and the control modules.
//! It does NOT own per-intent knowledge — control modules declare their
//! teardown needs via [`IntentTeardown`] descriptors.

use defmt::info;
use embassy_time::{Duration, Instant, Timer};

use crate::{
    system::state::motion,
    task::{
        drive::{
            api::{self, DriveCommandEnvelope},
            brake_coast::BrakeCoastState,
            differential,
            distance::DistanceDriveState,
            rotation::RotationState,
            sensors::{control as lifecycle, data::IMU_FEEDBACK_CHANNEL},
            state::{ActiveIntent, DriveLoop},
            types::{self, CompletionStatus, DriveAction, DriveCommand, DriveCompletion, IntentTeardown},
        },
        motor_driver::{self, MotorCommand},
    },
};

impl DriveLoop {
    /// Handle a dequeued command envelope, honouring epoch cancellation.
    pub(super) async fn handle_envelope(&mut self, envelope: DriveCommandEnvelope) {
        let current_epoch = api::CURRENT_EPOCH.load(core::sync::atomic::Ordering::Relaxed);
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
    pub(super) async fn handle_drive_action(&mut self, action: DriveAction, completion_requested: bool) {
        // Gate movement commands until the IMU is streaming stabilised data.
        // On first movement command: start IMU, wait for first sample, wait for
        // DMP filter stabilise (150 ms). Subsequent commands skip this.
        // Non-movement commands (Coast, Brake, Idle, Standby) pass through.
        if action_requires_imu(&action) && !ensure_imu_ready().await {
            api::send_completion(
                completion_requested,
                DriveCompletion {
                    status: CompletionStatus::Failed("IMU not responding"),
                    telemetry: types::CompletionTelemetry::None,
                },
            )
            .await;
            return;
        }

        // Wake from standby if a movement command arrives.
        self.wake_from_standby(&action).await;

        match action {
            DriveAction::Differential { left, right } => {
                self.handle_differential(left, right).await;
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

    /// Wake motor drivers from standby if the action is a movement command.
    async fn wake_from_standby(&mut self, action: &DriveAction) {
        if !self.standby_enabled {
            return;
        }
        match action {
            DriveAction::Differential { .. } | DriveAction::RotateExact { .. } | DriveAction::DriveDistance { .. } => {
                motor_driver::send_motor_command(MotorCommand::SetAllDriversEnable { enabled: true }).await;
                self.standby_enabled = false;
                Timer::after(Duration::from_millis(100)).await;
            }
            _ => {}
        }
    }

    // ── Per-action handlers ────────────────────────────────────────────────

    /// Handle a `Differential` command — fire-and-forget passthrough.
    async fn handle_differential(&self, left: i8, right: i8) {
        let (left_adjusted, right_adjusted) = differential::set_speeds(left, right);

        motor_driver::send_motor_command(MotorCommand::SetTracks {
            left_speed: left_adjusted,
            right_speed: right_adjusted,
        })
        .await;
        motion::set_track_speeds(left_adjusted, right_adjusted).await;
    }

    /// Handle a `RotateExact` command — delegates to `rotation::init()`.
    async fn handle_rotate_exact(
        &mut self,
        degrees: f32,
        direction: types::RotationDirection,
        motion: types::RotationMotion,
        completion_requested: bool,
    ) {
        let intent = RotationState::init(degrees, direction, motion, completion_requested).await;
        self.active_intent = Some(intent);
    }

    /// Handle a `DriveDistance` command — delegates to `distance::init()`.
    async fn handle_drive_distance(
        &mut self,
        kind: types::DriveDistanceKind,
        direction: types::DriveDirection,
        speed: u8,
        completion_requested: bool,
    ) {
        if let Some(intent) = DistanceDriveState::init(kind, direction, speed, completion_requested).await {
            self.active_intent = Some(intent);
        }
    }

    /// Handle a `Coast` command.
    async fn handle_coast(&mut self, completion_requested: bool) {
        info!("coast");
        motor_driver::send_motor_command(MotorCommand::CoastAll).await;
        motion::set_track_speeds(0, 0).await;

        let intent = BrakeCoastState::init(completion_requested).await;
        self.active_intent = Some(intent);
    }

    /// Handle a `Brake` command.
    async fn handle_brake(&mut self, completion_requested: bool) {
        info!("brake");
        motor_driver::send_motor_command(MotorCommand::BrakeAll).await;
        motion::set_track_speeds(0, 0).await;

        let intent = BrakeCoastState::init(completion_requested).await;
        self.active_intent = Some(intent);
    }

    /// Handle an `Idle` command.
    async fn handle_idle(&mut self, duration_ms: u64, completion_requested: bool) {
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
    async fn handle_standby(&mut self) {
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

    /// Handle an interrupt — cancel active intent, stop sensors, bump epoch, drain queue.
    pub(super) async fn handle_interrupt(&mut self, kind: types::InterruptKind) {
        match kind {
            types::InterruptKind::EmergencyBrake => {
                motor_driver::send_motor_command(MotorCommand::BrakeAll).await;
            }
            types::InterruptKind::Stop | types::InterruptKind::CancelCurrent => {
                motor_driver::send_motor_command(MotorCommand::CoastAll).await;
            }
        }

        let mut completion_sent = false;

        // Cancel and resolve the active intent using teardown descriptors.
        if let Some(intent) = self.active_intent.take() {
            let completion_requested = intent.completion_requested();
            let teardown = intent.teardown();
            let telemetry = intent.cancellation_telemetry();

            execute_intent_teardown(teardown).await;
            api::send_completion(
                completion_requested,
                DriveCompletion {
                    status: CompletionStatus::Cancelled,
                    telemetry,
                },
            )
            .await;

            if completion_requested {
                completion_sent = true;
            }
        }

        // Bump epoch to invalidate queued commands.
        api::CURRENT_EPOCH.fetch_add(1, core::sync::atomic::Ordering::Relaxed);

        // Drain queued commands.
        while let Ok(envelope) = api::DRIVE_QUEUE.receiver().try_receive() {
            if envelope.completion_requested {
                if completion_sent {
                    defmt::warn!("interrupt: additional completion-requested command dropped");
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

// ── Teardown execution ─────────────────────────────────────────────────────

/// Execute the sensor teardown declared by a control module.
pub(super) async fn execute_intent_teardown(teardown: IntentTeardown) {
    match teardown {
        IntentTeardown::RotationImu => {
            lifecycle::stop_rotation_imu();
        }
        IntentTeardown::DistanceImuAndMotors => {
            // Stop motors first (safety: covers failure paths where the controller
            // returned early without stopping motors, e.g. encoder timeout/stall).
            motor_driver::send_motor_command(MotorCommand::SetTracks {
                left_speed: 0,
                right_speed: 0,
            })
            .await;
            motion::set_track_speeds(0, 0).await;
            lifecycle::stop_encoder_sampling().await;
            lifecycle::stop_distance_imu(&types::DriveDistanceKind::Straight { distance_cm: 0.0 });
        }
        IntentTeardown::EncoderSettle => {
            lifecycle::stop_encoder_sampling().await;
        }
        IntentTeardown::None => {}
    }
}

/// Returns `true` if the drive action requires IMU data.
const fn action_requires_imu(action: &DriveAction) -> bool {
    matches!(
        action,
        DriveAction::Differential { .. } | DriveAction::DriveDistance { .. } | DriveAction::RotateExact { .. }
    )
}

/// Ensure the IMU is streaming stabilised data.
///
/// On first call: starts the IMU with default fusion mode, waits for the first
/// DMP FIFO sample (1 s timeout), waits 150 ms for DMP filter stabilisation,
/// drains stale samples. Returns `true` if the IMU is ready; `false` on timeout.
/// Subsequent calls return `true` immediately.
async fn ensure_imu_ready() -> bool {
    use core::sync::atomic::Ordering;

    use embassy_time::{Duration, Instant, Timer};

    const IMU_STABILISE_MS: u64 = 150;
    const IMU_WAIT_TIMEOUT_MS: u64 = 1000;

    static IMU_STABILISED: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

    if IMU_STABILISED.load(Ordering::Relaxed) {
        return true;
    }

    lifecycle::start_distance_imu(&types::DriveDistanceKind::Straight { distance_cm: 0.0 });

    // Wait for the first sample.
    let deadline = Instant::now() + Duration::from_millis(IMU_WAIT_TIMEOUT_MS);
    let mut got_sample = false;
    while Instant::now() < deadline {
        while IMU_FEEDBACK_CHANNEL.receiver().try_receive().is_ok() {
            got_sample = true;
        }
        if got_sample {
            break;
        }
        Timer::after(Duration::from_millis(10)).await;
    }

    if !got_sample {
        return false;
    }

    // DMP filter stabilisation.
    Timer::after(Duration::from_millis(IMU_STABILISE_MS)).await;

    // Drain any stale samples accumulated during stabilise.
    while IMU_FEEDBACK_CHANNEL.receiver().try_receive().is_ok() {}

    IMU_STABILISED.store(true, Ordering::Relaxed);
    true
}
