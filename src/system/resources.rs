//! Hardware Resource Management
//!
//! Manages and allocates hardware resources (pins, peripherals) to different system components.
//! This module ensures safe and organized access to the robot's hardware by:
//! - Defining clear ownership of hardware resources
//! - Preventing conflicts in hardware access
//! - Providing safe concurrent access to shared resources (e.g., ADC)
//!
//! # Resource Groups
//! - Distance Sensor: HC-SR04 ultrasonic sensor pins
//! - Battery Monitor: System voltage monitoring pin
//! - RGB LED: PWM-controlled indicator LED pins
//! - RC Control: Remote control button input pins
//! - Motor Control: Dual motor driver pins and PWM channels
//!
//! # Shared Resources
//! The ADC is shared between tasks and protected by a mutex to ensure
//! safe concurrent access. Tasks must acquire the mutex lock before
//! performing ADC operations and release it promptly after.

use assign_resources::assign_resources;
use embassy_rp::{
    adc::{Adc, Async as AdcAsync, InterruptHandler as AdcInterruptHandler},
    bind_interrupts,
    i2c::{Async as I2cAsync, I2c, InterruptHandler as I2cInterruptHandler},
    peripherals::{self, ADC, I2C0, PIO0},
    pio::InterruptHandler as PioInterruptHandler,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};

/// Global ADC (Analog-to-Digital Converter) instance protected by a mutex.
///
/// The mutex ensures safe concurrent access from multiple tasks that need to read analog values
/// (e.g., battery voltage monitoring). Only one task can access the ADC at a time, preventing
/// conflicts in hardware access.
static ADC: Mutex<CriticalSectionRawMutex, Option<Adc<'static, AdcAsync>>> = Mutex::new(None);

/// Initializes the ADC peripheral.
///
/// This should only be called once during system initialization in main.rs,
/// before any tasks are spawned.
pub fn init_adc(adc: ADC) {
    let adc = Adc::new(adc, Irqs, embassy_rp::adc::Config::default());
    critical_section::with(|_| {
        *ADC.try_lock().unwrap() = Some(adc);
    });
}

/// Returns a reference to the protected ADC instance.
///
/// The returned mutex ensures safe concurrent access to the ADC peripheral.
/// Tasks should acquire the mutex lock, perform their ADC operations,
/// and release the lock as quickly as possible.
pub fn get_adc() -> &'static Mutex<CriticalSectionRawMutex, Option<Adc<'static, AdcAsync>>> {
    &ADC
}

/// public type for better readability in main.rs and tasks
pub type I2c0BusShared = Mutex<CriticalSectionRawMutex, I2c<'static, I2C0, I2cAsync>>;

assign_resources! {
    /// HC-SR04 ultrasonic distance sensor pins
    us_distance_sensor: UltrasonicDistanceSensorResources {
        trigger_pin: PIN_15,
        echo_pin: PIN_14,
     },
     /// Battery voltage monitoring pin
    battery_charge: BatteryChargeResources {
       vsys_pin: PIN_29,
    },
    /// PWM-controlled RGB LED indicator pins
    rgb_led: RGBLedResources {
        pwm_red: PWM_SLICE1,
        pwm_green: PWM_SLICE2,
        red_pin: PIN_2,
        green_pin: PIN_4,
    },
    /// Remote control buttons
    rc: RCResources {
        btn_a: PIN_10,
        btn_b: PIN_16,
        btn_c: PIN_11,
        btn_d: PIN_17,
    },
    /// IR obstacle avoidance sensor - digital output
    ir_sensor: IRSensorResources {
        ir_left_pin: PIN_6,
        ir_right_pin: PIN_26,
    },
    /// TB6612FNG dual motor driver pins and PWM channels
    motor_driver: MotorDriverResources {
        standby_pin: PIN_22,
        // Motor drive PWM
        left_slice: PWM_SLICE6,
        left_pwm_pin: PIN_28,
        left_forward_pin: PIN_21,
        left_backward_pin: PIN_20,
        // Motor drive PWM
        right_slice: PWM_SLICE5,
        right_pwm_pin: PIN_27,
        right_forward_pin: PIN_19,
        right_backward_pin: PIN_18,
    },
    /// Motor encoder PWM input channels
    motor_encoders: MotorEncoderResources {
        left_encoder_slice: PWM_SLICE3,
        left_encoder_pin: PIN_7,
        right_encoder_slice: PWM_SLICE4,
        right_encoder_pin: PIN_9,
    },
    /// MPU6500 6-axis IMU
    inertial_measurement_unit: InertialMeasurementUnitResources {
        // I2C resources handled in I2C0 bus
        int: PIN_8,
        add: PIN_3
    },
    /// Servo for ultrasonic sweep
    sweep_servo: SweepServoResources {
        pin: PIN_5,
        pio: PIO0
    },
    /// I2C0 bus
    i2c: I2c0BusResources {
        scl: PIN_13,
        sda: PIN_12,
        i2c0: I2C0,
    },
    /// ADC
    adc: AdcResources {
        adc: ADC,
    },
}

bind_interrupts!(pub struct Irqs {
    ADC_IRQ_FIFO => AdcInterruptHandler;
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    I2C0_IRQ => I2cInterruptHandler<I2C0>;
});
