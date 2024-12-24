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
use embassy_rp::adc::InterruptHandler as AdcInterruptHandler;
use embassy_rp::adc::{Adc, Async};
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

/// Global ADC (Analog-to-Digital Converter) instance protected by a mutex.
///
/// The mutex ensures safe concurrent access from multiple tasks that need to read analog values
/// (e.g., battery voltage monitoring). Only one task can access the ADC at a time, preventing
/// conflicts in hardware access.
///
/// Usage pattern:
/// ```rust
/// let voltage = {
///     let mut adc_guard = get_adc().lock().await;
///     let adc = adc_guard.as_mut().unwrap();
///     // Perform ADC reading here
///     // Lock is automatically released when scope ends
/// };
/// ```
static ADC: Mutex<CriticalSectionRawMutex, Option<Adc<'static, Async>>> = Mutex::new(None);

/// Initializes the ADC peripheral.
///
/// This should only be called once during system initialization in main.rs,
/// before any tasks are spawned.
pub fn init_adc(adc: peripherals::ADC) {
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
pub fn get_adc() -> &'static Mutex<CriticalSectionRawMutex, Option<Adc<'static, Async>>> {
    &ADC
}

assign_resources! {
    /// HC-SR04 ultrasonic distance sensor pins
    distance_sensor: DistanceSensorResources {
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
    /// Remote control button A input
    rc_a: RCResourcesA {
        btn_a: PIN_16,
    },
    /// Remote control button B input
    rc_b: RCResourcesB {
        btn_b: PIN_17,
    },
    /// Remote control button C input
    rc_c: RCResourcesC {
        btn_c: PIN_10,
    },
    /// Remote control button D input
    rc_d: RCResourcesD {
        btn_d: PIN_11,
    },
    /// IR obstacle avoidance sensor (VMA330) - digital output
    ir_sensor: IRSensorResources {
        ir_pin: PIN_26, // Digital input pin
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
}

bind_interrupts!(pub struct Irqs {
    ADC_IRQ_FIFO => AdcInterruptHandler;
});
