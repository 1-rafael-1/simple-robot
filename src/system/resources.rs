//! Resource Allocation Module
//!
//! This module defines the hardware resources used by various components of the robot.
//! It uses the `assign_resources` macro to allocate specific pins and peripherals to each component.

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
    distance_sensor: DistanceSensorResources {
       trigger_pin: PIN_15,
       echo_pin: PIN_14,
    },
    battery_charge: BatteryChargeResources {
       vsys_pin: PIN_29,
    },
    rgb_led: RGBLedResources {
        pwm_red: PWM_SLICE1,
        pwm_green: PWM_SLICE2,
        red_pin: PIN_2,
        green_pin: PIN_4,
    },
    rc_a: RCResourcesA {
        btn_a: PIN_6,
    },
    rc_b: RCResourcesB {
        btn_b: PIN_7,
    },
    rc_c: RCResourcesC {
        btn_c: PIN_8,
    },
    rc_d: RCResourcesD {
        btn_d: PIN_9,
    },
    motor: MotorResources {
        standby_pin: PIN_22,
        left_slice: PWM_SLICE6,
        left_pwm_pin: PIN_28,
        left_forward_pin: PIN_21,
        left_backward_pin: PIN_20,
        right_slice: PWM_SLICE5,
        right_pwm_pin: PIN_27,
        right_forward_pin: PIN_19,
        right_backward_pin: PIN_18,
        },
}

bind_interrupts!(pub struct Irqs {
    ADC_IRQ_FIFO => AdcInterruptHandler;
});
