//! Resource Allocation Module
//!
//! This module defines the hardware resources used by various components of the robot.
//! It uses the `assign_resources` macro to allocate specific pins and peripherals to each component.

use assign_resources::assign_resources;
use embassy_rp::adc::InterruptHandler as AdcInterruptHandler;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals;

/// Allocates hardware resources to different components of the robot
///
/// This macro call defines the resources for various subsystems
assign_resources! {
    distance_sensor: DistanceSensorResources {
       trigger_pin: PIN_15,
       echo_pin: PIN_14,
    },
    status_led: StatusLedResources {
        pin: PIN_22,
    },
    battery_charge: BatteryChargeResources {
       vsys_pin: PIN_29,
       adc: ADC,
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

    // Commented out motor resources for future implementation
    //
    // /// Resources for the left motor
    // motor_left: MotorLeftResources {
    //     /// PWM slice for the left motor
    //     slice: PWM_SLICE7,
    //     /// Forward pin for the left motor
    //     fwdPin: PIN_14,
    //     /// Backward pin for the left motor
    //     backPin: PIN_15,
    // },
    //
    // /// Resources for the right motor
    // motor_right: MotorRightResources {
    //     /// PWM slice for the right motor
    //     slice: PWM_SLICE8,
    //     /// Forward pin for the right motor
    //     fwdPin: PIN_16,
    //     /// Backward pin for the right motor
    //     backPin: PIN_17,
    // },
}

/// Binds interrupt handlers for the robot's peripherals
///
/// Currently, this only includes the ADC interrupt handler.
bind_interrupts!(pub struct Irqs {
    ADC_IRQ_FIFO => AdcInterruptHandler;
});
