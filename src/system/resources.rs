//! Resource Allocation Module
//!
//! This module defines the hardware resources used by various components of the robot.
//! It uses the `assign_resources` macro to allocate specific pins and peripherals to each component.

use assign_resources::assign_resources;
use embassy_rp::adc::InterruptHandler as AdcInterruptHandler;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals;

assign_resources! {
    distance_sensor: DistanceSensorResources {
       trigger_pin: PIN_15,
       echo_pin: PIN_14,
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
