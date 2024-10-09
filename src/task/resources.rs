use assign_resources::assign_resources;
use embassy_rp::adc::InterruptHandler as AdcInterruptHandler;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals;

assign_resources! {
   distance_sensor: DistanceSensorResources {
       trigger_pin: PIN_15,
       echo_pin: PIN_14,
   },
   status_led: StatusLedResources {
       pin: PIN_22,
   },
//    motor_left: MotorLeftResources {
//        slice: PWM_SLICE7,
//        fwdPin: PIN_14,
//        backPin: PIN_15,
//    },
//    motor_right: MotorRightResources {
//        slice: PWM_SLICE8,
//        fwdPin: PIN_16,
//        backPin: PIN_17,
//    },
   battery_indicator: BatteryIndicatorResources {
       pwm_red: PWM_SLICE1,
       pwm_green: PWM_SLICE2,
       red_pin: PIN_2,
       green_pin: PIN_4,
       vsys_pin: PIN_29,
       adc: ADC,
   },
}

bind_interrupts!(pub struct Irqs {
    ADC_IRQ_FIFO => AdcInterruptHandler;
});
