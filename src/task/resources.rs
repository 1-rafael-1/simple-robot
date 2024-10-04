use assign_resources::assign_resources;
use embassy_rp::adc::InterruptHandler as AdcInterruptHandler;
use embassy_rp::gpio;
use embassy_rp::pio::InterruptHandler;
use embassy_rp::peripherals::{PWM_SLICE7, PWM_SLICE8};
use embassy_rp::{bind_interrupts, peripherals};
use embassy_rp::pwm::{Pwm, Slice, Config};

assign_resources! {
    distance_sensor: DistanceSensorResources {
        trigger_pin: PIN_13,
        echo_pin: PIN_5,
    },
    status_led: StatusLedResources {
        pin: PIN_25,
    },
    motor_left: MotorLeftResources {
        slice: PWM_SLICE7,
        pin: PIN_0,
    },
    motor_right: MotorRightResources {
        slice: PWM_SLICE8, 
        pin: PIN_16,
    },
}

// bind_interrupts!(pub struct Irqs {
//     PIO0_IRQ_0 => InterruptHandler<PIO0>;
//     ADC_IRQ_FIFO => AdcInterruptHandler;
// });
