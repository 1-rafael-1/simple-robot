use assign_resources::assign_resources;
use embassy_rp::adc::InterruptHandler as AdcInterruptHandler;
use embassy_rp::gpio;
use embassy_rp::pio::InterruptHandler;
use embassy_rp::{bind_interrupts, peripherals};

assign_resources! {
    // motor_left: MotorLeftResources {
    //     pwm: PWM_CH0,
    //     pin: PIN_0,
    // },
    // motor_right: MotorRightResources {
    //     pwm: PWM_CH1,
    //     pin: PIN_1,
    // },
    distance_sensor: DistanceSensorResources {
        trigger_pin: PIN_16,
        echo_pin: PIN_17,
    },
    led: LedResources {
        pin: PIN_25,
    },
}

// bind_interrupts!(pub struct Irqs {
//     PIO0_IRQ_0 => InterruptHandler<PIO0>;
//     ADC_IRQ_FIFO => AdcInterruptHandler;
// });
