use assign_resources::assign_resources;
use embassy_rp::peripherals;

assign_resources! {
   distance_sensor: DistanceSensorResources {
       trigger_pin: PIN_20,
       echo_pin: PIN_21,
   },
   status_led: StatusLedResources {
       pin: PIN_22,
   },
   motor_left: MotorLeftResources {
       slice: PWM_SLICE7,
       fwdPin: PIN_14,
       backPin: PIN_15,
   },
   motor_right: MotorRightResources {
       slice: PWM_SLICE8,
       fwdPin: PIN_16,
       backPin: PIN_17,
    },
}

// bind_interrupts!(pub struct Irqs {
//     PIO0_IRQ_0 => InterruptHandler<PIO0>;
//     ADC_IRQ_FIFO => AdcInterruptHandler;
// });
