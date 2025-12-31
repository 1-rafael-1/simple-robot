//! Robot firmware entry point
//!
//! Initializes system and spawns control tasks.

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::{
    Peri,
    adc::{Adc, Channel, Config as AdcConfig, InterruptHandler as AdcInterruptHandler},
    bind_interrupts,
    block::ImageDef,
    config::Config,
    gpio::{Input, Level, Output, Pull},
    i2c::{Config as I2cConfig, I2c, InterruptHandler as I2cInterruptHandler},
    peripherals::{I2C0, PIO0},
    pio::InterruptHandler as PioInterruptHandler,
    pwm::{InputMode, Pwm},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use panic_probe as _;
use static_cell::StaticCell;

use crate::{
    system::event::ButtonId,
    task::{
        autonomous_drive::autonomous_drive,
        battery_charge_read::battery_charge_read,
        display::display,
        drive::drive,
        encoder_read::{self, encoder_read},
        imu_read::inertial_measurement_read,
        ir_obstacle_detect::ir_obstacle_detect,
        monitor_motion::motion_correction_control,
        orchestrate::orchestrate,
        rc_control::rc_button_handle,
        rgb_led_indicate::rgb_led_indicate,
        sweep_ultrasonic::{self, ultrasonic_sweep},
        track_inactivity::track_inactivity,
    },
};

/// Firmware image type for bootloader
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

/// System core modules
mod system;
/// Task implementations
mod task;

// Bind interrupts on global scope for convenience
bind_interrupts!(pub struct Irqs {
    ADC_IRQ_FIFO => AdcInterruptHandler;
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    I2C0_IRQ => I2cInterruptHandler<I2C0>;
});

/// Public type for shared I2C bus
pub type I2cBusShared = Mutex<CriticalSectionRawMutex, I2c<'static, I2C0, embassy_rp::i2c::Async>>;

/// Firmware entry point
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Configure rp2350 to use the external oscillator and run at its usual 150Mhz
    let mut config = Config::default();
    config.clocks = embassy_rp::clocks::ClockConfig::system_freq(150_000_000).unwrap();
    let p = embassy_rp::init(config);

    // make peripheral handles and spawn tasks
    init_orchestrate(&spawner);
    init_battery_monitoring(&spawner, p.ADC, p.PIN_29);
    init_rgb_led(&spawner, p.PWM_SLICE1, p.PIN_2, p.PWM_SLICE2, p.PIN_4);
    init_rc_buttons(&spawner, p.PIN_10, p.PIN_16, p.PIN_11, p.PIN_17);
    init_motor_driver(
        &spawner,
        p.PWM_SLICE6,
        p.PIN_28,
        p.PWM_SLICE5,
        p.PIN_27,
        p.PIN_22,
        p.PIN_21,
        p.PIN_20,
        p.PIN_19,
        p.PIN_18,
    );
    init_autonomous_drive(&spawner);
    init_ir_obstacle_detect(&spawner, p.PIN_26);
    init_ultrasonic_sweep(&spawner, p.PIO0, p.PIN_5, p.PIN_15, p.PIN_14);
    let i2c_bus = init_i2c_bus(p.I2C0, p.PIN_13, p.PIN_12);
    init_display(&spawner, i2c_bus);
    init_imu_read(&spawner, i2c_bus, p.PIN_8, p.PIN_3);
    init_encoder_read(&spawner, p.PWM_SLICE3, p.PIN_7, p.PWM_SLICE4, p.PIN_9);
    init_motion_correction(&spawner);
    init_inactivity_tracker(&spawner);

    // main wishes you a great day
}

/// Initialize orchestrator task
fn init_orchestrate(spawner: &Spawner) {
    spawner.must_spawn(orchestrate());
}

/// Initialize battery monitoring task with ADC
fn init_battery_monitoring(
    spawner: &Spawner,
    adc: Peri<'static, embassy_rp::peripherals::ADC>,
    pin_29: Peri<'static, embassy_rp::peripherals::PIN_29>,
) {
    let adc = Adc::new(adc, Irqs, AdcConfig::default());
    let vsys_channel = Channel::new_pin(pin_29, Pull::None);
    spawner.must_spawn(battery_charge_read(adc, vsys_channel));
}

/// Initialize RGB LED indicator with PWM outputs
fn init_rgb_led(
    spawner: &Spawner,
    pwm_slice1: Peri<'static, embassy_rp::peripherals::PWM_SLICE1>,
    pin_2: Peri<'static, embassy_rp::peripherals::PIN_2>,
    pwm_slice2: Peri<'static, embassy_rp::peripherals::PWM_SLICE2>,
    pin_4: Peri<'static, embassy_rp::peripherals::PIN_4>,
) {
    let pwm_config = embassy_rp::pwm::Config::default();
    let pwm_red = Pwm::new_output_a(pwm_slice1, pin_2, pwm_config.clone());
    let pwm_green = Pwm::new_output_a(pwm_slice2, pin_4, pwm_config);
    spawner.must_spawn(rgb_led_indicate(pwm_red, pwm_green));
}

/// Initialize all four RC button inputs
fn init_rc_buttons(
    spawner: &Spawner,
    pin_10: Peri<'static, embassy_rp::peripherals::PIN_10>,
    pin_16: Peri<'static, embassy_rp::peripherals::PIN_16>,
    pin_11: Peri<'static, embassy_rp::peripherals::PIN_11>,
    pin_17: Peri<'static, embassy_rp::peripherals::PIN_17>,
) {
    let btn_a = Input::new(pin_10, Pull::Down);
    spawner.must_spawn(rc_button_handle(btn_a, ButtonId::A));

    let btn_b = Input::new(pin_16, Pull::Down);
    spawner.must_spawn(rc_button_handle(btn_b, ButtonId::B));

    let btn_c = Input::new(pin_11, Pull::Down);
    spawner.must_spawn(rc_button_handle(btn_c, ButtonId::C));

    let btn_d = Input::new(pin_17, Pull::Down);
    spawner.must_spawn(rc_button_handle(btn_d, ButtonId::D));
}

/// Initialize motor driver with PWM and direction control
fn init_motor_driver(
    spawner: &Spawner,
    pwm_slice6: Peri<'static, embassy_rp::peripherals::PWM_SLICE6>,
    pin_28: Peri<'static, embassy_rp::peripherals::PIN_28>,
    pwm_slice5: Peri<'static, embassy_rp::peripherals::PWM_SLICE5>,
    pin_27: Peri<'static, embassy_rp::peripherals::PIN_27>,
    pin_22: Peri<'static, embassy_rp::peripherals::PIN_22>,
    pin_21: Peri<'static, embassy_rp::peripherals::PIN_21>,
    pin_20: Peri<'static, embassy_rp::peripherals::PIN_20>,
    pin_19: Peri<'static, embassy_rp::peripherals::PIN_19>,
    pin_18: Peri<'static, embassy_rp::peripherals::PIN_18>,
) {
    let motor_standby = Output::new(pin_22, Level::Low);

    let desired_freq_hz = 10_000u32;
    let clock_freq_hz = embassy_rp::clocks::clk_sys_freq();
    let divider = ((clock_freq_hz / desired_freq_hz) / 65535 + 1) as u8;
    let period = (clock_freq_hz / (desired_freq_hz * divider as u32)) as u16 - 1;
    let mut motor_pwm_config = embassy_rp::pwm::Config::default();
    motor_pwm_config.divider = divider.into();
    motor_pwm_config.top = period;

    let left_pwm = Pwm::new_output_a(pwm_slice6, pin_28, motor_pwm_config.clone());
    let right_pwm = Pwm::new_output_b(pwm_slice5, pin_27, motor_pwm_config);

    let left_forward = Output::new(pin_21, Level::Low);
    let left_backward = Output::new(pin_20, Level::Low);
    let right_forward = Output::new(pin_19, Level::Low);
    let right_backward = Output::new(pin_18, Level::Low);

    spawner.must_spawn(drive(
        motor_standby,
        left_pwm,
        left_forward,
        left_backward,
        right_pwm,
        right_forward,
        right_backward,
    ));
}

/// Initialize IR obstacle detection
fn init_ir_obstacle_detect(spawner: &Spawner, pin_26: Peri<'static, embassy_rp::peripherals::PIN_26>) {
    let ir_right = Input::new(pin_26, Pull::Down);
    spawner.must_spawn(ir_obstacle_detect(ir_right));
}

/// Initialize ultrasonic sensor sweep with servo
fn init_ultrasonic_sweep(
    spawner: &Spawner,
    pio0: Peri<'static, PIO0>,
    pin_5: Peri<'static, embassy_rp::peripherals::PIN_5>,
    pin_15: Peri<'static, embassy_rp::peripherals::PIN_15>,
    pin_14: Peri<'static, embassy_rp::peripherals::PIN_14>,
) {
    let us_trigger = Output::new(pin_15, Level::Low);
    let us_echo = Input::new(pin_14, Pull::None);
    let sensor = sweep_ultrasonic::setup_ultrasonic_sensor(us_trigger, us_echo);
    let servo = sweep_ultrasonic::setup_servo(pio0, pin_5);
    spawner.must_spawn(ultrasonic_sweep(sensor, servo));
}

/// Initialize shared I2C bus for display and IMU
fn init_i2c_bus(
    i2c0: Peri<'static, I2C0>,
    pin_13: Peri<'static, embassy_rp::peripherals::PIN_13>,
    pin_12: Peri<'static, embassy_rp::peripherals::PIN_12>,
) -> &'static I2cBusShared {
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = 400_000;
    let i2c = I2c::new_async(i2c0, pin_13, pin_12, Irqs, i2c_config);
    static I2C_BUS: StaticCell<I2cBusShared> = StaticCell::new();
    I2C_BUS.init(Mutex::new(i2c))
}

/// Initialize display task with I2C bus
fn init_display(spawner: &Spawner, i2c_bus: &'static I2cBusShared) {
    spawner.must_spawn(display(i2c_bus));
}

/// Initialize IMU task with I2C bus and pins
fn init_imu_read(
    spawner: &Spawner,
    i2c_bus: &'static I2cBusShared,
    pin_8: Peri<'static, embassy_rp::peripherals::PIN_8>,
    pin_3: Peri<'static, embassy_rp::peripherals::PIN_3>,
) {
    spawner.must_spawn(inertial_measurement_read(i2c_bus, pin_8, pin_3));
}

/// Initialize autonomous drive task
fn init_autonomous_drive(spawner: &Spawner) {
    spawner.must_spawn(autonomous_drive());
}

/// Initialize motor encoder inputs
fn init_encoder_read(
    spawner: &Spawner,
    pwm_slice3: Peri<'static, embassy_rp::peripherals::PWM_SLICE3>,
    pin_7: Peri<'static, embassy_rp::peripherals::PIN_7>,
    pwm_slice4: Peri<'static, embassy_rp::peripherals::PWM_SLICE4>,
    pin_9: Peri<'static, embassy_rp::peripherals::PIN_9>,
) {
    let encoder_pwm_config = encoder_read::configure_encoder_pwm();
    let left_encoder = Pwm::new_input(
        pwm_slice3,
        pin_7,
        Pull::None,
        InputMode::RisingEdge,
        encoder_pwm_config.clone(),
    );
    let right_encoder = Pwm::new_input(pwm_slice4, pin_9, Pull::None, InputMode::RisingEdge, encoder_pwm_config);
    spawner.must_spawn(encoder_read(left_encoder, right_encoder));
}

/// Initialize motion correction control task
fn init_motion_correction(spawner: &Spawner) {
    spawner.must_spawn(motion_correction_control());
}

/// Initialize inactivity tracker task
fn init_inactivity_tracker(spawner: &Spawner) {
    spawner.must_spawn(track_inactivity());
}
