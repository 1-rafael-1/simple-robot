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
    flash::{Async, Flash},
    gpio::Pull,
    i2c::{Config as I2cConfig, I2c, InterruptHandler as I2cInterruptHandler},
    peripherals::I2C0,
    pwm::{InputMode, Pwm},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use panic_probe as _;
use static_cell::StaticCell;

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
    I2C0_IRQ => I2cInterruptHandler<I2C0>;
    ADC_IRQ_FIFO => AdcInterruptHandler;
});

/// Motor driver peripheral pins (for new architecture with PCA9555)
/// Standby pins are now controlled via PCA9555 port expander (Port 1, bits 4-5)
struct MotorDriverPins {
    lmot_pwm_slice: Peri<'static, embassy_rp::peripherals::PWM_SLICE0>,
    lmot_pwm_a: Peri<'static, embassy_rp::peripherals::PIN_0>,
    lmot_pwm_b: Peri<'static, embassy_rp::peripherals::PIN_1>,
    rmot_pwm_slice: Peri<'static, embassy_rp::peripherals::PWM_SLICE1>,
    rmot_pwm_a: Peri<'static, embassy_rp::peripherals::PIN_2>,
    rmot_pwm_b: Peri<'static, embassy_rp::peripherals::PIN_3>,
    encoder_left_front_slice: Peri<'static, embassy_rp::peripherals::PWM_SLICE3>,
    encoder_left_front_pin: Peri<'static, embassy_rp::peripherals::PIN_7>,
    encoder_left_rear_slice: Peri<'static, embassy_rp::peripherals::PWM_SLICE2>,
    encoder_left_rear_pin: Peri<'static, embassy_rp::peripherals::PIN_21>,
    encoder_right_front_slice: Peri<'static, embassy_rp::peripherals::PWM_SLICE4>,
    encoder_right_front_pin: Peri<'static, embassy_rp::peripherals::PIN_9>,
    encoder_right_rear_slice: Peri<'static, embassy_rp::peripherals::PWM_SLICE5>,
    encoder_right_rear_pin: Peri<'static, embassy_rp::peripherals::PIN_27>,
}

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

    let i2c_bus = init_i2c_bus(p.I2C0, p.PIN_13, p.PIN_12);

    init_port_expander(&spawner, i2c_bus, p.PIN_20);

    init_orchestrate(&spawner);
    init_battery_monitoring(&spawner, p.ADC, p.PIN_29);
    // init_rgb_led(&spawner, p.PWM_SLICE1, p.PIN_2, p.PWM_SLICE2, p.PIN_4);
    // init_rc_buttons(&spawner, p.PIN_10, p.PIN_16, p.PIN_11, p.PIN_17);

    init_motor_driver(
        &spawner,
        MotorDriverPins {
            lmot_pwm_slice: p.PWM_SLICE0,
            lmot_pwm_a: p.PIN_0,
            lmot_pwm_b: p.PIN_1,
            rmot_pwm_slice: p.PWM_SLICE1,
            rmot_pwm_a: p.PIN_2,
            rmot_pwm_b: p.PIN_3,
            encoder_left_front_slice: p.PWM_SLICE3,
            encoder_left_front_pin: p.PIN_7,
            encoder_left_rear_slice: p.PWM_SLICE2,
            encoder_left_rear_pin: p.PIN_21,
            encoder_right_front_slice: p.PWM_SLICE4,
            encoder_right_front_pin: p.PIN_9,
            encoder_right_rear_slice: p.PWM_SLICE5,
            encoder_right_rear_pin: p.PIN_27,
        },
    );
    // init_autonomous_drive(&spawner);
    // init_ir_obstacle_detect(&spawner, p.PIN_26);
    // init_ultrasonic_sweep(&spawner, p.PIO0, p.PIN_5, p.PIN_15, p.PIN_14);

    init_display(&spawner, i2c_bus);
    init_imu_read(&spawner, i2c_bus, p.PIN_18, p.PIN_19);
    init_flash_storage(&spawner, p.FLASH, p.DMA_CH0);

    // init_motion_correction(&spawner);
    // init_inactivity_tracker(&spawner);

    // main wishes you a great day
}

/// Initialize orchestrator task
fn init_orchestrate(spawner: &Spawner) {
    spawner.must_spawn(task::orchestrate::orchestrate());
    spawner.must_spawn(send_initialize_event());
    spawner.must_spawn(auto_start_calibration());
}

/// Send initialization event to orchestrator
#[embassy_executor::task]
async fn send_initialize_event() {
    // Small delay to ensure orchestrator is ready
    embassy_time::Timer::after(embassy_time::Duration::from_millis(100)).await;
    system::event::send_event(system::event::Events::Initialize).await;
}

/// Auto-start calibration for testing (TEMPORARY HACK)
///
/// This automatically triggers motor calibration after a few seconds.
/// Remove this once we have proper UI control (menu, rotary encoder, or remote).
#[embassy_executor::task]
async fn auto_start_calibration() {
    // Wait for system to initialize
    embassy_time::Timer::after(embassy_time::Duration::from_secs(3)).await;

    defmt::info!("🤖 AUTO-CALIBRATION: Starting motor calibration in 2 seconds...");
    embassy_time::Timer::after(embassy_time::Duration::from_secs(2)).await;

    defmt::info!("🤖 AUTO-CALIBRATION: Triggering calibration now!");
    task::drive::send_drive_command(task::drive::DriveCommand::RunMotorCalibration);
}

/// Initialize battery monitoring task with ADC
fn init_battery_monitoring(
    spawner: &Spawner,
    adc: Peri<'static, embassy_rp::peripherals::ADC>,
    pin_29: Peri<'static, embassy_rp::peripherals::PIN_29>,
) {
    let adc = Adc::new(adc, Irqs, AdcConfig::default());
    let battery_channel = Channel::new_pin(pin_29, Pull::None);
    spawner.must_spawn(task::battery_charge_read::battery_charge_read(adc, battery_channel));
}

// /// Initialize RGB LED indicator with PWM outputs
// fn init_rgb_led(
//     spawner: &Spawner,
//     pwm_slice1: Peri<'static, embassy_rp::peripherals::PWM_SLICE1>,
//     pin_2: Peri<'static, embassy_rp::peripherals::PIN_2>,
//     pwm_slice2: Peri<'static, embassy_rp::peripherals::PWM_SLICE2>,
//     pin_4: Peri<'static, embassy_rp::peripherals::PIN_4>,
// ) {
//     let pwm_config = embassy_rp::pwm::Config::default();
//     let pwm_red = Pwm::new_output_a(pwm_slice1, pin_2, pwm_config.clone());
//     let pwm_green = Pwm::new_output_a(pwm_slice2, pin_config);
//     spawner.must_spawn(rgb_led_indicate(pwm_red, pwm_green));
// }

// /// Initialize all four RC button inputs
// fn init_rc_buttons(
//     spawner: &Spawner,
//     pin_10: Peri<'static, embassy_rp::peripherals::PIN_10>,
//     pin_16: Peri<'static, embassy_rp::peripherals::PIN_16>,
//     pin_11: Peri<'static, embassy_rp::peripherals::PIN_11>,
//     pin_17: Peri<'static, embassy_rp::peripherals::PIN_17>,
// ) {
//     let btn_a = Input::new(pin_10, Pull::Down);
//     spawner.must_spawn(rc_button_handle(btn_a, ButtonId::A));

//     let btn_b = Input::new(pin_16, Pull::Down);
//     spawner.must_spawn(rc_button_handle(btn_b, ButtonId::B));

//     let btn_c = Input::new(pin_11, Pull::Down);
//     spawner.must_spawn(rc_button_handle(btn_c, ButtonId::C));

//     let btn_d = Input::new(pin_17, Pull::Down);
//     spawner.must_spawn(rc_button_handle(btn_d, ButtonId::D));
// }

/// Initialize motor driver with PWM channels and encoders
/// Direction and standby control are handled via PCA9555 port expander
/// Encoders are passed to the encoder_read task for sensing
fn init_motor_driver(spawner: &Spawner, pins: MotorDriverPins) {
    let desired_freq_hz = 20_000u32;
    let clock_freq_hz = embassy_rp::clocks::clk_sys_freq();
    let divider = ((clock_freq_hz / desired_freq_hz) / 65535 + 1) as u8;
    let period = (clock_freq_hz / (desired_freq_hz * divider as u32)) as u16 - 1;
    let mut motor_pwm_config = embassy_rp::pwm::Config::default();
    motor_pwm_config.divider = divider.into();
    motor_pwm_config.top = period;

    // Create PWM slices for 4 motors (2 drivers × 2 motors each)
    // Each PWM slice supports 2 channels (A and B)
    // Left Driver: Channel A = Front motor (GPIO 0), Channel B = Rear motor (GPIO 1)
    // Right Driver: Channel A = Front motor (GPIO 2), Channel B = Rear motor (GPIO 3)

    // Left Driver: GPIO 0 and GPIO 1 on PWM_SLICE0 (channels A and B)
    let pwm_driver_left = Pwm::new_output_ab(
        pins.lmot_pwm_slice,
        pins.lmot_pwm_a,
        pins.lmot_pwm_b,
        motor_pwm_config.clone(),
    );

    // Right Driver: GPIO 2 and GPIO 3 on PWM_SLICE1 (channels A and B)
    let pwm_driver_right = Pwm::new_output_ab(pins.rmot_pwm_slice, pins.rmot_pwm_a, pins.rmot_pwm_b, motor_pwm_config);

    // Configure encoder PWM inputs for pulse counting
    let mut encoder_pwm_config = embassy_rp::pwm::Config::default();
    encoder_pwm_config.divider = 1.into();
    encoder_pwm_config.phase_correct = false;

    // Create encoder inputs (PWM input mode on B channels)
    // Encoder 1: Left Front Motor (GPIO 7, PWM_SLICE3 Channel B)
    let encoder_left_front = Pwm::new_input(
        pins.encoder_left_front_slice,
        pins.encoder_left_front_pin,
        Pull::None,
        InputMode::RisingEdge,
        encoder_pwm_config.clone(),
    );

    // Encoder 2: Left Rear Motor (GPIO 21, PWM_SLICE2 Channel B)
    let encoder_left_rear = Pwm::new_input(
        pins.encoder_left_rear_slice,
        pins.encoder_left_rear_pin,
        Pull::None,
        InputMode::RisingEdge,
        encoder_pwm_config.clone(),
    );

    // Encoder 3: Right Front Motor (GPIO 9, PWM_SLICE4 Channel B)
    let encoder_right_front = Pwm::new_input(
        pins.encoder_right_front_slice,
        pins.encoder_right_front_pin,
        Pull::None,
        InputMode::RisingEdge,
        encoder_pwm_config.clone(),
    );

    // Encoder 4: Right Rear Motor (GPIO 27, PWM_SLICE5 Channel B)
    let encoder_right_rear = Pwm::new_input(
        pins.encoder_right_rear_slice,
        pins.encoder_right_rear_pin,
        Pull::None,
        InputMode::RisingEdge,
        encoder_pwm_config,
    );

    // Spawn motor driver task (handles PWM + coordinates with port expander)
    spawner.must_spawn(task::motor_driver::motor_driver(pwm_driver_left, pwm_driver_right));

    // Spawn encoder read task (handles encoder sensing)
    spawner.must_spawn(task::encoder_read::encoder_read(
        encoder_left_front,
        encoder_left_rear,
        encoder_right_front,
        encoder_right_rear,
    ));

    // Spawn drive task (high-level drive control)
    spawner.must_spawn(task::drive::drive());
}

// /// Initialize IR obstacle detection
// fn init_ir_obstacle_detect(spawner: &Spawner, pin_26: Peri<'static, embassy_rp::peripherals::PIN_26>) {
//     let ir_right = Input::new(pin_26, Pull::Down);
//     spawner.must_spawn(ir_obstacle_detect(ir_right));
// }

// /// Initialize ultrasonic sensor sweep with servo
// fn init_ultrasonic_sweep(
//     spawner: &Spawner,
//     pio0: Peri<'static, PIO0>,
//     pin_5: Peri<'static, embassy_rp::peripherals::PIN_5>,
//     pin_15: Peri<'static, embassy_rp::peripherals::PIN_15>,
//     pin_14: Peri<'static, embassy_rp::peripherals::PIN_14>,
// ) {
//     let us_trigger = Output::new(pin_15, Level::Low);
//     let us_echo = Input::new(pin_14, Pull::None);
//     let sensor = sweep_ultrasonic::setup_ultrasonic_sensor(us_trigger, us_echo);
//     let servo = sweep_ultrasonic::setup_servo(pio0, pin_5);
//     spawner.must_spawn(ultrasonic_sweep(sensor, servo));
// }

/// Initialize shared I2C bus for display and IMU
fn init_i2c_bus(
    i2c0: Peri<'static, I2C0>,
    scl: Peri<'static, embassy_rp::peripherals::PIN_13>,
    sda: Peri<'static, embassy_rp::peripherals::PIN_12>,
) -> &'static I2cBusShared {
    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = 400_000;
    let i2c = I2c::new_async(i2c0, scl, sda, Irqs, i2c_config);
    static I2C_BUS: StaticCell<I2cBusShared> = StaticCell::new();
    I2C_BUS.init(Mutex::new(i2c))
}

/// Initialize display task with I2C bus
fn init_display(spawner: &Spawner, i2c_bus: &'static I2cBusShared) {
    spawner.must_spawn(task::display::display(i2c_bus));
}

/// Initialize IMU task with I2C bus and pins
fn init_imu_read(
    spawner: &Spawner,
    i2c_bus: &'static I2cBusShared,
    int: Peri<'static, embassy_rp::peripherals::PIN_18>,
    add: Peri<'static, embassy_rp::peripherals::PIN_19>,
) {
    spawner.must_spawn(task::imu_read::inertial_measurement_read(i2c_bus, int, add));
}

/// Initialize port expander task with I2C bus and interrupt pin
fn init_port_expander(
    spawner: &Spawner,
    i2c_bus: &'static I2cBusShared,
    int: Peri<'static, embassy_rp::peripherals::PIN_20>,
) {
    let interrupt = embassy_rp::gpio::Input::new(int, Pull::Up);
    spawner.must_spawn(task::port_expander::port_expander(i2c_bus, interrupt));
}

/// Initialize flash storage task for persistent calibration data
fn init_flash_storage(
    spawner: &Spawner,
    flash_peripheral: Peri<'static, embassy_rp::peripherals::FLASH>,
    dma_ch0: Peri<'static, embassy_rp::peripherals::DMA_CH0>,
) {
    let flash = Flash::<_, Async, { 2048 * 1024 }>::new(flash_peripheral, dma_ch0);
    spawner.must_spawn(task::flash_storage::flash_storage(flash));
}

// /// Initialize autonomous drive task
// fn init_autonomous_drive(spawner: &Spawner) {
//     spawner.must_spawn(autonomous_drive());
// }

// /// Initialize motion correction control task
// fn init_motion_correction(spawner: &Spawner) {
//     spawner.must_spawn(motion_correction_control());
// }

// /// Initialize inactivity tracker task
// fn init_inactivity_tracker(spawner: &Spawner) {
//     spawner.must_spawn(track_inactivity());
// }
