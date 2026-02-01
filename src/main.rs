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
    gpio::{Input, Level, Output, Pull},
    i2c::{Config as I2cConfig, I2c, InterruptHandler as I2cInterruptHandler},
    peripherals::{I2C0, PIO0, PIO1},
    pio::{InterruptHandler as PioInterruptHandler, Pio},
    pio_programs::{
        pwm::{PioPwm, PioPwmProgram},
        rotary_encoder::{PioEncoder, PioEncoderProgram},
    },
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
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    PIO1_IRQ_0 => PioInterruptHandler<PIO1>;
});

/// Motor driver peripheral pins (for new architecture with PCA9555)
/// Standby pins are now controlled via PCA9555 port expander (Port 1, bits 4-5)
struct MotorDriverPins {
    /// Left motor driver PWM slice
    lmot_pwm_slice: Peri<'static, embassy_rp::peripherals::PWM_SLICE0>,
    /// Left motor driver PWM channel A pin
    lmot_pwm_a: Peri<'static, embassy_rp::peripherals::PIN_0>,
    /// Left motor driver PWM channel B pin
    lmot_pwm_b: Peri<'static, embassy_rp::peripherals::PIN_1>,
    /// Right motor driver PWM slice
    rmot_pwm_slice: Peri<'static, embassy_rp::peripherals::PWM_SLICE1>,
    /// Right motor driver PWM channel A pin
    rmot_pwm_a: Peri<'static, embassy_rp::peripherals::PIN_2>,
    /// Right motor driver PWM channel B pin
    rmot_pwm_b: Peri<'static, embassy_rp::peripherals::PIN_3>,
    /// Left front motor encoder PWM slice
    encoder_left_front_slice: Peri<'static, embassy_rp::peripherals::PWM_SLICE3>,
    /// Left front motor encoder PWM pin
    encoder_left_front_pin: Peri<'static, embassy_rp::peripherals::PIN_7>,
    /// Left rear motor encoder PWM slice
    encoder_left_rear_slice: Peri<'static, embassy_rp::peripherals::PWM_SLICE2>,
    /// Left rear motor encoder PWM pin
    encoder_left_rear_pin: Peri<'static, embassy_rp::peripherals::PIN_21>,
    /// Right front motor encoder PWM slice
    encoder_right_front_slice: Peri<'static, embassy_rp::peripherals::PWM_SLICE4>,
    /// Right front motor encoder PWM pin
    encoder_right_front_pin: Peri<'static, embassy_rp::peripherals::PIN_9>,
    /// Right rear motor encoder PWM slice
    encoder_right_rear_slice: Peri<'static, embassy_rp::peripherals::PWM_SLICE5>,
    /// Right rear motor encoder PWM pin
    encoder_right_rear_pin: Peri<'static, embassy_rp::peripherals::PIN_27>,
}

/// RGB LED indicator pins
struct RgbLedPins {
    /// Red channel pin
    red: Peri<'static, embassy_rp::peripherals::PIN_28>,
    /// Green channel pin
    green: Peri<'static, embassy_rp::peripherals::PIN_22>,
    /// Blue channel pin
    blue: Peri<'static, embassy_rp::peripherals::PIN_19>,
}

/// EC11 rotary encoder pins
struct Ec11Pins {
    /// Pin A
    a: Peri<'static, embassy_rp::peripherals::PIN_10>,
    /// Pin B
    b: Peri<'static, embassy_rp::peripherals::PIN_11>,
    /// Button pin
    // ToDo: route through port expander!
    button: Peri<'static, embassy_rp::peripherals::PIN_24>,
}

/// RC Control button pins
struct RCButtonPins {
    /// Button A pin
    a: Peri<'static, embassy_rp::peripherals::PIN_4>,
    /// Button B pin
    b: Peri<'static, embassy_rp::peripherals::PIN_5>,
    /// Button C pin
    c: Peri<'static, embassy_rp::peripherals::PIN_6>,
    /// Button D pin
    d: Peri<'static, embassy_rp::peripherals::PIN_8>,
}

/// Ultrasonic sensor sweep pins
struct UltrasonicSweepPins {
    /// Servo control pin
    servo: Peri<'static, embassy_rp::peripherals::PIN_14>,
    /// HC-SR04 trigger pin
    trigger: Peri<'static, embassy_rp::peripherals::PIN_15>,
    /// HC-SR04 echo pin
    echo: Peri<'static, embassy_rp::peripherals::PIN_18>,
}

/// Grove Vision AI v2 module UART pins
struct GroveVisionUartPins {
    /// UART TX pin
    tx: Peri<'static, embassy_rp::peripherals::PIN_16>,
    /// UART RX pin
    rx: Peri<'static, embassy_rp::peripherals::PIN_17>,
}

/// Ultrasonic Sweep Pins
struct UltrasonicPins {
    /// Servo control pin
    servo: Peri<'static, embassy_rp::peripherals::PIN_18>,
    /// Trigger pin
    trigger: Peri<'static, embassy_rp::peripherals::PIN_15>,
    /// Echo pin
    echo: Peri<'static, embassy_rp::peripherals::PIN_14>,
}

/// Public type for shared I2C bus
pub type I2cBusShared = Mutex<CriticalSectionRawMutex, I2c<'static, I2C0, embassy_rp::i2c::Async>>;

/// Firmware entry point
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Configure rp2350 to use the external oscillator and run at its usual 150Mhz
    let mut config = Config::default();
    config.clocks = embassy_rp::clocks::ClockConfig::system_freq(150_000_000)
        .unwrap_or_else(|e| defmt::panic!("Failed to configure system clocks {}", e));
    // get a peripheral handle
    let p = embassy_rp::init(config);

    // make peripheral handles and spawn tasks

    // Initialize shared I2C bus
    let i2c_bus = init_i2c_bus(p.I2C0, p.PIN_13, p.PIN_12);

    // Initialize PIO1 for various tasks
    let Pio {
        mut common,
        sm0,
        sm1,
        sm2,
        sm3,
        ..
    } = Pio::new(p.PIO1, Irqs);

    // Initialize PIO0 for ultrasonic sweep
    let Pio {
        common: mut us_common,
        sm0: us_sm0,
        ..
    } = Pio::new(p.PIO0, Irqs);

    // Initialize core tasks
    // Orchestrator
    init_orchestrate(spawner);
    // Battery monitoring
    init_battery_monitoring(spawner, p.ADC, p.PIN_26);
    // Display
    init_display(spawner, i2c_bus);

    // Initialize port expander task
    init_port_expander(spawner, i2c_bus, p.PIN_20);

    // Initialize the rgb indicattor led
    init_rgb_led(
        spawner,
        &mut common,
        sm0,
        sm1,
        sm2,
        RgbLedPins {
            red: p.PIN_28,
            green: p.PIN_22,
            blue: p.PIN_19,
        },
    );

    // Initialize the rotary encoder
    init_rotary_encoder(
        spawner,
        &mut common,
        sm3,
        Ec11Pins {
            a: p.PIN_10,
            b: p.PIN_11,
            button: p.PIN_24,
        },
    );

    // initialize the RC Control buttons
    init_rc_buttons(
        spawner,
        RCButtonPins {
            a: p.PIN_4,
            b: p.PIN_5,
            c: p.PIN_6,
            d: p.PIN_8,
        },
    );

    // Initialize motor driver and motor encoders
    init_motor_driver(
        spawner,
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
    init_ir_obstacle_detect(spawner);

    // Initialize the ultrasonic sweep task
    init_ultrasonic_sweep(
        spawner,
        &mut us_common,
        us_sm0,
        UltrasonicPins {
            servo: p.PIN_18,
            trigger: p.PIN_15,
            echo: p.PIN_14,
        },
    );

    // Initialize IMU task
    init_imu_read(spawner, i2c_bus);

    // Initialize flash storage task
    init_flash_storage(spawner, p.FLASH, p.DMA_CH0);

    // init_motion_correction(&spawner);
    // init_inactivity_tracker(&spawner);

    // Initialize testing task for development
    task::testing::init_testing(&spawner);

    // main wishes you a great day
}

/// Initialize orchestrator task
fn init_orchestrate(spawner: Spawner) {
    spawner.must_spawn(task::orchestrate::orchestrate());
}

/// Initialize battery monitoring task with ADC
fn init_battery_monitoring(
    spawner: Spawner,
    adc: Peri<'static, embassy_rp::peripherals::ADC>,
    adc_pin: Peri<'static, embassy_rp::peripherals::PIN_26>,
) {
    let adc = Adc::new(adc, Irqs, AdcConfig::default());
    let battery_channel = Channel::new_pin(adc_pin, Pull::None);
    spawner.must_spawn(task::battery_charge_read::battery_charge_read(adc, battery_channel));
}

/// Initialize RGB LED indicator with PIO PWM outputs
fn init_rgb_led(
    spawner: Spawner,
    common: &mut embassy_rp::pio::Common<'static, PIO1>,
    sm0: embassy_rp::pio::StateMachine<'static, PIO1, 0>,
    sm1: embassy_rp::pio::StateMachine<'static, PIO1, 1>,
    sm2: embassy_rp::pio::StateMachine<'static, PIO1, 2>,
    pins: RgbLedPins,
) {
    let rgb_program = PioPwmProgram::new(common);
    let pwm_red = PioPwm::new(common, sm0, pins.red, &rgb_program);
    let pwm_green = PioPwm::new(common, sm1, pins.green, &rgb_program);
    let pwm_blue = PioPwm::new(common, sm2, pins.blue, &rgb_program);

    spawner.must_spawn(task::rgb_led_indicate::rgb_led_indicate(pwm_red, pwm_green, pwm_blue));
}

/// Initialize EC11 rotary encoder (PIO quadrature + button input)
fn init_rotary_encoder(
    spawner: Spawner,
    common: &mut embassy_rp::pio::Common<'static, PIO1>,
    sm3: embassy_rp::pio::StateMachine<'static, PIO1, 3>,
    pins: Ec11Pins,
) {
    let encoder_program = PioEncoderProgram::new(common);
    let encoder = PioEncoder::new(common, sm3, pins.a, pins.b, &encoder_program);
    let encoder_button = Input::new(pins.button, Pull::Up);

    spawner.must_spawn(task::rotary_encoder::rotary_encoder_turns(encoder));
    spawner.must_spawn(task::rotary_encoder::rotary_encoder_button(encoder_button));
}

/// Initialize all four RC button inputs
fn init_rc_buttons(spawner: Spawner, pins: RCButtonPins) {
    // let btn_a = Input::new(btn_a, Pull::Down);
    // spawner.must_spawn(rc_button_handle(btn_a, ButtonId::A));

    // let btn_b = Input::new(btn_b, Pull::Down);
    // spawner.must_spawn(rc_button_handle(btn_b, ButtonId::B));

    // let btn_c = Input::new(btn_c, Pull::Down);
    // spawner.must_spawn(rc_button_handle(btn_c, ButtonId::C));

    // let btn_d = Input::new(btn_d, Pull::Down);
    // spawner.must_spawn(rc_button_handle(btn_d, ButtonId::D));
}

/// Initialize motor driver with PWM channels and encoders
/// Direction and standby control are handled via PCA9555 port expander
/// Encoders are passed to the `encoder_read` task for sensing
#[allow(clippy::cast_possible_truncation)]
fn init_motor_driver(spawner: Spawner, pins: MotorDriverPins) {
    let desired_freq_hz = 20_000u32;
    let clock_freq_hz = embassy_rp::clocks::clk_sys_freq();
    let divider = ((clock_freq_hz / desired_freq_hz) / 65535 + 1) as u8;
    let period = (clock_freq_hz / (desired_freq_hz * u32::from(divider))) as u16 - 1;
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

/// Initialize IR obstacle detection (signaled by port expander)
fn init_ir_obstacle_detect(spawner: Spawner) {
    spawner.must_spawn(task::ir_obstacle_detect::ir_obstacle_detect());
}

/// Initialize ultrasonic sensor sweep with servo
fn init_ultrasonic_sweep(
    spawner: Spawner,
    us_common: &mut embassy_rp::pio::Common<'static, PIO0>,
    us_sm0: embassy_rp::pio::StateMachine<'static, PIO0, 0>,
    pins: UltrasonicPins,
) {
    let us_trigger = Output::new(pins.trigger, Level::Low);
    let us_echo = Input::new(pins.echo, Pull::None);

    let us_pwm_program = PioPwmProgram::new(us_common);
    let us_pwm = PioPwm::new(us_common, us_sm0, pins.servo, &us_pwm_program);

    spawner.must_spawn(task::sweep_ultrasonic::ultrasonic_sweep(us_pwm, us_trigger, us_echo));
}

/// Initialize shared I2C bus for display and IMU
fn init_i2c_bus(
    i2c0: Peri<'static, I2C0>,
    scl: Peri<'static, embassy_rp::peripherals::PIN_13>,
    sda: Peri<'static, embassy_rp::peripherals::PIN_12>,
) -> &'static I2cBusShared {
    static I2C_BUS: StaticCell<I2cBusShared> = StaticCell::new();

    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = 400_000;

    let i2c = I2c::new_async(i2c0, scl, sda, Irqs, i2c_config);

    I2C_BUS.init(Mutex::new(i2c))
}

/// Initialize display task with I2C bus
fn init_display(spawner: Spawner, i2c_bus: &'static I2cBusShared) {
    spawner.must_spawn(task::display::display(i2c_bus));
}

/// Initialize IMU task with I2C bus and interrupt input
fn init_imu_read(spawner: Spawner, i2c_bus: &'static I2cBusShared) {
    spawner.must_spawn(task::imu_read::inertial_measurement_read(i2c_bus));
}

/// Initialize port expander task with I2C bus and interrupt pin
fn init_port_expander(
    spawner: Spawner,
    i2c_bus: &'static I2cBusShared,
    int: Peri<'static, embassy_rp::peripherals::PIN_20>,
) {
    let interrupt = embassy_rp::gpio::Input::new(int, Pull::Up);
    spawner.must_spawn(task::port_expander::port_expander(i2c_bus, interrupt));
}

/// Initialize flash storage task for persistent calibration data
fn init_flash_storage(
    spawner: Spawner,
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
