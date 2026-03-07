//! PCA9555 Port Expander task module
//!
//! This module provides an asynchronous task that manages a PCA9555 16-bit I2C I/O expander.
//! It provides a **generic pin control interface** - no motor-specific logic here!
//!
//! # Architecture
//!
//! The port expander operates as a generic I/O server:
//! 1. Listens for commands to set output pin states (generic bit manipulation)
//! 2. Monitors interrupt line for input changes
//! 3. Sends events when input states change
//!
//! # Pin Assignments (Hardware Configuration)
//!
//! ## Port 0 (All Outputs)
//! - Bits 0-7: General purpose outputs (currently used for motor direction pins)
//!
//! ## Port 1 (Mixed I/O)
//! - Bits 0-3: Inputs - RC Receiver Buttons A, B, C, D
//! - Bits 4-5: Outputs - General purpose (currently used for motor driver enables)
//! - Bit 6: Input - IR obstacle detect
//! - Bit 7: Output - Available for future expansion
//!
//! # Usage
//!
//! ```rust
//! // Set individual output pin
//! port_expander::send_command(PortExpanderCommand::OutputPin {
//!     port: PortNumber::Port0,
//!     pin: 0,
//!     state: true,
//! }).await;
//!
//! // Set multiple pins at once (more efficient)
//! port_expander::send_command(PortExpanderCommand::OutputByte {
//!     port: PortNumber::Port0,
//!     value: 0b10101010,
//! }).await;
//!
//! // Set specific bits using mask
//! port_expander::send_command(PortExpanderCommand::OutputBits {
//!     port: PortNumber::Port0,
//!     mask: 0b00001111,  // Affect bits 0-3
//!     value: 0b00001010, // Set bits 1 and 3 high
//! }).await;
//! ```

use defmt::{Format, debug, error, info};
use embassy_futures::select::{Either, select};
use embassy_rp::gpio::Input;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Timer};
use embedded_hal_async::i2c::I2c;

use crate::{
    I2cBusShared,
    system::event::{Events, RCButtonId, raise_event},
    task::{control::rotary_encoder::trigger_button_signal, sensors::ir_obstacle::signal_ir_obstacle},
};

/// PCA9555 I2C address (A0=high, A1=A2=low -> 0x21)
const PCA9555_ADDR: u8 = 0x21;

// Input register for Port 0. Not used ATM
// const REG_INPUT_PORT0: u8 = 0x00;
/// Input register for Port 1.
const REG_INPUT_PORT1: u8 = 0x01;
/// Output register for Port 0.
const REG_OUTPUT_PORT0: u8 = 0x02;
/// Output register for Port 1.
const REG_OUTPUT_PORT1: u8 = 0x03;
/// Configuration register for Port 0.
const REG_CONFIG_PORT0: u8 = 0x06;
/// Configuration register for Port 1.
const REG_CONFIG_PORT1: u8 = 0x07;

/// Command channel for port expander operations
static COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, PortExpanderOutputCommand, 16> = Channel::new();

/// Send a command to the port expander task
pub async fn send_command(command: PortExpanderOutputCommand) {
    COMMAND_CHANNEL.sender().send(command).await;
}

/// Port selection
#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub enum PortNumber {
    /// Port 0 (pins 0-7).
    Port0,
    /// Port 1 (pins 0-7).
    Port1,
}

/// Generic port expander commands - no motor-specific knowledge
#[derive(Debug, Clone, Copy, Format)]
pub enum PortExpanderOutputCommand {
    /// Set a single output pin state
    Pin {
        /// Target port.
        port: PortNumber,
        /// Pin index (0-7).
        pin: u8,
        /// Desired output state.
        state: bool,
    },

    /// Set entire port output byte at once
    Byte {
        /// Target port.
        port: PortNumber,
        /// Output value for all 8 pins.
        value: u8,
    },

    /// Set multiple bits using mask (efficient for updating specific bits)
    /// Only bits where mask=1 are affected
    Bits {
        /// Target port.
        port: PortNumber,
        /// Bitmask of pins to update.
        mask: u8,
        /// Values for masked pins.
        value: u8,
    },
    // Set both ports at once (atomic, most efficient)
    // BothPorts {
    //     /// Output value for port 0.
    //     port0: u8,
    //     /// Output value for port 1.
    //     port1: u8,
    // },
}

/// Input state tracking for Port 1
#[allow(clippy::struct_excessive_bools)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
struct InputState {
    /// RC Button A (active-low in hardware).
    rc_button_a: bool,
    /// RC Button B (active-low in hardware).
    rc_button_b: bool,
    /// RC Button C (active-low in hardware).
    rc_button_c: bool,
    /// RC Button D (active-low in hardware).
    rc_button_d: bool,
    /// IR obstacle signal (active-high in hardware).
    ir_obstacle: bool,
    /// Rotary encoder push button (active-low in hardware).
    rotary_encoder_button: bool,
}

impl InputState {
    /// Create from Port 1 input register value
    /// Note: RC buttons are inverted by NOT gates in hardware (active-LOW)
    const fn from_port1(value: u8) -> Self {
        Self {
            rc_button_a: (value & 0b0000_0001) == 0, // Inverted: LOW = pressed
            rc_button_b: (value & 0b0000_0010) == 0, // Inverted: LOW = pressed
            rc_button_c: (value & 0b0000_0100) == 0, // Inverted: LOW = pressed
            rc_button_d: (value & 0b0000_1000) == 0, // Inverted: LOW = pressed
            // IR sensor output is inverted in hardware: HIGH = obstacle detected
            ir_obstacle: (value & 0b0100_0000) != 0,
            // Rotary encoder button is inverted: LOW = pressed
            rotary_encoder_button: (value & 0b1000_0000) == 0,
        }
    }

    /// Compare and send events for changed inputs
    async fn send_changes(&self, previous: &Self) {
        if self.rc_button_a != previous.rc_button_a && self.rc_button_a {
            raise_event(Events::RCButtonPressed(RCButtonId::A)).await;
        }
        if self.rc_button_b != previous.rc_button_b && self.rc_button_b {
            raise_event(Events::RCButtonPressed(RCButtonId::B)).await;
        }
        if self.rc_button_c != previous.rc_button_c && self.rc_button_c {
            raise_event(Events::RCButtonPressed(RCButtonId::C)).await;
        }
        if self.rc_button_d != previous.rc_button_d && self.rc_button_d {
            raise_event(Events::RCButtonPressed(RCButtonId::D)).await;
        }
        if self.ir_obstacle != previous.ir_obstacle {
            signal_ir_obstacle(self.ir_obstacle).await;
        }
        if self.rotary_encoder_button != previous.rotary_encoder_button {
            trigger_button_signal(self.rotary_encoder_button).await;
        }
    }
}

/// Port expander state - just raw port values
struct PortExpanderState {
    /// Current output state for Port 0
    port0_output: u8,
    /// Current output state for Port 1
    port1_output: u8,
    /// Last known input state
    last_input_state: InputState,
}

impl PortExpanderState {
    /// Create new state with default values (all outputs low, no inputs pressed)
    const fn new() -> Self {
        Self {
            port0_output: 0x00,
            port1_output: 0x00, // No pull-ups - external pull-downs needed for RC inputs
            last_input_state: InputState {
                rc_button_a: false,
                rc_button_b: false,
                rc_button_c: false,
                rc_button_d: false,
                ir_obstacle: false,
                rotary_encoder_button: false,
            },
        }
    }

    /// Set a single output pin
    const fn set_output_pin(&mut self, port: PortNumber, pin: u8, state: bool) {
        if pin > 7 {
            return;
        }

        let port_value = match port {
            PortNumber::Port0 => &mut self.port0_output,
            PortNumber::Port1 => &mut self.port1_output,
        };

        if state {
            *port_value |= 1 << pin;
        } else {
            *port_value &= !(1 << pin);
        }
    }

    /// Set entire port output byte
    const fn set_output_byte(&mut self, port: PortNumber, value: u8) {
        match port {
            PortNumber::Port0 => self.port0_output = value,
            PortNumber::Port1 => self.port1_output = value,
        }
    }

    /// Set specific bits using mask
    const fn set_output_bits(&mut self, port: PortNumber, mask: u8, value: u8) {
        let port_value = match port {
            PortNumber::Port0 => &mut self.port0_output,
            PortNumber::Port1 => &mut self.port1_output,
        };

        // Clear bits specified by mask, then set new values
        *port_value = (*port_value & !mask) | (value & mask);
    }

    /// Set both ports at once
    const fn set_both_ports(&mut self, port0: u8, port1: u8) {
        self.port0_output = port0;
        self.port1_output = port1;
    }
}

/// Simple PCA9555 driver using shared I2C bus
struct Pca9555Driver {
    /// Shared I2C bus
    i2c_bus: &'static I2cBusShared,
}

impl Pca9555Driver {
    /// Create new driver instance with shared I2C bus
    const fn new(i2c_bus: &'static I2cBusShared) -> Self {
        Self { i2c_bus }
    }

    /// Initialize the PCA9555
    async fn init(&self) -> Result<(), ()> {
        // Configure Port 0: All outputs
        self.write_register(REG_CONFIG_PORT0, 0x00).await?;

        // Configure Port 1: Bits 0-3 inputs (buttons), bits 6-7 inputs (IR + encoder button), 4-5 outputs
        self.write_register(REG_CONFIG_PORT1, 0xCF).await?;

        // Initialize outputs to safe state (all low)
        self.write_register(REG_OUTPUT_PORT0, 0x00).await?;
        self.write_register(REG_OUTPUT_PORT1, 0x00).await?;

        info!("PCA9555 initialized");
        Ok(())
    }

    /// Write to a register
    async fn write_register(&self, register: u8, value: u8) -> Result<(), ()> {
        let mut i2c = self.i2c_bus.lock().await;
        i2c.write(PCA9555_ADDR, &[register, value]).await.map_err(|_| {
            error!("PCA9555 write error");
        })
    }

    /// Read from a register
    async fn read_register(&self, register: u8) -> Result<u8, ()> {
        let mut buf = [0u8; 1];
        let result = {
            let mut i2c = self.i2c_bus.lock().await;
            i2c.write_read(PCA9555_ADDR, &[register], &mut buf).await
        };
        result.map_err(|_| {
            error!("PCA9555 read error");
        })?;
        Ok(buf[0])
    }

    /// Write both output ports in ONE I2C transaction using auto-increment
    async fn write_outputs(&self, port0: u8, port1: u8) -> Result<(), ()> {
        let mut i2c = self.i2c_bus.lock().await;
        // Auto-increment: write to REG_OUTPUT_PORT0, then automatically to REG_OUTPUT_PORT1
        i2c.write(PCA9555_ADDR, &[REG_OUTPUT_PORT0, port0, port1])
            .await
            .map_err(|_| {
                error!("PCA9555 bulk write error");
            })
    }

    /// Read input state from Port 1
    async fn read_inputs(&self) -> Result<u8, ()> {
        self.read_register(REG_INPUT_PORT1).await
    }
}

/// Main port expander task
#[embassy_executor::task]
pub async fn port_expander(i2c_bus: &'static I2cBusShared, mut interrupt: Input<'static>) {
    info!("Port expander task starting");

    // Initialize hardware
    let driver = Pca9555Driver::new(i2c_bus);

    // Initialize the device
    if (driver.init().await).is_err() {
        error!("Failed to initialize PCA9555, task halting");
        return;
    }

    // Initialize state
    let mut state = PortExpanderState::new();

    // Read initial input state
    match driver.read_inputs().await {
        Ok(value) => {
            state.last_input_state = InputState::from_port1(value);
            debug!("Initial input state: {:?}", state.last_input_state);
            signal_ir_obstacle(state.last_input_state.ir_obstacle).await;
        }
        Err(()) => {
            error!("Failed to read initial input state");
        }
    }

    info!("Port expander initialized successfully");

    // Main task loop
    let receiver = COMMAND_CHANNEL.receiver();
    loop {
        // Wait for either a command or an interrupt
        let command_fut = receiver.receive();
        let interrupt_fut = interrupt.wait_for_low();
        match select(command_fut, interrupt_fut).await {
            // Command received
            Either::First(command) => {
                // debug!("Processing command: {:?}", command);
                if (process_command(&mut state, &driver, command).await).is_err() {
                    error!("Failed to process command");
                }
            }

            // Interrupt triggered (input changed)
            Either::Second(()) => {
                debug!("Interrupt detected, reading inputs");

                // Small delay to debounce
                Timer::after(Duration::from_millis(10)).await;

                match driver.read_inputs().await {
                    Ok(value) => {
                        let new_state = InputState::from_port1(value);
                        debug!("New input state: {:?}", new_state);

                        // Send events for changes
                        new_state.send_changes(&state.last_input_state).await;

                        // Update state
                        state.last_input_state = new_state;
                    }
                    Err(()) => {
                        error!("Failed to read inputs after interrupt");
                    }
                }
            }
        }
    }
}

/// Process a generic pin command and update hardware
async fn process_command(
    state: &mut PortExpanderState,
    driver: &Pca9555Driver,
    command: PortExpanderOutputCommand,
) -> Result<(), ()> {
    match command {
        PortExpanderOutputCommand::Pin {
            port,
            pin,
            state: pin_state,
        } => {
            state.set_output_pin(port, pin, pin_state);
        }

        PortExpanderOutputCommand::Byte { port, value } => {
            state.set_output_byte(port, value);
        }

        PortExpanderOutputCommand::Bits { port, mask, value } => {
            state.set_output_bits(port, mask, value);
        } // not currently used
          // PortExpanderCommand::BothPorts { port0, port1 } => {
          //     state.set_both_ports(port0, port1);
          // }
    }

    // Write the updated state to hardware
    driver.write_outputs(state.port0_output, state.port1_output).await?;

    Ok(())
}
