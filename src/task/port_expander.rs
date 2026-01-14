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
//! - Bits 6-7: Outputs - Available for future expansion
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
    system::event::{ButtonId, Events, send_event},
};

/// PCA9555 I2C address (A0=A1=A2=low -> 0x20)
const PCA9555_ADDR: u8 = 0x20;

/// PCA9555 Register addresses
const REG_INPUT_PORT0: u8 = 0x00;
const REG_INPUT_PORT1: u8 = 0x01;
const REG_OUTPUT_PORT0: u8 = 0x02;
const REG_OUTPUT_PORT1: u8 = 0x03;
const REG_CONFIG_PORT0: u8 = 0x06;
const REG_CONFIG_PORT1: u8 = 0x07;

/// Command channel for port expander operations
static COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, PortExpanderCommand, 16> = Channel::new();

/// Send a command to the port expander task
pub async fn send_command(command: PortExpanderCommand) {
    COMMAND_CHANNEL.sender().send(command).await;
}

/// Port selection
#[derive(Debug, Clone, Copy, PartialEq, Format)]
pub enum PortNumber {
    Port0,
    Port1,
}

/// Generic port expander commands - no motor-specific knowledge
#[derive(Debug, Clone, Copy, Format)]
pub enum PortExpanderCommand {
    /// Set a single output pin state
    OutputPin {
        port: PortNumber,
        pin: u8, // 0-7
        state: bool,
    },

    /// Set entire port output byte at once
    OutputByte { port: PortNumber, value: u8 },

    /// Set multiple bits using mask (efficient for updating specific bits)
    /// Only bits where mask=1 are affected
    OutputBits { port: PortNumber, mask: u8, value: u8 },

    /// Set both ports at once (atomic, most efficient)
    BothPorts { port0: u8, port1: u8 },
}

/// Input state tracking for Port 1
#[derive(Debug, Clone, Copy, PartialEq, Format)]
struct InputState {
    button_a: bool,
    button_b: bool,
    button_c: bool,
    button_d: bool,
}

impl InputState {
    /// Create from Port 1 input register value
    fn from_port1(value: u8) -> Self {
        Self {
            button_a: (value & 0b0000_0001) != 0,
            button_b: (value & 0b0000_0010) != 0,
            button_c: (value & 0b0000_0100) != 0,
            button_d: (value & 0b0000_1000) != 0,
        }
    }

    /// Compare and send events for changed inputs
    async fn send_changes(&self, previous: &InputState) {
        if self.button_a != previous.button_a && self.button_a {
            send_event(Events::ButtonPressed(ButtonId::A)).await;
        }
        if self.button_b != previous.button_b && self.button_b {
            send_event(Events::ButtonPressed(ButtonId::B)).await;
        }
        if self.button_c != previous.button_c && self.button_c {
            send_event(Events::ButtonPressed(ButtonId::C)).await;
        }
        if self.button_d != previous.button_d && self.button_d {
            send_event(Events::ButtonPressed(ButtonId::D)).await;
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
    fn new() -> Self {
        Self {
            port0_output: 0x00,
            port1_output: 0x00,
            last_input_state: InputState {
                button_a: false,
                button_b: false,
                button_c: false,
                button_d: false,
            },
        }
    }

    /// Set a single output pin
    fn set_output_pin(&mut self, port: PortNumber, pin: u8, state: bool) {
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
    fn set_output_byte(&mut self, port: PortNumber, value: u8) {
        match port {
            PortNumber::Port0 => self.port0_output = value,
            PortNumber::Port1 => self.port1_output = value,
        }
    }

    /// Set specific bits using mask
    fn set_output_bits(&mut self, port: PortNumber, mask: u8, value: u8) {
        let port_value = match port {
            PortNumber::Port0 => &mut self.port0_output,
            PortNumber::Port1 => &mut self.port1_output,
        };

        // Clear bits specified by mask, then set new values
        *port_value = (*port_value & !mask) | (value & mask);
    }

    /// Set both ports at once
    fn set_both_ports(&mut self, port0: u8, port1: u8) {
        self.port0_output = port0;
        self.port1_output = port1;
    }
}

/// Simple PCA9555 driver using shared I2C bus
struct Pca9555Driver {
    i2c_bus: &'static I2cBusShared,
}

impl Pca9555Driver {
    fn new(i2c_bus: &'static I2cBusShared) -> Self {
        Self { i2c_bus }
    }

    /// Initialize the PCA9555
    async fn init(&mut self) -> Result<(), ()> {
        // Configure Port 0: All outputs
        self.write_register(REG_CONFIG_PORT0, 0x00).await?;

        // Configure Port 1: Bits 0-3 inputs (buttons), 4-7 outputs
        self.write_register(REG_CONFIG_PORT1, 0x0F).await?;

        // Initialize outputs to safe state (all low)
        self.write_register(REG_OUTPUT_PORT0, 0x00).await?;
        self.write_register(REG_OUTPUT_PORT1, 0x00).await?;

        info!("PCA9555 initialized");
        Ok(())
    }

    /// Write to a register
    async fn write_register(&mut self, register: u8, value: u8) -> Result<(), ()> {
        let mut i2c = self.i2c_bus.lock().await;
        i2c.write(PCA9555_ADDR, &[register, value]).await.map_err(|_| {
            error!("PCA9555 write error");
        })
    }

    /// Read from a register
    async fn read_register(&mut self, register: u8) -> Result<u8, ()> {
        let mut buf = [0u8; 1];
        let mut i2c = self.i2c_bus.lock().await;
        i2c.write_read(PCA9555_ADDR, &[register], &mut buf).await.map_err(|_| {
            error!("PCA9555 read error");
        })?;
        Ok(buf[0])
    }

    /// Write both output ports in ONE I2C transaction using auto-increment
    async fn write_outputs(&mut self, port0: u8, port1: u8) -> Result<(), ()> {
        let mut i2c = self.i2c_bus.lock().await;
        // Auto-increment: write to REG_OUTPUT_PORT0, then automatically to REG_OUTPUT_PORT1
        i2c.write(PCA9555_ADDR, &[REG_OUTPUT_PORT0, port0, port1])
            .await
            .map_err(|_| {
                error!("PCA9555 bulk write error");
            })
    }

    /// Read input state from Port 1
    async fn read_inputs(&mut self) -> Result<u8, ()> {
        self.read_register(REG_INPUT_PORT1).await
    }
}

/// Main port expander task
#[embassy_executor::task]
pub async fn port_expander(i2c_bus: &'static I2cBusShared, mut interrupt: Input<'static>) {
    info!("Port expander task starting");

    // Initialize hardware
    let mut driver = Pca9555Driver::new(i2c_bus);

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
        }
        Err(_) => {
            error!("Failed to read initial input state");
        }
    }

    info!("Port expander initialized successfully");

    // Main task loop
    loop {
        // Wait for either a command or an interrupt
        match select(COMMAND_CHANNEL.receiver().receive(), interrupt.wait_for_low()).await {
            // Command received
            Either::First(command) => {
                debug!("Processing command: {:?}", command);
                if (process_command(&mut state, &mut driver, command).await).is_err() {
                    error!("Failed to process command");
                }
            }

            // Interrupt triggered (input changed)
            Either::Second(_) => {
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
                    Err(_) => {
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
    driver: &mut Pca9555Driver,
    command: PortExpanderCommand,
) -> Result<(), ()> {
    match command {
        PortExpanderCommand::OutputPin {
            port,
            pin,
            state: pin_state,
        } => {
            state.set_output_pin(port, pin, pin_state);
        }

        PortExpanderCommand::OutputByte { port, value } => {
            state.set_output_byte(port, value);
        }

        PortExpanderCommand::OutputBits { port, mask, value } => {
            state.set_output_bits(port, mask, value);
        }

        PortExpanderCommand::BothPorts { port0, port1 } => {
            state.set_both_ports(port0, port1);
        }
    }

    // Write the updated state to hardware
    driver.write_outputs(state.port0_output, state.port1_output).await?;

    Ok(())
}
