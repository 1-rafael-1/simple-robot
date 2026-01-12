# Pin Assignment Table for Simple Robot (Raspberry Pi Pico2)

## Overview
This document defines the GPIO pin assignments for the RP2350-based robot (Pico2 board) with 4 motors, PCA9555 port expander, and additional sensors.

**Hardware Platform**: Raspberry Pi Pico2 (RP2350, QFN-60 package)

## Pin Assignment Table

| GPIO | Function | Direction | PWM Slice | Notes |
|------|----------|-----------|-----------|-------|
| **0** | Motor Driver 1 - Left PWM | Output | 0A | Motor control |
| **1** | Motor Driver 1 - Right PWM | Output | 0B | Motor control |
| **2** | Motor Driver 2 - Left PWM | Output | 1A | Motor control |
| **3** | Motor Driver 2 - Right PWM | Output | 1B | Motor control |
| **4** | RGB LED - Red | Output | - | Via PIO0 (software PWM) |
| **5** | RGB LED - Green | Output | - | Via PIO0 (software PWM) |
| **6** | *Reserved* | - | - | Future expansion |
| **7** | Encoder 1 (Motor 1 Left) | Input | 3B | Frequency measurement mode |
| **8** | *Reserved* | - | - | Future expansion |
| **9** | Encoder 2 (Motor 1 Right) | Input | 4B | Frequency measurement mode |
| **10** | Servo Control (Ultrasonic Sweep) | Output | - | Via PIO0 |
| **11** | UART0 TX (Grove Vision AI V2) | Output | - | Serial communication |
| **12** | I2C0 SCL | Bidir | - | Shared I2C bus |
| **13** | I2C0 SDA | Bidir | - | Shared I2C bus |
| **14** | HC-SR04 Echo | Input | - | Ultrasonic sensor |
| **15** | HC-SR04 Trigger | Output | - | Ultrasonic sensor |
| **16** | IR Obstacle Sensor 1 | Input | - | Digital obstacle detection |
| **17** | IR Obstacle Sensor 2 | Input | - | Digital obstacle detection |
| **18** | IMU Address Select | Output | - | LSM6DSO32 AD0/SA0 |
| **19** | IMU Interrupt | Input | - | LSM6DSO32 INT1 |
| **20** | PCA9555 Interrupt | Input | - | Port expander interrupt |
| **21** | Encoder 3 (Motor 2 Left) | Input | 2B | Frequency measurement mode |
| **22** | Laser Rangefinder 2 XSHUT | Output | - | Address change control |
| **23** | UART0 RX (Grove Vision AI V2) | Input | - | Serial communication |
| **24** | EC11 Rotary Encoder A | Input | - | Quadrature signal via PIO1 |
| **25** | Encoder 4 (Motor 2 Right) | Input | 4B | Frequency measurement mode |
| **26** | EC11 Rotary Encoder B | Input | - | Quadrature signal via PIO1 |
| **27** | Motor Encoder 4 | Input | 5B | Frequency measurement mode |
| **28** | EC11 Push Button | Input | - | Digital input with pull-up |
| **29** | Battery Monitor (VSYS/3) | Input | - | ADC input (ADC_CHANNEL3) |

## I2C Bus Devices (I2C0 on GPIO 12/13)

| Device | I2C Address | Notes |
|--------|-------------|-------|
| SSD1306 Display | 0x3C | OLED display |
| LSM6DSO32 IMU | 0x6A or 0x6B | Accelerometer/Gyroscope, configurable via GPIO 18 |
| PCA9555 Port Expander | 0x20-0x27 | 16-bit I/O expander, configurable via A0-A2 pins |
| Laser Rangefinder 1 (Front) | 0x29 (default) | VL53L0X/VL53L1X, XSHUT tied to VCC |
| Laser Rangefinder 2 (Rear) | 0x30 (modified) | VL53L0X/VL53L1X, XSHUT controlled by GPIO 22 |

## PCA9555 Port Expander Pin Assignments

| Port | Bit | Function | Direction | Notes |
|------|-----|----------|-----------|-------|
| Port 0 | 0 | Motor 1 Left Forward | Output | Direction control |
| Port 0 | 1 | Motor 1 Left Backward | Output | Direction control |
| Port 0 | 2 | Motor 1 Right Forward | Output | Direction control |
| Port 0 | 3 | Motor 1 Right Backward | Output | Direction control |
| Port 0 | 4 | Motor 2 Left Forward | Output | Direction control |
| Port 0 | 5 | Motor 2 Left Backward | Output | Direction control |
| Port 0 | 6 | Motor 2 Right Forward | Output | Direction control |
| Port 0 | 7 | Motor 2 Right Backward | Output | Direction control |
| Port 1 | 0 | RC Receiver Button A | Input | Remote control input |
| Port 1 | 1 | RC Receiver Button B | Input | Remote control input |
| Port 1 | 2 | RC Receiver Button C | Input | Remote control input |
| Port 1 | 3 | RC Receiver Button D | Input | Remote control input |
| Port 1 | 4 | Motor Driver 1 Standby | Output | Enable/disable driver 1 |
| Port 1 | 5 | Motor Driver 2 Standby | Output | Enable/disable driver 2 |
| Port 1 | 6-7 | *Reserved* | - | Future expansion |

## PWM Slice Allocation Summary

| PWM Slice | Channel A (GPIO) | Channel B (GPIO) | Usage |
|-----------|------------------|------------------|-------|
| Slice 0 | 0 (Output) | 1 (Output) | Motor Driver 1 PWM (independent duty cycles) |
| Slice 1 | 2 (Output) | 3 (Output) | Motor Driver 2 PWM (independent duty cycles) |
| Slice 2 | - | 21 (Input) | Encoder 3 (frequency measurement) |
| Slice 3 | - | 7 (Input) | Encoder 1 (frequency measurement) |
| Slice 4 | - | 9 (Input) | Encoder 2 (frequency measurement) |
| Slice 5 | - | 27 (Input) | Encoder 4 (frequency measurement) |
| Slice 6-7 | - | - | Available for future use |

## Encoder Connections

| Encoder | Motor | GPIO | PWM Slice | PWM Channel | Mode |
|---------|-------|------|-----------|-------------|------|
| Encoder 1 | Motor 1 Left | GPIO 7 | Slice 3 | 3B | Rising Edge Input |
| Encoder 2 | Motor 1 Right | GPIO 9 | Slice 4 | 4B | Rising Edge Input |
| Encoder 3 | Motor 2 Left | GPIO 21 | Slice 2 | 2B | Rising Edge Input |
| Encoder 4 | Motor 2 Right | GPIO 27 | Slice 5 | 5B | Rising Edge Input |

## Laser Rangefinder Connections

| Sensor | I2C Address | XSHUT Pin | Initialization Sequence |
|--------|-------------|-----------|------------------------|
| Rangefinder 1 (Front) | 0x29 (default) | Tied to VCC (always on) | Powers up with default address |
| Rangefinder 2 (Rear) | 0x30 (modified) | GPIO 22 | Keep low during init, change Rangefinder 1 to 0x30, then bring high |

**Address Change Procedure**:
1. At startup, set GPIO 22 low (Rangefinder 2 in shutdown)
2. Rangefinder 1 powers up at default address 0x29
3. Change Rangefinder 1's I2C address to 0x30 via software command
4. Set GPIO 22 high (Rangefinder 2 powers up at default 0x29)
5. Now both sensors are accessible at different addresses

## Motor Driver Connections

### Motor Driver 1 (TB6612FNG or similar)
- **PWMA (Left Motor)**: GPIO 0 (PWM0A)
- **PWMB (Right Motor)**: GPIO 1 (PWM0B)
- **AIN1 (Left Forward)**: PCA9555 Port 0.0
- **AIN2 (Left Backward)**: PCA9555 Port 0.1
- **BIN1 (Right Forward)**: PCA9555 Port 0.2
- **BIN2 (Right Backward)**: PCA9555 Port 0.3
- **STBY (Standby)**: PCA9555 Port 1.4

### Motor Driver 2 (TB6612FNG or similar)
- **PWMA (Left Motor)**: GPIO 2 (PWM1A)
- **PWMB (Right Motor)**: GPIO 3 (PWM1B)
- **AIN1 (Left Forward)**: PCA9555 Port 0.4
- **AIN2 (Left Backward)**: PCA9555 Port 0.5
- **BIN1 (Right Forward)**: PCA9555 Port 0.6
- **BIN2 (Right Backward)**: PCA9555 Port 0.7
- **STBY (Standby)**: PCA9555 Port 1.5

## Communication Interfaces

### UART0 (Grove Vision AI V2)
- **TX**: GPIO 11
- **RX**: GPIO 23
- **Baud Rate**: 115200 (typical, check Grove Vision AI V2 documentation)

### I2C0 (Multiple Devices)
- **SCL**: GPIO 12
- **SDA**: GPIO 13
- **Frequency**: 400 kHz (Fast Mode)

### PIO0 State Machines
- **SM0**: Servo control (GPIO 10)
- **SM1**: RGB LED Red PWM (GPIO 4)
- **SM2**: RGB LED Green PWM (GPIO 5)
- **SM3**: Available

### PIO1 State Machines
- All available for future expansion

## Notes

### PWM Configuration
- **Motor PWM Frequency**: 10 kHz (configurable)
- **Motor PWM Independence**: Both A and B channels on same slice have same period but **independent duty cycles** (confirmed in RP2350 datasheet: "The two outputs on each slice have the same period, but independently varying duty cycles")
- **RGB LED**: Via PIO0 for software PWM generation, freeing up hardware PWM slices
- **Encoder Mode**: Input mode with rising edge detection for frequency measurement (uses B channel only, one encoder per PWM slice)

### Pin Rationale
1. **Motor PWM pins** use 2 slices (0-1), with independent duty cycle control on A/B channels for 4 motors total
2. **Encoders** require separate PWM slices since input mode uses only the B channel per slice (4 slices for 4 encoders)
3. **I2C bus** shared between display, IMU, port expander, and two laser rangefinders
4. **Motor direction AND standby pins** on PCA9555 to maximize available GPIO on Pico2
5. **RC receiver inputs** on PCA9555 to save GPIO and enable interrupt-driven detection via GPIO 20
6. **UART0** on standard pins (11/23) for Grove Vision AI V2
7. **RGB LED** via PIO instead of hardware PWM to conserve PWM slices
8. **ADC pin 29** measures VSYS through built-in voltage divider (no external divider needed)
9. **Ultrasonic servo** uses PIO for precise timing control
10. **Reserved pins** (6, 8, 25) available for future sensors or features
11. **Laser rangefinder XSHUT** on GPIO 22 enables address programming for dual sensor operation
12. **EC11 rotary encoder** uses PIO1 for efficient quadrature decoding without CPU overhead

### PIO Usage
- **PIO0 SM0**: Servo control (GPIO 10) - generates precise 50Hz PWM for servo positioning
- **PIO0 SM1**: RGB LED Red PWM (GPIO 4) - software PWM for brightness control
- **PIO0 SM2**: RGB LED Green PWM (GPIO 5) - software PWM for brightness control
- **PIO0 SM3**: Available
- **PIO1 SM0**: EC11 Rotary Encoder Quadrature Decoder (GPIO 24, 26) - tracks rotation direction and count
- **PIO1 SM1-3**: Available for future expansion

### Future Expansion Options
- Multiple GPIO pins reserved (6, 8, 25)
- Multiple PWM slices available (5-7, plus portions of 2-4)
- PCA9555 Port 1 bits 6-7 available for expansion (2 more I/O pins)
- PIO1 state machines 1-3 available (SM0 used for rotary encoder)
- Can add second I2C bus (I2C1) on different pins if needed
- Can add second PCA9555 expanders on different I2C addresses (up to 8 total at addresses 0x20-0x27)

### Migration from Current Code
The main changes from the existing `main.rs`:

1. **Move to PCA9555 Port Expander**:
   - RC button inputs: GPIOs 10, 11, 16, 17 → PCA9555 Port 1.0-1.3
   - Motor direction pins: GPIOs 18, 19, 20, 21 → PCA9555 Port 0.0-0.7 (8 pins for 4 motors)
   - Motor standby pins: GPIO 22 → PCA9555 Port 1.4-1.5 (2 pins for 2 drivers)

2. **Motor Driver Changes**:
   - Change motor PWM pins from GPIO 27/28 → GPIO 0/1/2/3
   - Add second motor driver on GPIO 2/3 (PWM) with directions on PCA9555 Port 0.4-0.7
   - Both drivers controlled via PCA9555 standby pins

3. **Encoder Changes**:
   - Current: GPIO 7, 9 (2 encoders)
   - New: GPIO 7, 9, 21, 27 (4 encoders, one per PWM slice)

4. **RGB LED Changes**:
   - From: Hardware PWM on GPIO 2/4 (PWM slices 1-2)
   - To: PIO software PWM on GPIO 4/5 (PIO0 SM1-2)

5. **IMU Pin Changes**:
   - From: GPIO 3 (interrupt), GPIO 8 (address)
   - To: GPIO 19 (interrupt), GPIO 18 (address)

6. **Servo Changes**:
   - From: GPIO 5 (PIO0)
   - To: GPIO 10 (PIO0)

7. **IR Sensor Changes**:
   - From: GPIO 26 (1 sensor)
   - To: GPIO 16/17 (2 sensors)

8. **New Additions**:
   - UART on GPIO 11/23 for Grove Vision AI V2
   - PCA9555 interrupt on GPIO 20
   - Two laser rangefinders on I2C bus (addresses 0x29, 0x30)

9. **New Additions**:
   - GPIO 22 for Laser Rangefinder 2 XSHUT control (address change management)
   - GPIO 24, 26, 28 for EC11 rotary encoder (quadrature + push button)

10. **Unchanged**:
   - I2C on GPIO 12/13
   - Ultrasonic sensor on GPIO 14/15
   - Battery ADC on GPIO 29

### Important PWM Notes from RP2350 Datasheet
- Each PWM slice has **one 16-bit counter** shared between A and B channels
- Both channels have **the same period** (frequency) but **independent compare values** (duty cycles)
- This means: two motors on same driver can have different speeds but same PWM frequency (which is fine for motor control)
- PWM input mode uses **only the B pin** of a slice for measuring frequency or duty cycle
- Therefore: **one encoder per PWM slice** is required (cannot use both A and B for separate encoder inputs)
- The B pin is specified as the input pin in the datasheet for edge-sensitive and level-sensitive input modes

### Pico2 Specific Notes
- The Raspberry Pi Pico2 uses the RP2350 in QFN-60 package
- GPIOs 0-29 are available (GPIO 30+ are not exposed on Pico2)
- PWM slices 0-7 are fully accessible (slices 8-11 require GPIO 32+ which are not on Pico2)
- Both I2C peripherals (I2C0, I2C1) available on multiple pin options
- Both UART peripherals (UART0, UART1) available on multiple pin options
- 8 PIO state machines total (4 per PIO block)
- Built-in voltage divider on GPIO 29 for VSYS monitoring (divide by 3)

### Bill of Materials Additions
For this pin configuration, you will need:
- **PCA9555** or **PCA9555A** 16-bit I2C I/O expander (TSSOP-24, SSOP-24, or QFN-24 package)
- **TB6612FNG** dual motor driver × 2 (or equivalent like DRV8833)
- **VL53L0X** or **VL53L1X** laser rangefinders × 2 (I2C ToF sensors)
- **Grove Vision AI V2** module with UART interface
- **EC11 rotary encoder** with integrated push button (5-pin, 20 detents per rotation)
- Pull-up resistors for I2C bus (typically 4.7kΩ, may be built into modules)
- Pull-up resistor for PCA9555 interrupt line (10kΩ recommended)
- Optional: 0.1µF capacitors for EC11 encoder pins (debouncing)

### Laser Rangefinder Address Programming Notes
- **VL53L0X/VL53L1X** sensors default to I2C address 0x29
- Both sensors share the same default address, so XSHUT pin is used for sequential initialization
- **Rangefinder 1 (Front)**: XSHUT hardwired to VCC (3.3V) - always powered
- **Rangefinder 2 (Rear)**: XSHUT controlled by GPIO 22 - powered on after address change
- XSHUT is **active high** (low = shutdown, high = active)
- During initialization, change the first sensor's address, then power up the second
- New addresses persist only while powered - repeat sequence after power cycle

### I2C Bus Utilization Analysis

**I2C Configuration**: 400 kHz (Fast Mode I2C)

**Device Communication Requirements**:

| Device | Update Rate | Bytes/Transaction | Bits/Transaction | Bits/Second |
|--------|-------------|-------------------|------------------|-------------|
| ICM-20948 IMU | 100 Hz | 18 (accel + gyro + mag) | ~193 | 19,300 |
| SSD1306 Display | 4 Hz | 1034 (128×64 buffer + commands) | ~9,326 | 37,304 |
| PCA9555 Port Expander | 20 Hz | 4 (read 2 + write 2) | ~100 | 2,000 |
| VL53L0X Rangefinder 1 | 10 Hz | 2 (range data) | ~80 | 800 |
| VL53L0X Rangefinder 2 | 10 Hz | 2 (range data) | ~80 | 800 |
| **TOTAL** | - | - | - | **60,204** |

**Bus Capacity**: 400,000 bits/second

**Utilization**: 60,204 / 400,000 = **15.05%**

**Conclusion**: ✅ The I2C bus is **NOT congested**. You have ~85% bandwidth remaining for future expansion.

**Timing Analysis**:
- IMU read cycle: 100 Hz = 10 ms period
- IMU transaction time: 193 bits / 400,000 = 0.48 ms
- Available time between IMU reads: 9.52 ms (plenty for other devices)
- Display update: 9,326 bits / 400,000 = 23.3 ms (completes well within 250 ms period)
- Port expander update: 0.25 ms (completes within 50 ms period at 20 Hz)
- Rangefinder reads: 0.2 ms each (completes within 100 ms period at 10 Hz)

**Recommendations**:
- Current configuration is well-balanced
- Could increase display refresh rate to 10 Hz if desired (~37% total utilization)
- Could increase rangefinder rate to 20-50 Hz if needed
- Consider I2C DMA for display updates to reduce CPU overhead
- IMU could use interrupt-driven reads (GPIO 19) to avoid polling

### Testing Strategy
1. Test I2C bus communication with all devices individually first
2. Test PCA9555 by toggling outputs and reading inputs
3. Test laser rangefinder address change procedure (GPIO 22 control)
4. Test motor drivers one at a time with simple forward/backward commands
5. Test encoders individually to verify PWM input mode frequency counting
6. Test UART communication with Grove Vision AI V2
7. Test PIO-based servo and RGB LED control
8. Test IMU at 100 Hz to verify timing and I2C bandwidth
9. Test EC11 rotary encoder quadrature decoding via PIO1
10. Monitor I2C bus timing with logic analyzer if issues arise
11. Integrate all systems with interrupt-driven PCA9555 and IMU input handling