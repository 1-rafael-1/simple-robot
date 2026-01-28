# Pin Assignment Reference — Simple Robot (Raspberry Pi Pico 2)

_Last updated: matches `src/main.rs` and the currently enabled Embassy tasks._

This document maps the Raspberry Pi Pico 2 (RP2350) GPIOs to the peripherals that the firmware actively touches today. When the code and previous plans disagree, the table below reflects what the firmware is really using so the first wiring pass stays truthful. Planned-but-disabled features are listed separately so you know which pins are still open.

---

## 1. Pico GPIO Summary

| GPIO | Direction (Firmware) | In-Use? | Subsystem / Signal | Notes |
|------|----------------------|---------|--------------------|-------|
| 0 | Output (PWM slice 0A) | ✅ | Motor driver A, channel 0 | H-bridge PWM output. |
| 1 | Output (PWM slice 0B) | ✅ | Motor driver A, channel 1 | H-bridge PWM output. |
| 2 | Output (PWM slice 1A) | ✅ | Motor driver B, channel 0 | H-bridge PWM output. |
| 3 | Output (PWM slice 1B) | ✅ | Motor driver B, channel 1 | H-bridge PWM output. |
| 4 | — | 🟡 | Planned | RGB LED (PIO PWM) Red channel. |
| 5 | — | 🟡 | Planned | Ultrasonic sweep servo (PIO). |
| 6 | — | 🟡 | Planned | RGB LED (PIO PWM) Green channel. |
| 7 | Input (PWM slice 3B) | ✅ | Motor encoder 0 | Rising-edge count input. |
| 8 | — | 🟡 | Planned | RGB LED (PIO PWM) Blue channel. |
| 9 | Input (PWM slice 4B) | ✅ | Motor encoder 1 | Rising-edge count input. |
| 10 | — | ⬜ | Available | Previously RC button; now unused. |
| 11 | — | ⬜ | Available | UART0 TX only in legacy plan. |
| 12 | Bidirectional (I²C0 SCL) | ✅ | Shared I²C bus | 400 kHz fast mode. |
| 13 | Bidirectional (I²C0 SDA) | ✅ | Shared I²C bus | 400 kHz fast mode. |
| 14 | Input (planned) | 🟡 | Ultrasonic sensor echo | HC-SR04 echo; task currently disabled—keep clear until the sweep task returns. |
| 15 | Output (planned) | 🟡 | Ultrasonic sensor trigger | HC-SR04 trigger; task currently disabled—keep clear until the sweep task returns. |
| 16 | — | 🟡 | Planned | IR obstacle sensor input (left). |
| 17 | — | 🟡 | Planned | IR obstacle sensor input (right). |
| 18 | Input (reserved) | 🟡 | IMU interrupt (planned) | Task signature reserves this pin; wire the IMU INT when available. |
| 19 | Output (reserved) | 🟡 | IMU address/config pin (planned) | Task signature reserves this pin; set the IMU address pin when wiring the sensor. |
| 20 | Input (pull-up) | ✅ | PCA9555 interrupt | Active-low interrupt from expander. |
| 21 | Input (PWM slice 2B) | ✅ | Motor encoder 2 | Rising-edge count input. |
| 22 | — | 🟡 | Planned | EC11 rotary encoder A (PIO input). |
| 23 | — | 🟡 | Planned | EC11 rotary encoder B (PIO input). |
| 24 | — | 🟡 | Planned | EC11 rotary encoder button (GPIO input). |
| 25 | On-board LED (output) | ⬜ | Pico built-in LED | Firmware never toggles it; safe to reuse if you don't need the onboard indicator. |
| 26 | Analog input (ADC0) | ✅ | Battery voltage monitor | Connected to battery pack via voltage divider (20kΩ/10kΩ). Measures battery voltage before the voltage regulator. |
| 27 | Input (PWM slice 5B) | ✅ | Motor encoder 3 | Rising-edge count input. |
| 28 | — | ⬜ | Available | Free for future use. |
| 29 | Analog input (ADC3) | ⬜ | Available | Free for future use. |

Legend: ✅ = actively used by running firmware; 🟡 = reserved in firmware (tasks expect hardware soon); ⬜ = free/available (may be mentioned in comments but not enabled).

---

## 2. PWM Slice Allocation

| PWM Slice | Channel A | Channel B | Firmware Role |
|-----------|-----------|-----------|---------------|
| 0 | GPIO0 (output) | GPIO1 (output) | Motor driver A PWM (two motors). |
| 1 | GPIO2 (output) | GPIO3 (output) | Motor driver B PWM (two motors). |
| 2 | — | GPIO21 (input) | Encoder 2 pulse counting. |
| 3 | — | GPIO7 (input) | Encoder 0 pulse counting. |
| 4 | — | GPIO9 (input) | Encoder 1 pulse counting. |
| 5 | — | GPIO27 (input) | Encoder 3 pulse counting. |
| 6–7 | — | — | Unused. |

> The encoder tasks configure slice B pins in `InputMode::RisingEdge`, one encoder per slice.

---

## 3. I²C0 Bus (GPIO12 = SCL, GPIO13 = SDA)

The shared I²C bus is initialized at 400 kHz in `init_i2c_bus`. Current tasks on that bus:

- SSD1306 display (`task::display`)
- PCA9555 port expander (`task::port_expander`)
- ICM-20948 IMU (`task::imu_read`)

Keep pull-ups sized for a 3V3, 400 kHz bus (4.7 kΩ is a good starting point).

---

## 4. PCA9555 Port Mapping

`task::port_expander` configures Port 0 as outputs and Port 1 as mixed I/O:

| Port | Bit | Direction | Firmware Usage |
|------|-----|-----------|----------------|
| 0 | 0 | Output | Motor A — forward (Track::Left, Motor::Front) |
| 0 | 1 | Output | Motor A — reverse (Track::Left, Motor::Front) |
| 0 | 2 | Output | Motor B — forward (Track::Left, Motor::Rear) |
| 0 | 3 | Output | Motor B — reverse (Track::Left, Motor::Rear) |
| 0 | 4 | Output | Motor C — forward (Track::Right, Motor::Front) |
| 0 | 5 | Output | Motor C — reverse (Track::Right, Motor::Front) |
| 0 | 6 | Output | Motor D — forward (Track::Right, Motor::Rear) |
| 0 | 7 | Output | Motor D — reverse (Track::Right, Motor::Rear) |
| 1 | 0 | Input (pull-up) | RC button A (interrupt driven) |
| 1 | 1 | Input (pull-up) | RC button B |
| 1 | 2 | Input (pull-up) | RC button C |
| 1 | 3 | Input (pull-up) | RC button D |
| 1 | 4 | Output | Motor driver A standby/enable |
| 1 | 5 | Output | Motor driver B standby/enable |
| 1 | 6 | Output | Spare |
| 1 | 7 | Output | Spare |

The port expander interrupt (GPIO20) fires on any input change. Outputs are always written in a single transaction (`BothPorts` command) so every update is atomic.

---

## 5. Currently Disabled / Reserved Features

These signals remain referenced in comments or helper modules but no task enables them yet:

- **Ultrasonic sweep** — would consume GPIO5 (servo PIO), GPIO14 (echo), GPIO15 (trigger).
- **RGB status LED** — planned on PIO PWM using GPIO4/6/8 (RGB channels).
- **IR obstacle sensors** — intended for GPIO16/17.
- **Rotary encoder (EC11)** — planned on GPIO22 (A), GPIO23 (B), GPIO24 (button).
- **Battery monitor** — Active on GPIO26 (ADC0) measuring 2S Li-Ion battery voltage (6V-8.4V) through 20kΩ/10kΩ voltage divider connected to battery pack before the voltage regulator.
- **Grove Vision UART** — Legacy plan on GPIO11 (TX) / GPIO23 (RX); RX now claimed by EC11, so this would need reassignment if revived.

Treat 🟡 pins as reserved for upcoming wiring; only ⬜ pins are truly free.

---

## 6. Wiring Notes

1. **Motor drivers**  
   - PWM pins (GPIO0–3) drive the TB6612FNG (or equivalent) PWMA/PWMB inputs.  
   - Direction + standby lines must be wired to the PCA9555 outputs as listed above.  
   - Don’t forget common grounds between Pico, drivers, encoders, and power electronics.

2. **Encoders**  
   - Each encoder output goes straight to its assigned GPIO (7, 9, 21, 27).  
   - Firmware expects single-channel rising-edge pulses (no quadrature decoding).

3. **I²C devices**  
   - Display, IMU, and port expander share GPIO12/13. Keep wires short on the first breadboard mock-up; noise margins are slimmer at 400 kHz.

4. **Battery voltage monitoring**  
   - GPIO26 measures battery voltage through a voltage divider (R1=20kΩ, R2=10kΩ to GND).  
   - The voltage divider input connects to the 2S Li-Ion battery pack (6V-8.4V range) before the voltage regulator.  
   - The divider ratio (0.333) scales 8.4V down to 2.8V for the 3.3V ADC.  
   - Source impedance is ~6.7kΩ, suitable for the RP2350 ADC.

5. **Port expander interrupt**  
   - GPIO20 needs a pull-up (external or relying on expander's INT default). The firmware enables an internal pull-up on the Pico side.

6. **IMU auxiliary pins**  
   - GPIO18 and GPIO19 are already consumed by the IMU task signature, so keep them free for INT and address-select wiring even though the driver has not consumed them yet.

---

## 7. Quick Checklist Before Prototyping

- [ ] Flash the current firmware so the PCA9555 task starts and holds motor drivers in standby.  
- [ ] Verify I²C pull-ups and device addresses (display, IMU, expander).  
- [ ] Wire battery voltage divider (20kΩ/10kΩ) from battery pack to GPIO26 before the voltage regulator.
- [ ] Tie each motor driver's STBY pin to the matching PCA9555 port bit (1.4 / 1.5).  
- [ ] Route encoder outputs with clean ground references to GPIO7/9/21/27.  
- [ ] Leave unused GPIOs floating only if the connected hardware also floats; otherwise add sensible pull resistors.

Once the above is wired you will have every pin that the shipping firmware expects, and the remaining GPIOs stay free for future sensors or UI hardware.