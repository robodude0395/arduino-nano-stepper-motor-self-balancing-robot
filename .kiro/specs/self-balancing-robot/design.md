# Implementation Plan

## Overview

This design describes the implementation of a remote-controlled self-balancing robot system using PlatformIO. The system comprises two firmware targets: a transmitter (TX) running on an Arduino Pro Mini and a robot (RX) running on an Arduino Nano. Communication is via nRF24L01 2.4GHz radios. The robot uses an MPU6050 IMU with a Madgwick sensor fusion filter feeding a PID balance controller that drives two stepper motors through DRV8825 drivers via a 20µs timer interrupt.

All firmware for each target is contained in a single `main.cpp` file, following the Arduino .ino sketch approach where all logic is inline — no separate header/source modules. A single shared header (`radio_config.h`) defines the packet structure and radio constants used by both targets.

## Architecture

### System Architecture

```
┌─────────────────────────────────────────────┐
│              TRANSMITTER (TX)                │
│         Arduino Pro Mini 16MHz              │
├─────────────────────────────────────────────┤
│                                             │
│  ┌──────────┐   ┌──────────┐   ┌────────┐  │
│  │Potentio- │   │ Battery  │   │ SSD1306│  │
│  │meter ADC │   │ Monitor  │   │  OLED  │  │
│  └────┬─────┘   └────┬─────┘   └───┬────┘  │
│       │               │             │       │
│       v               v             v       │
│  ┌────────────────────────────────────────┐ │
│  │         TX Main Loop (50Hz)            │ │
│  │  All logic inline in main.cpp:         │ │
│  │  - Radio init & transmit               │ │
│  │  - Potentiometer read & mapping        │ │
│  │  - Battery voltage monitoring          │ │
│  │  - OLED display update                 │ │
│  │  - LED status control                  │ │
│  └────────────────┬───────────────────────┘ │
│                   │                         │
│              ┌────▼─────┐  ┌────────┐       │
│              │ nRF24L01 │  │LED Pin4│       │
│              └────┬─────┘  └────────┘       │
└───────────────────┼─────────────────────────┘
                    │ 2.4GHz wireless
┌───────────────────┼─────────────────────────┐
│              ┌────▼─────┐                   │
│              │ nRF24L01 │                   │
│              └────┬─────┘                   │
│                   │                         │
│  ┌────────────────▼───────────────────────┐ │
│  │       RX Main Loop (250Hz / 4ms)       │ │
│  │  All logic inline in main.cpp:         │ │
│  │                                        │ │
│  │  ┌─────────────┐  ┌────────────────┐   │ │
│  │  │  IMU Read   │  │ Radio Receive  │   │ │
│  │  │  (MPU6050)  │  │ Command_Packet │   │ │
│  │  └──────┬──────┘  └───────┬────────┘   │ │
│  │         │                  │            │ │
│  │         v                  │            │ │
│  │  ┌─────────────┐          │            │ │
│  │  │  Madgwick   │          │            │ │
│  │  │   Filter    │          │            │ │
│  │  └──────┬──────┘          │            │ │
│  │         │                  │            │ │
│  │         v                  v            │ │
│  │  ┌──────────────────────────────────┐   │ │
│  │  │      PID Balance Controller      │   │ │
│  │  │  + Movement Command Processing  │   │ │
│  │  └──────────────┬──────────────────┘   │ │
│  │                 │                       │ │
│  │                 v                       │ │
│  │  ┌──────────────────────────────────┐   │ │
│  │  │  Non-linear Stepper Compensation │   │ │
│  │  └──────────────┬──────────────────┘   │ │
│  └─────────────────┼──────────────────────┘ │
│                    │                         │
│  ┌─────────────────▼──────────────────────┐ │
│  │   Timer2 ISR (20µs / 50kHz)            │ │
│  │   Stepper Pulse Generation             │ │
│  │   Left Motor ──► DRV8825 ──► Stepper   │ │
│  │   Right Motor ─► DRV8825 ──► Stepper   │ │
│  └────────────────────────────────────────┘ │
│              ROBOT (RX)                      │
│          Arduino Nano                        │
└──────────────────────────────────────────────┘
```

### Project Directory Structure

```
Self_Balancing_Robot/
├── platformio.ini              # Multi-environment config (TX + Robot)
├── shared/
│   └── radio_config.h          # CommandPacket struct & radio constants
├── src/
│   ├── tx/
│   │   └── main.cpp            # ALL transmitter firmware (single file)
│   └── robot/
│       └── main.cpp            # ALL robot firmware (single file)
├── include/
│   └── README
├── lib/
│   └── README
└── test/
    └── README
```

There are no separate `.h` / `.cpp` module files. Each target's entire firmware lives in its respective `main.cpp`, following the Arduino `.ino` monolithic sketch approach. The only shared file is `radio_config.h` which defines the `CommandPacket` struct and radio configuration constants.

## Shared Header: radio_config.h

**Purpose:** Defines the CommandPacket structure and radio constants shared between TX and Robot firmware.

```cpp
// shared/radio_config.h
#ifndef RADIO_CONFIG_H
#define RADIO_CONFIG_H

#include <stdint.h>

// Radio configuration constants
const uint8_t RADIO_CHANNEL = 108;
const uint8_t RADIO_PIPE_ADDRESS[5] = { 0xE8, 0xE8, 0xF0, 0xF0, 0xE1 };

// Command packet sent from TX to Robot every 20ms
struct CommandPacket {
  int8_t speed;   // -100 to +100 (forward/backward)
  int8_t turn;    // -100 to +100 (left/right), reserved for future
  uint8_t flags;  // bit flags for future use
};

#endif
```

**Key details:**
- Channel 108 avoids common Wi-Fi interference
- 5-byte pipe address shared by both endpoints
- Data rate: RF24_250KBPS for maximum range
- PA level: RF24_PA_MAX
- Auto-acknowledgment enabled for reliable delivery detection
- Packet is 3 bytes total — minimal overhead at 50Hz

## TX Firmware: src/tx/main.cpp

**Purpose:** Single file containing ALL transmitter logic — radio initialization and transmission, potentiometer reading with dead-band and hysteresis, battery voltage monitoring, OLED display rendering, and LED status control.

### Structure

```cpp
// src/tx/main.cpp — ALL transmitter firmware

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <U8g2lib.h>
#include <Wire.h>
#include "radio_config.h"

// ─── Pin Definitions ───
// CE, CSN for nRF24L01
// Potentiometer analog pin
// Battery voltage analog pin
// Status LED pin (pin 4)

// ─── Constants ───
// Dead-band boundaries (490-534)
// Hysteresis value (3 ADC counts)
// Battery voltage range (3.0V-4.2V)
// Loop interval (20ms)
// Connection timeout (500ms)

// ─── Global State ───
// RF24 radio object
// U8g2 display object
// CommandPacket packet
// Connection tracking (last ACK time, connected flag)
// Battery monitoring state
// Potentiometer state (previous value, in-dead-band flag)
// Timing variables

// ─── setup() ───
//   Initialize Serial (debug)
//   Initialize SPI + RF24 radio as transmitter
//     - Set channel, data rate, PA level, pipe address
//     - Enable auto-ack
//     - If init fails: blink LED at 2Hz, halt transmission
//   Initialize I2C + OLED display
//   Configure LED pin as output
//   Set initial loop timer

// ─── loop() ───  (runs every 20ms = 50Hz)
//   1. Read potentiometer (analogRead)
//   2. Apply dead-band (490-534 → speed = 0)
//      - Apply hysteresis (3 ADC counts at boundaries)
//      - Map below dead-band (0-489) → speed (-100 to -1)
//      - Map above dead-band (535-1023) → speed (+1 to +100)
//   3. Build CommandPacket with current speed
//   4. Transmit packet via RF24.write()
//      - Check ACK → update connection status & timestamp
//   5. Update connection state:
//      - If millis() - lastAckTime > 500ms → connected = false
//   6. Update Status LED:
//      - Connected: solid HIGH
//      - Disconnected: OFF
//      - Init failure: 2Hz blink
//   7. Every ~1 second (every 50th loop iteration):
//      - Read battery ADC → convert to voltage via divider ratio
//      - Map 3.0V-4.2V to 0%-100% (clamp at boundaries)
//      - Update OLED display (battery %, connection icon)
//      - Show low-battery warning if ≤ 10%
//   8. Wait for next 20ms interval
```

### Key Algorithms (Inline in main.cpp)

**Potentiometer Dead-Band with Hysteresis:**
```cpp
// Pseudocode within loop()
int raw = analogRead(POT_PIN);
if (inDeadBand) {
    // Exit dead-band only if past boundary + hysteresis
    if (raw < (490 - HYSTERESIS)) { inDeadBand = false; }
    else if (raw > (534 + HYSTERESIS)) { inDeadBand = false; }
    else { speed = 0; }
}
if (!inDeadBand) {
    if (raw >= 490 && raw <= 534) { inDeadBand = true; speed = 0; }
    else if (raw < 490) { speed = map(raw, 0, 489, -100, -1); }
    else { speed = map(raw, 535, 1023, 1, 100); }
}
```

**Battery Percentage Calculation:**
```cpp
// Linear interpolation: 3.0V = 0%, 4.2V = 100%
int voltage_mV = (analogRead(BAT_PIN) * dividerRatio) + diodeOffset;
int percent = constrain(map(voltage_mV, 3000, 4200, 0, 100), 0, 100);
```

**Satisfies Requirements:** 1.1, 1.3, 1.4, 1.7, 2.1, 2.2, 2.3, 2.4, 2.5, 3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7, 3.8

## Robot Firmware: src/robot/main.cpp

**Purpose:** Single file containing ALL robot logic — radio receive, IMU initialization and calibration, Madgwick sensor fusion filter, PID balance controller, Timer2 ISR stepper pulse generation, movement command processing, and safety protection.

### Structure

```cpp
// src/robot/main.cpp — ALL robot firmware

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include "radio_config.h"

// ─── Pin Definitions ───
// CE, CSN for nRF24L01
// Left motor: step pin 2, dir pin 3
// Right motor: step pin 4, dir pin 5
// Onboard LED: pin 13

// ─── Constants ───
// IMU I2C address (0x68)
// Gyro scale factor: 250.0 / 32768.0
// Accelerometer calibration value
// Accelerometer clamp: ±8200
// PID gains: Kp=15, Ki=1.5, Kd=30
// Integral/output clamp: ±400
// Dead-band: ±5
// Brake factor: 0.015
// Self-balance adjustment: ±0.0015 degrees/loop
// Tip-over threshold: ±30 degrees
// Start condition: ±0.5 degrees
// Max target speed: 150
// Turning speed: 30
// Movement rate: 0.05 degrees/loop
// Movement acceleration: 0.005 degrees/loop
// Max lean: ±2.5 degrees
// Loop interval: 4000µs (4ms)
// Radio timeout: 500ms
// Low battery: 1050 (10.5V)

// ─── Global Variables ───
// Motor control (volatile for ISR access):
//   throttle_left_motor, throttle_right_motor
//   throttle_counter_left_motor, throttle_counter_right_motor
//   throttle_left_motor_memory, throttle_right_motor_memory

// IMU data:
//   gyro_pitch_data_raw, gyro_yaw_data_raw, accelerometer_data_raw
//   gyro_pitch_calibration_value, gyro_yaw_calibration_value

// Sensor fusion state:
//   angle_gyro, angle_acc, angle (Madgwick output)
//   Madgwick quaternion state (q0, q1, q2, q3) or simplified 1-axis state
//   beta parameter (0.1)

// PID state:
//   pid_error_temp, pid_i_mem, pid_output, pid_last_d_error
//   pid_setpoint, self_balance_pid_setpoint
//   pid_output_left, pid_output_right

// System state:
//   start (byte: 0=stopped, 1=running)
//   low_bat (byte)
//   loop_timer (unsigned long)
//   last_receive_time (unsigned long)
//   received_packet (CommandPacket)

// ─── Timer2 ISR ───
// ISR(TIMER2_COMPA_vect) — executes every 20µs
//   Left motor:
//     Increment throttle_counter_left_motor
//     If counter > throttle_left_motor_memory:
//       Reset counter to 0
//       Load new throttle value from throttle_left_motor
//       Set direction via PORTD bit manipulation
//       If negative: invert, set reverse direction
//     If counter == 1: set step pin HIGH (PORTD)
//     If counter == 2: set step pin LOW (PORTD)
//   Right motor:
//     (mirror of left motor logic)

// ─── setup() ───
//   Initialize Serial (9600 baud, debug)
//   Initialize I2C (Wire.begin(), TWBR=12 for 400kHz)
//   Verify IMU WHO_AM_I register (0x75 should return 0x68)
//     - If fails: blink LED at 4Hz, halt
//   Configure IMU:
//     - Wake (register 0x6B = 0x00)
//     - Gyro ±250°/s (register 0x1B = 0x00)
//     - Accel ±4g (register 0x1C = 0x08)
//     - DLPF ~43Hz (register 0x1A = 0x03)
//   Configure motor pins (2,3,4,5) as OUTPUT
//   Configure LED pin 13 as OUTPUT
//   Calibrate gyroscope:
//     - 500 samples at ~4ms intervals
//     - Accumulate pitch and yaw raw readings
//     - Compute mean offset for each axis
//     - Blink LED during calibration
//   Configure Timer2:
//     - CTC mode, prescaler 8, OCR2A = 39 → 20µs period
//     - Enable OCIE2A interrupt
//   Initialize RF24 radio as receiver:
//     - Set channel, data rate, PA level, pipe address
//     - Open reading pipe, start listening
//     - If init fails: blink LED at 2Hz, keep motors at zero
//   Initialize Madgwick filter state (quaternion or simplified)
//   Set loop_timer = micros() + 4000

// ─── loop() ───  (runs every 4ms = 250Hz)
//   1. Check radio for new packet:
//      - If radio.available(): read packet, update last_receive_time
//   2. Check radio timeout:
//      - If millis() - last_receive_time > 500: clear movement commands
//   3. Read battery voltage:
//      - If < 10.5V and > 8.0V: set low_bat, illuminate LED
//   4. Read IMU accelerometer (register 0x3F, 2 bytes):
//      - Add calibration offset
//      - Clamp to ±8200
//      - Compute angle_acc = asin(raw/8200.0) * 57.296
//   5. Start condition check:
//      - If start == 0 and angle_acc within ±0.5°:
//        Initialize angle_gyro from angle_acc, set start = 1
//   6. Read IMU gyroscope (register 0x43, 4 bytes):
//      - Extract pitch and yaw raw values
//      - Subtract calibration offsets
//   7. Madgwick filter update:
//      - Convert gyro to degrees/s: raw * (250.0/32768.0)
//      - Run filter iteration with gyro rate and accel angle
//      - Output: corrected angle estimate
//      - Simplified implementation: angle_gyro += gyro_pitch * 0.000031
//        then blend: angle_gyro = angle_gyro * 0.9996 + angle_acc * 0.0004
//        (Madgwick gradient descent replaces this simple complementary blend)
//   8. PID computation:
//      - error = angle - self_balance_pid_setpoint - pid_setpoint
//      - Brake function: if |pid_output| > 10, error += pid_output * 0.015
//      - Integral: pid_i_mem += Ki * error, clamp ±400
//      - Output: Kp*error + pid_i_mem + Kd*(error - last_error), clamp ±400
//      - Dead-band: if |output| < 5, output = 0
//      - Store error for next derivative
//   9. Safety check:
//      - If |angle| > 30° OR start == 0 OR low_bat:
//        output = 0, reset integral, reset setpoints, start = 0
//   10. Re-start check:
//       - If start == 0 and angle_acc within ±0.5°:
//         Re-initialize angle from accelerometer, start = 1
//   11. Movement command processing:
//       - Forward (speed > 0): decrease pid_setpoint by 0.05/loop, clamp -2.5°
//         Additional 0.005/loop if |output| < max_target_speed
//       - Backward (speed < 0): increase pid_setpoint by 0.05/loop, clamp +2.5°
//         Additional 0.005/loop if |output| < max_target_speed
//       - No command: return pid_setpoint toward 0 at 0.05/loop
//         If |pid_setpoint| ≤ 0.5: set to exactly 0
//       - Turn left: pid_output_left += 30, pid_output_right -= 30
//       - Turn right: pid_output_left -= 30, pid_output_right += 30
//   12. Self-balance adjustment:
//       - If pid_setpoint == 0 and output < 0: self_balance += 0.0015
//       - If pid_setpoint == 0 and output > 0: self_balance -= 0.0015
//   13. Non-linear stepper compensation:
//       - If output_left > 0: output_left = 405 - (1/(output_left+9))*5500
//       - If output_left < 0: output_left = -405 - (1/(output_left-9))*5500
//       - Same for output_right
//   14. Compute throttle values:
//       - If output > 0: throttle = 400 - output_compensated
//       - If output < 0: throttle = -400 - output_compensated
//       - If output == 0: throttle = 0
//   15. Set motor throttle (atomic for ISR safety):
//       - throttle_left_motor = left_motor
//       - throttle_right_motor = right_motor
//   16. Wait for loop timer:
//       - while(loop_timer > micros());
//       - loop_timer += 4000;
```

### Timer2 ISR Detail (Inline in main.cpp)

```cpp
// Defined at file scope in src/robot/main.cpp
ISR(TIMER2_COMPA_vect) {
  // Left motor pulse generation
  throttle_counter_left_motor++;
  if (throttle_counter_left_motor > throttle_left_motor_memory) {
    throttle_counter_left_motor = 0;
    throttle_left_motor_memory = throttle_left_motor;
    if (throttle_left_motor_memory < 0) {
      PORTD &= 0b11110111;   // Dir pin 3 LOW (reverse)
      throttle_left_motor_memory *= -1;
    } else {
      PORTD |= 0b00001000;   // Dir pin 3 HIGH (forward)
    }
  }
  else if (throttle_counter_left_motor == 1) PORTD |= 0b00000100;   // Step pin 2 HIGH
  else if (throttle_counter_left_motor == 2) PORTD &= 0b11111011;   // Step pin 2 LOW

  // Right motor pulse generation
  throttle_counter_right_motor++;
  if (throttle_counter_right_motor > throttle_right_motor_memory) {
    throttle_counter_right_motor = 0;
    throttle_right_motor_memory = throttle_right_motor;
    if (throttle_right_motor_memory < 0) {
      PORTD |= 0b00100000;   // Dir pin 5 HIGH (reverse)
      throttle_right_motor_memory *= -1;
    } else {
      PORTD &= 0b11011111;   // Dir pin 5 LOW (forward)
    }
  }
  else if (throttle_counter_right_motor == 1) PORTD |= 0b00010000;  // Step pin 4 HIGH
  else if (throttle_counter_right_motor == 2) PORTD &= 0b11101111;  // Step pin 4 LOW
}
```

**Satisfies Requirements:** 1.2, 1.5, 1.6, 1.7, 4.1, 4.2, 4.3, 4.4, 4.5, 4.6, 5.1, 5.2, 5.3, 5.4, 5.5, 5.6, 5.7, 6.1, 6.2, 6.3, 6.4, 6.5, 6.6, 6.7, 7.1, 7.2, 7.3, 7.4, 7.5, 7.6, 7.7, 8.1, 8.2, 8.3, 8.4, 8.5, 8.6, 9.1, 9.2, 9.3, 9.4

## Components and Interfaces

### Component 1: Shared Radio Configuration (shared/radio_config.h)

**Purpose:** Single shared header defining the CommandPacket struct and radio constants used by both TX and Robot.

**Interface:**
```cpp
struct CommandPacket {
  int8_t speed;   // -100 to +100
  int8_t turn;    // -100 to +100
  uint8_t flags;  // reserved
};
const uint8_t RADIO_CHANNEL = 108;
const uint8_t RADIO_PIPE_ADDRESS[5] = { 0xE8, 0xE8, 0xF0, 0xF0, 0xE1 };
```

### Component 2: TX Firmware (src/tx/main.cpp)

**Purpose:** Monolithic transmitter firmware — all radio, input, battery, display, and LED logic in one file.

**Interface (public functions):**
```cpp
void setup();  // Initialize all hardware, radio, display
void loop();   // 50Hz: read pot, transmit, update display/LED
```

**Responsibilities:**
- Initialize nRF24L01 as transmitter (channel 108, 250kbps, auto-ack)
- Read potentiometer with dead-band (490-534) and hysteresis (3 ADC counts)
- Map pot values to speed -100..+100
- Transmit CommandPacket every 20ms
- Track connection via ACK (timeout 500ms)
- Read battery voltage, map 3.0-4.2V to 0-100%
- Render OLED display (battery %, connection status, low-bat warning)
- Drive Status LED (pin 4): solid=connected, off=disconnected, 2Hz blink=init failure

### Component 3: Robot Firmware (src/robot/main.cpp)

**Purpose:** Monolithic robot firmware — all radio receive, IMU, sensor fusion, PID, stepper ISR, movement, and safety logic in one file.

**Interface (public functions):**
```cpp
void setup();                  // Initialize all hardware, calibrate IMU
void loop();                   // 4ms control loop
ISR(TIMER2_COMPA_vect);        // 20µs stepper pulse generation
```

**Responsibilities:**
- Initialize nRF24L01 as receiver (channel 108, 250kbps, auto-ack)
- Initialize MPU6050 (verify WHO_AM_I, configure gyro ±250°/s, accel ±4g, DLPF 43Hz)
- Calibrate gyroscope (500 samples, compute pitch/yaw offsets)
- Configure Timer2 CTC for 20µs ISR
- Read IMU every 4ms, run Madgwick filter for angle estimation
- Compute PID output (Kp=15, Ki=1.5, Kd=30) with integral windup ±400, output clamp ±400
- Apply brake function, dead-band, self-balance adjustment
- Process movement commands (forward/backward lean, differential turning)
- Apply non-linear stepper compensation
- Generate step pulses via Timer2 ISR (direct PORTD manipulation)
- Safety: tip-over (±30°) → stop motors, radio timeout (500ms) → balance-only mode

## Data Models

### CommandPacket

```cpp
struct CommandPacket {
  int8_t speed;   // -100 (full backward) to +100 (full forward), 0 = stop
  int8_t turn;    // -100 (full left) to +100 (full right), 0 = straight
  uint8_t flags;  // bit 0-7 reserved for future use
};
```

**Validation Rules:**
- speed clamped to [-100, +100] before transmission
- Values within pot dead-band (490-534 raw) produce speed = 0
- Packet transmitted every 20ms regardless of value change

### Robot Control State (Global Variables)

| Variable | Type | Range | Description |
|----------|------|-------|-------------|
| angle_gyro | float | -180..180 | Fused angle estimate (degrees) |
| pid_output | float | -400..400 | PID controller output |
| pid_setpoint | float | -2.5..2.5 | Movement lean angle target |
| self_balance_pid_setpoint | float | ~±1 | Auto-discovered balance point |
| throttle_left_motor | int | -400..400 | Left motor pulse period (ISR) |
| throttle_right_motor | int | -400..400 | Right motor pulse period (ISR) |
| start | byte | 0 or 1 | 0=stopped, 1=balancing |
| low_bat | byte | 0 or 1 | Battery fault flag |

## Correctness Properties

### Property 1: Angle Boundedness
The Madgwick filter output shall always produce an angle in the range [-90°, +90°]. If the raw accelerometer value exceeds ±8200, it is clamped before angle computation.

**Validates: Requirements 5.3**

### Property 2: PID Output Boundedness
For any input angle and state, `|pid_output| <= 400` always holds after the clamp step.

**Validates: Requirements 6.4**

### Property 3: Integral Windup Protection
`|pid_i_mem| <= 400` is invariant across all loop iterations.

**Validates: Requirements 6.3**

### Property 4: Motor Safety on Tip-Over
When `|angle| > 30°`, motor output is set to zero within the same loop cycle (4ms).

**Validates: Requirements 9.1**

### Property 5: Radio Timeout Safety
If no packet received for 500ms (125 loops), movement commands are cleared and robot returns to balance-only mode.

**Validates: Requirements 9.3**

### Property 6: ISR Timing
The Timer2 ISR uses only integer operations and direct port manipulation — no floating-point, no function calls — guaranteeing completion within the 20µs period.

**Validates: Requirements 7.7**

### Property 7: Dead-Band Determinism
When potentiometer is in range [490, 534], speed output is always exactly 0 regardless of previous state (after hysteresis exit is not triggered).

**Validates: Requirements 2.4**

### Property 8: Non-Linear Compensation Monotonicity
For speed values 1..400, the compensation formula `405 - (1/(speed+9))*5500` is strictly monotonically increasing, producing pulse periods that decrease (faster stepping) with increasing speed.

**Validates: Requirements 7.4**

## Error Handling

### IMU Initialization Failure

**Condition:** WHO_AM_I register (0x75) does not return 0x68, or I2C transaction times out within 100ms during configuration.
**Response:** Robot halts operation, blinks onboard LED at 4Hz indefinitely.
**Recovery:** Power cycle required.

### Radio Initialization Failure (TX)

**Condition:** RF24 `begin()` or configuration calls fail (radio not detected on SPI bus).
**Response:** TX blinks Status LED (pin 4) at 2Hz, does not transmit packets, displays error on OLED.
**Recovery:** Power cycle required.

### Radio Initialization Failure (Robot)

**Condition:** RF24 `begin()` fails on robot side.
**Response:** Robot keeps motor output at zero, blinks onboard LED at 2Hz.
**Recovery:** Power cycle required.

### Radio Connection Loss

**Condition:** No ACK received (TX) or no packet received (Robot) for 500ms.
**Response TX:** Status LED turns off, OLED shows connection-lost icon.
**Response Robot:** Clears movement commands (pid_setpoint → 0), continues balance-only control.
**Recovery:** Automatic when packets resume.

### Tip-Over Detection

**Condition:** Estimated tilt angle exceeds ±30°.
**Response:** Motors immediately stopped (output=0), PID integral reset, self-balance setpoint reset, start=0.
**Recovery:** Automatic when robot is placed upright (accelerometer within ±0.5° of vertical).

### Low Battery (Robot)

**Condition:** Battery voltage < 10.5V and > 8.0V (excludes disconnected battery readings).
**Response:** Onboard LED illuminated, motors permanently disabled until power cycle.
**Recovery:** Power cycle with charged battery.

### Low Battery (TX)

**Condition:** Battery percentage ≤ 10%.
**Response:** Low-battery warning icon displayed on OLED.
**Recovery:** Charge or replace battery.

## Testing Strategy

### Manual Hardware Verification

Since both targets are monolithic single-file builds for ATmega328-based boards with no abstraction layers, testing is primarily done through hardware verification:

1. **Radio Link Test:** Flash both TX and Robot, verify ACK reception via Serial debug output and Status LED behavior.
2. **Potentiometer Mapping Test:** Read mapped speed values via Serial, verify dead-band (490-534 → 0) and linear mapping outside dead-band.
3. **IMU Calibration Test:** Monitor calibration offsets via Serial, verify they stabilize after 500 samples.
4. **Angle Estimation Test:** Output fused angle to Serial, tilt robot to known angles, verify accuracy within ±2°.
5. **PID Response Test:** Apply known tilt offset, verify motor output direction and magnitude, check dead-band and clamping.
6. **Stepper Pulse Test:** Use oscilloscope to verify 20µs pulse width and correct period-to-speed relationship.
7. **Safety Test:** Tilt robot beyond 30°, verify motors stop immediately. Remove radio link, verify timeout behavior after 500ms.

### Phased Integration Approach

- **Phase 1:** Hardware pin verification (LED blink, OLED hello, pot read via Serial)
- **Phase 2:** Radio link (TX sends, Robot receives, verify via Serial)
- **Phase 3:** IMU init + calibration + raw data streaming
- **Phase 4:** Sensor fusion angle output (compare to known reference angles)
- **Phase 5:** PID + stepper control (robot held in jig, verify motor response to tilt)
- **Phase 6:** Full free-standing balance (PID tuning)
- **Phase 7:** Remote movement control integration

## platformio.ini Configuration

```ini
[platformio]
default_envs = tx, robot

[env:tx]
platform = atmelavr
board = pro16MHzatmega328
framework = arduino
build_src_filter = +<tx/>
lib_deps =
    nRF24/RF24@^1.4.7
    olikraus/U8g2@^2.35.7
lib_extra_dirs = shared

[env:robot]
platform = atmelavr
board = nanoatmega328
framework = arduino
build_src_filter = +<robot/>
lib_deps =
    nRF24/RF24@^1.4.7
lib_extra_dirs = shared
```

**Key points:**
- `build_src_filter = +<tx/>` ensures only `src/tx/main.cpp` is compiled for the TX environment
- `build_src_filter = +<robot/>` ensures only `src/robot/main.cpp` is compiled for the Robot environment
- `lib_extra_dirs = shared` makes `shared/radio_config.h` available via `#include "radio_config.h"` in both targets
- Each environment declares only the libraries it actually uses

## Data Flow

### Control Loop Timing

```
Time (ms):  0      4      8      12     16     20
            │      │      │      │      │      │
Robot Loop: ├──────┼──────┼──────┼──────┼──────┤  (250Hz)
            │ IMU  │ IMU  │ IMU  │ IMU  │ IMU  │
            │ Fuse │ Fuse │ Fuse │ Fuse │ Fuse │
            │ PID  │ PID  │ PID  │ PID  │ PID  │
            │      │      │      │      │      │
TX Loop:    ├────────────────────┼──────────────┤  (50Hz)
            │ Read Pot           │ Read Pot     │
            │ Send Packet        │ Send Packet  │
            │                    │              │

ISR:        |||||||||||||||||||||||||||||||||||||| (50kHz / 20µs)
```

### Packet Flow

```
TX: ReadPot → BuildPacket → RF24.write(packet) ──► ACK/NACK
                                                       │
RX: RF24.read(packet) → processMovement → PID setpoint adjustment
```

## Dependencies

### PlatformIO Libraries

| Library | Version | Purpose | Target |
|---------|---------|---------|--------|
| nRF24/RF24 | ^1.4.7 | nRF24L01 radio driver | Both |
| olikraus/U8g2 | ^2.35.7 | SSD1306 OLED display driver | TX only |
| Wire (built-in) | — | I2C communication (IMU, OLED) | Both |
| SPI (built-in) | — | nRF24L01 SPI communication | Both |

## Performance Considerations

- **RAM (Arduino Nano — 2KB SRAM):** Madgwick filter uses minimal RAM (~20 bytes for quaternion state or ~12 bytes for simplified complementary approach). RF24 library uses ~32 bytes. Global variables for PID/motor state fit in ~100 bytes. Total firmware well under 2KB limit. No dynamic allocation.
- **RAM (Arduino Pro Mini — 2KB SRAM):** U8g2 configured with page buffer mode (128 bytes per page) instead of full framebuffer (1024 bytes). RF24 ~32 bytes. Leaves ample room for stack.
- **Flash (Nano — 30KB):** RF24 library ~4KB, Wire ~2KB. All robot logic in single main.cpp compiles to ~12-15KB total. Well within limit.
- **Flash (Pro Mini — 30KB):** RF24 ~4KB, U8g2 ~8KB (with selected font). Total ~16-18KB.
- **ISR Speed:** Timer2 ISR must complete in <20µs. Uses only direct port manipulation (PORTD bit operations), integer comparisons, and counter increments. No floating-point, no function calls, no library usage inside ISR.
- **I2C Timing:** MPU6050 burst read (2 bytes accel + 4 bytes gyro) at 400kHz takes ~200µs total. Well within the 4ms loop budget.
- **Radio Timing:** nRF24L01 auto-ack adds ~250µs per packet exchange. At 50Hz TX rate (20ms interval), this uses ~1.25% of available time.
- **Loop Precision:** Robot uses `while(loop_timer > micros())` busy-wait to ensure exactly 4ms per loop. TX uses similar approach for 20ms loops. No interrupts disable the loop timer.
- **Single-File Benefit:** No function-call overhead between modules. Compiler can inline and optimize all code as one translation unit. Reduced include complexity keeps compilation fast.
