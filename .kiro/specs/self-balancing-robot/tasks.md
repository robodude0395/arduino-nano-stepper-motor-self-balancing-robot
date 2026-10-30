# Implementation Plan: Self-Balancing Robot

## Overview

Incremental firmware implementation for a remote-controlled self-balancing robot using PlatformIO. The system has two monolithic firmware targets (TX on Arduino Pro Mini, Robot on Arduino Nano) communicating via nRF24L01 radios. Each phase adds functionality to the respective `main.cpp` file, building on the previous phase. Hardware checkpoints allow physical verification at each milestone.

## Tasks

- [x] 1. Set up project structure and shared radio header
  - [x] 1.1 Create platformio.ini with dual environments
    - Define `[env:tx]` for pro16MHzatmega328 with `build_src_filter = +<tx/>` and lib_deps for RF24@^1.4.7 and U8g2@^2.35.7
    - Define `[env:robot]` for nanoatmega328 with `build_src_filter = +<robot/>` and lib_deps for RF24@^1.4.7
    - Both environments: platform = atmelavr, framework = arduino, lib_extra_dirs = shared
    - _Requirements: 10.1, 10.2, 10.3, 10.4_
  - [x] 1.2 Create directory structure and shared/radio_config.h
    - Create `src/tx/` and `src/robot/` directories
    - Create `shared/radio_config.h` with `CommandPacket` struct (speed int8_t, turn int8_t, flags uint8_t)
    - Define RADIO_CHANNEL = 108 and RADIO_PIPE_ADDRESS 5-byte array
    - Include header guards and `<stdint.h>`
    - _Requirements: 10.4, 1.7_

- [x] 2. TX radio communication (initial minimal TX main.cpp)
  - [x] 2.1 Create src/tx/main.cpp with radio init and LED status
    - Include Arduino.h, SPI.h, RF24.h, and radio_config.h
    - Define pin constants: CE, CSN, LED_PIN (pin 4)
    - Initialize RF24 object with CE/CSN pins
    - In `setup()`: initialize Serial, SPI, RF24 (channel 108, 250kbps, PA_MAX, auto-ack, open writing pipe)
    - If RF24 init fails: blink LED at 2Hz, halt transmission
    - In `loop()`: build a zero-speed CommandPacket, call RF24.write(), check ACK
    - Track connection via lastAckTime — if >500ms since last ACK, set connected=false
    - LED: solid HIGH when connected, OFF when disconnected, 2Hz blink on init failure
    - Loop timing: 20ms interval (50Hz)
    - _Requirements: 1.1, 1.3, 1.4, 1.7, 3.6, 3.7_
  - [x] 2.2 ⚡ HARDWARE CHECKPOINT: Verify TX radio compiles and uploads
    - Build with `pio run -e tx` — verify successful compilation
    - Upload to Pro Mini with `pio run -e tx -t upload`
    - Verify LED blinks at 2Hz if no receiver present (no ACK)
    - Verify via Serial debug that RF24 initializes successfully
    - _Requirements: 1.1, 1.4_

- [x] 3. TX potentiometer input
  - [x] 3.1 Add potentiometer reading with dead-band and hysteresis to src/tx/main.cpp
    - Define POT_PIN analog pin constant
    - Define dead-band boundaries: 490 (lower), 534 (upper)
    - Define HYSTERESIS = 3 ADC counts
    - Add global state: inDeadBand flag, previous speed value
    - In `loop()`: read analogRead(POT_PIN)
    - Implement dead-band logic: if in dead-band, only exit if raw < (490 - 3) or raw > (534 + 3)
    - Map below dead-band (0-489) → speed (-100 to -1), above dead-band (535-1023) → speed (+1 to +100)
    - Set packet.speed to computed value, transmit every cycle
    - _Requirements: 2.1, 2.2, 2.3, 2.4, 2.5_
  - [x] 3.2 ⚡ HARDWARE CHECKPOINT: Verify potentiometer mapping via Serial
    - Upload updated TX firmware
    - Open Serial monitor, move potentiometer through full range
    - Verify: center position (490-534) outputs speed=0
    - Verify: below dead-band maps linearly to -100..-1
    - Verify: above dead-band maps linearly to +1..+100
    - Verify: hysteresis prevents jitter at dead-band boundaries
    - _Requirements: 2.3, 2.4, 2.5_

- [x] 4. TX display and battery monitoring (completing TX firmware)
  - [x] 4.1 Add OLED display and battery monitoring to src/tx/main.cpp
    - Include U8g2lib.h and Wire.h
    - Initialize U8g2 display object (SSD1306 128x64, page buffer mode for RAM efficiency)
    - Define BAT_PIN analog pin, voltage divider ratio, diode offset constants
    - In `setup()`: initialize I2C, initialize OLED display
    - In `loop()` every 50th iteration (~1 second): read battery ADC, compute voltage, map 3.0V-4.2V → 0%-100% (clamped)
    - Render OLED: battery percentage, connection-active or connection-lost icon
    - If battery ≤ 10%: display low-battery warning indicator
    - _Requirements: 3.1, 3.2, 3.3, 3.4, 3.5, 3.8_
  - [x] 4.2 ⚡ HARDWARE CHECKPOINT: Verify complete TX firmware
    - Upload final TX firmware, verify OLED displays battery % and connection icon
    - Verify connection-lost icon appears when no receiver
    - Verify LED status matches connection state
    - Verify all TX functionality works together (pot + radio + display + LED)
    - _Requirements: 3.1, 3.4, 3.5, 3.6, 3.7, 3.8_

- [x] 5. Robot radio receive and IMU initialization
  - [x] 5.1 Create src/robot/main.cpp with radio receive and IMU init/calibration
    - Include Arduino.h, SPI.h, RF24.h, Wire.h, and radio_config.h
    - Define pin constants: CE, CSN, motor step/dir pins (2,3,4,5), LED pin 13
    - Define IMU I2C address (0x68), gyro scale factor (250.0/32768.0)
    - Initialize RF24 as receiver: channel 108, 250kbps, PA_MAX, open reading pipe, startListening
    - If RF24 init fails: keep motor output at zero, blink LED at 2Hz
    - Initialize I2C (Wire.begin(), TWBR=12 for 400kHz)
    - Verify IMU WHO_AM_I register (0x75) returns 0x68 — if not, blink LED at 4Hz, halt
    - Configure IMU: wake (0x6B=0x00), gyro ±250°/s (0x1B=0x00), accel ±4g (0x1C=0x08), DLPF 43Hz (0x1A=0x03)
    - Calibrate gyroscope: 500 samples at 4ms intervals, compute mean offset for pitch and yaw
    - Blink LED during calibration, keep motors at zero
    - In `loop()`: check radio.available(), read CommandPacket, track last_receive_time
    - If millis() - last_receive_time > 500ms: clear movement commands
    - Set loop_timer for 4ms (250Hz) control cycle
    - _Requirements: 1.2, 1.5, 1.6, 1.7, 4.1, 4.2, 4.3, 4.4, 4.5, 4.6, 9.3_
  - [x] 5.2 ⚡ HARDWARE CHECKPOINT: Verify robot radio + IMU init
    - Build with `pio run -e robot` — verify successful compilation
    - Upload to Arduino Nano
    - Verify via Serial: IMU WHO_AM_I confirmed, calibration completes (500 samples)
    - Verify via Serial: radio receives packets from TX (when TX is powered on)
    - Verify: radio timeout detected when TX is powered off
    - _Requirements: 1.2, 4.1, 4.4, 9.3_

- [x] 6. Robot sensor fusion (Madgwick filter)
  - [x] 6.1 Add Madgwick sensor fusion filter to src/robot/main.cpp
    - Define accelerometer calibration value and clamp constant (±8200)
    - Add Madgwick filter state variables (quaternion or simplified complementary with gradient descent)
    - Define beta parameter = 0.1
    - In `loop()`: read accelerometer (register 0x3F, 2 bytes), add calibration offset, clamp to ±8200
    - Compute angle_acc = asin(clamped_raw / 8200.0) * 57.296 (radians to degrees)
    - Read gyroscope (register 0x43, 4 bytes): extract pitch and yaw, subtract calibration offsets
    - Convert gyro to degrees/s: raw * (250.0/32768.0)
    - Run Madgwick filter iteration: combine gyro rate and accel angle for corrected angle estimate
    - Add start condition: if angle_acc within ±0.5° of vertical, initialize angle_gyro and set start=1
    - Output fused angle estimate for PID consumption
    - Ensure filter state uses ≤200 bytes RAM
    - _Requirements: 5.1, 5.2, 5.3, 5.4, 5.5, 5.7_
  - [ ]* 6.2 Write property test for Angle Boundedness
    - **Property 1: Angle Boundedness** — The Madgwick filter output shall always produce an angle in the range [-90°, +90°]. If the raw accelerometer value exceeds ±8200, it is clamped before angle computation.
    - **Validates: Requirements 5.3**
  - [x] 6.3 ⚡ HARDWARE CHECKPOINT: Verify angle estimation
    - Upload updated robot firmware
    - Output fused angle to Serial at 250Hz
    - Tilt robot to known angles (0°, ±15°, ±30°), verify accuracy within ±2°
    - Verify angle stays bounded even at extreme tilts
    - Verify calibration offsets produce near-zero drift when stationary
    - _Requirements: 5.1, 5.3, 5.4_

- [x] 7. Robot PID balance controller
  - [x] 7.1 Add PID balance controller to src/robot/main.cpp
    - Define PID gains: Kp=15, Ki=1.5, Kd=30
    - Define integral/output clamp = ±400, dead-band = ±5
    - Define brake factor = 0.015, self-balance adjustment = ±0.0015°/loop
    - Add PID state variables: pid_error_temp, pid_i_mem, pid_output, pid_last_d_error, pid_setpoint, self_balance_pid_setpoint
    - In `loop()` after sensor fusion:
      - Compute error = angle - self_balance_pid_setpoint - pid_setpoint
      - Apply brake function: if |pid_output| > 10, error += pid_output * 0.015
      - Integral: pid_i_mem += Ki * error, clamp ±400
      - Output: Kp*error + pid_i_mem + Kd*(error - last_error), clamp ±400
      - Dead-band: if |output| < 5, output = 0
      - Store error for next derivative calculation
    - Safety check: if |angle| > 30° OR start==0: output=0, reset integral, reset setpoints, start=0
    - Re-start check: if start==0 and angle_acc within ±0.5°, re-initialize and set start=1
    - Self-balance adjustment: if pid_setpoint==0 and output<0: self_balance += 0.0015; if output>0: self_balance -= 0.0015
    - _Requirements: 6.1, 6.2, 6.3, 6.4, 6.5, 6.6, 6.7, 9.1, 9.2, 5.6_
  - [ ]* 7.2 Write property test for PID Output Boundedness
    - **Property 2: PID Output Boundedness** — For any input angle and state, `|pid_output| <= 400` always holds after the clamp step.
    - **Validates: Requirements 6.4**
  - [ ]* 7.3 Write property test for Integral Windup Protection
    - **Property 3: Integral Windup Protection** — `|pid_i_mem| <= 400` is invariant across all loop iterations.
    - **Validates: Requirements 6.3**
  - [ ]* 7.4 Write property test for Motor Safety on Tip-Over
    - **Property 4: Motor Safety on Tip-Over** — When `|angle| > 30°`, motor output is set to zero within the same loop cycle (4ms).
    - **Validates: Requirements 9.1**
  - [x] 7.5 ⚡ HARDWARE CHECKPOINT: Verify PID balance response
    - Upload updated robot firmware
    - Hold robot in jig, tilt slightly — verify motor output direction opposes tilt
    - Verify PID output stays within ±400 (Serial monitor)
    - Verify dead-band: no motor output when angle error < ±5 counts
    - Tilt beyond 30° — verify motors stop immediately (safety)
    - Return to vertical within ±0.5° — verify restart
    - _Requirements: 6.1, 6.4, 6.5, 9.1, 9.2_

- [ ] 8. Robot stepper pulse generation
  - [x] 8.1 Add Timer2 ISR and non-linear stepper compensation to src/robot/main.cpp
    - Define motor throttle variables as `volatile` for ISR safety: throttle_left_motor, throttle_right_motor, throttle_counter_left_motor, throttle_counter_right_motor, throttle_left_motor_memory, throttle_right_motor_memory
    - In `setup()`: configure Timer2 CTC mode, prescaler 8, OCR2A = 39 → 20µs period, enable OCIE2A interrupt
    - Implement ISR(TIMER2_COMPA_vect):
      - Left motor: increment counter, if > memory → reload throttle, set direction via PORTD bits, reset counter
      - Step pulse: counter==1 → step pin HIGH (PORTD), counter==2 → step pin LOW (PORTD)
      - Right motor: mirror logic for right motor pins
      - All operations: direct port manipulation only, no floating-point, no function calls
    - In `loop()` after PID: apply non-linear compensation
      - If output_left > 0: compensated = 405 - (1/(output_left+9))*5500
      - If output_left < 0: compensated = -405 - (1/(output_left-9))*5500
      - Same for output_right
    - Compute throttle values: if output>0: throttle = 400 - compensated; if output<0: throttle = -400 - compensated; if 0: throttle = 0
    - Write throttle_left_motor and throttle_right_motor (atomic for ISR)
    - Configure motor pins (2,3,4,5) as OUTPUT in setup()
    - _Requirements: 7.1, 7.2, 7.3, 7.4, 7.5, 7.6, 7.7_
  - [ ]* 8.2 Write property test for Non-Linear Compensation Monotonicity
    - **Property 8: Non-Linear Compensation Monotonicity** — For speed values 1..400, the compensation formula `405 - (1/(speed+9))*5500` is strictly monotonically increasing.
    - **Validates: Requirements 7.4**
  - [~] 8.3 ⚡ HARDWARE CHECKPOINT: Verify stepper pulse generation
    - Upload updated robot firmware
    - With robot in jig: tilt slightly, verify motors spin in correct direction
    - Verify step pulse width is ~20µs (oscilloscope if available)
    - Verify motors stop when throttle is zero
    - Verify speed increases smoothly with increasing tilt angle
    - Verify direction reversal works correctly
    - _Requirements: 7.1, 7.2, 7.3, 7.5_

- [ ] 9. Robot movement command processing
  - [~] 9.1 Add movement command processing to src/robot/main.cpp
    - Define movement constants: max_target_speed=150, turning_speed=30, movement_rate=0.05°/loop, movement_acceleration=0.005°/loop, max_lean=±2.5°
    - Add pid_output_left and pid_output_right variables
    - In `loop()` after PID computation, before non-linear compensation:
      - Initialize pid_output_left = pid_output, pid_output_right = pid_output
      - Forward command (received_packet.speed > 0): decrease pid_setpoint by 0.05/loop, clamp at -2.5°, add 0.005/loop acceleration if |output| < 150
      - Backward command (received_packet.speed < 0): increase pid_setpoint by 0.05/loop, clamp at +2.5°, add 0.005/loop acceleration if |output| < 150
      - No command: return pid_setpoint toward 0 at 0.05/loop; if |pid_setpoint| ≤ 0.5, set to exactly 0
      - Turn left (received_packet.turn < 0): pid_output_left += 30, pid_output_right -= 30
      - Turn right (received_packet.turn > 0): pid_output_left -= 30, pid_output_right += 30
    - Apply non-linear compensation and throttle computation to pid_output_left and pid_output_right separately
    - _Requirements: 8.1, 8.2, 8.3, 8.4, 8.5, 8.6_
  - [ ]* 9.2 Write property test for Radio Timeout Safety
    - **Property 5: Radio Timeout Safety** — If no packet received for 500ms (125 loops), movement commands are cleared and robot returns to balance-only mode.
    - **Validates: Requirements 9.3**
  - [~] 9.3 ⚡ HARDWARE CHECKPOINT: Verify remote movement control
    - Upload updated robot firmware, power on TX
    - With robot in jig: push pot forward → verify motors lean robot forward direction
    - Push pot backward → verify reverse direction
    - Verify speed ramps gradually (0.05°/loop rate)
    - Verify lean angle caps at ±2.5°
    - Release pot to center → verify pid_setpoint returns to 0
    - Power off TX → verify robot returns to balance-only after 500ms
    - _Requirements: 8.1, 8.2, 8.5, 8.6, 9.3_

- [ ] 10. Full integration, safety, and final verification
  - [~] 10.1 Finalize robot main.cpp with battery monitoring and atomic writes
    - Add battery voltage reading: analogRead battery pin each loop
    - If voltage < 1050 (10.5V) and > 800 (8.0V): set low_bat flag, illuminate LED pin 13
    - In safety check: if low_bat → output=0, permanently disable motors until power cycle
    - Ensure all throttle writes to volatile ISR variables are atomic (cli/sei or single-word writes)
    - Verify all global state is correctly ordered in the 4ms loop
    - Final code review: ensure no dynamic memory allocation, all variables fit in 2KB RAM
    - _Requirements: 9.4, 7.6_
  - [ ]* 10.2 Write property test for Dead-Band Determinism
    - **Property 7: Dead-Band Determinism** — When potentiometer is in range [490, 534], speed output is always exactly 0 regardless of previous state (after hysteresis exit is not triggered).
    - **Validates: Requirements 2.4**
  - [ ]* 10.3 Write property test for ISR Timing
    - **Property 6: ISR Timing** — The Timer2 ISR uses only integer operations and direct port manipulation — no floating-point, no function calls — guaranteeing completion within the 20µs period.
    - **Validates: Requirements 7.7**
  - [~] 10.4 ⚡ HARDWARE CHECKPOINT: Full free-standing balance test
    - Upload final robot firmware, power on TX
    - Place robot upright on flat surface — verify it achieves stable balance
    - Verify forward/backward movement responds to potentiometer
    - Verify recovery from gentle pushes
    - Tilt beyond 30° — verify motors stop, place upright — verify restart
    - Power off TX — verify balance-only mode continues
    - Verify battery monitoring LED behavior
    - _Requirements: 6.1, 8.1, 8.2, 9.1, 9.2, 9.3, 9.4_

- [~] 11. Final checkpoint
  - Ensure all code compiles cleanly for both environments (`pio run`), ask the user if questions arise.

## Notes

- Tasks marked with `*` are optional property-based tests and can be skipped for faster MVP
- Each task references specific requirements for traceability
- ⚡ HARDWARE CHECKPOINT tasks require physical hardware — the user must manually upload firmware and test on actual boards
- All firmware is monolithic: each phase ADDS code to the existing `main.cpp` rather than creating new files
- The only separate file created is `shared/radio_config.h` — everything else is inline in the respective main.cpp
- Property tests validate correctness properties defined in the design document
- PID tuning (Kp, Ki, Kd) will likely need adjustment during hardware checkpoints — these are starting values from the design

## Task Dependency Graph

```json
{
  "waves": [
    { "id": 0, "tasks": ["1.1", "1.2"] },
    { "id": 1, "tasks": ["2.1"] },
    { "id": 2, "tasks": ["2.2"] },
    { "id": 3, "tasks": ["3.1"] },
    { "id": 4, "tasks": ["3.2"] },
    { "id": 5, "tasks": ["4.1"] },
    { "id": 6, "tasks": ["4.2", "5.1"] },
    { "id": 7, "tasks": ["5.2", "6.1"] },
    { "id": 8, "tasks": ["6.2", "6.3"] },
    { "id": 9, "tasks": ["7.1"] },
    { "id": 10, "tasks": ["7.2", "7.3", "7.4"] },
    { "id": 11, "tasks": ["7.5"] },
    { "id": 12, "tasks": ["8.1"] },
    { "id": 13, "tasks": ["8.2", "8.3"] },
    { "id": 14, "tasks": ["9.1"] },
    { "id": 15, "tasks": ["9.2", "9.3"] },
    { "id": 16, "tasks": ["10.1"] },
    { "id": 17, "tasks": ["10.2", "10.3"] },
    { "id": 18, "tasks": ["10.4"] }
  ]
}
```
