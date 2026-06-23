# Requirements Document

## Introduction

This document defines the requirements for a remote-controlled self-balancing robot system. The system consists of two subsystems: a custom wireless transmitter (TX) built with an Arduino Pro Mini and nRF24L01 radio, and a self-balancing robot built with an Arduino Nano, MPU6050 IMU, and two stepper motors driven by DRV8825 drivers in quarter-step mode. The project aims to replace a lost firmware implementation with an improved version featuring better sensor fusion, smoother motor control, and wireless communication via nRF24L01 radios. Development follows a phased approach from basic hardware verification through full balancing control.

## Glossary

- **TX_Controller**: The Arduino Pro Mini (16MHz ATmega328) based transmitter unit that reads user inputs and sends commands wirelessly
- **Robot_Controller**: The Arduino Nano based robot unit that balances and responds to remote commands
- **Radio_Module**: The nRF24L01 2.4GHz wireless transceiver used on both TX and robot
- **IMU**: The MPU6050 inertial measurement unit providing gyroscope and accelerometer data via I2C
- **Stepper_Driver**: The DRV8825 stepper motor driver operating in quarter-step mode
- **OLED_Display**: The SSD1306 128x64 OLED display on the transmitter showing system status
- **Sensor_Fusion_Filter**: The algorithm that combines gyroscope and accelerometer data to estimate tilt angle
- **Balance_Controller**: The control algorithm (PID or advanced) that computes motor output to maintain balance
- **Stepper_Pulse_Generator**: The timer-interrupt-driven routine that generates precisely timed step pulses for the motors
- **Joystick**: The 2-axis analog joystick on the TX_Controller with X-axis (left/right turn), Y-axis (forward/backward speed), and a push button for future functionality
- **Command_Packet**: The wireless data structure sent from TX_Controller to Robot_Controller containing speed and turn fields
- **Status_LED**: The LED on TX_Controller pin 4 indicating radio connection status
- **Battery_Monitor**: The circuit and software on TX_Controller that measures LiPo battery voltage

## Requirements

### Requirement 1: Radio Communication Initialization

**User Story:** As a user, I want both the transmitter and robot to establish a wireless radio link on power-up, so that I can remotely control the robot.

#### Acceptance Criteria

1. WHEN powered on, THE TX_Controller SHALL initialize the Radio_Module on the SPI bus and begin transmitting Command_Packets at a fixed interval of 20ms (50Hz)
2. WHEN powered on, THE Robot_Controller SHALL initialize the Radio_Module on the SPI bus and begin listening for Command_Packets
3. WHEN a Command_Packet is transmitted, THE Radio_Module SHALL use hardware auto-acknowledgment so that the TX_Controller can detect whether delivery succeeded or failed within a single packet cycle
4. IF the Radio_Module fails to initialize on the TX_Controller, THEN THE TX_Controller SHALL blink the Status_LED at 2Hz to indicate a connection failure and SHALL NOT transmit Command_Packets
5. IF the Radio_Module fails to initialize on the Robot_Controller, THEN THE Robot_Controller SHALL keep motor output at zero and blink the onboard LED at 2Hz to indicate radio failure
6. IF no Command_Packet is received for 500ms, THEN THE Robot_Controller SHALL enter a safe-stop state and set motor output to zero
7. THE TX_Controller and Robot_Controller SHALL both configure the Radio_Module to channel 108, 250kbps data rate, maximum PA level, and a shared 5-byte pipe address

### Requirement 2: Transmitter User Input

**User Story:** As a user, I want to control the robot's movement using a 2-axis joystick on the transmitter, so that I can steer the robot intuitively with forward/backward speed and left/right turning.

#### Acceptance Criteria

1. THE TX_Controller SHALL read the Joystick Y-axis and X-axis analog values at a minimum rate of 50Hz
2. THE TX_Controller SHALL encode the current mapped speed value (from Y-axis) and turn value (from X-axis) into a Command_Packet and transmit it via the Radio_Module every loop cycle, regardless of whether the values have changed since the previous cycle
3. THE TX_Controller SHALL map the Joystick Y-axis range below the dead-band (0-489) linearly to speed values (-100 to -1) and the range above the dead-band (535-1023) linearly to speed values (+1 to +100)
4. WHEN the Joystick Y-axis is in the center dead-band (490-534), THE TX_Controller SHALL set the speed field in the Command_Packet to zero
5. IF the Joystick Y-axis value transitions from inside the dead-band to outside (or vice versa), THEN THE TX_Controller SHALL apply a hysteresis of 3 ADC counts at the dead-band boundaries before changing the speed output
6. THE TX_Controller SHALL map the Joystick X-axis range below the dead-band (0-489) linearly to turn values (-100 to -1) and the range above the dead-band (535-1023) linearly to turn values (+1 to +100)
7. WHEN the Joystick X-axis is in the center dead-band (490-534), THE TX_Controller SHALL set the turn field in the Command_Packet to zero
8. IF the Joystick X-axis value transitions from inside the dead-band to outside (or vice versa), THEN THE TX_Controller SHALL apply a hysteresis of 3 ADC counts at the dead-band boundaries before changing the turn output
9. THE TX_Controller SHALL read the Joystick push button state each loop cycle and include it in the Command_Packet as a boolean field reserved for future use

### Requirement 3: Transmitter Status Display

**User Story:** As a user, I want to see battery level and connection status on the transmitter's OLED screen, so that I know the system state at a glance.

#### Acceptance Criteria

1. THE TX_Controller SHALL display the battery percentage on the OLED_Display, updated at least once per second
2. THE Battery_Monitor SHALL measure the LiPo voltage via a voltage divider on an analog input pin at least once per second
3. THE TX_Controller SHALL map the measured voltage from the LiPo operating range (3.0V-4.2V) to a percentage (0%-100%) using linear interpolation, clamping values at 0% for voltages at or below 3.0V and at 100% for voltages at or above 4.2V
4. WHEN the Radio_Module receives an acknowledgment from the Robot_Controller within 500ms of the last transmitted Command_Packet, THE TX_Controller SHALL display a connection-active icon on the OLED_Display
5. WHEN the Radio_Module loses connection (no acknowledgment for 500ms), THE TX_Controller SHALL display a connection-lost icon on the OLED_Display
6. WHILE the radio link is active (acknowledgment received within the last 500ms), THE TX_Controller SHALL illuminate the Status_LED solid
7. WHEN the Radio_Module loses connection (no acknowledgment for 500ms), THE TX_Controller SHALL turn off the Status_LED
8. IF the battery percentage drops to 10% or below, THEN THE TX_Controller SHALL display a low-battery warning indicator on the OLED_Display

### Requirement 4: IMU Initialization and Calibration

**User Story:** As a user, I want the robot to calibrate its sensors on startup, so that angle measurements are accurate from the first moment of operation.

#### Acceptance Criteria

1. WHEN powered on, THE Robot_Controller SHALL verify the IMU identity by reading the WHO_AM_I register and confirming the expected device ID before proceeding with configuration
2. WHEN the IMU identity is verified, THE Robot_Controller SHALL initialize the IMU over I2C with gyroscope full-scale set to +/-250 degrees/s and accelerometer full-scale set to +/-4g
3. WHEN the IMU identity is verified, THE Robot_Controller SHALL configure the IMU digital low-pass filter to approximately 43Hz bandwidth
4. WHEN initialization is complete, THE Robot_Controller SHALL collect 500 gyroscope samples at 4ms intervals for both pitch and yaw axes, and compute a mean calibration offset for each axis
5. WHILE calibrating, THE Robot_Controller SHALL keep motor outputs at zero
6. IF the IMU WHO_AM_I register does not return the expected device ID, or if the IMU fails to acknowledge an I2C transaction within 100ms during initialization, THEN THE Robot_Controller SHALL halt operation and blink the onboard LED at 4Hz to indicate sensor failure

### Requirement 5: Sensor Fusion and Angle Estimation

**User Story:** As a user, I want the robot to accurately estimate its tilt angle, so that it can balance reliably without drift or excessive noise.

#### Acceptance Criteria

1. THE Sensor_Fusion_Filter SHALL combine gyroscope and accelerometer data to produce a tilt angle estimate in degrees once every 4ms control loop cycle
2. THE Sensor_Fusion_Filter SHALL use the Madgwick filter algorithm with an initial beta parameter of 0.1, and SHALL consume no more than 200 bytes of RAM for filter state variables to remain within the Arduino Nano 2KB RAM constraint
3. THE Sensor_Fusion_Filter SHALL clamp the accelerometer raw value to +/-8200 before computing the accelerometer angle, limiting the accelerometer angle contribution to the range +/-45 degrees to prevent singularity errors in the asin calculation
4. THE Sensor_Fusion_Filter SHALL subtract the gyroscope calibration offset (computed at startup per Requirement 4) from each raw gyroscope sample before using it in the filter update
5. WHEN the Robot_Controller first starts balancing, THE Sensor_Fusion_Filter SHALL initialize the angle estimate to the current accelerometer-derived angle, provided that angle is within +/-0.5 degrees of vertical
6. WHEN the estimated tilt angle exceeds +/-30 degrees, THE Robot_Controller SHALL disable motor output, reset the PID integral term, and enter a stopped state
7. THE Sensor_Fusion_Filter SHALL convert gyroscope raw data to degrees/s using the scale factor derived from the +/-250 degrees/s full-scale setting (raw multiplied by 250.0 / 32768.0) before applying the filter update

### Requirement 6: Balance Control Algorithm

**User Story:** As a user, I want the robot to maintain upright balance smoothly and with minimal jitter, so that it appears stable and controlled.

#### Acceptance Criteria

1. THE Balance_Controller SHALL compute a motor speed command using a PID algorithm combining proportional (tilt angle error), integral (accumulated angle error), and derivative (angular velocity of error) terms
2. THE Balance_Controller SHALL execute at a fixed loop interval of 4ms (250Hz)
3. THE Balance_Controller SHALL implement integral windup protection by clamping the integral term to +/-400
4. THE Balance_Controller SHALL clamp total output to +/-400 to protect the stepper drivers
5. WHEN the tilt angle error is within +/-0.5 degrees and angular velocity is below 1 degree/s, THE Balance_Controller SHALL apply a dead-band of +/-5 to the output to reduce motor jitter at the balance point
6. WHEN the absolute value of the PID output exceeds 10, THE Balance_Controller SHALL add the product of the current output and 0.015 to the angle error to provide overshoot braking
7. IF no movement command is active and the PID movement setpoint is zero, THEN THE Balance_Controller SHALL adjust the self-balance setpoint by +0.0015 degrees per loop cycle when output is negative and by -0.0015 degrees per loop cycle when output is positive

### Requirement 7: Stepper Motor Pulse Generation

**User Story:** As a user, I want the motors to respond smoothly to control commands, so that the robot balances without jerky movements.

#### Acceptance Criteria

1. THE Stepper_Pulse_Generator SHALL use a hardware timer interrupt at 20us intervals (50kHz) to generate step pulses
2. THE Stepper_Pulse_Generator SHALL produce step pulses with a minimum pulse width of 20us (one timer tick high, one timer tick low) as required by the DRV8825 driver
3. THE Stepper_Pulse_Generator SHALL independently control left and right motor speed and direction using a signed throttle value in the range -400 to +400, where positive values drive the motor forward and negative values drive the motor in reverse
4. THE Stepper_Pulse_Generator SHALL apply non-linear speed compensation to linearize stepper motor response: for positive speed, pulse_period = 405 - (1/(speed + 9)) x 5500; for negative speed, pulse_period = -405 - (1/(speed - 9)) x 5500
5. WHEN motor speed is set to zero, THE Stepper_Pulse_Generator SHALL cease generating step pulses for the corresponding motor
6. THE Stepper_Pulse_Generator SHALL load updated throttle values only at the beginning of each new pulse cycle (when the pulse counter resets) to prevent mid-pulse glitches
7. THE Stepper_Pulse_Generator SHALL complete all ISR operations within the 20us timer interval using direct port manipulation without any blocking calls

### Requirement 8: Remote Movement Control

**User Story:** As a user, I want to drive the robot forward, backward, left, and right using the transmitter, so that I can navigate the robot around.

#### Acceptance Criteria

1. WHEN a forward command is received, THE Robot_Controller SHALL gradually decrease the PID setpoint angle (lean forward) at a rate of 0.05 degrees per loop cycle, limited to -2.5 degrees maximum lean, and SHALL apply an additional 0.005 degrees per loop cycle acceleration while the PID output magnitude has not yet reached max_target_speed (150)
2. WHEN a backward command is received, THE Robot_Controller SHALL gradually increase the PID setpoint angle (lean backward) at a rate of 0.05 degrees per loop cycle, limited to +2.5 degrees maximum lean, and SHALL apply an additional 0.005 degrees per loop cycle acceleration while the PID output magnitude has not yet reached max_target_speed (150)
3. WHEN a turn-left command is received, THE Robot_Controller SHALL add a differential speed offset of 30 (turning_speed) to the left motor output and subtract 30 from the right motor output
4. WHEN a turn-right command is received, THE Robot_Controller SHALL subtract a differential speed offset of 30 (turning_speed) from the left motor output and add 30 to the right motor output
5. WHEN no forward or backward command is active and the PID setpoint magnitude is greater than 0.5 degrees, THE Robot_Controller SHALL gradually return the PID setpoint toward zero at a rate of 0.05 degrees per loop cycle
6. WHEN no forward or backward command is active and the PID setpoint magnitude is 0.5 degrees or less, THE Robot_Controller SHALL set the PID setpoint to exactly 0 degrees

### Requirement 9: Safety and Fault Protection

**User Story:** As a user, I want the robot to stop safely when it falls or loses communication, so that the hardware is protected.

#### Acceptance Criteria

1. WHEN the tilt angle exceeds +/-30 degrees, THE Robot_Controller SHALL within the same control loop cycle (4ms) set motor output to zero, reset the integral term, reset the self-balance setpoint to 0 degrees, and enter a stopped state
2. WHILE in stopped state, IF the accelerometer angle is within +/-0.5 degrees of vertical, THEN THE Robot_Controller SHALL re-initialize the gyro angle from the accelerometer and resume PID balance control
3. IF no Command_Packet is received for 500ms (125 consecutive loop cycles), THEN THE Robot_Controller SHALL reset the PID setpoint angle to zero and continue balance-only control without accepting directional commands until a new Command_Packet is received
4. IF the battery voltage on the Robot_Controller drops below 10.5V (and remains above 8.0V to exclude disconnected readings), THEN THE Robot_Controller SHALL illuminate the onboard LED and permanently disable motor output until power is cycled

### Requirement 10: PlatformIO Project Structure

**User Story:** As a developer, I want a well-organized PlatformIO project with separate environments for the transmitter and robot, so that I can build and flash each independently.

#### Acceptance Criteria

1. THE platformio.ini SHALL define separate build environments for the TX_Controller (board: pro16MHzatmega328) and the Robot_Controller (board: nanoatmega328), each using the atmelavr platform and Arduino framework
2. THE project SHALL contain all transmitter firmware in a single source file src/tx/main.cpp and all robot firmware in a single source file src/robot/main.cpp, with each environment using a build_src_filter that compiles only the main.cpp from its corresponding directory
3. THE TX_Controller environment SHALL declare lib_deps for nRF24/RF24@^1.4.7 and olikraus/U8g2@^2.35.7, and THE Robot_Controller environment SHALL declare lib_deps for nRF24/RF24@^1.4.7
4. THE project SHALL include a shared/ directory at the project root containing a radio_config.h header that defines the CommandPacket structure and radio configuration constants (radio channel and pipe address), and both environments SHALL reference this directory via lib_extra_dirs so the shared header is available to both TX and robot firmware

### Requirement 11: Repository Cleanup

**User Story:** As a developer, I want the repository to be clean and well-organized after development is complete, so that it is easy to navigate and maintain.

#### Acceptance Criteria

1. THE project SHALL remove any unused placeholder files (such as .gitkeep files in directories that now contain source code)
2. THE project SHALL remove or archive the SELF_BALANCING_ROBOT_SOFTWARE_CONTEXT directory since the original reference code has been superseded by the new firmware
3. THE project SHALL include a README.md at the project root written as a quickstart guide that is concise and to the point, covering: what the project is, hardware needed (components and wiring), how to build and flash each target, pin assignments for both TX and Robot, and how to use the remote — written so that someone unfamiliar with the repo (or the original developer returning months later) can get up and running quickly
4. THE project SHALL ensure all source files contain a brief header comment describing their purpose
5. THE project SHALL remove any debug Serial.print statements that are not needed for production use, or gate them behind a DEBUG preprocessor flag
6. THE project SHALL verify that both environments compile cleanly with no warnings using pio run
