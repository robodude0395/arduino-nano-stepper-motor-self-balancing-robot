// src/robot/main.cpp — Self-Balancing Robot Firmware
// Working version: IMU + complementary filter + PID + stepper ISR + tuning display
// Radio receive present but NOT checked in control loop (to be re-enabled later)

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <U8g2lib.h>

// ─── Pin Definitions ───
const uint8_t LED_PIN = 13;

// Motor pins (back to original wiring)
const uint8_t M1_ENABLE = 6;
const uint8_t M1_STEP   = 2;
const uint8_t M1_DIR    = 4;
const uint8_t M2_ENABLE = 7;
const uint8_t M2_STEP   = 3;
const uint8_t M2_DIR    = 5;

// PID tuning buttons (analog pins used as digital)
const uint8_t BTN_LEFT   = A0;
const uint8_t BTN_MID    = A1;
const uint8_t BTN_RIGHT  = A2;

// EEPROM addresses
const int EEPROM_P_ADDR = 0;
const int EEPROM_I_ADDR = 4;
const int EEPROM_D_ADDR = 8;
const int EEPROM_CAL_ADDR = 12;
const int EEPROM_MAGIC_ADDR = 14;
const byte EEPROM_MAGIC = 0xAB;

// ─── Constants ───
const int IMU_ADDRESS = 0x68;
const unsigned long LOOP_INTERVAL_US = 4000;   // 4ms (250Hz)
const int GYRO_CALIBRATION_SAMPLES   = 500;

// Sensor fusion
int acc_calibration_value = 1600;  // Default (your tuned value)

// PID gains (loaded from EEPROM or defaults)
float pid_p_gain = 20.5;
float pid_i_gain = 1.5;
float pid_d_gain = 0.0;
float turning_speed = 30.0;
float max_target_speed = 150.0;

// ─── Global State ───
U8G2_SSD1306_128X64_NONAME_F_HW_I2C tuningDisplay(U8G2_R0, U8X8_PIN_NONE);

unsigned long loop_timer = 0;
long gyro_pitch_calibration_value = 0;
long gyro_yaw_calibration_value = 0;

byte start = 0;

// Sensor fusion state
float angle_gyro = 0;
float angle_acc = 0;
int16_t gyro_pitch_data_raw = 0;
int16_t gyro_yaw_data_raw = 0;
int16_t accelerometer_data_raw = 0;

// PID state
float pid_error_temp = 0;
float pid_i_mem = 0;
float pid_setpoint = 0;
float pid_output = 0;
float pid_last_d_error = 0;
float pid_output_left = 0, pid_output_right = 0;
float self_balance_pid_setpoint = 0;
byte low_bat = 0;

// Motor control (volatile for ISR)
volatile int throttle_left_motor = 0, throttle_right_motor = 0;
volatile int throttle_counter_left_motor = 0, throttle_counter_right_motor = 0;
volatile int throttle_left_motor_memory = 0, throttle_right_motor_memory = 0;
int left_motor = 0, right_motor = 0;

// Tuning interface
uint8_t tuningParam = 0;  // 0=P, 1=I, 2=D, 3=CAL
bool lastBtnLeft = false, lastBtnMid = false, lastBtnRight = false;

// ─── EEPROM helpers ───
void savePidToEeprom() {
  EEPROM.put(EEPROM_P_ADDR, pid_p_gain);
  EEPROM.put(EEPROM_I_ADDR, pid_i_gain);
  EEPROM.put(EEPROM_D_ADDR, pid_d_gain);
  EEPROM.put(EEPROM_CAL_ADDR, acc_calibration_value);
  EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
}

void loadPidFromEeprom() {
  if (EEPROM.read(EEPROM_MAGIC_ADDR) == EEPROM_MAGIC) {
    EEPROM.get(EEPROM_P_ADDR, pid_p_gain);
    EEPROM.get(EEPROM_I_ADDR, pid_i_gain);
    EEPROM.get(EEPROM_D_ADDR, pid_d_gain);
    EEPROM.get(EEPROM_CAL_ADDR, acc_calibration_value);
  }
}

// ─── Tuning buttons ───
void handleTuningButtons() {
  bool btnLeft = !digitalRead(BTN_LEFT);
  bool btnMid = !digitalRead(BTN_MID);
  bool btnRight = !digitalRead(BTN_RIGHT);

  if (btnMid && !lastBtnMid) {
    tuningParam = (tuningParam + 1) % 4;
  }
  if (btnLeft && !lastBtnLeft) {
    switch (tuningParam) {
      case 0: pid_p_gain -= 0.5; if (pid_p_gain < 0) pid_p_gain = 0; break;
      case 1: pid_i_gain -= 0.05; if (pid_i_gain < 0) pid_i_gain = 0; break;
      case 2: pid_d_gain -= 0.5; if (pid_d_gain < 0) pid_d_gain = 0; break;
      case 3: acc_calibration_value -= 100; break;
    }
    savePidToEeprom();
  }
  if (btnRight && !lastBtnRight) {
    switch (tuningParam) {
      case 0: pid_p_gain += 0.5; break;
      case 1: pid_i_gain += 0.05; break;
      case 2: pid_d_gain += 0.5; break;
      case 3: acc_calibration_value += 100; break;
    }
    savePidToEeprom();
  }
  lastBtnLeft = btnLeft;
  lastBtnMid = btnMid;
  lastBtnRight = btnRight;
}

// ─── Tuning display ───
void updateTuningDisplay() {
  tuningDisplay.clearBuffer();
  tuningDisplay.setFont(u8g2_font_6x10_tr);
  tuningDisplay.setCursor(0, 10);
  tuningDisplay.print(tuningParam == 0 ? F(">P:") : F(" P:"));
  tuningDisplay.print(pid_p_gain, 1);
  tuningDisplay.setCursor(0, 22);
  tuningDisplay.print(tuningParam == 1 ? F(">I:") : F(" I:"));
  tuningDisplay.print(pid_i_gain, 2);
  tuningDisplay.setCursor(0, 34);
  tuningDisplay.print(tuningParam == 2 ? F(">D:") : F(" D:"));
  tuningDisplay.print(pid_d_gain, 1);
  tuningDisplay.setCursor(0, 46);
  tuningDisplay.print(tuningParam == 3 ? F(">CAL:") : F(" CAL:"));
  tuningDisplay.print(acc_calibration_value);
  tuningDisplay.setCursor(0, 60);
  tuningDisplay.print(F("Ang:"));
  tuningDisplay.print(angle_gyro, 1);
  tuningDisplay.sendBuffer();
}

// ─── IMU helper ───
void imuWriteRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void haltWithBlink(unsigned long halfPeriodMs) {
  while (true) {
    digitalWrite(LED_PIN, HIGH);
    delay(halfPeriodMs);
    digitalWrite(LED_PIN, LOW);
    delay(halfPeriodMs);
  }
}

// ─── Timer1 ISR: Stepper pulse generation (every 20µs) ───
ISR(TIMER1_COMPA_vect) {
  // Left motor (M1) - step pin 2, dir pin 4
  throttle_counter_left_motor++;
  if (throttle_counter_left_motor > throttle_left_motor_memory) {
    throttle_counter_left_motor = 0;
    throttle_left_motor_memory = throttle_left_motor;
    if (throttle_left_motor_memory < 0) {
      PORTD |= (1 << 4);
      throttle_left_motor_memory *= -1;
    } else {
      PORTD &= ~(1 << 4);
    }
  }
  else if (throttle_counter_left_motor == 1) PORTD |= (1 << 2);
  else if (throttle_counter_left_motor == 2) PORTD &= ~(1 << 2);

  // Right motor (M2) - step pin 3, dir pin 5
  throttle_counter_right_motor++;
  if (throttle_counter_right_motor > throttle_right_motor_memory) {
    throttle_counter_right_motor = 0;
    throttle_right_motor_memory = throttle_right_motor;
    if (throttle_right_motor_memory < 0) {
      PORTD |= (1 << 5);
      throttle_right_motor_memory *= -1;
    } else {
      PORTD &= ~(1 << 5);
    }
  }
  else if (throttle_counter_right_motor == 1) PORTD |= (1 << 3);
  else if (throttle_counter_right_motor == 2) PORTD &= ~(1 << 3);
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  TWBR = 12;

  // Pins
  pinMode(M1_STEP, OUTPUT);
  pinMode(M1_DIR, OUTPUT);
  pinMode(M1_ENABLE, OUTPUT);
  pinMode(M2_STEP, OUTPUT);
  pinMode(M2_DIR, OUTPUT);
  pinMode(M2_ENABLE, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BTN_LEFT, INPUT_PULLUP);
  pinMode(BTN_MID, INPUT_PULLUP);
  pinMode(BTN_RIGHT, INPUT_PULLUP);

  digitalWrite(M1_ENABLE, LOW);
  digitalWrite(M2_ENABLE, LOW);

  // Load PID from EEPROM (or use defaults if no saved values)
  loadPidFromEeprom();

  // Init display
  tuningDisplay.begin();

  // IMU init
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(0x75);
  Wire.endTransmission();
  Wire.requestFrom(IMU_ADDRESS, 1);
  if (!Wire.available() || Wire.read() != 0x68) {
    haltWithBlink(125);
  }

  imuWriteRegister(0x6B, 0x00);  // Wake
  imuWriteRegister(0x1B, 0x00);  // Gyro ±250°/s
  imuWriteRegister(0x1C, 0x08);  // Accel ±4g
  imuWriteRegister(0x1A, 0x03);  // DLPF 43Hz

  // Gyro calibration
  for (int i = 0; i < GYRO_CALIBRATION_SAMPLES; i++) {
    if (i % 15 == 0) digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    Wire.beginTransmission(IMU_ADDRESS);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(IMU_ADDRESS, 4);
    gyro_yaw_calibration_value += (int16_t)(Wire.read() << 8 | Wire.read());
    gyro_pitch_calibration_value += (int16_t)(Wire.read() << 8 | Wire.read());
    delayMicroseconds(3700);
  }
  gyro_pitch_calibration_value /= GYRO_CALIBRATION_SAMPLES;
  gyro_yaw_calibration_value /= GYRO_CALIBRATION_SAMPLES;
  digitalWrite(LED_PIN, LOW);

  // Timer1: 20µs ISR (uses Timer1 to avoid pin 3/Timer2 conflict)
  TCCR1A = 0;
  TCCR1B = 0;
  TIMSK1 |= (1 << OCIE1A);          // Enable compare match A interrupt
  TCCR1B |= (1 << CS11);            // Prescaler = 8
  OCR1A = 39;                        // 20µs period: 16MHz / 8 / (39+1) = 50kHz
  TCCR1B |= (1 << WGM12);           // CTC mode

  loop_timer = micros() + LOOP_INTERVAL_US;
}

void loop() {
  // ─── NO RADIO CHECK IN LOOP (disabled to confirm balancing works) ───

  // ─── Sensor Fusion (Complementary Filter) ───
  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(0x3F);
  Wire.endTransmission();
  Wire.requestFrom(IMU_ADDRESS, 2);
  accelerometer_data_raw = (int16_t)(Wire.read() << 8 | Wire.read());
  accelerometer_data_raw += acc_calibration_value;
  if (accelerometer_data_raw > 8200) accelerometer_data_raw = 8200;
  if (accelerometer_data_raw < -8200) accelerometer_data_raw = -8200;
  angle_acc = asin((float)accelerometer_data_raw / 8200.0) * RAD_TO_DEG;

  if (start == 0 && angle_acc > -2.0 && angle_acc < 2.0) {
    angle_gyro = angle_acc;
    start = 1;
  }

  Wire.beginTransmission(IMU_ADDRESS);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(IMU_ADDRESS, 4);
  gyro_yaw_data_raw = (int16_t)(Wire.read() << 8 | Wire.read());
  gyro_pitch_data_raw = (int16_t)(Wire.read() << 8 | Wire.read());
  gyro_pitch_data_raw -= gyro_pitch_calibration_value;
  gyro_yaw_data_raw -= gyro_yaw_calibration_value;

  angle_gyro += gyro_pitch_data_raw * 0.000031;
  angle_gyro = angle_gyro * 0.9996 + angle_acc * 0.0004;

  // ─── PID Balance Controller ───
  pid_error_temp = angle_gyro - self_balance_pid_setpoint - pid_setpoint;
  if (pid_output > 10 || pid_output < -10) pid_error_temp += pid_output * 0.015;

  pid_i_mem += pid_i_gain * pid_error_temp;
  if (pid_i_mem > 400) pid_i_mem = 400;
  else if (pid_i_mem < -400) pid_i_mem = -400;

  pid_output = pid_p_gain * pid_error_temp + pid_i_mem + pid_d_gain * (pid_error_temp - pid_last_d_error);
  if (pid_output > 400) pid_output = 400;
  else if (pid_output < -400) pid_output = -400;

  pid_last_d_error = pid_error_temp;

  if (pid_output < 5 && pid_output > -5) pid_output = 0;

  if (angle_gyro > 30 || angle_gyro < -30 || start == 0 || low_bat == 1) {
    pid_output = 0;
    pid_i_mem = 0;
    start = 0;
    self_balance_pid_setpoint = 0;
  }

  if (pid_setpoint == 0) {
    if (pid_output < 0) self_balance_pid_setpoint += 0.0015;
    if (pid_output > 0) self_balance_pid_setpoint -= 0.0015;
  }

  // Copy to left/right
  pid_output_left = pid_output;
  pid_output_right = pid_output;

  // ─── Non-linear stepper compensation ───
  if (pid_output_left > 0) pid_output_left = 405 - (1 / (pid_output_left + 9)) * 5500;
  else if (pid_output_left < 0) pid_output_left = -405 - (1 / (pid_output_left - 9)) * 5500;

  if (pid_output_right > 0) pid_output_right = 405 - (1 / (pid_output_right + 9)) * 5500;
  else if (pid_output_right < 0) pid_output_right = -405 - (1 / (pid_output_right - 9)) * 5500;

  if (pid_output_left > 0) left_motor = 400 - pid_output_left;
  else if (pid_output_left < 0) left_motor = -400 - pid_output_left;
  else left_motor = 0;

  if (pid_output_right > 0) right_motor = 400 - pid_output_right;
  else if (pid_output_right < 0) right_motor = -400 - pid_output_right;
  else right_motor = 0;

  throttle_left_motor = left_motor;
  throttle_right_motor = right_motor;

  // ─── PID Tuning (only when not balancing) ───
  handleTuningButtons();
  if (start == 0) {
    updateTuningDisplay();
  }

  // ─── Wait for loop timer ───
  while (loop_timer > micros());
  loop_timer += LOOP_INTERVAL_US;
}
