// src/tx/main.cpp — TX Radio Communication with LED Status
// Phase 2: Radio init, zero-speed packet transmission, connection tracking, LED status
//
// LED behavior:
//   - OFF             = radio chip failed to initialize (SPI issue)
//   - Solid ON        = radio chip connected successfully via SPI
//   - Fast blink (5x) = robot connection confirmed (first ACK received)

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <U8g2lib.h>
#include <Wire.h>
#include "radio_config.h"

// ─── Pin Definitions ───
const uint8_t CE_PIN  = 7;
const uint8_t CSN_PIN = 8;
const uint8_t LED_PIN = 4;
const uint8_t JOY_Y_PIN = A1;   // Joystick Y-axis (forward/backward)
const uint8_t JOY_X_PIN = A0;   // Joystick X-axis (left/right)
const uint8_t JOY_BTN_PIN = A2;  // Joystick push button (active LOW)
const uint8_t BAT_PIN = A2;     // Battery voltage via voltage divider

// ─── Constants ───
const unsigned long LOOP_INTERVAL_MS   = 20;   // 50Hz transmission rate
const unsigned long CONNECTION_TIMEOUT = 500;  // 500ms ACK timeout
const uint8_t ROBOT_CONNECT_BLINKS    = 5;    // Number of fast blinks on robot connect

// Joystick dead-band and hysteresis
const int DEADBAND_HALF = 22;   // ±22 around center
const int HYSTERESIS    = 3;    // ADC counts
int joyYCenter = 512;           // Calibrated at startup
int joyXCenter = 512;           // Calibrated at startup

// ─── Global State ───
RF24 radio(CE_PIN, CSN_PIN);
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C display(U8G2_R2, /* reset=*/ U8X8_PIN_NONE);

bool radioInitFailed  = false;
bool radioChipOk      = false;  // true = nRF24L01 chip detected on SPI
bool robotConnected   = false;  // true = received ACK from robot
bool robotFirstConnect = false; // true = first-time robot connection (triggers blink pattern)
unsigned long lastAckTime    = 0;
unsigned long previousLoopMs = 0;

// Potentiometer state
bool yInDeadBand  = true;
bool xInDeadBand  = true;
int8_t lastSpeed  = 0;
int8_t lastTurn   = 0;

// Battery monitoring state
uint16_t loopCounter = 0;
int batteryPercent   = 0;
int batteryVoltageMv = 0;

// Blink animation state
uint8_t blinkCount     = 0;
unsigned long lastBlinkTime = 0;
bool blinkPhase        = false;  // toggling state during blink animation
bool blinkingActive    = false;  // true while blink animation is running

void setup() {
  // Initialize Serial for debug output
  Serial.begin(9600);
  Serial.println(F("TX: Starting..."));

  // Configure LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Configure joystick button pin (active LOW with internal pull-up)
  pinMode(JOY_BTN_PIN, INPUT_PULLUP);

  // Initialize RF24 radio
  if (!radio.begin()) {
    Serial.println(F("TX: Radio chip NOT detected on SPI!"));
    radioInitFailed = true;
    // LED stays OFF — indicates radio chip failure
    return;
  }

  // Radio chip detected — turn LED solid ON
  radioChipOk = true;
  digitalWrite(LED_PIN, HIGH);
  Serial.println(F("TX: Radio chip OK (LED solid ON)"));

  // Configure radio as transmitter
  radio.setChannel(RADIO_CHANNEL);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(true);
  radio.openWritingPipe(RADIO_PIPE_ADDRESS);
  radio.stopListening();

  Serial.print(F("TX: Channel="));
  Serial.println(RADIO_CHANNEL);

  // Calibrate joystick center (average 16 samples)
  long sumY = 0, sumX = 0;
  for (uint8_t i = 0; i < 16; i++) {
    sumY += analogRead(JOY_Y_PIN);
    sumX += analogRead(JOY_X_PIN);
    delay(2);
  }
  joyYCenter = sumY / 16;
  joyXCenter = sumX / 16;
  Serial.print(F("TX: Joy center Y="));
  Serial.print(joyYCenter);
  Serial.print(F(" X="));
  Serial.println(joyXCenter);

  // Initialize OLED display (I2C)
  display.begin();
  Serial.println(F("TX: OLED display initialized"));

  // Initialize timing
  previousLoopMs = millis();
}

void loop() {
  unsigned long currentMs = millis();

  // ─── Radio Init Failure: LED stays OFF, do nothing ───
  if (radioInitFailed) {
    return;
  }

  // ─── Handle blink animation (robot connection celebration) ───
  if (blinkingActive) {
    if (currentMs - lastBlinkTime >= 100) {  // 100ms per toggle = fast blink
      lastBlinkTime = currentMs;
      blinkPhase = !blinkPhase;
      digitalWrite(LED_PIN, blinkPhase ? HIGH : LOW);
      if (!blinkPhase) {
        blinkCount++;
      }
      if (blinkCount >= ROBOT_CONNECT_BLINKS) {
        // Done blinking — return to solid ON (radio chip still connected)
        blinkingActive = false;
        digitalWrite(LED_PIN, HIGH);
      }
    }
    // Continue with normal loop during blink animation
  }

  // ─── 20ms loop interval (50Hz) ───
  if (currentMs - previousLoopMs < LOOP_INTERVAL_MS) {
    return;
  }
  previousLoopMs = currentMs;

  // ─── Read joystick Y-axis (speed: forward/backward) ───
  int rawY = analogRead(JOY_Y_PIN);
  int8_t speed = 0;
  int yLow = joyYCenter - DEADBAND_HALF;
  int yHigh = joyYCenter + DEADBAND_HALF;

  if (yInDeadBand) {
    if (rawY < (yLow - HYSTERESIS)) {
      yInDeadBand = false;
    } else if (rawY > (yHigh + HYSTERESIS)) {
      yInDeadBand = false;
    }
  } else {
    if (rawY >= yLow && rawY <= yHigh) {
      yInDeadBand = true;
    }
  }

  if (yInDeadBand) {
    speed = 0;
  } else if (rawY < yLow) {
    // Map from 0..(yLow-1) → -100..-1
    speed = map(rawY, 0, yLow - 1, -100, -1);
  } else {
    // Map from (yHigh+1)..max using symmetric range from center
    // Use distance from center to 0 as reference for both sides
    int yMax = joyYCenter + (joyYCenter - 0);  // Mirror the low range
    if (yMax > 1023) yMax = 1023;
    int clamped = min(rawY, yMax);
    speed = map(clamped, yHigh + 1, yMax, 1, 100);
  }
  speed = constrain(speed, -100, 100);
  lastSpeed = speed;

  // ─── Read joystick X-axis (turn: left/right) ───
  int rawX = analogRead(JOY_X_PIN);
  int8_t turn = 0;
  int xLow = joyXCenter - DEADBAND_HALF;
  int xHigh = joyXCenter + DEADBAND_HALF;

  if (xInDeadBand) {
    if (rawX < (xLow - HYSTERESIS)) {
      xInDeadBand = false;
    } else if (rawX > (xHigh + HYSTERESIS)) {
      xInDeadBand = false;
    }
  } else {
    if (rawX >= xLow && rawX <= xHigh) {
      xInDeadBand = true;
    }
  }

  if (xInDeadBand) {
    turn = 0;
  } else if (rawX < xLow) {
    // Map from 0..(xLow-1) → -100..-1
    turn = map(rawX, 0, xLow - 1, -100, -1);
  } else {
    // Map using symmetric range from center, shrunk slightly to ensure full +100
    int xMax = joyXCenter + (joyXCenter - 0);
    xMax = xMax * 95 / 100;  // Shrink 5% to account for pot not reaching full range
    if (xMax > 1023) xMax = 1023;
    int clamped = min(rawX, xMax);
    turn = map(clamped, xHigh + 1, xMax, 1, 100);
  }
  turn = constrain(turn, -100, 100);
  lastTurn = turn;

  // ─── Read joystick button ───
  bool btnPressed = !digitalRead(JOY_BTN_PIN);  // Active LOW

  // ─── Build CommandPacket ───
  CommandPacket packet;
  packet.speed = speed;
  packet.turn  = turn;
  packet.flags = btnPressed ? 0x01 : 0x00;

  // ─── Transmit packet ───
  bool ackReceived = radio.write(&packet, sizeof(packet));

  if (ackReceived) {
    lastAckTime = currentMs;

    // First time getting an ACK from robot — trigger blink celebration
    if (!robotConnected) {
      robotConnected = true;
      robotFirstConnect = true;
      blinkingActive = true;
      blinkCount = 0;
      blinkPhase = false;
      lastBlinkTime = currentMs;
      Serial.println(F("TX: Robot connected! (LED blinking)"));
    }
  }

  // ─── Update connection state ───
  if (currentMs - lastAckTime > CONNECTION_TIMEOUT) {
    if (robotConnected) {
      robotConnected = false;
      Serial.println(F("TX: Robot connection lost"));
      // LED stays solid ON — radio chip is still working
      // (only robot link is lost, not the local SPI connection)
      digitalWrite(LED_PIN, HIGH);
    }
  }

  // ─── Battery monitoring and OLED display (every 50th loop ~ 1 second) ───
  loopCounter++;
  if (loopCounter >= 50) {
    loopCounter = 0;

    // Read battery voltage using bandgap method (no divider needed)
    // Measures VCC against internal 1.1V reference: VCC = 1.1V * 1023 / ADC
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    delay(2);
    ADCSRA |= _BV(ADSC);
    while (bit_is_set(ADCSRA, ADSC));
    batteryVoltageMv = (int)(1083740L / ADC);

    // Map voltage from 3000mV (0%) to 4200mV (100%), clamped
    batteryPercent = constrain(map(batteryVoltageMv, 3000, 4200, 0, 100), 0, 100);

    // Update OLED display using full frame buffer
    display.clearBuffer();
    display.setFont(u8g2_font_7x14B_tr);

    // Row 1: Battery voltage (V) for testing
    display.setCursor(0, 13);
    display.print(F("BAT: "));
    display.print(batteryVoltageMv / 1000);
    display.print(F("."));
    int decimals = (batteryVoltageMv % 1000) / 10;
    if (decimals < 10) display.print(F("0"));
    display.print(decimals);
    display.print(F("V"));

    // Row 2: Connection status
    display.setCursor(0, 29);
    if (robotConnected) {
      display.print(F("LINKED"));
    } else {
      display.print(F("NO LINK"));
    }

    // Low battery warning (replaces connection text)
    if (batteryPercent <= 10) {
      display.setCursor(70, 29);
      display.print(F("LOW!"));
    }

    display.sendBuffer();
  }

  // ─── Debug output ───
  Serial.print(F("TX: ACK="));
  Serial.print(ackReceived ? "Y" : "N");
  Serial.print(F(" robot="));
  Serial.print(robotConnected ? "Y" : "N");
  Serial.print(F(" Y="));
  Serial.print(rawY);
  Serial.print(F(" spd="));
  Serial.print(speed);
  Serial.print(F(" X="));
  Serial.print(rawX);
  Serial.print(F(" trn="));
  Serial.print(turn);
  Serial.print(F(" btn="));
  Serial.println(btnPressed ? "1" : "0");
}
