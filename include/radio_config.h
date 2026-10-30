#ifndef RADIO_CONFIG_H
#define RADIO_CONFIG_H

#include <stdint.h>

// Radio configuration constants
const uint8_t RADIO_CHANNEL = 108;
const uint8_t RADIO_PIPE_ADDRESS[5] = { 0xE8, 0xE8, 0xF0, 0xF0, 0xE1 };

// Command packet sent from TX to Robot every 20ms
struct CommandPacket {
  int8_t speed;   // -100 to +100 (forward/backward from joystick Y-axis)
  int8_t turn;    // -100 to +100 (left/right from joystick X-axis)
  uint8_t flags;  // bit 0: joystick button pressed, bits 1-7: reserved
};

#endif
