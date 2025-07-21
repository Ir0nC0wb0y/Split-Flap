#include <Arduino.h>

// Create character message (ID33) (2 bytes)
struct I2CTx_33 {
    byte id = 33; // one byte
    char payload; // one byte
};
#define MESSAGE_SIZE_33 2

// Create status request message (ID34) (one byte)
struct I2CTx_34 {
  byte id = 34; // one byte
};
#define MESSAGE_SIZE_34 1

// Create status response message (ID128) (2 bytes)
struct I2CTx_128 {
  byte id = 128; // one byte
  bool status = false; // one byte
};
#define MESSAGE_SIZE_128 2