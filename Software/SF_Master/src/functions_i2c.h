#ifndef FUNCTIONS_I2C_H
#define FUNCTIONS_I2C_H
#include <Arduino.h>

#include <Wire.h>

#include "I2C_messages.h"

#define PIN_SCL 22
#define PIN_SDA 21
#define I2C_BUS_DELAY 1000 // Wait a moment before scanning to give slower digits a chance to join the bus
#define I2C_DIGIT_ADDR_START 30
#define I2C_DIGIT_ADDR_END   62

extern byte address_list[32];
extern int address_count;
extern char display_chars[33];

void I2C_Address_Scan();
void I2C_Message_33(int digit, I2CTx_33 payload);
bool I2C_Message_34(int digit);

#endif