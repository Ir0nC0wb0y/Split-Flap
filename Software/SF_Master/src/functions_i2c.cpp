#include "functions_i2c.h"

void I2C_Address_Scan() {
  delay(I2C_BUS_DELAY); // should change this to a "if millis > some value"
  byte error, address;
  int nDevices = 0;

  Serial.println("Scanning for connected digits...");

  #ifdef DEBUG_DIGITS
    for(int digit = 0; digit <DEBUG_DIGITS; digit++) {
      address_list[address_count] = I2C_DIGIT_ADDR_START + digit;
      address_count++;
    }
  #endif
  #ifndef DEBUG_DIGITS
    for(address = 1; address < 127; address++ ) {
      // The i2c_scanner uses the return value of
      // the Wire.endTransmisstion to see if
      // a device did acknowledge to the address.
      Wire.beginTransmission(address);
      error = Wire.endTransmission();

      if (error == 0) {
        Serial.print("I2C device found at address 0x");
        if (address<16) Serial.print("0");
        Serial.print(address,HEX);
        Serial.println(" !");
        nDevices++;

        // Add address to list
        if (address >= I2C_DIGIT_ADDR_START && address <= I2C_DIGIT_ADDR_END) {
          address_count++;
          address_list[address_count] = address;
        }
        

      } else if (error==4) {
        Serial.print("Unknow error at address 0x");
        if (address<16) Serial.print("0");
        Serial.print(address,HEX);
        Serial.println(" !");
      }
    }
  #endif
}

void send_character(int digit, char payload) {
  // check status
  bool digit_status = false;
  display_chars[digit] = payload;
  //Serial.print("current char at position ");
  //  Serial.print(digit);
  //  Serial.print(": ");
  //  Serial.print(display_string.charAt(digit));
  display_string.setCharAt(digit, payload);
  //  Serial.print(" after change: ");
  //  Serial.println(display_string.charAt(digit));
  while (!digit_status) {
    digit_status = I2C_Message_34(digit);
    delay(100);
  }
  // send character
    I2CTx_33 message;
    message.payload = payload;
    I2C_Message_33(digit, message);
}

void I2C_Message_33(int digit, I2CTx_33 payload) {
  byte send_addr = address_list[digit];
  if (send_addr >= I2C_DIGIT_ADDR_START && send_addr <= I2C_DIGIT_ADDR_END) {
    // Query digit for readiness
      // if not ready, wait

    // Send Character
    #ifndef DEBUG_DIGITS
      Wire.beginTransmission(send_addr);
      Wire.write((byte*) &payload, sizeof(payload));
      Wire.endTransmission();    // this is what actually sends the data

      Serial.print("Sent id ");
      Serial.print(payload.id);
      Serial.print(" ");
      Serial.print(payload.payload);
      Serial.print(" to ");
      Serial.print(send_addr,HEX);
      Serial.println();
    #endif

    
  } else {
    Serial.print("Invalid address: ");
      Serial.println(send_addr);
  }
}

bool I2C_Message_34(int digit) {
  // sends message 34 and requests message 128
  byte send_addr = address_list[digit];
  #ifdef DEBUG_DIGITS
    bool digit_status = true;
  #endif
  #ifndef DEBUG_DIGITS
    bool digit_status = false;
    if (send_addr >= I2C_DIGIT_ADDR_START && send_addr <= I2C_DIGIT_ADDR_END) {
      // Transmit data
      I2CTx_34 payload;
        Wire.beginTransmission(send_addr);
        Wire.write((byte*) &payload, sizeof(payload));
        Wire.endTransmission();
      // Receive data
        //int stop = (int)true;
        I2CTx_128 rxData;
        Wire.requestFrom(send_addr, MESSAGE_SIZE_128);
        // the request is immediately followed by the read for the response
        Wire.readBytes( (byte*) &rxData, MESSAGE_SIZE_128);
        digit_status = rxData.status;
        Serial.print("Digit status: ");
        Serial.println(digit_status);
    }
    #endif
  return digit_status;
}