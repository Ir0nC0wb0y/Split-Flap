#include <Arduino.h>
#include <Wire.h>

// Message structure
#include "I2C_messages.h"

// I2C
  #define PIN_SCL 22
  #define PIN_SDA 21

// Digits
  #define DEBUG_DIGITS 8 // Comment this macro to search for connected digits
  void I2C_Address_Scan();
  void I2C_Message_33(int digit, I2CTx_33 payload);
  bool I2C_Message_34(int digit);
  byte address_list[32];
  int address_count = -1;
  #define I2C_DIGIT_ADDR_START 30
  #define I2C_DIGIT_ADDR_END   62

// Flaps
  #define I2C_BUS_DELAY 1000 // Wait a moment before scanning to give slower digits a chance to join the bus
  #define FLAPS_NUM 40
  const char char_order[] = " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.:/";

// Display Functions
  char display_chars[33]; // global display
  void send_character(int digit, char payload);
  // Pretty Serial
  void Display_PrettySerial();
  // Countdown
  // Display Message(s)
  // Discord Message
  // Time

  // Demo Function
    #define DEMO_PAUSE_TIME 2000
    unsigned long demo_pause_last = 0;
    int demo_state = 40;
    void Demo_Run(bool all_digits = false);
    char Demo_NewChar(int demo_state);
  

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Starting sketch!");
  
  Wire.begin(PIN_SDA, PIN_SCL);

  // Connect digits
  delay(I2C_BUS_DELAY);
  I2C_Address_Scan();

  memset(display_chars, '\0', sizeof(display_chars));

}

void loop() {
  // Scan for addresses
  
  Demo_Run();

  delay(5000);
}


void I2C_Address_Scan() {
  byte error, address;
  int nDevices = 0;

  Serial.println("Scanning...");

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
    #endif

    Serial.print("Sent id ");
      Serial.print(payload.id);
      Serial.print(" ");
      Serial.print(payload.payload);
      Serial.print(" to ");
      Serial.print(send_addr,HEX);
      Serial.println();
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
//################################################################################
//###                         Display Functions                                ###
//################################################################################

void Display_PrettySerial() {
  // Output looks like:
  // 30 31 32 33
  //  A  B  C  D

  for (int i=0; i<address_count; i++) {
    Serial.print(address_list[i]);
      Serial.print(" ");
  }
  
  for (int i=0; i<address_count; i++) {
    Serial.print(" ");
      Serial.print(display_chars[i]);
      if (i < address_count-1) {
        Serial.print("");
      }
  }
}

void Demo_Run(bool all_digits) {
  // the all_digits flag toggles whether all digits are the same random character,
  // or independent random characters
  if (demo_pause_last + DEMO_PAUSE_TIME <= millis()) {
    if (all_digits) {
      // Pick new character
      char all_char = Demo_NewChar(demo_state);
      // for loop through digits
      for (int i=0; i<address_count; i++) {
        // send character to digit
        send_character(i, all_char);
      }
    } else {
      char new_char;
      // for loop through digits
      for (int i=0; i<address_count; i++) {
        // pick new character
        new_char = Demo_NewChar(demo_state);
        // send character to digit
        send_character(i, new_char);
      }
    }

    #ifdef DEBUG_DIGITS
      Display_PrettySerial();
    #endif
    
    demo_pause_last = millis();
  }
}

char Demo_NewChar(int demo_state) {
  // Pick new character
  int new_char_idx = 0;
  if (demo_state < FLAPS_NUM) {
    new_char_idx = demo_state;
    demo_state++;
  } else {
    new_char_idx = random(0,FLAPS_NUM);
  }

  char new_char = char_order[new_char_idx];
  if (demo_state < FLAPS_NUM) {
    Serial.print("Cycle to character: ");
  } else {
    Serial.print("Random character: ");
  }
  Serial.println(new_char);

  return new_char;
}