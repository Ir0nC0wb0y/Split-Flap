#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <FS.h>
#include <LittleFS.h>
//#include <AsyncTCP.h>
//#include <ESPAsyncWebServer.h>
#include <WiFiUdp.h>
#include <NTP.h>

// Message structure
#include "I2C_messages.h"

// I2C
  #define PIN_SCL 22
  #define PIN_SDA 21

// WiFi
  #define WIFI_CONNECT_TIME 5000
  #define WIFI_HOSTNAME "SF-Master"
  #define WIFI_AP_SSID "SplitFlap"
  #define WIFI_AP_PASS "SomeSecurePassword"
  void connect2WiFi();

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

// NTP & Date/Time
  WiFiUDP ntpUDP;
  NTP ntp(ntpUDP);
  void handle_time();
  #define NTP_UPDATE_PERIOD 900000 // 900 seconds, or 15 minutes
  // Time
    int time_hour_raw = 0;
    int time_hour     = 0;
    int time_minute   = 0;
  // Date
    int date_year     = 0;
    int date_month    = 0;
    int date_day      = 0;
  // Timezone
    const char* TZ_STD = "CST";
    const char* TZ_DST = "CDT";
    const char* NTP_server = "north-america.pool.ntp.org";
    long utcOffsetInSeconds_DST  = -300; // UTC Offset, minutes
    long utcOffsetInSeconds_STD  = -360; // UTC Offset, minutes
    bool dst_flag                = 1; // flag whether to perform DST
    bool dst_state               = 1; // flag for current DST state\
    // DST START
      // DST starts on the second sunday of March
      #define TZ_DST_WEEK            Second
      #define TZ_DST_WDAY            Sun
      #define TZ_DST_MONTH           Mar
      #define TZ_DST_HOUR            2
    // DST END
      // DST Ends the first sunday of November
      #define TZ_STD_WEEK            First
      #define TZ_STD_WDAY            Sun
      #define TZ_STD_MONTH           Nov
      #define TZ_STD_HOUR            3

// Display Functions
  char display_chars[33]; // global display
  unsigned long display_framerate = 5000;
  unsigned long display_frame_last = 0;
  int frame_ID_last = -1;
  void send_character(int digit, char payload);
  void HandleDisplay();
  int frame_IDs[]   = {1, 2, 3, 4, 99}; // update to keep consistent
  int frame_valid[] = {0, 0, 0, 0,  0}; // boolean turn on
  // Pretty Serial
  void Display_PrettySerial();
  // Countdown            (frame ID:  1)
    time_t countdown_event = 1755468000; // value used for testing, should equate to 8/17/25 @ 17:00:00 CDT
    void Display_Countdown();
  // Display Message(s)   (frame ID:  2)
  // Time                 (frame ID:  3)
    void Display_Time();
  // Date                 (frame ID:  4)
    void Display_Date();
  // Discord Message      (frame ID: XX)
  // Demo Function        (frame ID: 99)
    //#define DEMO_PAUSE_TIME 2000
    //unsigned long demo_pause_last = 0;
    int demo_state = 40;
    void Demo_Run(bool all_digits = false);
    char Demo_NewChar(int demo_state);
  

// --- Split Flap Web API ---
  WebServer server(80);
  //AsyncWebServer server(80);
  String current_letters = "      "; // Default 6 spaces, update as needed

  void handleGetMessage() {
      server.send(200, "application/json", '"' + current_letters + '"');
  }

  void handlePostMessage() {
      String body = server.arg("plain");
      if (body.length() == 0) {
          body = server.arg("letters"); // Fallback for form data
      }
      body.toUpperCase();
      // Validate input
      for (size_t i = 0; i < body.length(); ++i) {
          char c = body[i];
          if (strchr(" ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.:/", c) == nullptr) {
              server.send(400, "text/plain", "Invalid input");
              return;
          }
      }
      // Pad or truncate to FLAPS_NUM (or 6 for demo)
      int num_chars = 6; // Change to FLAPS_NUM if needed
      if (body.length() < num_chars) {
          for (int i = body.length(); i < num_chars; ++i) {
              body += ' ';
          }
      }
      if (body.length() > num_chars) body = body.substring(0, num_chars);
      current_letters = body;
      // TODO: Send to hardware
      server.send(200, "text/plain", "OK");
  }

  void handleFileRequest() {
      String path = server.uri();
      if (path == "/") path = "/index.html";
      if (!LittleFS.exists(path)) {
          server.sendHeader("Location", "/", true);
          server.send(302, "text/plain", "Resource Not Found");
          return;
      }
      String contentType = "text/plain";
      if (path.endsWith(".html")) contentType = "text/html";
      else if (path.endsWith(".css")) contentType = "text/css";
      else if (path.endsWith(".js")) contentType = "application/javascript";
      else if (path.endsWith(".png")) contentType = "image/png";
      else if (path.endsWith(".jpg")) contentType = "image/jpeg";
      else if (path.endsWith(".ico")) contentType = "image/x-icon";
      File file = LittleFS.open(path, "r");
      server.streamFile(file, contentType);
      file.close();
  }

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Starting sketch!");
  
  Wire.begin(PIN_SDA, PIN_SCL);

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS Mount Failed");
    while(true) {
      yield();
    }
  }

  connect2WiFi(); // must be done after LittleFS is successful

  // Connect digits
  delay(I2C_BUS_DELAY); // shouldn't be necessary if 
  I2C_Address_Scan();

  memset(display_chars, '\0', sizeof(display_chars));

  // Web API routes
  server.on("/v1/message", HTTP_GET, handleGetMessage);
  server.on("/v1/message", HTTP_POST, handlePostMessage);
  // Serve static files
  server.onNotFound(handleFileRequest);
  server.begin();

  // NTP Setup
  ntp.updateInterval(NTP_UPDATE_PERIOD);
  ntp.ruleDST(TZ_DST, TZ_DST_WEEK, TZ_DST_WDAY, TZ_DST_MONTH, TZ_DST_HOUR, utcOffsetInSeconds_DST);
  ntp.ruleSTD(TZ_STD,  TZ_STD_WEEK, TZ_STD_WDAY, TZ_STD_MONTH, TZ_STD_HOUR, utcOffsetInSeconds_STD);
  ntp.begin(NTP_server);
}

void loop() {
  // Scan for addresses
  handle_time();

  server.handleClient();

}

void connect2WiFi() {
  // Search for WiFi config files
  WiFi.setHostname(WIFI_HOSTNAME);
  WiFi.mode(WIFI_STA);
  String WiFi_Path = "/WiFi/";
  File Wifi_conf = LittleFS.open(WiFi_Path);
  if (!Wifi_conf) {
    Serial.println("Failed to open WiFi conf directory");
  } else if (!Wifi_conf.isDirectory()) {
    Serial.println("Wifi conf dir not directory");
  } else {
    File Wifi_conf_file = Wifi_conf.openNextFile();
    int n_SSIDs = WiFi.scanNetworks();
    if (n_SSIDs > 0) {
      while (Wifi_conf_file) {
        String conf_name = String(Wifi_conf_file.name());
        if (!Wifi_conf_file.isDirectory() && conf_name.endsWith(".conf")) {
          Serial.print("Found WiFi Config File: ");
            Serial.println(conf_name);
          // Read file contents
          // Line 1: AP name
          String AP_name = Wifi_conf_file.readStringUntil('\n');
          const char* AP_name_char = AP_name.c_str();
          // Line 2: AP password
          String AP_pass = Wifi_conf_file.readStringUntil('\n');
          const char* AP_pass_char = AP_pass.c_str();
          
          // Check if AP exists
          Serial.print("Searching for AP: ");
            Serial.print(AP_name);
          bool AP_exists = false;
          for (int i = 0; i < n_SSIDs; ++i) {
            if (WiFi.SSID(i) == AP_name) {
              AP_exists = true;
              Serial.println(" Found!");
              break;
            }
          }
          if (!AP_exists) {
            Serial.println(" not found!");
          }

          if (AP_exists) {
            // Attempt to connect
            Serial.print("Connecting to AP: ");
              Serial.print(AP_name);
              Serial.print(" ");
            WiFi.begin(AP_name_char,AP_pass_char);
            unsigned long connect_time = millis();
            while (WiFi.status() != WL_CONNECTED || connect_time <= WIFI_CONNECT_TIME) {
              Serial.print(".");
              delay(100);
            }
            if (WiFi.status() == WL_CONNECTED) {
              Serial.println(" Success!");
              break; // break out of file loop
            } else {
              Serial.print(" Failed!");
            }
          }
        }
        Wifi_conf_file = Wifi_conf.openNextFile();
      }
    }
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.mode(WIFI_AP);
      WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
      IPAddress IP = WiFi.softAPIP();
      Serial.print("AP IP address: ");
      Serial.println(IP);
    } else {
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
    }
  }
}


void I2C_Address_Scan() {
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

void handle_time() {
  ntp.update();

}


//################################################################################
//###                         Display Functions                                ###
//################################################################################
  // Countdown            (frame ID:  1)
  // Display Message(s)   (frame ID:  2)
  // Time                 (frame ID:  3)
  // Date                 (frame ID:  4)
  // Discord Message      (frame ID: XX)
  // Demo                 (frame ID: 99)

void HandleDisplay() {
  // check timing
  if (display_frame_last + display_framerate <= millis()) {
    int frame_next = -1;
    // check for next "turned on" frame type
    int Frame_ArrayLength = sizeof(frame_valid) / sizeof(frame_valid[0]);
    for (int i=0; i < Frame_ArrayLength; i++) {
      if (frame_valid[i]) {
        if (frame_IDs[i] > frame_ID_last) {
          frame_next = frame_IDs[i];
          break;
        }
      }
    }
    if (frame_next == 0) {
      Serial.println("Next frame not found, wrapping around");
      for (int i=0; i < Frame_ArrayLength; i++) {
        if (frame_valid[i]) {
          frame_next = frame_IDs[i];
          break;
        }
      }
    }
    Serial.print("Found next frame: ");
      Serial.print(frame_next);
    // run frame function
    switch (frame_next) {
      case 1:
        Serial.println("Displaying countdown");
        Display_Countdown();
        break;

      case 2:
        Serial.println("Displaying message");
        break;

      case 3:
        Serial.println("Displaying Time");
        Display_Time();
        break;
      
      case 4:
        Serial.println("Displaying Date");
        Display_Date();
        break;

      case 99:
        Serial.println("Displaying Demo");
        Demo_Run();
        break;
    }
  }

  #ifdef DEBUG_DIGITS
    Display_PrettySerial();
  #endif
}

void Display_PrettySerial() {
  // Output looks like:
  // 30 31 32 33
  //  A  B  C  D

  for (int i=0; i<address_count; i++) {
    Serial.print(address_list[i]);
      Serial.print(" ");
  }
  Serial.println();

  for (int i=0; i<address_count; i++) {
    Serial.print(" ");
      Serial.print(display_chars[i]);
      if (i < address_count-1) {
        Serial.print(" ");
      }
  }
  Serial.println();
}

void Display_Countdown() {
  time_t time_now = ntp.epoch();
  double countdown_seconds = difftime(countdown_event, time_now);
  // Convert the time
    // if countdown_seconds > {seconds in year}
    // if countdown_seconds > {seconds in month??}
    // if countdown_seconds > {seconds in }
  // Now figure out what you can display, based on how many {largest units}
    // Units:
      // Y - year
      // M - month ??
      // W - weeks ??
      // D - day
      // H - Hour
      // M - Minutes
    // 3 digits: up to 2 digit number and unit
    // 4 digits: up to 3 digit number and unit
    // 6 digits: up to (2) 2 digit numbers and their units
    // 7 digits: (1) 3 digit number and (1) 2 digit number and units
    // 8 digits: up to (2) 3 digit numbers and their units
    // 9 digits: up to (3) 2 digit numbers and their units
}

void Display_Time() {
  // needs to change format based on number of characters
  if (address_count >= 5) {
    // center time in digits, preferring fewer spaces left
  } else if (address_count == 4) {
    // remove colon
  } else {
    // remove time frame validity
  }
}

void Display_Date() {
  // needs to change format based on number of characters
  if (address_count >= 10) {
    // Date format: MM/DD/YYYY
    // center date in digits, preferring fewer spaces to the left
  } else if (address_count >= 8) {
    // Date Format: MM/DD/YY
    // center date in digits, preferring fewer spaces to the left
  } else if (address_count >= 5) {
    // Date Format: MM/DD
    // center date in digits, preferring fewer spaces to the left
  } else {
    // remove date frame validity
  }
}

void Demo_Run(bool all_digits) {
  // the all_digits flag toggles whether all digits are the same random character,
  // or independent random characters
  //if (demo_pause_last + DEMO_PAUSE_TIME <= millis()) {
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
    
    //demo_pause_last = millis();
  //}
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
  /*
  if (demo_state < FLAPS_NUM) {
    Serial.print("Cycle to character: ");
  } else {
    Serial.print("Random character: ");
  }
  Serial.println(new_char);
  */

  return new_char;
}