#include <Arduino.h>
//#include <Wire.h>
//#include <WiFi.h>
//#include <WebServer.h>
//#include <FS.h>
//#include <LittleFS.h>
//#include <AsyncTCP.h>
//#include <ESPAsyncWebServer.h>
//#include <WiFiUdp.h>
//#include <NTP.h>

// Options to include in Webserver
  // Timezone
    const char* TZ_STD = "CST";
    const char* TZ_DST = "CDT";
    const char* NTP_server = "north-america.pool.ntp.org";
    long utcOffsetInSeconds_DST  = -300; // UTC Offset, minutes
    long utcOffsetInSeconds_STD  = -360; // UTC Offset, minutes
    bool dst_flag                = 1; // flag whether to perform DST
    bool dst_state               = 1; // flag for current DST state
  // Display Options
    unsigned long display_framerate = 5000; // there will be a minimum value
    bool use_12_hr_time = true;
    bool countdown_years_show   = true;
    bool countdown_months_show  = true;
    bool countdown_weeks_show   = false;
    bool countdown_days_show    = true;
    bool countdown_hours_show   = true;
    bool countdown_minutes_show = true;
    time_t countdown_event = 1755468000; // value used for testing, should equate to 8/17/25 @ 17:00:00 CDT

// I2C
  #include "functions_i2c.h"
  byte address_list[32];
  int address_count = 0;

// WiFi
  #include "functions_wifi.h"

// Digits
  #define DEBUG_DIGITS 8 // Comment this macro to search for connected digits

// NTP & Date/Time
  #include "functions_time.h"
  WiFiUDP ntpUDP;
  NTP ntp(ntpUDP);
  

// Display Functions
  static String character_order = " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.:/";
  #include "functions_display.h"
  char display_chars[33]; // global display
  String display_string;
  const char char_order[41] = " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.:/";
  


// Webserver
  #include "functions_webserver.h"
  WebServer server(80);
  //AsyncWebServer server(80);
  //String current_letters = "      ";
  String display_string_server;
  

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
  I2C_Address_Scan();
  display_string.reserve(address_count);
  display_string_server.reserve(address_count);
  for (int i = 0; i< address_count; i++) {
    if (i == 0) {
      display_string = ' ';
      display_string_server = ' ';
    } else {
      display_string += ' ';
      display_string_server += ' ';
    }
    
  }
  

  memset(display_chars, '\0', sizeof(display_chars));

  setup_webserver();

  setup_time();
}

void loop() {
  // Scan for addresses
  handle_time();

  server.handleClient();

  checkWiFi();

  HandleDisplay();
}