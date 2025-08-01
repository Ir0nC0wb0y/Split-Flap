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

// Message structure


// I2C
  #include "functions_i2c.h"
  byte address_list[32];
  int address_count = -1;

// WiFi
  #include "functions_wifi.h"

// Digits
  #define DEBUG_DIGITS 8 // Comment this macro to search for connected digits

// NTP & Date/Time
  #include "functions_time.h"
  WiFiUDP ntpUDP;
  NTP ntp(ntpUDP);
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
    bool dst_state               = 1; // flag for current DST state

// Display Functions
  #include "functions_display.h"
  char display_chars[33]; // global display
  unsigned long display_framerate = 5000;
  unsigned long display_frame_last = 0;
  int frame_ID_last = -1;
  int frame_IDs[5]   = {1, 2, 3, 4, 99}; // update to keep consistent
  int frame_valid[5] = {0, 0, 0, 0,  0}; // boolean turn on
  time_t countdown_event = 1755468000; // value used for testing, should equate to 8/17/25 @ 17:00:00 CDT
  int demo_state = 40;

// Webserver
  #include "functions_webserver.h"
  WebServer server(80);
  //AsyncWebServer server(80);
  String current_letters = "      "; // Default 6 spaces, update as needed
  

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

  memset(display_chars, '\0', sizeof(display_chars));

  setup_webserver();

  setup_time();
}

void loop() {
  // Scan for addresses
  handle_time();

  server.handleClient();

}