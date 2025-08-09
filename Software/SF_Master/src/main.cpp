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

String character_order = " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.:/";

// Include function files
#include "functions_i2c.h"
#include "functions_wifi.h"
#include "functions_time.h"
#include "functions_display.h"
#include "functions_webserver.h"

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
    int  display_alignment = ALIGN_CENTER; // should be selectable from the list of the "alignment" enum
    // Countdown
      bool countdown_years_show   = true;
      bool countdown_months_show  = true;
      bool countdown_weeks_show   = false;
      bool countdown_days_show    = true;
      bool countdown_hours_show   = true;
      bool countdown_minutes_show = true;
      //time_t countdown_event = 1755468000; // value used for testing, should equate to 8/17/25 @ 17:00:00 CDT
      time_t countdown_event = 1805341501; // value used for testing, should be 3/17/27 @ 10:45:01 PM CDT
      //time_t countdown_event = 1710733501; // value used for testing, should be 3/17/24 @ 10:45:01 PM CDT

// I2C
  byte address_list[32];
  int address_count = 0;

// WiFi
  // Nothing here (yet)

// Digits
  //#define DEBUG_DIGITS 8 // Comment this macro to search for connected digits
    // DEBUG_DIGITS is now in "functions_i2c.h"

// NTP & Date/Time
  WiFiUDP ntpUDP;
  NTP ntp(ntpUDP);

// Display Functions
  String display_string;
  //const char char_order[41] = " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.:/";

// Webserver
  WebServer server(80);
  //AsyncWebServer server(80);
  //String current_letters = "      ";
  String display_string_server;
  

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Starting sketch!");
  
  Wire.begin(PIN_SDA, PIN_SCL);

  Serial.print("Starting File System ");
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS Mount Failed");
    while(true) {
      yield();
    }
  }
  Serial.println("... Success!");

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

  setup_webserver();

  setup_time();
}

void loop() {
  // Scan for addresses
  unsigned long loop_start = micros();

  unsigned long time_start = micros();
  bool ran_time = handle_time();
  unsigned long time_stop  = micros();

  unsigned long server_start = micros();
  server.handleClient();
  unsigned long server_stop = micros();

  unsigned long wifi_start = micros();
  bool ran_wifi = checkWiFi();
  unsigned long wifi_stop  = micros();

  unsigned long display_start = micros();
  bool ran_display = HandleDisplay();
  unsigned long display_stop = micros();

  // Display length of loop if over 5ms
  unsigned long loop_stop = micros();
  if(ran_time || server_stop - server_start > 2000 || ran_wifi || ran_display || loop_stop-loop_start > 5000) {
    Serial.print("Loop time: ");
      Serial.print(loop_stop-loop_start);
      Serial.println("us and ran:");
      if (ran_time) {
        Serial.print("handle_time - ");
        Serial.print(time_stop - time_start);
        Serial.println("us");
      }
      if (server_stop - server_start > 500) { //anything over 2ms counts as running
        Serial.print("sever.handleClient - ");
        Serial.print(server_stop - server_start);
        Serial.println("us");
      } 
      if (ran_wifi) {
        Serial.print("checkWiFi - ");
        Serial.print(wifi_stop - wifi_start);
        Serial.println("us");
      }
      if (ran_display) {
        Serial.print("HandleDisplay - ");
        Serial.print(display_stop - display_start);
        Serial.println("us");
      }
      Serial.println();
  }

}