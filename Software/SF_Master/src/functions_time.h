#pragma once
#include <Arduino.h>

#include <WiFiUdp.h>
#include <NTP.h>

// Collect macros
#define NTP_UPDATE_PERIOD 900000 // 900 seconds, or 15 minutes
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

// Collect objects and variables
// NTP
  extern WiFiUDP ntpUDP;
  extern NTP ntp;
// Time
  extern int time_hour_raw;
  extern int time_hour;
  extern int time_minute;
// Date
  extern int date_year;
  extern int date_month;
  extern int date_day;
// Timezone
  extern const char* TZ_STD;
  extern const char* TZ_DST;
  extern const char* NTP_server;
  extern long utcOffsetInSeconds_DST; // UTC Offset, minutes
  extern long utcOffsetInSeconds_STD; // UTC Offset, minutes
  extern bool dst_flag; // flag whether to perform DST
  extern bool dst_state; // flag for current DST state
  
// Collect functions
void setup_time();
void handle_time();

