#include "functions_time.h"

  // Time
    int time_hour_raw = 0;
    //int time_hour     = 0;
    int time_minute   = 0;
  // Date
    int date_year     = 0;
    int date_month    = 0;
    int date_day      = 0;

void setup_time() {
  // NTP Setup
  Serial.print("Setting up NTP... ");
  ntp.updateInterval(NTP_UPDATE_PERIOD);
  ntp.ruleDST(TZ_DST, TZ_DST_WEEK, TZ_DST_WDAY, TZ_DST_MONTH, TZ_DST_HOUR, utcOffsetInSeconds_DST);
  ntp.ruleSTD(TZ_STD,  TZ_STD_WEEK, TZ_STD_WDAY, TZ_STD_MONTH, TZ_STD_HOUR, utcOffsetInSeconds_STD);
  ntp.begin(NTP_server);
  Serial.println(" Success!");
}

void handle_time() {
  ntp.update();
  // set date & time variables to updated time
  time_hour_raw = ntp.hours();
  time_minute = ntp.minutes();
  date_year = ntp.year();
  date_month = ntp.month();
  date_day = ntp.day();
}