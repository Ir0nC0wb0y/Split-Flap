#include "functions_time.h"

void setup_time() {
  // NTP Setup
  ntp.updateInterval(NTP_UPDATE_PERIOD);
  ntp.ruleDST(TZ_DST, TZ_DST_WEEK, TZ_DST_WDAY, TZ_DST_MONTH, TZ_DST_HOUR, utcOffsetInSeconds_DST);
  ntp.ruleSTD(TZ_STD,  TZ_STD_WEEK, TZ_STD_WDAY, TZ_STD_MONTH, TZ_STD_HOUR, utcOffsetInSeconds_STD);
  ntp.begin(NTP_server);
}

void handle_time() {
  ntp.update();
  // set date & time variables to updated time
}