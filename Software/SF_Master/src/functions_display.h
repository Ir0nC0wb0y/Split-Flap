#ifndef FUNCTIONS_DISPLAY_H
#define FUNCTIONS_DISPLAY_H
#include <Arduino.h>

#include "functions_i2c.h"
#include "functions_time.h"

#define FLAPS_NUM 40
const char char_order[] = " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.:/";



// Settings from main
  extern char display_chars[33]; // global display
  extern bool use_12_hr_time;
  extern unsigned long display_framerate;
  //extern unsigned long display_frame_last;
  extern time_t countdown_event;

// Values taken from "functions_time"
  // Time
    extern int time_hour_raw;
    extern int time_minute;
  // Date
    extern int date_year;
    extern int date_month;
    extern int date_day;

void send_character(int digit, char payload);
void HandleDisplay();
// Pretty Serial
  void Display_PrettySerial();
// Countdown            (frame ID:  1)
  void Display_Countdown();
// Display Message(s)   (frame ID:  2)
// Time                 (frame ID:  3)
  void Display_Time();
// Date                 (frame ID:  4)
  void Display_Date();
// Discord Message      (frame ID: XX)
// Demo Function        (frame ID: 99)
  void Demo_Run(bool all_digits = false);
  char Demo_NewChar(int demo_state);

#endif