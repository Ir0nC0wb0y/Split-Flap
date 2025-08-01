#pragma once
#include <Arduino.h>

#include "functions_i2c.h"
#include "functions_time.h"

#define FLAPS_NUM 40
const char char_order[] = " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.:/";

extern char display_chars[33]; // global display
extern unsigned long display_framerate;
extern unsigned long display_frame_last;
extern int frame_ID_last;

extern int frame_IDs[5]; // update to keep consistent
extern int frame_valid[5]; // boolean turn on
extern time_t countdown_event;
extern int demo_state;

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