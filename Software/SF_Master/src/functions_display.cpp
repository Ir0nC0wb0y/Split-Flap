#include "functions_display.h"

//################################################################################
//###                         Display Functions                                ###
//################################################################################
  // Countdown            (frame ID:  1)
  // Display Message(s)   (frame ID:  2)
  // Time                 (frame ID:  3)
  // Date                 (frame ID:  4)
  // Discord Message      (frame ID: XX)
  // Demo                 (frame ID: 99)

// local variables
  int frame_ID_last = -1;
  int frame_IDs[5]   = {1, 2, 3, 4, 99}; // update to keep consistent
  int frame_valid[5] = {0, 0, 0, 0,  0}; // boolean turn on
  int demo_state = 40;
  unsigned long display_frame_last = 0;
  int time_hour = 0;

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
    if (frame_next == -1) {
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