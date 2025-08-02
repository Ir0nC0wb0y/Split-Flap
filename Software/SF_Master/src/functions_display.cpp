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
  const char char_order[] = " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.:/";
  int frame_ID_last = -1;
  int frame_IDs[5]   = {1, 2, 3, 4, 99}; // update to keep consistent
  int frame_valid[5] = {1, 0, 0, 0,  0}; // boolean turn on
  int demo_state = 40;
  unsigned long display_frame_last = 0;
  int time_hour = 0;

void HandleDisplay() {
  // check timing
  if (display_frame_last + display_framerate <= millis()) {
    Serial.print("Framerate: ");
      Serial.println(display_framerate);
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
      Serial.println(frame_next);
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

  #ifdef DEBUG_DIGITS
    Display_PrettySerial();
  #endif
  
  display_frame_last = millis();
  }
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
  Serial.print("Countdown seconds: ");
    Serial.println(countdown_seconds);
  long countdown[6] = {0, 0, 0, 0, 0, 0};
  //long countdown_years   = 0;
  //long countdown_months  = 0;
  //long countdown_weeks   = 0;
  //long countdown_days    = 0;
  //long countdown_hours   = 0;
  //long countdown_minutes = 0;
  bool disp_unit[6] = {0, 0, 0, 0, 0, 0};

  // Convert the time
    if (countdown_years_show && countdown_seconds >= COUNTDOWN_SEC_YEAR) {
      countdown[0]   = floor(countdown_seconds/COUNTDOWN_SEC_YEAR);
      countdown_seconds -= countdown[0] * COUNTDOWN_SEC_YEAR;
      disp_unit[0] = true;
      Serial.print("Years: ");
        Serial.print(countdown[0]);
        Serial.print(" remaining seconds: ");
        Serial.println(countdown_seconds);
    }
    if (countdown_months_show && countdown_seconds >= COUNTDOWN_SEC_MON) {
      countdown[1]   = floor(countdown_seconds/COUNTDOWN_SEC_MON);
      countdown_seconds -= countdown[1] * COUNTDOWN_SEC_MON;
      disp_unit[1] = true;
      Serial.print("Months: ");
        Serial.print(countdown[1]);
        Serial.print(" remaining seconds: ");
        Serial.println(countdown_seconds);
    }
    if (countdown_weeks_show && countdown_seconds >= COUNTDOWN_SEC_WEEK) {
      countdown[2]   = floor(countdown_seconds/COUNTDOWN_SEC_WEEK);
      countdown_seconds -= countdown[2] * COUNTDOWN_SEC_WEEK;
      disp_unit[2] = true;
      Serial.print("Weeks: ");
        Serial.print(countdown[2]);
        Serial.print(" remaining seconds: ");
        Serial.println(countdown_seconds);
    }
    if (countdown_days_show && countdown_seconds >= COUNTDOWN_SEC_DAY) {
      countdown[3]   = floor(countdown_seconds/COUNTDOWN_SEC_DAY);
      countdown_seconds -= countdown[3] * COUNTDOWN_SEC_DAY;
      disp_unit[3] = true;
      Serial.print("Days: ");
        Serial.print(countdown[3]);
        Serial.print(" remaining seconds: ");
        Serial.println(countdown_seconds);
    }
    if (countdown_hours_show && countdown_seconds >= COUNTDOWN_SEC_HOUR) {
      countdown[4]   = floor(countdown_seconds/COUNTDOWN_SEC_HOUR);
      countdown_seconds -= countdown[4] * COUNTDOWN_SEC_HOUR;
      disp_unit[4] = true;
      Serial.print("Hours: ");
        Serial.print(countdown[4]);
        Serial.print(" remaining seconds: ");
        Serial.println(countdown_seconds);
    }
    if (countdown_minutes_show && countdown_seconds >= COUNTDOWN_SEC_MIN) {
      countdown[5]   = floor(countdown_seconds/COUNTDOWN_SEC_MIN);
      countdown_seconds -= countdown[5] * COUNTDOWN_SEC_MIN;
      disp_unit[5] = true;
      Serial.print("Minutes: ");
        Serial.print(countdown[5]);
        Serial.print(" remaining seconds: ");
        Serial.println(countdown_seconds);
    }


  // build message, cut to size
  char countdown_message[33];
  int countdown_message_idx = 0;
  int unit_length[6] = {0, 0, 0, 0, 0, 0};
  memset(display_chars, '\0', sizeof(display_chars));
  for (int i = 0; i<6; i++) {
    bool started_chars = false;
    int unit_length_temp = 0;
    if (disp_unit[i]) {
      unit_length_temp = 1;
      int digit = floor((countdown[i]% 10000)/1000);
      if (digit > 0) {
        // pick up the thousandths digit
        started_chars = true;
        unit_length_temp++;
        char char_digit = '0' + digit;
        //Serial.print(char_digit);
        countdown_message[countdown_message_idx] = char_digit;
        countdown_message_idx++;
      }
      digit = floor((countdown[i]% 1000)/100);
      if (digit > 0 || started_chars) {
        // pick up the hundreds digit
        started_chars = true;
        unit_length_temp++;
        char char_digit = '0' + digit;
        //Serial.print(char_digit);
        countdown_message[countdown_message_idx] = char_digit;
        countdown_message_idx++;
      }
      digit = floor((countdown[i]% 100)/10);
      if (digit > 0 || started_chars) {
        // pick up the hundreds digit
        started_chars = true;
        unit_length_temp++;
        char char_digit = '0' + digit;
        //Serial.print(char_digit);
        countdown_message[countdown_message_idx] = char_digit;
        countdown_message_idx++;
      }
      digit = floor(countdown[i]% 10);
      if (digit > 0 || started_chars) {
        // pick up the hundreds digit
        started_chars = true;
        unit_length_temp++;
        char char_digit = '0' + digit;
        //Serial.print(char_digit);
        countdown_message[countdown_message_idx] = char_digit;
        countdown_message_idx++;
      }
      
      switch (i) {
        case 0:
          countdown_message_idx++;
          countdown_message[countdown_message_idx] = 'Y';
          //Serial.println(countdown_message[countdown_message_idx]);
          break;
        case 1:
          countdown_message_idx++;
          countdown_message[countdown_message_idx] = 'M';
          //Serial.println(countdown_message[countdown_message_idx]);
          break;
        case 2:
          countdown_message_idx++;
          countdown_message[countdown_message_idx] = 'W';
          //Serial.println(countdown_message[countdown_message_idx]);
          break;
        case 3:
          countdown_message_idx++;
          countdown_message[countdown_message_idx] = 'D';
          //Serial.println(countdown_message[countdown_message_idx]);
          break;
        case 4:
          countdown_message_idx++;
          countdown_message[countdown_message_idx] = 'H';
          //Serial.println(countdown_message[countdown_message_idx]);
          break;
        case 5:
          countdown_message_idx++;
          countdown_message[countdown_message_idx] = 'M';
          //Serial.println(countdown_message[countdown_message_idx]);
          break;
      }
    }
    unit_length[i] = unit_length_temp;
  }

  for (int i = 0; i < 6; i++) {
    Serial.print("Unit [");
      Serial.print(i);
      Serial.print("] length: ");
      Serial.println(unit_length[i]);
  }

  Serial.print("Countdown message: ");
    Serial.println(countdown_message);

  int countdown_message_characters = 0;
  for (int i = 0; i<6; i++) {
    if (unit_length[i] + countdown_message_characters <= address_count) {
      countdown_message_characters+= unit_length[i];
    }
  }

  for (int i = 0; i<address_count; i++) {
    if (i < countdown_message_characters) {
      send_character(i,countdown_message[i]);
    } else {
      send_character(i,' ');
    }
  }

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

void ClearDisplay() {
  for (int i; i<33; i++) {
    if (i < address_count) {
      display_chars[i] = ' ';
    } else {
      display_chars[i] = '\0';
    }
  }
}