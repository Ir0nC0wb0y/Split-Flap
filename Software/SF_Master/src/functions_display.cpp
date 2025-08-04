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
  int frame_next = -1;
  int frame_IDs[5]   = {1, 2, 3, 4, 99}; // update to keep consistent
  int frame_valid[5] = {1, 0, 1, 1,  0}; // boolean turn on
  int demo_state = 40;
  unsigned long display_frame_last = 0;
  int time_hour = 0;
  bool frame_complete = true;

void string_align(String& str, char pad_char, int alignment, int length) {
  int pad_length = length - str.length();
  int pad_right = 0;
  int pad_left = 0;
  
  switch (alignment) {
    case ALIGN_LEFT:
      // Align left
      // padding goes to right of string
      pad_right = pad_length;
      break;
    
    case ALIGN_CENTER:
      // Align center
        // padding goes half right/half left, with remainder on left
      pad_right = floor(pad_length/2);
      pad_left = pad_right + pad_length % 2;
      break;
    
    case ALIGN_RIGHT:
      // Align right
        // padding goes to left of string
      pad_left = pad_length;
      break;
  }

  // Do padding operations
  if (pad_right > 0) {
    for (int i = 0; i < pad_right; i++) {
      str = str + String(pad_char);
    }
  }
  if (pad_left > 0) {
    for (int i = 0; i < pad_left; i++) {
      str = String(pad_char) + str;
    }
  }
}

void send_String(String& str) {
  Serial.print("Sending message to digits ");
  unsigned long send_time = millis();
  int countdown_length = str.length();
  for (int i = 0; i<address_count; i++) {
    if (i < countdown_length) {
      send_character(i,str.charAt(i));
    } else {
      send_character(i,' ');
    }
    Serial.print(".");
  }
  Serial.print(" Complete in ");
    Serial.print(millis() - send_time);
    Serial.println("ms");
}

bool HandleDisplay() {
  // check timing
  bool ran_display = false;
  if (display_frame_last + display_framerate <= millis()) {
    ran_display = true;
    //Serial.print("Framerate: ");
    //  Serial.println(display_framerate);
    
    // Clear frame
    //ClearDisplay(); // not necessary with string_align
    if (frame_complete) {
      frame_complete = false;
      // check for next "turned on" frame type
      int Frame_ArrayLength = sizeof(frame_valid) / sizeof(frame_valid[0]);
      Serial.println("Searching for frame");
      for (int i=0; i < Frame_ArrayLength; i++) {
        if (frame_valid[i]) {
          if (frame_IDs[i] > frame_ID_last) {
            frame_next = frame_IDs[i];
            break;
          }
        }
      }
      if (frame_next == -1) {
        //Serial.println("Next frame not found, wrapping around");
        for (int i=0; i < Frame_ArrayLength; i++) {
          if (frame_valid[i]) {
            frame_next = frame_IDs[i];
            break;
          }
        }
      }
      Serial.print("Found next frame: ");
        Serial.println(frame_next);
    }
    // run frame function
    display_frame_last = millis();
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
  
  Serial.println();
  frame_ID_last = frame_next;
  }
  return ran_display;
}

void Display_PrettySerial() {
  // Output looks like:
  // 30 31 32 33
  //  A  B  C  D

  // addresses
  for (int i=0; i<address_count; i++) {
    Serial.print(address_list[i]);
      Serial.print(" ");
  }
  Serial.println();

  // String
  for (int i=0; i< address_count; i++) {
    Serial.print(" ");
      Serial.print(display_string.charAt(i));
    if (i < address_count-1) {
      Serial.print(" ");
    }
  }
  Serial.println();
  //Serial.print("Sanity check: ");
  //  Serial.print(display_string);
}

void Display_Countdown() {
  time_t time_now = ntp.epoch();
  double countdown_seconds = difftime(countdown_event, time_now);
  Serial.print("Countdown seconds: ");
    Serial.println(countdown_seconds);
  if (countdown_seconds < 0) {
    frame_valid[0] = 0;
    Serial.println("Countdown Display no longer valid, fails sign check");
    display_frame_last = 0;
    frame_complete = true;
    return;
  }
  long countdown[6] = {0, 0, 0, 0, 0, 0};
  bool disp_unit[6] = {0, 0, 0, 0, 0, 0};
  bool message_built = false;

  // Convert the time
    // Years
    if (countdown_years_show && countdown_seconds >= COUNTDOWN_SEC_YEAR) {
      countdown[0]   = floor(countdown_seconds/COUNTDOWN_SEC_YEAR);
      countdown_seconds -= countdown[0] * COUNTDOWN_SEC_YEAR;
      disp_unit[0] = true;
      Serial.print("Years: ");
        Serial.print(countdown[0]);
        //Serial.print(" remaining seconds: ");
        //Serial.print(countdown_seconds);
        Serial.println();
    }
    // Months
    if (countdown_months_show && countdown_seconds >= COUNTDOWN_SEC_MON) {
      countdown[1]   = floor(countdown_seconds/COUNTDOWN_SEC_MON);
      countdown_seconds -= countdown[1] * COUNTDOWN_SEC_MON;
      disp_unit[1] = true;
      Serial.print("Months: ");
        Serial.print(countdown[1]);
        Serial.print(" remaining seconds: ");
        Serial.println(countdown_seconds);
    }
    // Weeks
    if (countdown_weeks_show && countdown_seconds >= COUNTDOWN_SEC_WEEK) {
      countdown[2]   = floor(countdown_seconds/COUNTDOWN_SEC_WEEK);
      countdown_seconds -= countdown[2] * COUNTDOWN_SEC_WEEK;
      disp_unit[2] = true;
      Serial.print("Weeks: ");
        Serial.print(countdown[2]);
        //Serial.print(" remaining seconds: ");
        //Serial.print(countdown_seconds);
        Serial.println();
    }
    // Days
    if (countdown_days_show && countdown_seconds >= COUNTDOWN_SEC_DAY) {
      countdown[3]   = floor(countdown_seconds/COUNTDOWN_SEC_DAY);
      countdown_seconds -= countdown[3] * COUNTDOWN_SEC_DAY;
      disp_unit[3] = true;
      Serial.print("Days: ");
        Serial.print(countdown[3]);
        //Serial.print(" remaining seconds: ");
        //Serial.print(countdown_seconds);
        Serial.println();
    }
    // Hours
    if (countdown_hours_show && countdown_seconds >= COUNTDOWN_SEC_HOUR) {
      countdown[4]   = floor(countdown_seconds/COUNTDOWN_SEC_HOUR);
      countdown_seconds -= countdown[4] * COUNTDOWN_SEC_HOUR;
      disp_unit[4] = true;
      Serial.print("Hours: ");
        Serial.print(countdown[4]);
        //Serial.print(" remaining seconds: ");
        //Serial.print(countdown_seconds);
        Serial.println();
    }
    // Minutes
    if (countdown_minutes_show && countdown_seconds >= COUNTDOWN_SEC_MIN) {
      countdown[5]   = floor(countdown_seconds/COUNTDOWN_SEC_MIN);
      countdown_seconds -= countdown[5] * COUNTDOWN_SEC_MIN;
      disp_unit[5] = true;
      Serial.print("Minutes: ");
        Serial.print(countdown[5]);
        //Serial.print(" remaining seconds: ");
        //Serial.print(countdown_seconds);
        Serial.println();
    }


  // build message, cut to size
  String countdown_string;
  for (int i = 0; i<6; i++) {
    String unit_msg;
    if (disp_unit[i]) {
      //Serial.print("Value of unit: ");
      //  Serial.print(countdown[i]);
      unit_msg = String(countdown[i]);
      //  Serial.print(" now as string: ");
      //  Serial.println(unit_msg);

      switch (i) {
        case 0:
          unit_msg += 'Y';
          //Serial.println(countdown_message[countdown_message_idx]);
          break;
        case 1:
          unit_msg += 'M';
          //Serial.println(countdown_message[countdown_message_idx]);
          break;
        case 2:
          unit_msg += 'W';
          //Serial.println(countdown_message[countdown_message_idx]);
          break;
        case 3:
          unit_msg += 'D';
          //Serial.println(countdown_message[countdown_message_idx]);
          break;
        case 4:
          unit_msg += 'H';
          //Serial.println(countdown_message[countdown_message_idx]);
          break;
        case 5:
          unit_msg += 'M';
          //Serial.println(countdown_message[countdown_message_idx]);
          break;
      }
      //Serial.print("Current Unit: ");
      //  Serial.print(unit_msg);

      if(countdown_string.length() + unit_msg.length() <= address_count) {
        countdown_string += unit_msg;
        message_built = true;
      //  Serial.println("  Adding unit to countdown");
      } else {
      //  Serial.println("  Countdown message too long, not added");
      }
    }
  }
  if (!message_built) {
    frame_valid[0] = 0;
    Serial.println("Countdown Display no longer valid, fails message length");
    display_frame_last = 0;
    frame_complete = true;
    return;
  }
  
  string_align(countdown_string, ' ', display_alignment, address_count);

  send_String(countdown_string);
  frame_complete = true;
}

void Display_Message() {
  // With between 5 and 10 digits, "Hello World" gets parsed as
    // Frame1: "HELLO"
    // Frame2: "WORLD"
  // "Hello my little World." gets parsed differently, with 8 digits:
    // Frame1: "HELLO MY"
    // Frame2: "LITTLE"
    // Frame3: "WORLD."
  // longer words can be broken into pieces, using a "-" (not currently available)
}

void Display_Time() {
  // needs to change format based on number of characters
  if (address_count < 4) {
    frame_valid[2] = 0;
    Serial.println("Time Display no longer valid");
    display_frame_last = 0;
    frame_complete = true;
    return;
  }
  String time_message;
  if (address_count >= 4) {
    // add hour to message
    // format time_hour
    if (use_12_hr_time) {
      if (time_hour_raw > 12) {
        time_hour = time_hour_raw - 12;
      } else if (time_hour_raw == 0) {
        time_hour = 12;
      } else {
        time_hour = time_hour_raw;
      }
    } else {
      time_hour = time_hour_raw;
    }
    
    if (time_hour < 10) {
      if (use_12_hr_time) {
        time_message = String(" ") + String(time_hour);
      } else {
        time_message = String("0") + String(time_hour);
      }
    } else {
      time_message = String(time_hour);
    }
  }
  if (address_count >= 5 && use_12_hr_time) {
    // add colon
    time_message += String(':');
  }
  if (address_count >= 4) {
    // add minute
    if (time_minute < 10) {
      time_message += String("0") + String(time_minute);
    } else {
      time_message += String(time_minute);
    }
  }

  // Add time formatting
  if (use_12_hr_time) {
    if (address_count == 6) {
      // add letter A/P (as in Am/Pm)
      if(time_hour_raw > 11) {
        time_message += String('P');
      } else {
        time_message += String('A');
      }
    }
    if (address_count == 7) {
      // add AM/PM
      if(time_hour_raw > 11) {
        time_message += String("PM");
      } else {
        time_message += String("AM");
      }
    }
    if (address_count >= 8) {
      // add " AM"/" PM"
      if(time_hour_raw > 11) {
        time_message += String(" PM");
      } else {
        time_message += String(" AM");
      }
    }
  } else {
    if (address_count == 5) {
      time_message += String("H");
    }
    if (address_count > 5 && address_count < 10) {
      time_message += String(" H");
    }
    if (address_count >= 10) {
      time_message += String(" HOURS");
    }
  }

  string_align(time_message, ' ', display_alignment, address_count);

  send_String(time_message);
  frame_complete = true;
}

void Display_Date() {
  // needs to change format based on number of characters
  if (address_count < 5) {
    frame_valid[3] = 0;
    Serial.println("Date Display no longer valid");
    display_frame_last = 0;
    frame_complete = true;
    return;
  }
  String Date_message;
  if (address_count >= 10) {
    // Date format: MM/DD/YYYY
    Date_message = String(date_month);
    Date_message += String("/");
    Date_message += String(date_day);
    Date_message += String("/");
    Date_message += String(date_year);
  } else if (address_count >= 8) {
    // Date Format: MM/DD/YY
    Date_message = String(date_month);
    Date_message += String("/");
    Date_message += String(date_day);
    Date_message += String("/");
    int date_year_show = date_year % 100;
    Date_message += String(date_year_show);
  } else if (address_count >= 5) {
    // Date Format: MM/DD
    Date_message = String(date_month);
    Date_message += String("/");
    Date_message += String(date_day);
  }

  string_align(Date_message, ' ', display_alignment, address_count);

  send_String(Date_message);
  frame_complete = true;
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

  char new_char = character_order.charAt(new_char_idx);

  return new_char;
}

void ClearDisplay() {
  Serial.println("Clearing display string");
  // char array
  /*for (int i; i<33; i++) {
    if (i < address_count) {
      display_chars[i] = ' ';
    } else {
      display_chars[i] = '\0';
    }
  }*/

  // String
  for (int i = 0; i < address_count; i++) {
    display_string.setCharAt(i, ' ');
  }
}