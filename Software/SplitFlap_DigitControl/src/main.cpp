#include <Arduino.h>
#include <AccelStepper.h>

#include "expFilter.h"

// coil order: blue, pink, yellow, orange
// wire order: yellow(A), orange(B), pink(C), blue(D)
// Stepper expects: blue-yellow, pink-orange


#define PIN_MOTOR_A 17  // yellow
#define PIN_MOTOR_B 16  // orange
#define PIN_MOTOR_C 15  // pink
#define PIN_MOTOR_D 14  // blue
#define PIN_ENDSTOP  3
#define PIN_ADDR_0  10
#define PIN_ADDR_1   9
#define PIN_ADDR_2   8
#define PIN_ADDR_3   7
#define PIN_ADDR_4   6

// Address Setup
  int Address_Calc();
  void Address_Show();
  #define DISPLAY_ADDRESS_TIME 5000
  unsigned long addr_show_last = 0;

// Endstop Setup
  //void Endstop_Check();
  //#define ENDSTOP_CHECK_TIME 250
  //unsigned long endstop_check_last = 0;
  long last_endstop_trigger = 0;
  void Endstop_Interrupt();
  int is_homed = 0;
  long cross_endstop = 0;
  void Endstop_Cross();
  

// Motor Setup
  #define HALFSTEP 8
  #define FULLSTEP 4
  AccelStepper stepper1(HALFSTEP, PIN_MOTOR_B, PIN_MOTOR_C, PIN_MOTOR_D, PIN_MOTOR_A);
  //#define MOTOR_TARGET_POSITION 12288
  #define MOTOR_SPEED_HOMING   300.0
  #define MOTOR_SPEED_RUN     1500.0
  #define MOTOR_ACCEL         3000.0
  // Motor steps/rev filter
    expFilter motor_sr;
    #define MOTOR_SR_DEFAULT 4096
    //#define MOTOR_SR_DEFAULT 2038
    #define MOTOR_SR_WEIGHT 0.95
    #define MOTOR_SR_DEBOUNCE 800
  #define MOTOR_CALIBRATE_TURNS 4
  #define MOTOR_CALIBRATE_SPEED 1000
  #define MOTOR_CALIBRATE_FORCE 100
  int motor_turns = -1;
  int motor_calibrate[MOTOR_CALIBRATE_TURNS+1] = {0};
  void Motor_Calibrate();
  void Motor_Home();


// Flap
  int flap_idx_current = -1;
  #define FLAPS_NUM 40
  #define FLAPS_ROTATION .025 // 1/40
  const char char_order[] = " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.:/";
  long Flap_Pos_Abs(int flap_idx);
  int Flap_Char(char incoming_char);
  void Flap_Idx2Pos(int flap_idx);
  int flap_offset = -400;
  void Flap_Move_Offset();
  
// Demo Function
  #define DEMO_PAUSE_TIME 2000
  unsigned long demo_pause_last = 0;
  int demo_state = 26;
  void Demo_Run();

// Serial Comms
  int incomingByte = 0; // for incoming serial data
  #define SERIAL_BUFFER_LEN 16
  char message[SERIAL_BUFFER_LEN];
  void Serial_Read_Buffer();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Starting Sketch:");

  // Set up pins
    //pinMode(PIN_MOTOR_A, OUTPUT);
    //pinMode(PIN_MOTOR_C, OUTPUT);
    //pinMode(PIN_MOTOR_B, OUTPUT);
    //pinMode(PIN_MOTOR_D, OUTPUT);

    // Endstop, also needs to become an interrupt
    pinMode(PIN_ENDSTOP, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_ENDSTOP), Endstop_Interrupt, FALLING);

    // Address Pins
    pinMode(PIN_ADDR_0, INPUT_PULLUP);
    pinMode(PIN_ADDR_1, INPUT_PULLUP);
    pinMode(PIN_ADDR_2, INPUT_PULLUP);
    pinMode(PIN_ADDR_3, INPUT_PULLUP);
    pinMode(PIN_ADDR_4, INPUT_PULLUP);
    
    Address_Show();


  // Motor SR Filter
    motor_sr.setValue(MOTOR_SR_DEFAULT);
    motor_sr.setWeight(MOTOR_SR_WEIGHT);

  // Set up Motor
    stepper1.setSpeed(MOTOR_SPEED_RUN);         
    stepper1.setAcceleration(MOTOR_ACCEL); 
    stepper1.setMaxSpeed(MOTOR_SPEED_RUN);

  //Motor_Home();
  Motor_Calibrate();
  demo_pause_last = millis() - DEMO_PAUSE_TIME;
  Demo_Run();
}

void loop() {

  //Serial_Read_Buffer();

  if (stepper1.distanceToGo() != 0) {
    stepper1.run();
  } else {
    if (demo_pause_last == 0){
      if (cross_endstop != 0) {
        Endstop_Cross();
      }
      // set pause at current position
      demo_pause_last = millis();
      Serial.println();
      //Serial.print("Current position: "); Serial.println(stepper1.currentPosition());
    }
    stepper1.disableOutputs();
    Demo_Run();
  }

  // Force motor recalibration
  if (motor_turns >= MOTOR_CALIBRATE_FORCE) {
    Serial.println("Forcing recalibration ...");
    Motor_Calibrate();
    Serial.println(" ... Done!");
  }

}


int Address_Calc() {
  bool bit0 = digitalRead(PIN_ADDR_0);
  bool bit1 = digitalRead(PIN_ADDR_1);
  bool bit2 = digitalRead(PIN_ADDR_2);
  bool bit3 = digitalRead(PIN_ADDR_3);
  bool bit4 = digitalRead(PIN_ADDR_4);

  int addr = 0;

  if (bit0) {
    addr += 1;
  }

  if (bit1) {
    addr += 2;
  }

  if (bit2) {
    addr += 4;
  }

  if (bit3) {
    addr += 8;
  }

  if (bit4) {
    addr += 16;
  }

  return addr;
}

void Address_Show() {
  if (addr_show_last + DISPLAY_ADDRESS_TIME <= millis()) {
    int address = Address_Calc();
    Serial.print("Address is: ");
      Serial.println(address);
    addr_show_last = millis();
  }
}

/*
void Endstop_Check() {
  if (endstop_check_last + ENDSTOP_CHECK_TIME <= millis()) {
    bool endstop_state = digitalRead(PIN_ENDSTOP);
    Serial.print("Endstop State: ");
      Serial.println(endstop_state);
    endstop_check_last = millis();
  }
}
  */

void Endstop_Interrupt() {
  if (is_homed == 1) {
    // used for homing the motor
    if (stepper1.currentPosition() > MOTOR_SR_DEBOUNCE && cross_endstop == 0) {
      // Set endstop position
      cross_endstop = stepper1.currentPosition();
      motor_turns++;
    }
  } else if (is_homed == 2) {
    // used for calibrating the motor
    if (stepper1.currentPosition() - cross_endstop >= MOTOR_SR_DEBOUNCE) {
      cross_endstop = stepper1.currentPosition();
      if (motor_turns >= 0) {
        motor_calibrate[motor_turns] = stepper1.currentPosition();
      }
      motor_turns++;
      cross_endstop = stepper1.currentPosition();
    }
  } else {
    // used for typical use
    stepper1.setCurrentPosition(flap_offset);
    is_homed = 1;
  }
}

void Endstop_Cross() {
  // calculate absolute position of flap index?
  long flap_pos = Flap_Pos_Abs(flap_idx_current);
  Serial.print("Flap Position:    ");
    Serial.println(flap_pos);

  long current_position = stepper1.currentPosition() - cross_endstop + flap_offset;
  Serial.print("Current Position: ");
    Serial.println(current_position);

  long position_error = current_position-flap_pos;
  Serial.print("Position error: ");
    Serial.println(position_error);

  stepper1.setCurrentPosition(current_position);
  //Serial.print("Updated position: "); Serial.println(stepper1.currentPosition());

  Serial.print("Crossed endstop, current turns: ");
    Serial.println(motor_turns);

  cross_endstop = 0;
}

void Motor_Calibrate() {
  is_homed = 2; // set interrupt to calibrate state
  motor_turns = -1;
  Serial.print("Current SR value: ");
    Serial.println(motor_sr.getValue());

  // Gather rotation data
  stepper1.setSpeed(MOTOR_CALIBRATE_SPEED);
  while (motor_turns <= MOTOR_CALIBRATE_TURNS) {
    stepper1.runSpeed();
    yield();
  }

  // filter collected data
  for (int i = 0; i < MOTOR_CALIBRATE_TURNS; i++) {
    int sr_turn = motor_calibrate[i+1] - motor_calibrate[i];
    if (abs(sr_turn - MOTOR_SR_DEFAULT) > MOTOR_SR_DEBOUNCE) {
      Serial.print("Turn ");
        Serial.print(i);
        Serial.print(" returned an error with ");
        Serial.print(sr_turn);
        Serial.println(" steps");
    } else {
      motor_sr.filter(sr_turn);
      Serial.print("Turn ");
        Serial.print(i);
        Serial.print(" filtering S/R: ");
        Serial.println(sr_turn);
    }
  }
  Serial.print("New SR value: ");
    Serial.println(motor_sr.getValue());

  // Not necessary after position calculation is corrected
  // Move off the endstop
  int new_position = stepper1.currentPosition() + ((int)motor_sr.getValue() + flap_offset - 100);
  stepper1.moveTo(new_position);
  while (stepper1.distanceToGo() != 0) {
    stepper1.run();
    yield();
  }

  cross_endstop = 0;
  motor_turns = 0;
  // Home Motor
  Motor_Home();

}

void Motor_Home() {
  is_homed = 0;
  stepper1.setSpeed(MOTOR_SPEED_HOMING);
  while (is_homed == 0) {
    stepper1.runSpeed();
    yield();
  }
  Serial.println("Motor is Homed!");
  Flap_Move_Offset();
}

void Flap_Move_Offset() {
  stepper1.setSpeed(MOTOR_SPEED_RUN);
  stepper1.moveTo(0);
  while (stepper1.distanceToGo() != 0) {
    stepper1.run();
    yield();
  }
  flap_idx_current = 0;
  //cross_endstop = flap_offset;
  cross_endstop = 0;
  Serial.print("At position ");
    Serial.println(stepper1.currentPosition());
  delay(2000);
}

long Flap_Pos_Abs(int flap_idx) {
  float pos_float_abs = flap_idx * FLAPS_ROTATION;
  pos_float_abs = pos_float_abs * motor_sr.getValue();
  long pos_abs = (long)pos_float_abs;
  return pos_abs;
}

int Flap_Char(char incoming_char) {
  int index = -1;
  char * pch;
  pch=strchr(char_order,incoming_char);
  if (pch != nullptr) {
    index = pch - char_order;
  }
  return index;
}

void Flap_Idx2Pos(int flap_idx) {
  // Calculation for absolute movement
    long flap_pos_new = Flap_Pos_Abs(flap_idx);
    long stepper_position = stepper1.currentPosition();
    // new pos > zero pos (must pass go to get to new position)
    if (stepper1.currentPosition() < 0) {
      if (flap_pos_new - motor_sr.getValue() > stepper1.currentPosition()) {
        flap_pos_new = flap_pos_new - motor_sr.getValue();
      }
    } else if (flap_pos_new < stepper_position) {
      flap_pos_new += motor_sr.getValue();
    }

    // 

    // Communicate the move
    Serial.print("Moving to character id ");
      Serial.print(flap_idx);
      Serial.print(" at position ");
      Serial.println(flap_pos_new);

  stepper1.setSpeed(MOTOR_SPEED_RUN);
  //stepper1.moveTo(position);
  stepper1.moveTo(flap_pos_new);
  flap_idx_current = flap_idx;
}

/*
void Serial_Read_Buffer() {
  if (Serial.available() >0 ) {
    unsigned int message_pos = 0;
    while (Serial.available() > 0) {
      // Create Place to store message
      

      // Read bytes
      char inByte = Serial.read();
      if ( inByte != '\n' && (message_pos < SERIAL_BUFFER_LEN - 1) ) {
        //Add the incoming byte to our message
        message[message_pos] = inByte;
        message_pos++;
      } else {
        //Add null character to string
        message[message_pos] = '\0';
        //Reset for the next message
        message_pos = 0;
      }
    }
  }
}
*/

void Demo_Run() {
  if (demo_pause_last != 0 && demo_pause_last + DEMO_PAUSE_TIME <= millis()) {
    // set demo_pause_last to 0 so we can set it when we turn off the motor
    demo_pause_last = 0;
    
    // Pick new character
    int new_char_idx = 0;
    if (demo_state < FLAPS_NUM) {
      new_char_idx = demo_state;
      demo_state++;
    } else {
      new_char_idx = random(0,FLAPS_NUM);
    }

    char new_char = char_order[new_char_idx];
    if (demo_state < FLAPS_NUM) {
      Serial.print("Cycle to character: ");
    } else {
      Serial.print("Random character: ");
    }
    Serial.println(new_char);

    int move_idx = 0;
    //int flap_position = 0;
    move_idx = Flap_Char(new_char);
    Flap_Idx2Pos(move_idx);
  }
}