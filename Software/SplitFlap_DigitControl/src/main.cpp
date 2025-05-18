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
    #define MOTOR_SR_DEFAULT 4076
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
  int Flap_Char(char incoming_char);
  void Flap_Idx2Pos(int flap_idx);
  int flap_offset = -425;
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
      //Serial.print("Current position: "); Serial.println(stepper1.currentPosition());
    }
    stepper1.disableOutputs();
    Demo_Run();
  }

  // Force motor recalibration
  if (motor_turns > MOTOR_CALIBRATE_FORCE) {
    Motor_Calibrate();
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
    }
  } else {
    // used for typical use
    stepper1.setCurrentPosition(flap_offset);
    is_homed = 1;
  }
}

void Endstop_Cross() {
  //Serial.print("Endstop Cross point: "); Serial.println(cross_endstop);
  //Serial.print("Current position: "); Serial.println(stepper1.currentPosition());

  // Where I am: current position
  // Endstop Cross Point: cross_endstop
  // Zero Position: current position - motor_sr
  // steps/rotation = cross_endstop - flap_offset

  // Filter steps for rotation
  //Serial.print("Filtering: "); Serial.println(cross_endstop - flap_offset);
  //motor_sr.filter(cross_endstop);
  //Serial.print("Filtered "); Serial.print(cross_endstop);
  //  Serial.print(" Current SR: "); Serial.println(motor_sr.getValue());

  // calculate absolute position of flap index?
  stepper1.setCurrentPosition(stepper1.currentPosition() - cross_endstop);
  //Serial.print("Updated position: "); Serial.println(stepper1.currentPosition());
  Serial.println();

  cross_endstop = 0;
}

void Motor_Calibrate() {
  is_homed = 2; // set interrupt to calibrate state
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

  // Move off the endstop
  int new_position = stepper1.currentPosition() + (int)(motor_sr.getValue()/2.0);
  stepper1.moveTo(new_position);
  while (stepper1.distanceToGo() != 0) {
    stepper1.run();
    yield();
  }

  motor_turns = 0;
  // Home Motor
  //is_homed = 0;
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
    float pos_float = motor_sr.getValue() * FLAPS_ROTATION * (float)flap_idx;
    if (flap_idx < flap_idx_current && (long)stepper1.currentPosition() > pos_float) {
      pos_float += motor_sr.getValue();
    }
    
    pos_float -= flap_offset;

    long position = (long)pos_float;

    // Communicate the move
    //Serial.print("Moving to character id ");
    //  Serial.print(flap_idx);
    //  Serial.print(" at position ");
    //  Serial.println(position);

    /* // Take 1 at absolute calc
    // calculate flap position from flap index
    float pos_float = motor_sr.getValue() * FLAPS_ROTATION * (float)flap_idx;
    Serial.print("Pos (float): "); Serial.print(pos_float);
    position = (long)pos_float;
    Serial.print(" Pos (int): "); Serial.println(position);

    // determine if current position is greater than new flap position
    if (flap_idx < flap_idx_current) {
      position += (long)motor_sr.getValue();
      Serial.print("Added Rotation, new position: "); Serial.println(position);
    }

    // Communicate the move
    Serial.print("Moving to character id ");
      Serial.print(flap_idx);
      Serial.print(" at position ");
      Serial.println(position);
      */

  /* //Calculation for relative movement
  int move = 0;
  if (flap_idx < flap_idx_current) {
    // go to end, then keep on till morning
    move = FLAPS_NUM - flap_idx_current;
    move += flap_idx;
  } else if (flap_idx > flap_idx_current) {
    move = flap_idx - flap_idx_current;
  }
  if (move > 0) {
    steps = (int)motor_sr.getValue() * FLAPS_ROTATION * move;
  }
  
  Serial.print("Moving ");
    Serial.print(move);
    Serial.print(" flaps, ");
    Serial.print(steps);
    Serial.print(", to character id ");
    Serial.println(flap_idx);
  */
  
  stepper1.setSpeed(MOTOR_SPEED_RUN);
  //stepper1.moveTo(position);
  stepper1.moveTo(position);
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