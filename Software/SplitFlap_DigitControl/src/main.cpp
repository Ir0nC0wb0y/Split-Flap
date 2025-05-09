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
  #define ENDSTOP_CHECK_TIME 250
  unsigned long endstop_check_last = 0;
  void Endstop_Interrupt();
  bool is_homed = false;
  

// Motor Setup
  #define HALFSTEP 8
  #define FULLSTEP 4
  AccelStepper stepper1(HALFSTEP, PIN_MOTOR_D, PIN_MOTOR_A, PIN_MOTOR_B, PIN_MOTOR_C);
  #define MOTOR_TARGET_POSITION 12288
  #define MOTOR_SPEED_HOMING   500.0
  #define MOTOR_SPEED_RUN     2000.0
  #define MOTOR_ACCEL         4000.0
  // Motor steps/rev filter
    expFilter motor_sr;
    #define MOTOR_SR_DEFAULT 4096
    #define MOTOR_SR_WEIGHT 0.25
    #define MOTOR_SR_DEBOUNCE 200
  void Motor_Home();


// Flap
  int flap_idx_current = -1;
  #define FLAPS_NUM 40
  #define FLAPS_ROTATION .025 // 1/40
  const char char_order[] = " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.:/";
  int Flap_Char(char incoming_char);
  int Flap_Steps2Go(int flap_idx);
  int flap_offset = 100;
  void Flap_Move_Offset();
  
// Demo Function
  #define DEMO_PAUSE_TIME 2000
  unsigned long demo_pause_last = 0;
  int demo_state = 0;
  void Demo_Run();

// Serial Comms
  int incomingByte = 0; // for incoming serial data
  #define SERIAL_BUFFER_LEN 16
  char message[SERIAL_BUFFER_LEN];
  void Serial_Read_Buffer();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
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

  // Motor SR Filter
  motor_sr.setValue(MOTOR_SR_DEFAULT);
  motor_sr.setWeight(MOTOR_SR_WEIGHT);

  // Set up Motor
    stepper1.setSpeed(MOTOR_SPEED_RUN);         
    stepper1.setAcceleration(MOTOR_ACCEL); 
    stepper1.setMaxSpeed(MOTOR_SPEED_RUN);

  Motor_Home();
  demo_pause_last = millis() - DEMO_PAUSE_TIME;
  Demo_Run();
}

void loop() {

  //Serial_Read_Buffer();

  if (stepper1.distanceToGo() != 0) {
    stepper1.run();
  } else {
    if (demo_pause_last == 0){
      // set pause at current position
      demo_pause_last = millis();
    }
    stepper1.disableOutputs();
    Demo_Run();
  }

  Address_Show();

  //Endstop_Check();

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
  if (is_homed) {
    if (stepper1.currentPosition() > MOTOR_SR_DEBOUNCE) {
      // Filter steps for rotation
      motor_sr.filter(stepper1.currentPosition());
      // Also zero motor?
      stepper1.setCurrentPosition(-flap_offset);
    }
  } else {
    // zero out motor position
    stepper1.setCurrentPosition(-flap_offset);
    is_homed = true;
    flap_idx_current = 0;
  }
}

void Motor_Home() {
  stepper1.setSpeed(MOTOR_SPEED_HOMING);
  while (!is_homed) {
    stepper1.runSpeed();
    yield();
  }
  Serial.println("Motor is Homed!");
  Flap_Move_Offset();
}

void Flap_Move_Offset() {
  stepper1.moveTo(0);
  stepper1.setSpeed(MOTOR_SPEED_RUN);
  while (stepper1.distanceToGo() != 0) {
    stepper1.run();
    yield();
  }
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

int Flap_Steps2Go(int flap_idx) {
  int steps = 0;
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
  
  return steps;
}


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
    Serial.print("Random character: ");
      Serial.println(new_char);

    int move_idx = 0;
    int move_steps = 0;
    move_idx = Flap_Char(new_char);
    move_steps = Flap_Steps2Go(move_idx);

    stepper1.setSpeed(MOTOR_SPEED_RUN);
    stepper1.move(move_steps);
  }
}