#include <Arduino.h>
#include <Wire.h>


// Messages:
//  - Master
//      ID  | Type        | Payload | Digit Response 
//      0   | handshake   | empty   | Return Message 128
//      1   | Character   | char    | Digit move to char
//      2   | Set Doff    | short   | Set Digit Offset
//      3   | Get Doff    | empty   | Return message 129
//      4   | Get State   | empty   | Return message 130
//      5   | Recalibrate | empty   | Run recalibrate routine
//      6   | Re-home     | empty   | Run home routine
//      7   | Get Speed   | empty   | Return message 131
//      8   | Set Speed   | short   | Set Speed
//      9   | Get Accel   | empty   | Return message 132
//      10  | Set Accel   | short   | Set Accel
//  - Digit
//      ID  | Type        | Payload
//      128 | Handshake   | empty
//      129 | Doff RTN    | short
//      130 | CurState    | {???}
//      131 | Speed RTN   | short
//      132 | Accel RTN   | short

/*
struct msg2master { // 8 byte total
  byte id;          // one byte
  bool ready = 0;   // one byte
  short doff = 0;   // two byte
  short speed = 0;  // two byte
  short accel = 0;  // two byte
};

struct msg2digit { // 9 byte total
  byte id;         // one byte
  char character;  // one byte
  short doff;      // two byte
  byte state;      // one byte
  short speed;     // two byte
  short accel;     // two byte
};
*/

struct message_status { // 8 byte total
  byte id;         // one byte
  bool ready = 0;  // one byte
  short doff = 0;  // two byte
  short speed = 0; // two byte
  short accel = 0; // two byte
};

struct i2c_message { // 12 byte total
  byte id;                // 1 byte
  char character;         // 1 byte
  short value;            // 2 bytes
  message_status status;  // 8 bytes
};

// need some persistent 
void _onReceive(int numbytes);

class i2c_master {
  public:
    i2c_master();
    void begin(); // starts I2C bus
    //void begin(int pin_SDA, int pin_SCL); // optional SDA/SCL pin
    bool HandleMessage(); // to be run from loop to process received message
    void SendMessage(byte address, i2c_message message);
    i2c_message message;

  private:
    // useful variables
    bool _new_message = false;
    //i2c_message _last_message;

    // Wire event handlers
    
    //void _onRequest();

    // Messages
    void _clear_message();
    void _msg_snd_0(byte address);
    void _msg_snd_1(byte address, char payload);
    void _msg_snd_2(byte address, short payload);
    void _msg_snd_3(byte address);
    void _msg_snd_4(byte address);
    void _msg_snd_5(byte address);
    void _msg_snd_6(byte address);
    void _msg_snd_7(byte address);
    void _msg_snd_8(byte address, short payload);
    void _msg_snd_9(byte address);
    void _msg_snd_10(byte address, short payload);
};

class i2c_digit {
  public:
    i2c_digit();
    void begin();
    bool HandleMessage(); // to be run from loop to process received message
    void SendMessage(i2c_message message);
    i2c_message message;

  private:
    bool _new_message = false;

    // Wire event Handlers
    void _onReceive(int numbytes);
    
    // Messages
    void _msg_snd_128(byte address);
    void _msg_snd_129(byte address, short payload);
    void _msg_snd_130(byte address, message_status payload);
    void _msg_snd_131(byte address, short payload);
    void _msg_snd_132(byte address, short payload);
};