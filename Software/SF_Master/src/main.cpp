#include <Arduino.h>
#include <Wire.h>
#include <WebServer.h>
#include <FS.h>
#include <SPIFFS.h>

struct I2CTx_33 {
    byte id = 33;
    char payload;
};

// I2C
  #define PIN_SCL 22
  #define PIN_SDA 21
  void I2C_Address_Scan();
  void I2C_Transmit(int digit, I2CTx_33 payload);
  byte address_list[32];
  int address_count = -1;
  #define I2C_DIGIT_ADDR_START 30
  #define I2C_DIGIT_ADDR_END   63

// Flaps
  #define I2C_BUS_DELAY 1000 // Wait a moment before scanning to give slower digits a chance to join the bus
  #define FLAPS_NUM 40
  const char char_order[] = " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.:/";

// Demo Function
  #define DEMO_PAUSE_TIME 2000
  unsigned long demo_pause_last = 0;
  int demo_state = 40;
  void Demo_Run();
  

// --- Split Flap Web API ---
WebServer server(80);
String current_letters = "      "; // Default 6 spaces, update as needed

void handleGetMessage() {
    server.send(200, "application/json", '"' + current_letters + '"');
}

void handlePostMessage() {
    String body = server.arg("plain");
    if (body.length() == 0) {
        body = server.arg("letters"); // Fallback for form data
    }
    body.toUpperCase();
    // Validate input
    for (size_t i = 0; i < body.length(); ++i) {
        char c = body[i];
        if (strchr(" ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.:/", c) == nullptr) {
            server.send(400, "text/plain", "Invalid input");
            return;
        }
    }
    // Pad or truncate to FLAPS_NUM (or 6 for demo)
    int num_chars = 6; // Change to FLAPS_NUM if needed
    if (body.length() < num_chars) {
        for (int i = body.length(); i < num_chars; ++i) {
            body += ' ';
        }
    }
    if (body.length() > num_chars) body = body.substring(0, num_chars);
    current_letters = body;
    // TODO: Send to hardware
    server.send(200, "text/plain", "OK");
}

void handleFileRequest() {
    String path = server.uri();
    if (path == "/") path = "/index.html";
    if (!SPIFFS.exists(path)) {
        server.send(404, "text/plain", "File Not Found");
        return;
    }
    String contentType = "text/plain";
    if (path.endsWith(".html")) contentType = "text/html";
    else if (path.endsWith(".css")) contentType = "text/css";
    else if (path.endsWith(".js")) contentType = "application/javascript";
    else if (path.endsWith(".png")) contentType = "image/png";
    else if (path.endsWith(".jpg")) contentType = "image/jpeg";
    else if (path.endsWith(".ico")) contentType = "image/x-icon";
    File file = SPIFFS.open(path, "r");
    server.streamFile(file, contentType);
    file.close();
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Starting sketch!");
  
  Wire.begin(PIN_SDA, PIN_SCL);

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
    return;
  }

  delay(I2C_BUS_DELAY);
  I2C_Address_Scan();

  // Web API routes
  server.on("/v1/message", HTTP_GET, handleGetMessage);
  server.on("/v1/message", HTTP_POST, handlePostMessage);
  // Serve static files
  server.onNotFound(handleFileRequest);
  server.begin();
}

void loop() {
  // Scan for addresses
  
  Demo_Run();

  delay(5000);
  server.handleClient();
}


void I2C_Address_Scan() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    // The i2c_scanner uses the return value of
    // the Wire.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) Serial.print("0");
      Serial.print(address,HEX);
      Serial.println(" !");
      nDevices++;

      // Add address to list
      if (address >= 30 && address <= 64) {
        address_count++;
        address_list[address_count] = address;
      }
      

    } else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) Serial.print("0");
      Serial.print(address,HEX);
      Serial.println(" !");
    }
  }
}

void I2C_Transmit(int digit, I2CTx_33 payload) {
  byte send_addr = address_list[digit];
  if (send_addr >= I2C_DIGIT_ADDR_START && send_addr <= I2C_DIGIT_ADDR_END) {
    // Query digit for readiness
      // if not ready, wait

    // Send Character
    Wire.beginTransmission(send_addr);
    Wire.write((byte*) &payload, sizeof(payload));
    Wire.endTransmission();    // this is what actually sends the data

    Serial.print("Sent id ");
      Serial.print(payload.id);
      Serial.print(" ");
      Serial.print(payload.payload);
      Serial.print(" to ");
      Serial.print(send_addr,HEX);
      Serial.println();
  } else {
    Serial.print("Invalid address: ");
      Serial.println(send_addr);
  }
}

void Demo_Run() {
  if (demo_pause_last + DEMO_PAUSE_TIME <= millis()) {
    // set demo_pause_last to 0 so we can set it when we turn off the motor
    //demo_pause_last = 0;
    
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

    I2CTx_33 message;
    message.payload = new_char;
    I2C_Transmit(0, message);

    demo_pause_last = millis();
  }
}