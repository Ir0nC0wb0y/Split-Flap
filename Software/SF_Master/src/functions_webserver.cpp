#include "functions_webserver.h"

void setup_webserver() {
  // Web API routes
  Serial.print("Setting up Webserver ...");
  server.on("/v1/message", HTTP_GET, handleGetMessage);
  server.on("/v1/message", HTTP_POST, handlePostMessage);
  // Serve static files
  server.onNotFound(handleFileRequest);
  server.begin();
  Serial.println(" Success!");
}

void handleGetMessage() {
      server.send(200, "application/json", '"' + display_string_server + '"');
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
    display_string_server = body;
    // TODO: Send to hardware
    server.send(200, "text/plain", "OK");
}

void handleFileRequest() {
    String path = server.uri();
    if (path == "/") path = "/index.html";
    if (!LittleFS.exists(path)) {
        server.sendHeader("Location", "/", true);
        server.send(302, "text/plain", "Resource Not Found");
        return;
    }
    String contentType = "text/plain";
    if (path.endsWith(".html")) contentType = "text/html";
    else if (path.endsWith(".css")) contentType = "text/css";
    else if (path.endsWith(".js")) contentType = "application/javascript";
    else if (path.endsWith(".png")) contentType = "image/png";
    else if (path.endsWith(".jpg")) contentType = "image/jpeg";
    else if (path.endsWith(".ico")) contentType = "image/x-icon";
    File file = LittleFS.open(path, "r");
    server.streamFile(file, contentType);
    file.close();
}