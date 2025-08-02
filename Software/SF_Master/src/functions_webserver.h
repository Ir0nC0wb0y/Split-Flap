#ifndef FUNCTIONS_WEBSERVER_H
#define FUNCTIONS_WEBSERVER_H
#include <Arduino.h>

#include <FS.h>
#include <LittleFS.h>

#include <WebServer.h>

extern WebServer server;
//extern String current_letters;
extern String display_string_server;
//extern String display_string; // probably need to set this at the appropriate time

void setup_webserver();

void handleGetMessage();
void handlePostMessage();
void handleFileRequest();

#endif