#ifndef FUNCTIONS_WEBSERVER_H
#define FUNCTIONS_WEBSERVER_H
#include <Arduino.h>

#include <FS.h>
#include <LittleFS.h>

#include <WebServer.h>

extern WebServer server;
extern String current_letters;

void setup_webserver();

void handleGetMessage();
void handlePostMessage();
void handleFileRequest();

#endif