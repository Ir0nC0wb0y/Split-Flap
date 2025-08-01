#pragma once
#include <Arduino.h>

#include <FS.h>
#include <LittleFS.h>
#include <WiFi.h>


#define WIFI_CONNECT_TIME 5000
#define WIFI_HOSTNAME "SF-Master"
#define WIFI_AP_SSID "SplitFlap"
#define WIFI_AP_PASS "SomeSecurePassword"

void connect2WiFi();
void checkWiFi();