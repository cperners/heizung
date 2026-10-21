#pragma once

// Copy to secrets.h and enter your local credentials.
#define SECRET_SSID "YOUR_WIFI_SSID"
#define SECRET_PASS "YOUR_WIFI_PASSWORD"
#define MQTT_USERNAME "YOUR_MQTT_USERNAME"
#define MQTT_PASSWORD "YOUR_MQTT_PASSWORD"

// MQTT-Einstellungen an die jeweilige Installation anpassen.
#include <IPAddress.h>
#define MQTT_CLIENT_ID "ESP32_Heizung_Beispiel"
#define MQTT_HOST IPAddress(192, 168, 0, 1)
#define MQTT_PORT 1883
#define MQTT_TEXT "/SmartHome/Beispiel/Heizung/"
