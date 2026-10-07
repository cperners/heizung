//updates
//2023-10-15 Mischer Zu - sonst heizt Raum mit. -> BoilerBetrieb();

//ende updates

/*
https://wiki.ta.co.at/Heizkreisregelung_(Funktion)
2020-01-24 chckBoiler() geändert - Kesseltemperatur war kleiner als Boilertemperatur
           chckKessel() geändert - nKesselDiff um beim BoilerAufheizen eine höhere Temperatur zu haben
https://github.com/fedorweems/YouTube/blob/Arduino-Game-V1/ESP8266%20Home%20Automation%20MQTT%20-%20Arduino
*/
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include <math.h>
#include <WiFi.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>
#include <WiFiUdp.h>
#include <sys/time.h>
#include <time.h> // Clock einbinden
#include <AsyncTCP.h>
#include <AsyncMqttClient.h>
#include <LiquidCrystal_I2C.h> //This library you can add via Include Library > Manage Library >
#include <Wire.h>
#include <OneWire.h> //fuer DS18B20
#include <EEPROM.h>
#include <Preferences.h>
#include <Keypad_I2C.h>
#include <Ticker.h>
#include <Update.h>
#include <ArduinoJson.h>

//define externe Mischersteuerung per I2c
#include "config.h"



#include "web/page.h"
#include "diagnostics/cpu_load.h"
#include "ntp_diagnostics/ntp_diagnostics.h"

#include "firmware/shared_state.inc"
#include "firmware/runtime_support.inc"
#include "firmware/web_status.inc"
#include "firmware/setup.inc"
#include "firmware/loop.inc"
#include "firmware/lcd_menu.inc"
#include "firmware/clock.inc"
#include "firmware/network.inc"
#include "firmware/sensor_reading.inc"
#include "firmware/uptime.inc"
#include "firmware/heating.inc"
#include "firmware/mqtt.inc"
#include "firmware/lcd_display.inc"
#include "firmware/mixer.inc"
#include "firmware/sensor_bus.inc"
#include "firmware/serial_console.inc"
#include "firmware/web_handlers.inc"
