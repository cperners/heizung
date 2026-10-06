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


struct NtpProbeResult { double offsetMs; double delayMs; unsigned long measuredAt; int state; };
NtpProbeResult ntpProbeResults[3] = {};
portMUX_TYPE ntpProbeMux = portMUX_INITIALIZER_UNLOCKED;
const char* ntpProbeServers[3] = {NTP_SERVER1, NTP_SERVER2, NTP_SERVER3};
static double ntpProbeNow() {
  timeval tv; gettimeofday(&tv, nullptr);
  return double(tv.tv_sec) + double(tv.tv_usec) / 1000000.0;
}
static uint32_t ntpProbeWord(const uint8_t* p) {
  return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | p[3];
}
static double ntpProbeStamp(const uint8_t* p) {
  return double(ntpProbeWord(p)) - 2208988800.0 + double(ntpProbeWord(p+4)) / 4294967296.0;
}
void ntpProbeTask(void*) {
  WiFiUDP udp;
  for (;;) {
    for (int i = 0; i < 3; ++i) {
      NtpProbeResult result = {};
      result.state = 1; // No usable reply.
      if (WiFi.status() == WL_CONNECTED && time(nullptr) > 1700000000) {
        IPAddress address;
        if (WiFi.hostByName(ntpProbeServers[i], address) && udp.begin(0)) {
          uint8_t request[48] = {}; request[0] = 0x23;
          double t1 = ntpProbeNow();
          uint32_t seconds = uint32_t(t1 + 2208988800.0);
          uint32_t fraction = uint32_t((t1 - floor(t1)) * 4294967296.0);
          for (int b=0; b<4; ++b) { request[40+b] = seconds >> (24-8*b); request[44+b] = fraction >> (24-8*b); }
          udp.beginPacket(address, 123); udp.write(request, sizeof(request));
          if (udp.endPacket()) {
            unsigned long start = millis();
            while (millis() - start < 2000UL) {
              int size = udp.parsePacket();
              if (size >= 48) {
                const double t4 = ntpProbeNow(); uint8_t response[48];
                int count = udp.read(response, sizeof(response));
                if (count == 48 && udp.remoteIP() == address && udp.remotePort() == 123 &&
                    (response[0] & 7) == 4 && (response[0] >> 6) != 3 && response[1] > 0 && response[1] < 16 &&
                    memcmp(response+24, request+40, 8) == 0) {
                  const double t2 = ntpProbeStamp(response+32), t3 = ntpProbeStamp(response+40);
                  const double elapsed = (millis()-start)/1000.0;
                  const double delay = (t4-t1)-(t3-t2);
                  if (t2 > 1700000000 && t3 >= t2 && fabs((t4-t1)-elapsed) < 0.1 && delay >= -0.001 && delay < 2.0) {
                    result.offsetMs = ((t2-t1)+(t3-t4))*500.0;
                    result.delayMs = fmax(0.0, delay*1000.0); result.state = 2; result.measuredAt = millis();
                    break;
                  }
                }
              }
              vTaskDelay(pdMS_TO_TICKS(10));
            }
          }
          udp.stop();
        }
      } else result.state = 0;
      portENTER_CRITICAL(&ntpProbeMux); ntpProbeResults[i] = result; portEXIT_CRITICAL(&ntpProbeMux);
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelay(pdMS_TO_TICKS(60000));
  }
}
String ntpProbeDisplay() {
  NtpProbeResult snapshot[3];
  portENTER_CRITICAL(&ntpProbeMux); memcpy(snapshot, ntpProbeResults, sizeof(snapshot)); portEXIT_CRITICAL(&ntpProbeMux);
  String text;
  for (int i=0; i<3; ++i) {
    text += ntpProbeServers[i]; text += ": ";
    if (snapshot[i].state == 2) {
      char line[120]; snprintf(line, sizeof(line), "%+.1f ms (Laufzeit %.1f ms, vor %lu s)", snapshot[i].offsetMs, snapshot[i].delayMs, (millis()-snapshot[i].measuredAt)/1000UL); text += line;
    } else text += snapshot[i].state == 1 ? "Keine gueltige Antwort" : "Warte auf WLAN und gueltige Zeit";
    text += "\n";
  }
  return text;
}

//3 seconds WDT

#include "secrets.h"

//#define SECRET_SSID "MyAP4Me"
//#define SECRET_PASS "dasisteintest"




//define PIN für DS18B20
//https://github.com/JChristensen/Timezone/tree/master/examples/Change_TZ_1
//define Ausgänge

//A0-A1-A2 dip switch to off position

/*  https://haus-automatisierung.com/nodered/2017/12/13/node-red-tutorial-reihe-part-4-verbindung-fhem.html */


//zwischen Menupage 1 unt 5 soll *C angezeigt werden
//zwischen Menaupage 6 und 8 soll nur die Zahl angezeigt werden
//zwischen Menupage 9 und 11 soll HH.M angezeigt werden /float
/* *******************************************************************************************************
                                         EEPROM
******************************************************************************************************* */


float kessel_min_temp = KESSEL_MIN_TEMP; // for incoming serial data

//Werte zum 1-maligen setzen im EEPROM
const float eeBoiler = 55.0; //55.0 Grad
const float eeRaum = 22.9; //22.5 Grad
const float eeRaumNacht = 22.0; //22.0 Grad
const float eeKessel = 72.0; //68.0 Grad
const float eeDiffRaum = 0.1; //0.2 Grad
const float eeDiffKessel = 14.0; //10.0 Grad
const float eeDiffBoiler = 10.0; //10.0 Grad
const float eeNacht = 21.45; //21:30 H:MM
const float eeTag = 5.45; //5:30 H:MM
const byte eeWinter = 1; // bei 1 WinterBetrieb mit Boiler wenn NurHeizung 0
const byte eeBoilerBetrieb = 0; // bei 1 für Boiler Sommberbetrieb
const byte eeNurHeizung = 0; //bei 1 nur Heizung ohne Boiler
const long eeBrennerLaufzeit = 0;
const byte eeSommerzeit_EinAus = 1; // 1 Automatik Sommerzeit aktiv
//https://www.arduino.cc/en/Reference/EEPROMGet
//https://www.arduino.cc/en/Reference/EEPROMPut
const float eetvmax = 62.0; //VorlaufMaxTemperatur bei AussentemperaturRegelung
const float eetaumin = 15.0; //MaxAussentemperatur bei AussentemperaturRegelung
const float een = 1.9; //Kurvenfaktor bei AussentemperaturRegelung
const int eeAuTempRegel = 1; 
/* *******************************************************************************************************
                                         Zeit via NTP
******************************************************************************************************* */
byte dayticker_hr=0;
bool ntpTimeValid = false;
/* *******************************************************************************************************
                                         Netzwerk
******************************************************************************************************* */
//https://github.com/micw/ArduinoProjekte/blob/master/HeizungsSteuerung/HeizungsSteuerung.ino
int wifi_retry=0;
int wifi_rssi=0;
// Insert your WiFi secrets here:
const char* ssid     = SECRET_SSID;
const char* password = SECRET_PASS;
// Set your Static IP address
IPAddress local_IP(192, 168, 0, 2);
// Set your Gateway IP address
IPAddress gateway(192, 168, 0, 254);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(192, 168, 0, 1);   //optional
IPAddress secondaryDNS(8, 8, 8, 8); //optional

bool connect2wifi, connect2mqtt = false;

int avg_time_ms;
/* *******************************************************************************************************
                                         Temperaturen
******************************************************************************************************* */
volatile bool temp_update=false;
float tVorlauf,tAussen,tKessel,tKesselDest,tKesselDiff,tBoiler,tBoilerDest,tBoilerDiff,tRoom,tRoomTag,tRoomDiff,
  tRoomNacht;
float vorlaufTemperatur; // errechnete vorlaufTemperatur für Regelung
bool heizkurveGueltig = false;
float tmyRoomdest;//Zieltemperature je nach Tageszeit
byte WinterBetrieb,BoilerBetrieb,NurHeizung;
int AussentemperaturRegelung,AussentemperaturRegelungAlt = 0;
byte Pumpenloesen = 0;
byte PumpenNachlauf = 0;
//Pumpenloesen von 18:25 bis 18:27
bool KesselHeizen, RoomHeizen, BoilerHeizen, RoomAnforderung, BoilerAnforderung = false;
bool BrennerRelais, HeizungsRelais, BoilerRelais, BoilerAufheizen = false;
bool MischerAufRelais, MischerZuRelais = false;
bool mischer_init_laeuft, mischer_init_auf, mischer_init_zu = false;

uint8_t mischer_wait, mischer_drive = 0;
const long mischer_init_time_auf = MISCHER_INIT_TIME_AUF;
const long mischer_init_time_zu = MISCHER_INIT_TIME_ZU;
char daynight='N';
char Betriebsart = '0';
long BrennerLaufzeit;
byte Sommerzeit_EinAus;
float tvmax,taumin,n;

// V2 Kachelofen-Erkennung (externer Sensor via MQTT)
// Der Kachelofen ist hydraulisch nicht mit der Heizung verbunden.
// Seine Temperatur entscheidet nur, ob Raum- oder Aussentemperaturregelung aktiv ist.
static const char KACHELOFEN_MQTT_TOPIC[] = MQTT_TEXT "kachelofenTemp";
float tKachelofen = NAN;
bool kachelofenAktiv = false;
unsigned long kachelofenLastUpdate = 0;
float kachelofenEinTemp = 50.0;
float kachelofenAusTemp = 40.0;
const unsigned long kachelofenTimeout = 10UL * 60UL * 1000UL;
enum RegelungsModus { REGELUNG_AUTO, REGELUNG_RAUM, REGELUNG_AUSSEN };
RegelungsModus regelungsModus = REGELUNG_AUTO;
volatile int requestedWebMode = -1;
int requestedOperatingMode = -1;
struct RoomSetpointRequest { bool pending; float day; float night; };
RoomSetpointRequest requestedRoomSetpoints = {};
portMUX_TYPE roomSetpointMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE operatingModeMux = portMUX_INITIALIZER_UNLOCKED;

unsigned int jumptoDefault = 0;
char jump = '0';
/* *******************************************************************************************************
                                              Zeit
******************************************************************************************************* */
float TagBegin,NachtBegin;
volatile int TagBeginHr, TagBeginMi, NachtBeginHr, NachtBeginMi;
/* *******************************************************************************************************
                                              mqtt
******************************************************************************************************* */
uint32_t wait_for_connect = 0;
//IPAddress MqttServer(192,168,000,002);
WiFiClient net;
//MQTTClient mqtt;
static const char mqttUser[] = MQTT_USERNAME;
static const char mqttPassword[] = MQTT_PASSWORD;
static const char mqttClientID[] = MQTT_CLIENT_ID;
static AsyncMqttClient asyncMqttClient;

uint8_t my_str[6]; // sting to store the incoming data from the publisher
//void callback(char* topic, byte* payload, unsigned int length);
//PubSubClient client(MqttServer, 1883, callback, net);
char mqtt_payload[32];
bool mqtt2update=false;
int mqtt_message=0;
/* *******************************************************************************************************
                                               OTA
******************************************************************************************************* */
char TimeString[32];
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta charset="utf-8">
  <title>ESP Web Server</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="icon" href="data:,">
  <style>
/* Darstellung der Heizungsuebersicht */
* { box-sizing: border-box; }
html { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; color: #20343c; text-align: left; }
body { margin: 0; background: #f1f5f6; line-height: 1.5; }
.topnav { background: #163b45; padding: 14px 20px; border-bottom: 4px solid #36b6a5; }
.topnav h1 { display: flex; align-items: center; justify-content: space-between; gap: 12px; max-width: 1120px; margin: 0 auto; font-size: clamp(.9rem, 2.5vw, 1.4rem); line-height: 1.4; color: white; white-space: nowrap; }
.topnav time { font-size: inherit; font-weight: inherit; }
.topnav hr { border: 0; border-top: 1px solid #ffffff30; margin: 10px 0; }
.layout { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 16px; max-width: 1168px; margin: 20px auto; padding: 0 24px; }
.layout > p, body > p { display: none; }
.layout > div { min-width: 0; padding: 20px; border: 1px solid #dce5e8; border-radius: 16px; background: white; box-shadow: 0 4px 16px #163b4508; overflow-wrap: anywhere; }
.layout > .content { padding: 0; }
.content { margin: 0; max-width: none; }
.card { padding: 22px; border-radius: 16px; background: white; box-shadow: none; }
h3 { margin: 0 0 12px; color: #20343c; font-size: 1rem; font-weight: 600; }
.state { margin: 12px 0; font-size: 1.1rem; color: #536b74; }
.state span { display: inline-block; padding: 4px 12px; border-radius: 20px; background: #edf3f5; color: #20343c; font-weight: 700; }
p { font-size: 1rem; margin: 14px 0; }
.dht-labels { display: block; font-size: .9rem; color: #607681; padding-bottom: 8px; }
#troom, #taussen, #tkessel, #tvorlauf, #tboiler, #tkachelofen { font-size: 1.8rem; font-weight: 700; letter-spacing: -.04em; color: #163b45; }
.units { font-size: .9rem; color: #607681; }
strong { display: block; margin-top: 8px; color: #14796c; font-size: 1rem; }
.button, input[type="submit"] { border: 0; border-radius: 9px; background: #14796c; color: white; padding: 10px 18px; font: inherit; font-weight: 600; cursor: pointer; }
.button:hover, input[type="submit"]:hover { background: #105f55; }
button:focus-visible, input:focus-visible { outline: 3px solid #36b6a5; outline-offset: 3px; }
input[type="number"] { border: 1px solid #bdcdd3; border-radius: 7px; padding: 8px; margin: 5px 0 10px; font: inherit; }
input[type="range"] { width: 100%; accent-color: #14796c; }
form { line-height: 2; font-size: .9rem; }
@media (max-width: 540px) { .layout { grid-template-columns: 1fr; padding: 0 16px; gap: 12px; margin: 16px auto; } .topnav { padding: 20px 16px; } }

/* Bedientasten fuer die Kachelofen-Schaltschwellen */
.threshold-card { grid-column: span 1; }
@media (max-width: 540px) { .threshold-card { grid-column: span 1; } }
.threshold-form { line-height: 1.5; }
.threshold-form label { display: block; margin: 18px 0 8px; font-size: .85rem; font-weight: 600; color: #536b74; }
.threshold-row { display: grid; grid-template-columns: 32px minmax(0, 1fr) 32px; gap: 4px; align-items: center; }
.threshold-row button { height: 38px; border: 1px solid #b9d7d2; border-radius: 9px; background: #eaf5f2; color: #126a5f; font: inherit; font-size: 1.4rem; cursor: pointer; }
.threshold-row button:hover { background: #d8eee7; }
.threshold-value { display: flex; align-items: center; justify-content: center; gap: 3px; min-width: 0; white-space: nowrap; }
.threshold-value input[type="number"] { width: 5ch; min-width: 0; max-width: 100%; flex: 0 1 5ch; margin: 0; padding: 8px 0; border: 0; background: transparent; text-align: center; font-size: 1.2rem; font-weight: 700; color: #163b45; appearance: textfield; -moz-appearance: textfield; }
.threshold-value input::-webkit-inner-spin-button, .threshold-value input::-webkit-outer-spin-button { appearance: none; margin: 0; }
.threshold-value span { flex: 0 0 auto; white-space: nowrap; color: #607681; font-size: .9rem; }
.threshold-hint { font-size: .78rem; color: #607681; margin: 16px 0; }
.threshold-save { width: 100%; padding: 12px; border: 0; border-radius: 9px; background: #14796c; color: white; font: inherit; font-weight: 600; cursor: pointer; }
.threshold-save:disabled { opacity: .5; cursor: default; }

.output-status {display:grid;gap:12px;}
.output-row {display:flex;flex-direction:row-reverse;align-items:center;justify-content:flex-end;gap:12px;}
.output-indicator {min-width:62px;flex-shrink:0;}
.output-led {display:inline-block;width:16px;height:16px;border-radius:50%;background:#94a3b8;box-shadow:inset 0 0 3px #0005;flex-shrink:0;}
.output-led.on {background:#16a34a;box-shadow:0 0 6px #16a34a66;}
.output-led.off {background:#dc2626;}
.output-indicator {display:flex;align-items:center;gap:8px;}
</style>
<title>ESP32 Heizungs-Server</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<link rel="stylesheet" href="https://use.fontawesome.com/releases/v5.7.2/css/all.css" 
        integrity="sha384-fnmOCqbTlWIlj8LyTjo7mOUStjsKC4pOpQbqyi7RrhN7udi9RwhKkMHpvLbHG9Sr" crossorigin="anonymous">
<link rel="icon" href="data:,">
</head>
<body>
  <div class="topnav">
    <h1><span>ESP32 Heizung</span><time id="header-clock">%WEBTIME%</time></h1>
  </div>
  <section class="layout">
  <div>
    <i class="fas fa-thermometer-half" style="color:#059e8a;"></i> 
    <span class="dht-labels">K&uuml;che</span> 
    <span id="troom">%TROOM%</span>
    <sup class="units">&deg;C</sup>
  </div>
  <div>
    <i class="fas fa-thermometer-half" style="color:#059e8a;"></i> 
    <span class="dht-labels">Aussen</span> 
    <span id="taussen">%TAUSSEN%</span>
    <sup class="units">&deg;C</sup>
  </div>
  <div>
    <i class="fas fa-thermometer-half" style="color:#059e8a;"></i> 
    <span class="dht-labels">Gaskessel</span> 
    <span id="tkessel">%TKESSEL%</span>
    <div id="brennersperre" role="status">%BRENNERSPERRE%</div>
    <sup class="units">&deg;C</sup>
  </div>
  <div>
    <i class="fas fa-thermometer-half" style="color:#059e8a;"></i> 
    <span class="dht-labels">Vorlauf</span> 
    <span id="tvorlauf">%TVORLAUF%</span>
    <sup class="units">&deg;C</sup>
  </div>
  <div>
    <i class="fas fa-thermometer-half" style="color:#059e8a;"></i> 
    <span class="dht-labels">Boiler</span> 
    <span id="tboiler">%TBOILER%</span>
    <sup class="units">&deg;C</sup>
  </div>
  <p>
  </section>
  <section class="layout">
  <div><h3>Ausgangsstatus</h3>
    <div class="output-status">
      <div class="output-row"><span>Brenner</span><span class="output-indicator"><span id="led-brenner" class="output-led" aria-hidden="true"></span><span id="status-brenner">Wird geladen</span></span></div>
      <div class="output-row"><span>Boilerpumpe</span><span class="output-indicator"><span id="led-boiler" class="output-led" aria-hidden="true"></span><span id="status-boiler">Wird geladen</span></span></div>
      <div class="output-row"><span>Heizungspumpe</span><span class="output-indicator"><span id="led-heizung" class="output-led" aria-hidden="true"></span><span id="status-heizung">Wird geladen</span></span></div>
      <div class="output-row"><span>Mischer auf</span><span class="output-indicator"><span id="led-mischerauf" class="output-led" aria-hidden="true"></span><span id="status-mischerauf">Wird geladen</span></span></div>
      <div class="output-row"><span>Mischer zu</span><span class="output-indicator"><span id="led-mischerzu" class="output-led" aria-hidden="true"></span><span id="status-mischerzu">Wird geladen</span></span></div>
    </div>
    <small>Gr&uuml;n: Ausgang ein &middot; Rot: Ausgang aus</small>
  </div>
  <div>
    <h3>Gasverbrauch heute</h3>
    <strong id="gas-day">%GASDAY%</strong> m&sup3;
    <p>Seit erster Meldung des Tages</p>
    <small>Gesamtstand: <span id="gas-total">%GASTOTAL%</span> m&sup3;</small>
  </div>
  <div>
    <h3>Regelung</h3>
    <strong id="regelungsart">%REGELUNGSART%</strong>
    <form action="/regelung" method="post">
      <p><label for="regelung-auswahl">Regelungsart ausw&auml;hlen</label></p>
      <p><select id="regelung-auswahl" name="modus" style="width:100%;padding:12px;font:inherit;border:1px solid #cbd9dd;border-radius:8px">
        <option value="auto" %MODEAUTO%>Automatik</option>
        <option value="raum" %MODERAUM%>Raumtemperatur</option>
        <option value="aussen" %MODEAUSSEN%>Au&szlig;entemperatur</option>
      </select></p>
      <button class="threshold-save" type="submit">&Uuml;bernehmen</button>
    </form>
  </div>
  <div>
    <h3>Betriebsart</h3>
    <strong id="betriebsart">%BETRIEBSART%</strong>
    <form action="/betriebsart" method="post">
      <p><label for="betrieb-auswahl">Betriebsart ausw&auml;hlen</label></p>
      <p><select id="betrieb-auswahl" name="modus" style="width:100%;padding:12px;font:inherit;border:1px solid #cbd9dd;border-radius:8px">
        <option value="auto" %BETRIEBAUTO%>Automatikbetrieb</option>
        <option value="boiler" %BETRIEBBOILER%>Nur Boiler</option>
        <option value="heizung" %BETRIEBHEIZUNG%>Nur Heizung</option>
        <option value="aus" %BETRIEBAUS%>Aus</option>
      </select></p>
      <button class="threshold-save" type="submit">&Uuml;bernehmen</button>
    </form>
  </div>
  <div><h3>NTP-Zeitserver</h3>
    <p>Abweichung zur ESP32-Uhr: Plus = Server voraus</p>
    <pre id="ntp-abweichung" style="white-space:pre-wrap;font:inherit;overflow-wrap:anywhere">Messung wird geladen</pre>
    <small>Messung etwa jede Minute; Netzwerkwege beeinflussen die Genauigkeit.</small>
  </div>
  </section>
  <section class="layout">
  <div class="threshold-card">
    <h3>Wunschraumtemperatur</h3>
    <form id="room-setpoints" class="threshold-form" action="/raumtemperaturen" method="post">
      <label for="room-tag">Tag</label>
      <div class="threshold-row">
        <button type="button" aria-label="Tagtemperatur senken" onclick="document.getElementById('room-tag').stepDown()">&minus;</button>
        <div class="threshold-value"><input id="room-tag" name="tag" type="number" min="5" max="40" step="0.1" value="%RAUMTAG%" readonly required><span>&deg;C</span></div>
        <button type="button" aria-label="Tagtemperatur erh&ouml;hen" onclick="document.getElementById('room-tag').stepUp()">+</button>
      </div>
      <label for="room-nacht">Nacht</label>
      <div class="threshold-row">
        <button type="button" aria-label="Nachttemperatur senken" onclick="document.getElementById('room-nacht').stepDown()">&minus;</button>
        <div class="threshold-value"><input id="room-nacht" name="nacht" type="number" min="5" max="40" step="0.1" value="%RAUMNACHT%" readonly required><span>&deg;C</span></div>
        <button type="button" aria-label="Nachttemperatur erh&ouml;hen" onclick="document.getElementById('room-nacht').stepUp()">+</button>
      </div>
      <p id="room-save-status" class="threshold-hint" role="status">&Auml;nderungen werden erst mit Speichern &uuml;bernommen.</p>
      <button id="room-save" class="threshold-save" type="submit">Wunschtemperaturen speichern</button>
    </form>
  </div>
  <div>
    <span class="dht-labels">Kachelofen</span>
    <span id="tkachelofen">%TKACHELOFEN%</span>
    <sup class="units">&deg;C</sup>
  </div>
  <div>Kachelofen: <strong>%KACHELOFENSTATUS%</strong></div>
  <div class="threshold-card">
    <form class="threshold-form" action="/kachelofen" method="get">
  <h3>Kachelofen-Schaltschwellen</h3>
  <label for="threshold-ein">Aktiv ab</label>
  <div class="threshold-row">
    <button type="button" aria-label="Einschalttemperatur senken" onclick="adjustThreshold('threshold-ein', -1)">&minus;</button>
    <div class="threshold-value"><input id="threshold-ein" name="ein" type="number" min="10" max="150" step="0.5" value="%KACHELOFENEIN%" readonly required><span>&deg;C</span></div>
    <button type="button" aria-label="Einschalttemperatur erhöhen" onclick="adjustThreshold('threshold-ein', 1)">+</button>
  </div>
  <label for="threshold-aus">Inaktiv bei oder unter</label>
  <div class="threshold-row">
    <button type="button" aria-label="Ausschalttemperatur senken" onclick="adjustThreshold('threshold-aus', -1)">&minus;</button>
    <div class="threshold-value"><input id="threshold-aus" name="aus" type="number" min="0" max="140" step="0.5" value="%KACHELOFENAUS%" readonly required><span>&deg;C</span></div>
    <button type="button" aria-label="Ausschalttemperatur erhöhen" onclick="adjustThreshold('threshold-aus', 1)">+</button>
  </div>
  <p class="threshold-hint" id="threshold-hint" aria-live="polite">Änderungen werden erst mit Speichern übernommen.</p>
  <button class="threshold-save" id="threshold-save" type="submit">Schaltschwellen speichern</button>
</form>
<script>
function adjustThreshold(id, direction) {
  const field = document.getElementById(id);
  if (direction > 0) field.stepUp(); else field.stepDown();
  const ein = document.getElementById('threshold-ein').valueAsNumber;
  const aus = document.getElementById('threshold-aus').valueAsNumber;
  const valid = Number.isFinite(ein) && Number.isFinite(aus) && aus < ein;
  document.getElementById('threshold-save').disabled = !valid;
  document.getElementById('threshold-hint').textContent = valid
    ? 'Änderungen werden erst mit Speichern übernommen.'
    : 'Die Ausschalttemperatur muss niedriger als die Einschalttemperatur sein.';
}
</script>
  </div>
  </section>
  <p>
  <section class="layout">
  </section>
  <p>
  <section class="layout">
  <div>WLAN-SSID: <span id="wifi_rssi">%WIFISSID%</span></div>
  <div>WLAN-Signal: <span id="wifi_rssi">%WIFIRSSI%</span></div>
  <div><input type="range" onchange="updateSliderTimer(this)" id="wifi_rssis" min="-100" max="-10" value="%WIFIRSSI%" step="1" class="slider2"></div>
  </section>
  <p>
<script>
  setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("troom").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/troom", true);
  xhttp.send();
}, 10000 ) ;

setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("taussen").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/taussen", true);
  xhttp.send();
}, 10000 ) ;

setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("tkessel").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/tkessel", true);
  xhttp.send();
}, 10000 ) ;

setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("tvorlauf").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/tvorlauf", true);
  xhttp.send();
}, 10000 ) ;

setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("tboiler").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/tboiler", true);
  xhttp.send();
}, 10000 ) ;

setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("tkachelofen").innerHTML = this.responseText;
    }
  };
  xhttp.open("GET", "/tkachelofen", true);
  xhttp.send();
}, 10000 ) ;

function updateBrennerSensorLock() {
  fetch('/brennersperre', {cache: 'no-store'})
    .then(function(response) {
      if (!response.ok) throw new Error('Status nicht erreichbar');
      return response.text();
    })
    .then(function(status) { document.getElementById('brennersperre').textContent = status; })
    .catch(function() { document.getElementById('brennersperre').textContent = 'Sperrstatus nicht erreichbar'; });
}
setInterval(updateBrennerSensorLock, 3000);

var clockRequestPending = false;
function updateHeaderClock() {
  if (clockRequestPending) return;
  clockRequestPending = true;
  var request = new XMLHttpRequest();
  request.open('GET', '/uhrzeit?tick=' + Date.now(), true);
  request.timeout = 2500;
  request.onload = function() {
    if (request.status === 200) document.getElementById('header-clock').textContent = request.responseText;
  };
  request.onloadend = function() { clockRequestPending = false; };
  request.send();
}
setInterval(updateHeaderClock, 1000);
updateHeaderClock();

document.getElementById('room-setpoints').addEventListener('submit', function(event) {
  event.preventDefault();
  var button = document.getElementById('room-save');
  var status = document.getElementById('room-save-status');
  button.disabled = true; status.textContent = 'Wird gespeichert...';
  var body = new URLSearchParams(new FormData(event.target));
  fetch('/raumtemperaturen', {method:'POST', body:body})
    .then(function(response) { if (!response.ok) throw new Error();
      status.textContent = 'Zur Übernahme vorgemerkt. Bitte anschließend neu laden und Werte prüfen.';
    })
    .catch(function() { status.textContent = 'Speichern nicht bestätigt. Bitte Verbindung prüfen.'; })
    .finally(function() { button.disabled = false; });
});

var outputKeys = ['brenner','boiler','heizung','mischerauf','mischerzu'];
var outputStatusPending = false;
function updateOutputStatus() {
  if (outputStatusPending) return;
  outputStatusPending = true;
  var request = new XMLHttpRequest();
  request.open('GET', '/ausgangsstatus', true); request.timeout = 2500;
  function unavailable() {
    outputKeys.forEach(function(key) {
      document.getElementById('led-'+key).className = 'output-led';
      document.getElementById('status-'+key).textContent = 'Nicht erreichbar';
    });
  }
  request.onload = function() {
    if (request.status !== 200) { unavailable(); return; }
    try {
      var values = JSON.parse(request.responseText);
      if (!Array.isArray(values) || values.length !== 5) throw new Error();
      outputKeys.forEach(function(key, index) {
        var active = values[index] === 1;
        document.getElementById('led-'+key).className = 'output-led ' + (active ? 'on' : 'off');
        document.getElementById('status-'+key).textContent = active ? 'Ein' : 'Aus';
      });
    } catch (error) { unavailable(); }
  };
  request.onerror = unavailable; request.ontimeout = unavailable;
  request.onloadend = function() { outputStatusPending = false; };
  request.send();
}
setInterval(updateOutputStatus, 1000); updateOutputStatus();

function updateNtpProbe() {
  fetch('/ntp-abweichung', {cache:'no-store'})
    .then(function(response) { if (!response.ok) throw new Error(); return response.text(); })
    .then(function(text) { document.getElementById('ntp-abweichung').textContent = text; })
    .catch(function() { document.getElementById('ntp-abweichung').textContent = 'Nicht erreichbar'; });
}
setInterval(updateNtpProbe, 5000); updateNtpProbe();
setInterval(function() {
  fetch('/betriebsart' , {cache: 'no-store'})
    .then(function(response) { if (!response.ok) throw new Error(); return response.text(); })
    .then(function(text) { document.getElementById('betriebsart').textContent = text; })
    .catch(function() { document.getElementById('betriebsart').textContent = 'Betriebsstatus nicht erreichbar'; });
}, 3000);

setInterval(function() {
  fetch('/regelungsart', {cache: 'no-store'})
    .then(function(response) { if (!response.ok) throw new Error(); return response.text(); })
    .then(function(text) { document.getElementById('regelungsart').textContent = text; })
    .catch(function() { document.getElementById('regelungsart').textContent = 'Regelungsstatus nicht erreichbar'; });
}, 3000);

setInterval(function() {
  [['/gas?daily=1', 'gas-day'], ['/gas', 'gas-total']].forEach(function(item) {
    fetch(item[0], {cache: 'no-store'})
      .then(function(response) { if (!response.ok) throw new Error(); return response.text(); })
      .then(function(text) { document.getElementById(item[1]).textContent = text; })
      .catch(function() { document.getElementById(item[1]).textContent = 'Nicht erreichbar'; });
  });
}, 10000);
</script>
</body>
</html>)rawliteral";

/* *******************************************************************************************************
                                               NTP
******************************************************************************************************* */
//NTPClient
//GMT Time Zone with sign

//change this to a random number between 0-255 to force time update

//closest NTP Server
//#define NTP_SERVER "0.at.pool.ntp.org"
const long utcOffsetInSeconds = 3600;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
unsigned long timeUpdated = 0;
bool set2myuhr=false;
byte myhours=0;
byte myminutes=0;
byte mysecunds=0;
byte mymonth=0;
byte myday=0;
byte myweekday=0;
int myyear=0;
//brennerlaufzeit
long brhours=0;
byte brminutes=0;
byte brsecunds=0;

//uptimeZeit
long uDay=0;
int uHour =0;
int uMinute=0;
int uSecond=0;
int uHighMillis=0;
int uRollover=0;


/*ReadDS18B20mk19222*/
const String sPrgmName="Heizung (c)perni.at";
const String sVers="V1.03";

/* *******************************************************************************************************
                                         Timing
******************************************************************************************************* */
//const unsigned long ANSWER_TIME = 1900;
unsigned long previousTime = 0;
unsigned long previousTime1 = 0;
unsigned long previousTime_MischerInit = 0;
unsigned long previousTime_wifi = 0;
bool wifiRecon = false;
bool mqttRecon = false;
unsigned long previousTime_mqtt = 0;
volatile bool readSensor = false;
/* *******************************************************************************************************
                                         LCD I2C
******************************************************************************************************* */
volatile int lcd_display_light = 30;
bool lcdPresent = false;
bool keypadPresent = false;
bool ioExtenderPresent = false;

// Auch print/printf laufen ueber write: ohne Display keine Buszugriffe.
class OptionalLCD : public LiquidCrystal_I2C {
public:
  OptionalLCD(uint8_t address, uint8_t columns, uint8_t rows)
    : LiquidCrystal_I2C(address, columns, rows) {}
  void begin() { if (lcdPresent) LiquidCrystal_I2C::begin(); }
  void clear() { if (lcdPresent) LiquidCrystal_I2C::clear(); }
  void setCursor(uint8_t column, uint8_t row) {
    if (lcdPresent) LiquidCrystal_I2C::setCursor(column, row);
  }
  void setBacklight(uint8_t value) {
    if (lcdPresent) LiquidCrystal_I2C::setBacklight(value);
  }
  size_t write(uint8_t value) override {
    return lcdPresent ? LiquidCrystal_I2C::write(value) : 1;
  }
};
OptionalLCD lcd(lcd_addr, 20, 4); //0x3F wird ersetzt - Davor den i2c scanner laufen lassen!!! 16 Zeichen, 2 Zeilen
// Pin 4, 5 (D2, D1) für I2C
/* *******************************************************************************************************
                                         Keypad
******************************************************************************************************* */
//KeyPad
const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
 byte rowPins[4] = {0,1,2,3}; //P0-P3 to R1-R4
 byte colPins[4] = {4,5,6,7}; //P4-P6 to C1-C4
Keypad_I2C I2C_Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS, keypad_addr, PCF8574);
const byte KEYSLENGTH_Z = 5;
const byte KEYSLENGTH_T = 4;
const byte KEYSLENGTH_N = 1;

char keyBuffer[KEYSLENGTH_Z+1] = {'-','-','-','-','-'};
volatile byte data_count = 0;
volatile bool data_ready=false;


/* *******************************************************************************************************
                                         MenuPage
******************************************************************************************************* */

volatile int MenuPage=0;

/* *******************************************************************************************************
                                         I2C_IO_Extender
******************************************************************************************************* */
uint8_t ioextender0_indicate, dataToI2C;
static const uint8_t exMischerAuf_Pin = 0; //1
static const uint8_t exMischerZu_Pin = 1; //1
static const uint8_t exBrenner_Pin = 2; //14
static const uint8_t exHeizung_Pin = 3; //12
static const uint8_t exBoiler_Pin = 4; //13

/* *******************************************************************************************************
                                         Board-PINs
******************************************************************************************************* */

static const uint8_t BrennerPin = BRENNER_PIN;
static const uint8_t HeizungPin = HEIZUNG_PIN;
static const uint8_t BoilerPin = BOILER_PIN;
static const uint8_t MischerAuf_Pin = MISCHER_AUF_PIN;
static const uint8_t MischerZu_Pin = MISCHER_ZU_PIN;

/*
NEU
youtube.com/watch?v=7h2bE2vNoaY
NEU ESP32:
                                   GND 1|      | 38 GND
                                   VCC 2|      | 37   GPIO_23---MOSI
                                    EN 3|      | 36   GPIO_22---*I2C SCL -LCD, Keypad
                               GPIO_36 4|      | 35   GPIO_1----TxD0
                               GPIO_39 5|      | 34   GPIO_3----RxD0
                               GPIO_34 6|      | 33   GPIO_21----*I2C SDA -LCD, Keypad
                               GPIO_35 7|      | 32   GPIO_20----*
             T3_Kesseltemp*----GPIO_32 8|      | 31   GPIO_19---MISO
             T4_Boilertemp*----GPIO_33 9|      | 30   GPIO_18----*MischerAuf
           T1_Vorlauftemp*----GPIO_25 10|      | 29   GPIO_5    CS0
            T2_Aussentemp*----GPIO_26 11|      | 28   GPIO_17----*Heizungspumpe
             T3_Küchetemp*----GPIO_27 12|      | 27   GPIO_16----*Brenner
                              GPIO_14 13|      | 26   GPIO_4-----*Boilerpumpe
                              GPIO_12 14|      | 25   GPIO_0
                                               |----------unten------- 
                                               |24   GPIO_2----*MischerZu
                                               |23   GPIO_15
                                               |22   GPIO_8
                                               |21   GPIO_7
                                               |20   GPIO_6
                                               |19   GPIO_11
                                               |18   GPIO_10
                                               |17   GPIO_9
                                               |16   GPIO_13----*T0_Kesseltemp
                                               |15 GND

*/
/* *******************************************************************************************************
                                         DS18B20 Sensoren
******************************************************************************************************* */

//--------------------------fuer OneWire.h-------------------
OneWire sensorDS1820[5]{
  OneWire(KESSELTEMP),
  OneWire(VORLAUFTEMP),
  OneWire(AUSSENTEMP),
  OneWire(KUECHENTEMP),
  OneWire(BOILERTEMP),
};
//byte addr[4][8];
byte addr[8];
byte type_s;
int setup_sensorDS1820=5;
//--------------------------fuer OneWire.h-------------------
static const byte kTtureSensorMaxIndex=4;
static const int tture[kTtureSensorMaxIndex+1] = {KESSELTEMP,VORLAUFTEMP,AUSSENTEMP,KUECHENTEMP,BOILERTEMP};

byte bLoopCounter = 0;//used for misc loop counters.
    //Values in bLoopCounter do not need to persist
    //  in places not nearby where it is set.
    //In older versions of this code, "x" was used
    //  as bLoopCounter is now used.
    //It was made global only because having a
    //  variable for this sort of task is often useful.

volatile byte bWhichSensor = 0;//This variable was called "count"
  //in earlier versions of this code.

//Following globals used to communicate results back
//from readTturePt1, Pt2, and to send data to printTture...
//See webpage for what they hold.

int TReading[kTtureSensorMaxIndex+1], SignBit[kTtureSensorMaxIndex+1],
        Whole[kTtureSensorMaxIndex+1],Fract[kTtureSensorMaxIndex+1];
//Whole[] holds the ABSOLUTE VALUE of the integer part
//  of the reading. E.g. for either 12.7 r -12.7, Whole
//  holds 12. (SignBit[] tells you if it is +ve or -ve.)

float fTc_100[kTtureSensorMaxIndex+1];


/* *******************************************************************************************************
                                         Funktion Deklarationen
******************************************************************************************************* */
//----------------------------------------------------------
void setmyuhr();
void myuhr();
void Schaltuhr();
void dayticker();
void ntpupdate();
void set2mqttupdate();
void mqttupdate();
void connect();
void WiFiEvent(WiFiEvent_t event);
void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
void onMqttSubscribe(uint16_t packetId, uint8_t qos);
void connectToMqtt();
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total);
void processMqttMessages();
void connectToWifi();
bool OneWireReset(int Pin);
void OneWireOutByte(int Pin, byte d);
byte OneWireInByte(int Pin);
void readTturePt1(byte Pin);
void readTturePt2(byte Pin, const byte tmp_bWhichSensor);
void printTture();
void KeyPad();
void addToKeyBuffer(char inkey);
void checkEingabe();
void EEPROMWrite(int address, int value);
int EEPROMRead(int address);
void MenueMain();
void MenueDefault();
void MenueBooting();
void MenuRoomTemp();
void MenuBoilerTemp();
void MenuKesseltemp();
void MenuDifRaumtemp();
void MenuDifKesseltemp();
void MenuRaumTempNacht();
void MenuDifBoilerTemp();
void Menu_tvmax();
void Menu_taumin();
void Menu_n();
void MenuBetriebWinter();
void MenuAussentempRegelung();
void uptime();
void print_Uptime();
void Automatik();
void Boilerbetrieb();
void Heizungsbetrieb();
void RoomAnforderungf();
void updateKachelofenStatus();
void BoilerAnforderungf();
void MenuBetriebBoiler();
void MenuBetriebNurHeizung();
void MenuHeizbeginnTag();
void MenuHeizEndeNacht();
void Time2LCD();
void SetOutPin();
bool heatingPumpSensorFault();
bool boilerPumpSensorFault();
void kein_Betrieb();
int chckKessel();
void chckRoom();
void chckBoiler();
void Regelungs_Switch_AI();
void Serial_Read();
//void clearstring();
void print_Main_LCD_Values();
boolean summertime_EU(int year, byte month, byte day, byte hour, byte tzHours);
void MenuSommerzeitEinAus();
void hpumpe_nachlauf();
void I2C_IO_Init(uint8_t address, uint8_t data);
void I2C_IO_BitWrite(uint8_t address, uint8_t data);
uint8_t I2C_IO_ReadInputs(uint8_t address);
void MischerInit();
void MischerAuf();
void MischerZu();
void MischerStop();
bool enforceVorlaufSensorLock();
void SetToRT_Regelung();
void sensorDS1820_indicateChip(byte pin);
void sensorDS1820_reset(byte pin);
void sensorDS1820_read(byte pin);
void serial_go_home();
void serial_clear_screen();
void serial_newline();
void notifyClients();
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len);
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len);
void initWebSocket();
String processor(const String& var);
String readTemperature(const float& var);
String readValue(const int& var);

Ticker timer0_time(setmyuhr, 1000);
Ticker timer1_2lcd(Time2LCD, 1000);
Ticker timer2_ntp(dayticker, 3600000);
Ticker timer3_mqttupdate(set2mqttupdate,10000);//alle 25 Sekunden Update zu MQTT-Server
char doppelp = ' ';


/* *******************************************************************************************************
                                         SETUP
******************************************************************************************************* */
// Sensorstatus ist getrennt vom letzten Messwert, damit Fehler sichtbar bleiben.
const unsigned long SENSOR_MAX_AGE_MS = 30000UL;
bool sensorReadingValid[5] = {};
unsigned long sensorLastSuccess[5] = {};
const char* sensorNames[5] = {"Kessel", "Vorlauf", "Aussen", "Raum", "Boiler"};

Preferences gasStorage;
bool gasStorageReady = false;
double gasTotal = NAN;
double gasDayBase = NAN;
int gasDayKey = 0;
bool gasDayReady = false;
portMUX_TYPE gasMux = portMUX_INITIALIZER_UNLOCKED;
double pendingGasTotal = 0;
bool pendingGas = false;
bool pendingGasRetained = false;

int currentGasDay() {
  const time_t utc = time(nullptr);
  if (utc < static_cast<time_t>(1577836800UL)) return 0;
  time_t local = utc + GMT_TIME_ZONE * utcOffsetInSeconds;
  struct tm calendar = {};
  if (!gmtime_r(&local, &calendar)) return 0;
  if (Sommerzeit_EinAus && summertime_EU(calendar.tm_year + 1900,
      calendar.tm_mon + 1, calendar.tm_mday, calendar.tm_hour, GMT_TIME_ZONE)) {
    local += 3600;
    if (!gmtime_r(&local, &calendar)) return 0;
  }
  return (calendar.tm_year + 1900) * 10000 + (calendar.tm_mon + 1) * 100 + calendar.tm_mday;
}

void processGasReading() {
  double value;
  bool retained;
  portENTER_CRITICAL(&gasMux);
  const bool available = pendingGas;
  value = pendingGasTotal;
  retained = pendingGasRetained;
  pendingGas = false;
  portEXIT_CRITICAL(&gasMux);
  const int day = currentGasDay();
  if (day != gasDayKey) gasDayReady = false;
  if (!available) return;
  if ((isfinite(gasTotal) && value < gasTotal) ||
      (day == gasDayKey && isfinite(gasDayBase) && value < gasDayBase)) {
    Serial.println("Gaszaehler: ruecklaeufigen Stand verworfen");
    return;
  }
  gasTotal = value;
  if (day == 0) return;
  if (day != gasDayKey || !isfinite(gasDayBase)) {
    // Ein alter Retain-Wert ist keine neue Tagesmessung.
    if (retained) return;
    gasDayKey = day;
    gasDayBase = value;
    if (gasStorageReady) {
      gasStorage.putDouble("base", gasDayBase);
      gasStorage.putInt("day", gasDayKey);
    }
  }
  gasDayReady = true;
}

String gasDisplay(bool daily) {
  if (!isfinite(gasTotal)) return "Noch keine Meldung";
  if (daily && (!gasDayReady || gasDayKey != currentGasDay())) return "Warte auf Tagesmessung";
  char text[32];
  snprintf(text, sizeof(text), "%.3f", daily ? gasTotal - gasDayBase : gasTotal);
  return String(text);
}

enum BoilerDiagnostic { BOILER_WAITING, BOILER_OK, BOILER_NO_RESPONSE, BOILER_ZERO_DATA, BOILER_HIGH_DATA,
                        BOILER_BAD_CRC, BOILER_BAD_VALUE };
volatile BoilerDiagnostic boilerDiagnostic = BOILER_WAITING;
byte boilerRawBytes[9] = {};
bool boilerRawAvailable = false;
portMUX_TYPE boilerDiagnosticMux = portMUX_INITIALIZER_UNLOCKED;


const char* boilerDiagnosticText() {
  switch (boilerDiagnostic) {
    case BOILER_NO_RESPONSE: return "Keine Sensorantwort bei Anwesenheitspruefung";
    case BOILER_ZERO_DATA: return "Datenblock durchgehend Null";
    case BOILER_HIGH_DATA: return "Datenblock durchgehend 255";
    case BOILER_BAD_CRC: return "Pruefsumme fehlerhaft";
    case BOILER_BAD_VALUE: return "Temperaturwert ungueltig";
    case BOILER_OK:
      return (unsigned long)(millis() - sensorLastSuccess[4]) > SENSOR_MAX_AGE_MS ?
        "Letzter gueltiger Messwert zu alt" : "Messung gueltig";
    default: return "Warte auf erste Boiler-Messung";
  }
}

String boilerDiagnosticDetails() {
  byte bytes[9];
  bool available;
  portENTER_CRITICAL(&boilerDiagnosticMux);
  memcpy(bytes, boilerRawBytes, sizeof(bytes));
  available = boilerRawAvailable;
  portEXIT_CRITICAL(&boilerDiagnosticMux);
  String text = boilerDiagnosticText();
  if (available) {
    char raw[40];
    snprintf(raw, sizeof(raw), "%02X %02X %02X %02X %02X %02X %02X %02X %02X",
             bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7], bytes[8]);
    text += " | Letzter gelesener Datenblock (Hex): ";
    text += raw;
  }
  return text;
}

bool sensorIsUsable(byte index) {
  return index < 5 && sensorReadingValid[index] &&
         millis() - sensorLastSuccess[index] <= SENSOR_MAX_AGE_MS;
}

// Status bei Aenderung und nach Wiederverbindung erneut senden.
void publishSensorStates() {
  static bool published[5] = {};
  static bool lastValid[5] = {};
  static float lastTemperature[5] = {};
  static unsigned long lastAttempt = 0;
  if (!asyncMqttClient.connected()) {
    for (byte i = 0; i < 5; ++i) published[i] = false;
    return;
  }
  if (millis() - lastAttempt < 1000UL) return;
  lastAttempt = millis();
  static String lastBoilerReason;
  const String boilerReason = boilerDiagnosticText();
  if (!published[4] || boilerReason != lastBoilerReason) {
    const String diagnosticTopic = String(MQTT_TEXT) + "sensordiagnose/boiler";
    if (asyncMqttClient.publish(diagnosticTopic.c_str(), 1, true, boilerReason.c_str()) != 0)
      lastBoilerReason = boilerReason;
  }
  const char* suffixes[5] = {"kessel", "vorlauf", "aussen", "raum", "boiler"};
  const float* values[5] = {&tKessel, &tVorlauf, &tAussen, &tRoom, &tBoiler};
  for (byte i = 0; i < 5; ++i) {
    const float temperature = *values[i];
    const bool valid = sensorIsUsable(i) && isfinite(temperature);
    if (published[i] && valid == lastValid[i] &&
        (!valid || temperature == lastTemperature[i])) continue;
    const String topic = String(MQTT_TEXT) + "sensorstatus/" + suffixes[i];
    char message[32];
    if (valid) snprintf(message, sizeof(message), "%.1f", temperature);
    else snprintf(message, sizeof(message), "SENSORFEHLER");
    if (asyncMqttClient.publish(topic.c_str(), 1, true, message) != 0) {
      published[i] = true;
      lastValid[i] = valid;
      lastTemperature[i] = temperature;
    }
  }
}

void publishBrennerSensorLock() {
  static bool published = false;
  static bool lastLocked = false;
  static unsigned long lastAttempt = 0;
  if (!asyncMqttClient.connected()) { published = false; return; }
  const bool locked = !sensorIsUsable(0);
  if (published && locked == lastLocked) return;
  if (millis() - lastAttempt < 1000UL) return;
  lastAttempt = millis();
  const String topic = String(MQTT_TEXT) + "brennersperre";
  const char* message = locked ? "BRENNER GESPERRT - Kesselsensor ungueltig" :
                                 "Kesselsensor OK - keine Sensorsperre";
  if (asyncMqttClient.publish(topic.c_str(), 1, true, message) != 0) {
    lastLocked = locked;
    published = true;
  }
}

void enforceKesselSensorLock() {
  static bool lastLocked = false;
  const bool locked = !sensorIsUsable(0);
  if (locked) {
    BrennerRelais = false;
    digitalWrite(BrennerPin, LOW);
  }
  if (locked != lastLocked) {
    Serial.println(locked ? "Brenner gesperrt: Kesselsensor ungueltig" :
                            "Kesselsensor gueltig: Brennersperre aufgehoben");
    lastLocked = locked;
  }
}

bool validSensorScratchpad(const byte* data) {
  bool allZero = true;
  bool allHigh = true;
  for (byte i = 0; i < 9; ++i) {
    allZero &= (data[i] == 0);
    allHigh &= (data[i] == 0xff);
  }
  return !allZero && !allHigh && OneWire::crc8(data, 8) == data[8];
}

void markSensorFailure(byte index) {
  if (index >= 5) return;
  if (sensorReadingValid[index] || sensorLastSuccess[index] == 0)
    Serial.printf("Sensorfehler: %s\n", sensorNames[index]);
  sensorReadingValid[index] = false;
}

bool markSensorSuccess(byte index, float temperature) {
  if (index >= 5) return false;
  if (!isfinite(temperature) || temperature < -55.0f || temperature > 125.0f) {
    if (index == 4) boilerDiagnostic = BOILER_BAD_VALUE;
    markSensorFailure(index);
    return false;
  }
  if (!sensorReadingValid[index]) Serial.printf("Sensor wieder gueltig: %s\n", sensorNames[index]);
  if (index == 4) boilerDiagnostic = BOILER_OK;
  sensorReadingValid[index] = true;
  sensorLastSuccess[index] = millis();
  return true;
}

// Bestehende Einstellungen erhalten; uninitialisierte Werte einzeln reparieren.
template <typename T>
bool repairSetting(int address, T fallback, double minimum, double maximum) {
  T value;
  EEPROM.get(address, value);
  if (isfinite(static_cast<double>(value)) && value >= minimum && value <= maximum) return false;
  EEPROM.put(address, fallback);
  Serial.printf("Einstellung an Speicherstelle %d auf Standardwert gesetzt\n", address);
  return true;
}

bool repairSchedule(int address, float fallback) {
  float value;
  EEPROM.get(address, value);
  if (isfinite(value) && value >= 0.0f && value < 24.0f) {
    const int hours = static_cast<int>(value);
    const int minutes = static_cast<int>(roundf((value - hours) * 100.0f));
    if (minutes >= 0 && minutes <= 59) return false;
  }
  EEPROM.put(address, fallback);
  Serial.printf("Schaltzeit an Speicherstelle %d auf Standardwert gesetzt\n", address);
  return true;
}

void initializeStoredSettings() {
  bool changed = false;
  changed |= repairSetting(EEADDRESS_BOILER, eeBoiler, 0, 100);
  changed |= repairSetting(EEADDRESS_RAUM, eeRaum, 5, 40);
  changed |= repairSetting(EEADDRESS_RAUMNACHT, eeRaumNacht, 5, 40);
  changed |= repairSetting(EEADDRESS_KESSEL, eeKessel, 0, 100);
  changed |= repairSetting(EEADDRESS_DIFFRAUM, eeDiffRaum, 0, 20);
  changed |= repairSetting(EEADDRESS_DIFFKESSEL, eeDiffKessel, 0, 100);
  changed |= repairSetting(EEADDRESS_DIFFBOILER, eeDiffBoiler, 0, 100);
  changed |= repairSetting(EEADDRESS_TVMAX, eetvmax, 0, 100);
  changed |= repairSetting(EEADDRESS_TAUMIN, eetaumin, 0, 50);
  changed |= repairSetting(EEADDRESS_NN, een, 0.1, 10);
  changed |= repairSetting(EEADDRESS_AUSSENTEMPREGELUNG, eeAuTempRegel, 0, 1);
  changed |= repairSetting(EEADDRESS_WINTER, eeWinter, 0, 1);
  changed |= repairSetting(EEADDRESS_BOILER_SOMMERBETRIEB, eeBoilerBetrieb, 0, 1);
  changed |= repairSetting(EEADDRESS_NUR_HEIZUNG, eeNurHeizung, 0, 1);
  changed |= repairSetting(EEADDRESS_SOMMERZEIT_EINAUS, eeSommerzeit_EinAus, 0, 1);
  changed |= repairSetting(EEADDRESS_BR_LAUFZEIT, eeBrennerLaufzeit, 0, 2147483647);
  changed |= repairSchedule(EEADDRESS_TAG, eeTag);
  changed |= repairSchedule(EEADDRESS_NACHT, eeNacht);
  if (changed && !EEPROM.commit()) Serial.println("Einstellungen konnten nicht gespeichert werden");
}


struct PendingMqttMessage {
  char topic[128];
  char payload[64];
  AsyncMqttClientMessageProperties properties;
  size_t length;
  unsigned long receivedAt;
};
constexpr unsigned MQTT_QUEUE_LENGTH = 8;
QueueHandle_t mqttMessageQueue = nullptr;

void setup() {
  mqttMessageQueue = xQueueCreate(MQTT_QUEUE_LENGTH, sizeof(PendingMqttMessage));

  Serial.begin(BAUD_RATE);
  esp_task_wdt_init(WDT_TIMEOUT, true); //disable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch

  pinMode(BrennerPin, OUTPUT);
  pinMode(HeizungPin, OUTPUT);
  pinMode(BoilerPin, OUTPUT);
  pinMode(MischerAuf_Pin, OUTPUT);
  pinMode(MischerZu_Pin, OUTPUT);

  if (!EEPROM.begin(EE_SIZE)) {
    Serial.println("Einstellungsspeicher konnte nicht gestartet werden");
    while (true) { delay(1000); }
  }
  initializeStoredSettings();
  gasStorageReady = gasStorage.begin("gas-meter", false);
  if (gasStorageReady) {
    gasDayBase = gasStorage.getDouble("base", NAN);
    gasDayKey = gasStorage.getInt("day", 0);
  }


//float tVorlauf,tAussen,tKessel,tKesselDest,tKesselDiff,tBoiler,tBoilerDest,tBoilerDiff,tRoom,tRoomTag,tRoomDiff;
  EEPROM.get( EEADDRESS_BOILER, tBoilerDest );
  Serial.println( tBoilerDest, 1 );
  Serial.print("...\n");
  EEPROM.get( EEADDRESS_RAUM, tRoomTag );
  Serial.println( tRoomTag, 1 );
  Serial.print("...\n");
  EEPROM.get( EEADDRESS_RAUMNACHT, tRoomNacht );
  Serial.println( tRoomNacht, 1 );
  Serial.print("...\n");
  EEPROM.get( EEADDRESS_KESSEL, tKesselDest );
  Serial.println( tKesselDest, 1 );
  Serial.print("...\n");
  EEPROM.get( EEADDRESS_DIFFRAUM, tRoomDiff );
  Serial.println( tRoomDiff, 1 );
  Serial.print("...\n");
  EEPROM.get( EEADDRESS_DIFFKESSEL, tKesselDiff );
  Serial.println( tKesselDiff, 1 );
  Serial.print("...\n");
  EEPROM.get( EEADDRESS_DIFFBOILER, tBoilerDiff );
  Serial.println( tBoilerDiff, 1 );
  Serial.print("...\n");
  EEPROM.get( EEADDRESS_TVMAX, tvmax );
  Serial.println( tvmax, 1 );
  Serial.print("...\n");
  EEPROM.get( EEADDRESS_TAUMIN, taumin );
  taumin*=-1.0;
  Serial.println( taumin, 1 );
  Serial.print("...\n");
  EEPROM.get( EEADDRESS_NN, n );
  Serial.println( n, 1 );
  Serial.print("...\n");
  EEPROM.get( EEADDRESS_AUSSENTEMPREGELUNG, AussentemperaturRegelung);
  Serial.println( AussentemperaturRegelung, 1 );
  Serial.print("...\n");

  EEPROM.get( EEADDRESS_KACHELOFEN_EIN, kachelofenEinTemp );
  EEPROM.get( EEADDRESS_KACHELOFEN_AUS, kachelofenAusTemp );
  // Neue EEPROM-Felder koennen beim ersten Start uninitialisiert sein.
  if(!isfinite(kachelofenEinTemp) || !isfinite(kachelofenAusTemp) ||
     kachelofenEinTemp < 10.0 || kachelofenEinTemp > 150.0 ||
     kachelofenAusTemp < 0.0 || kachelofenAusTemp > 140.0 ||
     kachelofenAusTemp >= kachelofenEinTemp){
    kachelofenEinTemp = 50.0;
    kachelofenAusTemp = 40.0;
    EEPROM.put( EEADDRESS_KACHELOFEN_EIN, kachelofenEinTemp );
    EEPROM.put( EEADDRESS_KACHELOFEN_AUS, kachelofenAusTemp );
    EEPROM.commit();
  }
  Serial.print("Kachelofen EIN: ");
  Serial.println(kachelofenEinTemp, 1);
  Serial.print("Kachelofen AUS: ");
  Serial.println(kachelofenAusTemp, 1);
  EEPROM.get( EEADDRESS_TAG, TagBegin );
  TagBeginHr = (int)(TagBegin);
  TagBeginMi = static_cast<int>(roundf((TagBegin - TagBeginHr)*100.0f));
  Serial.println( TagBegin, 2 );
  Serial.print("Tag - \n");
  Serial.println( TagBeginHr );
  Serial.print(":");
  Serial.println( TagBeginMi );
  Serial.print("...\n");
  EEPROM.get( EEADDRESS_NACHT, NachtBegin );
  NachtBeginHr = (int)(NachtBegin);
  NachtBeginMi = static_cast<int>(roundf((NachtBegin - NachtBeginHr)*100.0f));
  Serial.println( TagBegin, 2 );
  Serial.print("Nacht - \n");
  Serial.println( NachtBeginHr );
  Serial.print(":");
  Serial.println( NachtBeginMi );
  Serial.print("...\n");
  EEPROM.get( EEADDRESS_BOILER_SOMMERBETRIEB, BoilerBetrieb );
  Serial.println(BoilerBetrieb);
  Serial.print("-BoilerBetrieb\n");
  EEPROM.get( EEADDRESS_NUR_HEIZUNG, NurHeizung );
  Serial.println(NurHeizung);
  Serial.print("-NurHeizung\n");
  EEPROM.get( EEADDRESS_WINTER, WinterBetrieb );
  Serial.println(WinterBetrieb);
  Serial.print("-WinterBetrieb\n");
  EEPROM.get( EEADDRESS_BR_LAUFZEIT, BrennerLaufzeit );
  Serial.println(BrennerLaufzeit);
  Serial.print("-BrennerLaufzeit\n");
  EEPROM.get( EEADDRESS_SOMMERZEIT_EINAUS, Sommerzeit_EinAus );
  Serial.println(Sommerzeit_EinAus);
  Serial.print("-Sommerzeit_EinAus");
  Wire.begin();
  auto hardwarePresent = [](uint8_t address) {
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
  };
  lcdPresent = hardwarePresent(lcd_addr);
  keypadPresent = hardwarePresent(keypad_addr);
  ioExtenderPresent = hardwarePresent(ioextender0_addr);
  Serial.printf("Zusatzhardware: Display %s, Tastatur %s, Erweiterung %s\n",
                lcdPresent ? "vorhanden" : "fehlt", keypadPresent ? "vorhanden" : "fehlt",
                ioExtenderPresent ? "vorhanden" : "fehlt");
  //I2C ESP32 -> SDA (default is GPIO 21), SCL (default is GPIO 22)
// i2c ioextender
  ioextender0_indicate = 0b11111111;
  I2C_IO_Init(ioextender0_addr,ioextender0_indicate);
//
  lcd.begin();
  lcd.setBacklight(HIGH);
  //MenueBooting();
  if (keypadPresent) I2C_Keypad.begin();
  if (lcdPresent || keypadPresent || ioExtenderPresent) delay(1000);
   //For each tture sensor: Do a pinMode and a digitalWrite
   for (bLoopCounter = 0;
     bLoopCounter <= kTtureSensorMaxIndex;
     bLoopCounter++)
   {
      pinMode(tture[bLoopCounter], INPUT);
      digitalWrite(tture[bLoopCounter], LOW);//Disable internal pull-up.
   }
   //pinMode(pLED,OUTPUT);//Just so it can "pulse" to show Arduino
   // is working
   delay(300);//Wait for newly restarted system to stabilize
   Serial.println(sPrgmName);
   Serial.println(sVers);
   Serial.println();
   Serial.println("See http://sheepdogguides.com/arduino/ar3ne1tt2.htm");
   Serial.println("Temperature measurement, Multiple Dallas DSxxxx sensors");
   Serial.println("The 'S' value at the start of each line identifies the sensor reading was from.");
   Serial.println("*************************************************");
   Serial.println("Die MQTT Variablen\n");
   String bez = MQTT_TEXT;
   Serial.println(bez);
   Serial.println("Per Serial den Arbeitsmodus umschalten:");
   Serial.println("a ->AutomatikBetrieb");
   Serial.println("b ->BoilerBetrieb");
   Serial.println("h ->NurHeizungsBetrieb");
   Serial.println("r ->Aussen-/Raum-Regelung");
   Serial.println("s ->AUS");
   Serial.println("*************************************************");
   Serial.println("\n\n\n");
   // Configures static IP address
  // Production address is assigned by the router via DHCP.
   Serial.print("Connecting to :");
   Serial.println(ssid);
   WiFi.onEvent(WiFiEvent);
   WiFi.mode(WIFI_STA);
   WiFi.begin(ssid, password);
   configTime(0, 0,
           NTP_SERVER1,
           NTP_SERVER2,
           NTP_SERVER3);
   Serial.println("");
   Serial.println("WiFi connected");
   Serial.println("IP address: ");
   Serial.println(WiFi.localIP());
   lcd.clear();

   if (WiFi.status()) {
     wifi_rssi = WiFi.RSSI();
     ntpupdate();
     Schaltuhr();
     lcd.setCursor(1, 0);
     lcd.print(WiFi.localIP());
     Serial.println(WiFi.localIP());
     //onMqttConnect();
     esp_task_wdt_reset(); //watchdog Zeit rücksetzen
     Serial.print("connecting to MQTT broker...\n");
   }
   
   Serial.println("Reading from EEPROM\n");
   Serial.println(EEPROM.read(EEADDRESS_KESSEL));
   asyncMqttClient.onConnect(onMqttConnect);
   asyncMqttClient.onDisconnect(onMqttDisconnect); 
   asyncMqttClient.onSubscribe(onMqttSubscribe);
   asyncMqttClient.setCredentials(mqttUser, mqttPassword);
   asyncMqttClient.setClientId(mqttClientID);
   asyncMqttClient.onMessage(onMqttMessage);
   asyncMqttClient.setServer(MQTT_HOST,MQTT_PORT);
   

   timer0_time.start();
   timer1_2lcd.start();
   timer2_ntp.start();
   timer3_mqttupdate.start();
  serial_clear_screen();
  initWebSocket();
  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
  String page = FPSTR(index_html);
  const char* placeholders[] = {
    "TimeString", "MQTTUPDATE", "HEIZUNGSPUMPE", "BOILERPUMPE",
    "TROOM", "TAUSSEN", "TKESSEL", "TVORLAUF", "TBOILER",
    "TKACHELOFEN", "KACHELOFENSTATUS", "REGELUNGSART",
    "KACHELOFENEIN", "KACHELOFENAUS", "WIFISSID", "WIFIRSSI", "BRENNERSPERRE", "WEBTIME", "MODEAUTO", "MODERAUM", "MODEAUSSEN", "GASDAY", "GASTOTAL", "BETRIEBSART", "BETRIEBAUTO", "BETRIEBBOILER", "BETRIEBHEIZUNG", "BETRIEBAUS", "RAUMTAG", "RAUMNACHT"
  };
  for (const char* name : placeholders) {
    page.replace(String("%") + name + "%", processor(String(name)));
  }
  request->send(200, "text/html; charset=utf-8", page);
});
  server.on("/gas", HTTP_GET, [](AsyncWebServerRequest *request){
    const bool daily = request->hasParam("daily");
    request->send(200, "text/plain; charset=utf-8", gasDisplay(daily));
  });
  server.on("/raumtemperaturen", HTTP_POST, [](AsyncWebServerRequest *request){
    if (!request->hasParam("tag", true) || !request->hasParam("nacht", true)) {
      request->send(400, "text/plain; charset=utf-8", "Tag und Nacht erforderlich"); return;
    }
    const String dayText = request->getParam("tag", true)->value();
    const String nightText = request->getParam("nacht", true)->value();
    char *dayEnd = nullptr, *nightEnd = nullptr;
    const float day = strtof(dayText.c_str(), &dayEnd), night = strtof(nightText.c_str(), &nightEnd);
    if (dayText.length() > 16 || nightText.length() > 16 || dayEnd == dayText.c_str() || *dayEnd != '\0' ||
        nightEnd == nightText.c_str() || *nightEnd != '\0' || !isfinite(day) || !isfinite(night) ||
        day < 5 || day > 40 || night < 5 || night > 40) {
      request->send(400, "text/plain; charset=utf-8", "Ungueltige Wunschtemperaturen (5 bis 40 Grad)"); return;
    }
    portENTER_CRITICAL(&roomSetpointMux);
    requestedRoomSetpoints = {true, day, night};
    portEXIT_CRITICAL(&roomSetpointMux);
    request->send(202, "text/plain; charset=utf-8", "Uebernahme vorgemerkt");
  });
  server.on("/ausgangsstatus", HTTP_GET, [](AsyncWebServerRequest *request){
    char result[32];
    snprintf(result, sizeof(result), "[%d,%d,%d,%d,%d]",
      digitalRead(BrennerPin) == HIGH, digitalRead(BoilerPin) == HIGH,
      digitalRead(HeizungPin) == HIGH, digitalRead(MischerAuf_Pin) == HIGH,
      digitalRead(MischerZu_Pin) == HIGH);
    AsyncWebServerResponse* response = request->beginResponse(200, "application/json", result);
    response->addHeader("Cache-Control", "no-store");
    request->send(response);
  });
  server.on("/ntp-abweichung", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain; charset=utf-8", ntpProbeDisplay());
  });
  server.on("/betriebsart", HTTP_POST, [](AsyncWebServerRequest *request){
    if (!request->hasParam("modus", true)) {
      request->send(400, "text/plain", "Betriebsart fehlt"); return;
    }
    const String value = request->getParam("modus", true)->value();
    const int mode = value == "auto" ? 0 : value == "boiler" ? 1 : value == "heizung" ? 2 : value == "aus" ? 3 : -1;
    if (mode < 0) { request->send(400, "text/plain", "Ungueltige Betriebsart"); return; }
    portENTER_CRITICAL(&operatingModeMux);
    requestedOperatingMode = mode;
    portEXIT_CRITICAL(&operatingModeMux);
    request->redirect("/");
  });
  server.on("/betriebsart", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain; charset=utf-8", processor("BETRIEBSART"));
  });
  server.on("/regelung", HTTP_POST, [](AsyncWebServerRequest *request){
    if (!request->hasParam("modus", true)) {
      request->send(400, "text/plain", "Regelungsart fehlt"); return;
    }
    const String mode = request->getParam("modus", true)->value();
    if (mode == "auto") requestedWebMode = REGELUNG_AUTO;
    else if (mode == "raum") requestedWebMode = REGELUNG_RAUM;
    else if (mode == "aussen") requestedWebMode = REGELUNG_AUSSEN;
    else { request->send(400, "text/plain", "Ungueltige Regelungsart"); return; }
    request->redirect("/");
  });
  server.on("/regelungsart", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain; charset=utf-8", processor("REGELUNGSART"));
  });
  server.on("/uhrzeit", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain; charset=utf-8", processor("WEBTIME"));
  });
  server.on("/boilerdiagnose", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain; charset=utf-8", boilerDiagnosticDetails());
  });
  server.on("/brennersperre", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain; charset=utf-8", processor("BRENNERSPERRE"));
  });
  server.on("/tkachelofen", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", readTemperature(tKachelofen));
  });
  server.on("/kachelofen", HTTP_GET, [](AsyncWebServerRequest *request){
    if(!request->hasParam("ein") || !request->hasParam("aus")){
      request->send(400, "text/plain", "Parameter ein und aus erforderlich");
      return;
    }
    String einText = request->getParam("ein")->value();
    String ausText = request->getParam("aus")->value();
    char* einEnd = nullptr;
    char* ausEnd = nullptr;
    float newEin = strtof(einText.c_str(), &einEnd);
    float newAus = strtof(ausText.c_str(), &ausEnd);
    if(einEnd == einText.c_str() || *einEnd != '\0' ||
       ausEnd == ausText.c_str() || *ausEnd != '\0' ||
       !isfinite(newEin) || !isfinite(newAus) ||
       newEin < 10.0 || newEin > 150.0 ||
       newAus < 0.0 || newAus > 140.0 ||
       newAus >= newEin){
      request->send(400, "text/plain", "Ungueltige Kachelofen-Schaltschwellen");
      return;
    }
    kachelofenEinTemp = newEin;
    kachelofenAusTemp = newAus;
    EEPROM.put( EEADDRESS_KACHELOFEN_EIN, kachelofenEinTemp );
    EEPROM.put( EEADDRESS_KACHELOFEN_AUS, kachelofenAusTemp );
    EEPROM.commit();
    updateKachelofenStatus();
    request->redirect("/");
  });
  // Start ElegantOTA
  ElegantOTA.begin(&server);
  server.begin();
  esp_task_wdt_reset(); //watchdog Zeit rücksetzen
  serial_clear_screen();
}//end of setup()


/* *******************************************************************************************************
                                         MAIN LOOP
******************************************************************************************************* */
void loop(){
  static bool ntpProbeStarted = false;
  if (!ntpProbeStarted) {
    ntpProbeStarted = xTaskCreate(ntpProbeTask, "ntp-probe", 4096, nullptr, 1, nullptr) == pdPASS;
  }
  ElegantOTA.loop();
  processMqttMessages();
  processGasReading();
  portENTER_CRITICAL(&operatingModeMux);
  const int operatingMode = requestedOperatingMode;
  requestedOperatingMode = -1;
  portEXIT_CRITICAL(&operatingModeMux);
  if (operatingMode >= 0 && operatingMode <= 3) {
    WinterBetrieb = operatingMode == 0;
    BoilerBetrieb = operatingMode == 1;
    NurHeizung = operatingMode == 2;
    Betriebsart = operatingMode == 0 ? 'A' : operatingMode == 1 ? 'B' : operatingMode == 2 ? 'H' : '0';
    EEPROM.put(EEADDRESS_WINTER, WinterBetrieb);
    EEPROM.put(EEADDRESS_BOILER_SOMMERBETRIEB, BoilerBetrieb);
    EEPROM.put(EEADDRESS_NUR_HEIZUNG, NurHeizung);
    EEPROM.commit();
  }
  portENTER_CRITICAL(&roomSetpointMux);
  const RoomSetpointRequest roomRequest = requestedRoomSetpoints;
  requestedRoomSetpoints.pending = false;
  portEXIT_CRITICAL(&roomSetpointMux);
  if (roomRequest.pending) {
    tRoomTag = roomRequest.day; tRoomNacht = roomRequest.night;
    EEPROM.put(EEADDRESS_RAUM, tRoomTag);
    EEPROM.put(EEADDRESS_RAUMNACHT, tRoomNacht);
    if (!EEPROM.commit()) Serial.println("Wunschtemperaturen: Speichern fehlgeschlagen");
  }
  const int webMode = requestedWebMode;
  if (webMode >= 0) {
    requestedWebMode = -1;
    regelungsModus = static_cast<RegelungsModus>(webMode);
    updateKachelofenStatus();
  }
  enforceKesselSensorLock();
  enforceVorlaufSensorLock();
  publishBrennerSensorLock();
  publishSensorStates();
  static unsigned long lastWsCleanup = 0;
    const unsigned long wsNow = millis();
    if (wsNow - lastWsCleanup >= 1000UL) {
      lastWsCleanup = wsNow;
      ws.cleanupClients();
    }
  unsigned long currentTime;
  updateKachelofenStatus();
  SetOutPin(); // Enforce pump fault deadlines on every loop pass.
  MischerInit();
  if(WinterBetrieb){
    Automatik();
    Betriebsart='A';
  }else if(BoilerBetrieb){
    Boilerbetrieb();
    Betriebsart='B';
  }else if (NurHeizung) {
    Betriebsart='H';
    Heizungsbetrieb();
  }else {
    //HeizungAus
    Betriebsart='0';
    kein_Betrieb();
  }
  if(connect2wifi){
    connect2wifi=false;
    WiFi.disconnect();
    WiFi.begin(ssid,password);
  }
  if(connect2mqtt){
    connect2mqtt=false;
    Serial.println("MQTT-Verbindungsversuch");
    asyncMqttClient.connect();
  }
  if(set2myuhr){
    set2myuhr=false;
    myuhr();
  }
  if(mqtt2update){
    mqtt2update=false;
      mqttupdate();
  }
  esp_task_wdt_reset(); //watchdog Zeit rücksetzen
  uptime();//um zu wissen, wie lange der Arduino durchläuft
  KeyPad();
  Serial_Read();
  currentTime = millis();
  esp_task_wdt_reset(); //watchdog Zeit rücksetzen
  if(readSensor == false){
    //ds_alt    readTturePt1(tture[bWhichSensor]);//N.B.: Values passed back in globals
    if (bWhichSensor==BOILER_NUMBER){readTturePt1(tture[bWhichSensor]);}
    else{
      if(setup_sensorDS1820){sensorDS1820_indicateChip(bWhichSensor); setup_sensorDS1820--;}//Setup nur einmal beim Start ausführen
      sensorDS1820_reset(bWhichSensor);
   } //ds_neu
    // Conversion time starts when the command has actually been sent.
    previousTime = millis();
    currentTime = previousTime;
    readSensor = true;
  }
  if (readSensor && (unsigned long)(currentTime - previousTime) > ANSWER_TIME){ //delay 1900
    if (lcd_display_light){
      lcd.setBacklight(HIGH);
      lcd_display_light--;
    }else{
       lcd.setBacklight(LOW);
    }
    previousTime = currentTime;
//ds_alt      readTturePt2(tture[bWhichSensor],bWhichSensor);//N.B.: Values passed back in globals
//ds_alt      printTture();//N.B.: Takes values from globals.
if (bWhichSensor==BOILER_NUMBER){
readTturePt2(tture[bWhichSensor],bWhichSensor);
}
else {sensorDS1820_read(bWhichSensor);} //ds_neu
      delay(5);//war 50
      bWhichSensor++;
      readSensor = false;
      if (wait_for_connect>0){
        wait_for_connect--;
        if (!MenuPage) {
          Serial.printf("Zeit: %s\n", TimeString);
        }
      }
      if (jumptoDefault){
        if (jumptoDefault==1){
          memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
          jumptoDefault=0;
          MenuPage=0;
          if (jump==1)jump=0;
          MenueDefault();
        }else{
          jumptoDefault--;
        }
      }
      switch (Betriebsart) {
        case 'A':
          Automatik();
          break;
        case 'B':
          Boilerbetrieb();
          break;
        case 'H':
          Heizungsbetrieb();
          break;
        case '0':
          // Anlage ausschalten
          kein_Betrieb();
          //Serial.print("Anlage ist ausgeschaltet\n");
          break;
      }
    }
   if(bWhichSensor == (kTtureSensorMaxIndex+1)){
      bWhichSensor = 0;
      if(!MenuPage)Serial.print("\n");//Start new line
   }
   if (currentTime - previousTime_wifi > WIFI_RECON_TIMER && wifiRecon){
      previousTime_wifi = currentTime;
      wifiRecon=false;
      mqttRecon=true;
      connectToWifi();
   }
   if (currentTime - previousTime_mqtt > MQTT_RECON_TIMER && mqttRecon){
      previousTime_mqtt = currentTime;
      mqttRecon=false;
      connectToMqtt();
   }
   if (currentTime - previousTime1 > NTP_UPDATE){
     previousTime1 = currentTime;
     if(!MenuPage)print_Uptime();
   }
   if (myhours==PUMPENL_HR && myminutes >= PUMPENL_MIN_B && myminutes <=PUMPENL_MIN_E){
     Pumpenloesen=1;
   }else{
     Pumpenloesen=0;
   }
   timer0_time.update();
   timer1_2lcd.update();
   timer2_ntp.update();
   timer3_mqttupdate.update();
  esp_task_wdt_reset(); //watchdog Zeit wieder rücksetzen
}//end of loop()






/* *******************************************************************************************************
                                         KeyPad
******************************************************************************************************* */
void KeyPad(){
  if (!keypadPresent) return;
  // Gedrückte Taste abfragen
    char i2cKey = I2C_Keypad.getKey();
    if (i2cKey) {
      lcd_display_light=30;
      lcd.setBacklight(HIGH);
      if (i2cKey == 'A') { // Menue verlassen -> zurueck zur Standard-Menue-Seite 0
        memset(keyBuffer, 0, sizeof keyBuffer);//Alle Eingaben werden verworfen
        MenuPage=0;
        data_ready=false;
        data_count=0;
        MenueMain();
      }else if (i2cKey == 'B' && MenuPage==0) { // Menue verlassen -> zurueck zur Standard-Menue-Seite 0
        memset(keyBuffer, 0, sizeof keyBuffer);//Alle Eingaben werden verworfen
        MenuPage=0;
        data_ready=false;
        data_count=0;
        Regelungs_Switch_AI();
      }else if (i2cKey == 'D' || i2cKey == 'C' || i2cKey == '*') { //C oder D zum Scrollen im Menue
        memset(keyBuffer, 0, sizeof keyBuffer);//Alle Eingaben werden verworfen
        data_ready=false;
        data_count=0;
        if (i2cKey == 'D'){ if(MenuPage < MENUPAGE_MAX) MenuPage++;}
        if (i2cKey == 'C'){ if(MenuPage) MenuPage--;}
        if (i2cKey == '*'){
          ntpupdate();
          Schaltuhr();
          MenuPage=12;
        }
        switch (MenuPage)
        {
            case 0:
              MenueMain();
              break;
            case 1:
              MenuRoomTemp();
              break;
            case 2:
              MenuBoilerTemp();
              break;
            case 3:
              MenuKesseltemp();
              break;
            case 4:
              MenuDifRaumtemp();
              break;
            case 5:
              MenuDifKesseltemp();
              break;
            case 6:
              MenuRaumTempNacht();
              break;
            case 7:
              MenuBetriebWinter();
              break;
            case 8:
              MenuBetriebBoiler();
              break;
            case 9:
              MenuBetriebNurHeizung();
              break;
            case 10:
              MenuHeizbeginnTag();
              break;
            case 11:
              MenuHeizEndeNacht();
              break;
            case 12:
              Time2LCD();
              break;
            case 13:
              MenuSommerzeitEinAus();
              break;
            case 14:
              MenuDifBoilerTemp();
              break;
            case 15:
              Menu_tvmax();
              break;
            case 16:
              Menu_taumin();
              break;
            case 17:
              Menu_n();
                break;
            case 18:
              MenuAussentempRegelung();
                break;
        }
      }
      // Check, ob ASCII Wert des Char einer Ziffer zwischen 0 und 9 entspricht
      else if ((int(i2cKey) >= 48) && (int(i2cKey) <= 57) && MenuPage > 0){ //Nummerntasten wurden gedrückt
        addToKeyBuffer(i2cKey);
        lcd.setCursor(0, 3);
        lcd.print(keyBuffer);//schreibe den eingegebenen Text ans Display
        if(data_ready==true){lcd.print(" OK");}
      }else if ((i2cKey == '#') && (data_ready == true) && MenuPage > 0) { //# ist die Enter-Taste
        checkEingabe();
        data_ready=false;
        data_count=0;
        memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
      }
    }
}
/* *******************************************************************************************************
                                         KeyPad-Buffer
******************************************************************************************************* */
void addToKeyBuffer(char inkey) {
  switch (MenuPage) {
      case MENUPAGE_TEMPERATUR:
      case MENUPAGE_TEMPERATUR1:
      if (data_count < KEYSLENGTH_T && data_ready==false){
        keyBuffer[data_count] = inkey;
        data_count++;
        if(data_count == KEYSLENGTH_T) {
          data_ready=true;
        }
        if(data_count==2){
          keyBuffer[data_count]='.';
          data_count++;
        }
      }
      break;
    case MENUPAGE_NUM:
      if (data_count < KEYSLENGTH_N && data_ready==false){
        keyBuffer[data_count] = inkey;
        data_count++;
        data_ready=true;
      }
      break;
    case MENUPAGE_TIME:
      if (data_count < KEYSLENGTH_Z && data_ready==false){
        if (data_count==3 && (int(inkey) > 53)){inkey='5';} //neu Minuten max 50
        keyBuffer[data_count] = inkey;
        data_count++;
        if(data_count == KEYSLENGTH_Z) {
          data_ready=true;
        }
        if(data_count==2){
          keyBuffer[data_count]='.';
          data_count++;
        }
      }
      break;
      case MENUPAGE_SZ:
        if (data_count < KEYSLENGTH_N && data_ready==false){
          keyBuffer[data_count] = inkey;
          data_count++;
          data_ready=true;
        }
        break;
    case MENUPAGE_NUM_AT:
      if (data_count < KEYSLENGTH_N && data_ready==false){
        keyBuffer[data_count] = inkey;
        data_count++;
        data_ready=true;
      }
      break;
  }
}
/* *******************************************************************************************************
                                         KeyPad-checkEingabe
******************************************************************************************************* */
void checkEingabe() {
  if ((MenuPage >= 1 && MenuPage <= 6) || (MenuPage >= 14 && MenuPage <= 17)) {//case MENUPAGE_TEMPERATUR:
    lcd.print('#');
    //  if(sizeof(keyBuffer)==(KEYSLENGTH_T+1)){
        float f_char;
        f_char = atof(keyBuffer);
        Serial.println(f_char);
        Serial.print("*C");
        switch (MenuPage) {
          case 1:
            tRoomTag=f_char;
            EEPROM.put(EEADDRESS_RAUM, f_char);
            lcd.print(" ->");
            lcd.print(f_char);
            break;
          case 2:
            tBoilerDest=f_char;
            EEPROM.put(EEADDRESS_BOILER, f_char);
            lcd.print(" ->");
            lcd.print(f_char);
            break;
          case 3:
            tKesselDest=f_char;
            EEPROM.put(EEADDRESS_KESSEL, f_char);
            lcd.print(" ->");
            lcd.print(f_char);
            break;
          case 4:
            tRoomDiff=f_char;
            EEPROM.put(EEADDRESS_DIFFRAUM, f_char);
            lcd.print(" ->");
            lcd.print(f_char);
            break;
          case 5:
            tKesselDiff=f_char;
            EEPROM.put(EEADDRESS_DIFFKESSEL, f_char);
            lcd.print(" ->");
            lcd.print(f_char);
            break;
          case 6:
            tRoomNacht=f_char;
            EEPROM.put(EEADDRESS_RAUMNACHT, f_char);
            lcd.print(" ->");
            lcd.print(f_char);
            break;
          case 14:
            tBoilerDiff=f_char;
            EEPROM.put(EEADDRESS_DIFFBOILER, f_char);
            lcd.print(" ->");
            lcd.print(f_char);
            break;
          case 15:
            tvmax=f_char;
            EEPROM.put(EEADDRESS_TVMAX, f_char);
            lcd.print(" ->");
            lcd.print(f_char);
            break;
          case 16:
            taumin=f_char*-1.0;
            EEPROM.put(EEADDRESS_TAUMIN, f_char);
            lcd.print(" -> -");
            lcd.print(f_char);
            break;
          case 17:
            n=f_char;
            EEPROM.put(EEADDRESS_NN, f_char);
            lcd.print(" ->");
            lcd.print(f_char);
            break;
        }
        EEPROM.commit();
      //}
    }
    if ((MenuPage >= 7 && MenuPage <= 9) || MenuPage == 13 || MenuPage == 18) // case MENUPAGE_NUM:
    {
    //if(sizeof(keyBuffer)==(KEYSLENGTH_N+1)){
      int i_char;
      i_char = atoi(keyBuffer);
      if(i_char>1){i_char=1;}
      switch (MenuPage) {
        case 7:
          WinterBetrieb=i_char;
          EEPROM.put(EEADDRESS_WINTER, i_char);
          lcd.print(" ->");
          lcd.print(i_char);
          break;
        case 8:
          BoilerBetrieb=i_char;
          EEPROM.put(EEADDRESS_BOILER_SOMMERBETRIEB, i_char);
          lcd.print(" ->");
          lcd.print(i_char);
          break;
        case 9:
          NurHeizung=i_char;
          EEPROM.put(EEADDRESS_NUR_HEIZUNG, i_char);
          lcd.print(" ->");
          lcd.print(i_char);
          break;
        case 13:
          Sommerzeit_EinAus = (i_char != 0);
          EEPROM.put(EEADDRESS_SOMMERZEIT_EINAUS, Sommerzeit_EinAus);
          ntpupdate();
          Schaltuhr();
          lcd.print(" ->");
          lcd.print(Sommerzeit_EinAus);
          break;
        case 18:
          EEPROM.put(EEADDRESS_AUSSENTEMPREGELUNG, i_char);
          AussentemperaturRegelung=i_char;
          lcd.print(" ->");
          lcd.print(i_char);
          break;
        }
        EEPROM.commit();
      //}
    }
      if (MenuPage >= 10 && MenuPage <= 11){ //case MENUPAGE_TIME:
    //  if(sizeof(keyBuffer)==KEYSLENGTH_Z+1){
        float f_char;
        f_char = atof(keyBuffer);
        if(f_char<24){
        Serial.print(f_char);
        switch (MenuPage) {
          case 10:
            TagBegin=f_char;
            TagBeginHr = (int)(TagBegin);
            TagBeginMi = static_cast<int>(roundf((TagBegin - TagBeginHr)*100.0f));
            EEPROM.put(EEADDRESS_TAG, f_char);
            lcd.print(" ->");
            lcd.print(f_char);
            break;
          case 11:
            NachtBegin=f_char;
            NachtBeginHr = (int)(NachtBegin);
            NachtBeginMi = static_cast<int>(roundf((NachtBegin - NachtBeginHr)*100.0f));
            EEPROM.put(EEADDRESS_NACHT, f_char);
            lcd.print(" ->");
            lcd.print(f_char);
            break;
          }
          EEPROM.commit();
          lcd.print(" OK!");
        }
        else{
          lcd.print(" Ungültig!");
        }
    //  }
    }
    jumptoDefault=3;
}
/* *******************************************************************************************************
                                         MenüDisplay
******************************************************************************************************* */
void MenueMain() {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("0 >MainMenue");
    //lcd.setCursor(0,1);
    jumptoDefault=60;
}
void MenueDefault(){
    lcd.clear();
}
void MenueBooting(){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("System startet");
    Serial.print("System startet\n");
    lcd.setCursor(0,1);
    for (int i = 0; i <= 100; i++){  // you can change the increment value here
      lcd.setCursor(8,1);
      if (i<=100) {
        lcd.print(" ");
        //print a space if the percentage is < 100
        Serial.print(" ");
      }
      if (i<10) {
        lcd.print(" ");  //print a space if the percentage is < 10
        Serial.print(" ");
      }
      lcd.print(i);
      serial_clear_screen();
      Serial.print(i);
      lcd.print("%");
      Serial.print("%");
      delay(25);  //change the delay to change how fast the boot up screen changes
    }
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(sVers); // Start Print text to Line 1
    lcd.setCursor(0, 1);
    lcd.print(sPrgmName); // Start Print Test to Line 2
    jumptoDefault=60;
}
void MenuRoomTemp() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("1 >RoomTemp");
  lcd.setCursor(0,1);
  lcd.print("2  BoilerTemp");
  lcd.setCursor(0,2);
  char float_str[32];
  char line0[21];
  snprintf(float_str, sizeof(float_str), "%4.2f", static_cast<double>(tRoomTag));
  snprintf(line0, sizeof(line0), "TempNow: %-9sC", float_str); // %6s right pads the string
  lcd.print(line0);
  memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
  jumptoDefault=60;
}
void MenuBoilerTemp() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("2 >BoilerTemp");
  lcd.setCursor(0,1);
  lcd.print("3  KesselTemp");
  lcd.setCursor(0,2);
  char float_str[32];
  char line0[21];
  snprintf(float_str, sizeof(float_str), "%4.2f", static_cast<double>(tBoilerDest));
  snprintf(line0, sizeof(line0), "TempNow: %-9sC", float_str);
  lcd.print(line0);
  memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
  jumptoDefault=60;
}
void MenuKesseltemp() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("3 >KesselTemp");
  lcd.setCursor(0,1);
  lcd.print("4  DiffRaumtemp");
  lcd.setCursor(0,2);
  char float_str[32];
  char line0[21];
  snprintf(float_str, sizeof(float_str), "%4.2f", static_cast<double>(tKesselDest));
  snprintf(line0, sizeof(line0), "TempNow: %-9sC", float_str);
  lcd.print(line0);
  memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
  jumptoDefault=60;
}
void MenuDifRaumtemp() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("4 >DiffRaumtemp");
  lcd.setCursor(0,1);
  lcd.print("5  DiffKesselTemp");
  lcd.setCursor(0,2);
  char float_str[32];
  char line0[21];
  snprintf(float_str, sizeof(float_str), "%4.2f", static_cast<double>(tRoomDiff));
  snprintf(line0, sizeof(line0), "TempNow: %-9sC", float_str);
  lcd.print(line0);
  memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
  jumptoDefault=60;
}
void MenuDifKesseltemp() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("5 >DiffKesselTemp");
  lcd.setCursor(0,1);
  lcd.print("6  RaumNachtTemp");
  lcd.setCursor(0,2);
  char float_str[32];
  char line0[21];
  snprintf(float_str, sizeof(float_str), "%4.2f", static_cast<double>(tKesselDiff));
  snprintf(line0, sizeof(line0), "TempNow: %-9sC", float_str);
  lcd.print(line0);
  memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
  jumptoDefault=60;
}
void MenuRaumTempNacht(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("6 >RaumNachtTemp");
  lcd.setCursor(0,1);
  lcd.print("7  WinterBetrieb");
  lcd.setCursor(0,2);
  char float_str[32];
  char line0[21];
  snprintf(float_str, sizeof(float_str), "%4.2f", static_cast<double>(tRoomNacht));
  snprintf(line0, sizeof(line0), "TempNow: %-9sC", float_str);
  lcd.print(line0);
  memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
  jumptoDefault=60;
}
void MenuBetriebWinter(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("7 >WinterBetrieb");
  lcd.setCursor(0,1);
  lcd.print("8  BoilerBetrieb");
  lcd.setCursor(0,2);
  char line0[21];
  snprintf(line0, sizeof(line0), "WinterBetr: %d", WinterBetrieb);
  lcd.print(line0);
  memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
  jumptoDefault=60;
}
void MenuBetriebBoiler(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("8 >BoilerBetrieb");
  lcd.setCursor(0,1);
  lcd.print("9 NurHeizungBetr");
  lcd.setCursor(0,2);
  char line0[21];
  snprintf(line0, sizeof(line0), "BoilerBetrieb: %d", BoilerBetrieb);
  lcd.print(line0);
  memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
  jumptoDefault=60;
}
void MenuBetriebNurHeizung(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("9 >NurHeizungBetr");
  lcd.setCursor(0,1);
  lcd.print("10  Tagbetrieb");
  lcd.setCursor(0,2);
  char line0[21];
  snprintf(line0, sizeof(line0), "NurHeizBetr: %d", NurHeizung);
  lcd.print(line0);
  memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
  jumptoDefault=60;
}
void MenuHeizbeginnTag(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("10 >Tagbetrieb");
  lcd.setCursor(0,1);
  lcd.print("11  Nachtbetrieb");
  lcd.setCursor(0,2);
  char float_str[32];
  char line0[21];
  snprintf(float_str, sizeof(float_str), "%4.2f", static_cast<double>(TagBegin));
  snprintf(line0, sizeof(line0), "ZeitNow: %-5s", float_str);
  lcd.print(line0);
  memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
  jumptoDefault=60;
}
void MenuHeizEndeNacht(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("11 >Nachtbetrieb");
  lcd.setCursor(0,1);
  lcd.print("12 Uptime/NTP_Sync");
  lcd.setCursor(0,2);
  char float_str[32];
  char line0[21];
  snprintf(float_str, sizeof(float_str), "%4.2f", static_cast<double>(NachtBegin));
  snprintf(line0, sizeof(line0), "ZeitNow: %-5s", float_str);
  lcd.print(line0);
  memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
  jumptoDefault=60;
}
void MenuDifBoilerTemp() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("14 >DiffBoilerTemper");
  lcd.setCursor(0,1);
  lcd.print("15  VorlaufMaxTemp");
  lcd.setCursor(0,2);
  char float_str[32];
  char line0[21];
  snprintf(float_str, sizeof(float_str), "%4.2f", static_cast<double>(tBoilerDiff));
  snprintf(line0, sizeof(line0), "TempNow: %-9sC", float_str);
  lcd.print(line0);
  memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
  jumptoDefault=60;
}

void Menu_tvmax() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("15 >VorlaufMaxTemp");
  lcd.setCursor(0,1);
  lcd.print("16  AussenMinTemp");
  lcd.setCursor(0,2);
  char float_str[32];
  char line0[21];
  snprintf(float_str, sizeof(float_str), "%4.2f", static_cast<double>(tvmax));
  snprintf(line0, sizeof(line0), "TempNow: %-9sC", float_str);
  lcd.print(line0);
  memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
  jumptoDefault=60;
}
void Menu_taumin() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("16 >AussenMinTemp");
  lcd.setCursor(0,1);
  lcd.print("17  Kurvenfaktor n");
  lcd.setCursor(0,2);
  char float_str[32];
  char line0[21];
  snprintf(float_str, sizeof(float_str), "%4.2f", static_cast<double>(taumin));
  snprintf(line0, sizeof(line0), "TempNow: %-9sC", float_str);
  lcd.print(line0);
  memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
  jumptoDefault=60;
}
void Menu_n() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("17 >Kurvenfaktor n");
  lcd.setCursor(0,1);
  lcd.print("18 >AussentempRegel");
  lcd.setCursor(0,2);
  char float_str[32];
  char line0[21];
  snprintf(float_str, sizeof(float_str), "%4.2f", static_cast<double>(n));
  snprintf(line0, sizeof(line0), "FaktorNow: %-9s", float_str);
  lcd.print(line0);
  memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
  jumptoDefault=60;
}
void MenuAussentempRegelung(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("18 >AussentempRegel");
  lcd.setCursor(0,1);
  lcd.print("                    ");
  lcd.setCursor(0,2);
  char float_str[32];
  char line0[21];
  snprintf(float_str, sizeof(float_str), "%4.2f", static_cast<double>(n));
  snprintf(line0, sizeof(line0), "Ja(1)/Nein(0): %d", AussentemperaturRegelung);
  lcd.print(line0);
  memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
  jumptoDefault=60;
}

void Time2LCD(){
  if(mischer_wait>0)mischer_wait--;
  if(mischer_drive>0)mischer_drive--;
  char line0[21];
  if (doppelp==' '){
    doppelp=':';
  }else{
    doppelp=' ';
  }

  if (MenuPage==12){
    lcd.clear();
    lcd.setCursor(0,0);
    snprintf(line0, sizeof(line0), "UP: %ld, %02d:%02d%c%02d", uDay,uHour,uMinute,doppelp,uSecond);
    lcd.print(line0);
    lcd.setCursor(0, 1);
    lcd.printf("H%ld A%c %s %d",BrennerLaufzeit,Betriebsart,mqtt_payload,mqtt_message );
    lcd.setCursor(0, 2);
    lcd.printf("UpdTime:%ld, wr:%d",timeUpdated,wifi_retry);
    lcd.setCursor(0, 3);
    lcd.print(line0);
    lcd.setCursor(8,3);
    lcd.printf("CalVT:%.2f",vorlaufTemperatur);
    if (jump == 0){
      jump=1;
      jumptoDefault=60;
    }
 }
  if(MenuPage==0){
    print_Main_LCD_Values();
  }
}
/* *******************************************************************************************************
                                         Sommerzeit EIN / AUS
******************************************************************************************************* */
void MenuSommerzeitEinAus(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("13 >SommerzeitEINAUS");
  lcd.setCursor(0,1);
  lcd.print("14 DiffBoilerTemper ");
  lcd.setCursor(0,2);
  char line0[21];
  snprintf(line0, sizeof(line0), "Sommerzeit: %d", Sommerzeit_EinAus);
  lcd.print(line0);
  memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
  jumptoDefault=10;
}
/* *******************************************************************************************************
                                         Schaltuhr
******************************************************************************************************* */
void Schaltuhr() {
  if (!ntpTimeValid) {
    return;
  }

  const long mytime = 100L * myhours + myminutes;
  const long t_begin = 100L * TagBeginHr + TagBeginMi;
  const long t_ende = 100L * NachtBeginHr + NachtBeginMi;

  // Bisherige Grenzen des Zeitprogramms beibehalten.
  daynight = (mytime > t_begin && mytime < t_ende) ? 'D' : 'N';
}
/* *******************************************************************************************************
                                         mytime update - every seconds
******************************************************************************************************* */
void setmyuhr(){
  set2myuhr=true;
}
void myuhr() {
  ntpupdate();
  Schaltuhr();

  // Vorhandene Brenner-Laufzeitzählung beibehalten.
  if (BrennerRelais) {
    brsecunds++;
  }
  if (brsecunds >= 60) {
    brminutes += brsecunds / 60;
    brsecunds %= 60;
  }
  if (brminutes >= 60) {
    brhours += brminutes / 60;
    brminutes %= 60;
  }
}
/* *******************************************************************************************************
                                         NTP update - via ticker every 24 hours
******************************************************************************************************* */
void dayticker(){
  if(dayticker_hr<=23){dayticker_hr++;
  }else{
    dayticker_hr=0;
    Schaltuhr();
  }
  mqttRecon=true;//mqtt neu starten
  previousTime_mqtt=millis();//mqtt neu starten
}
/* *******************************************************************************************************
                                         NTP update
******************************************************************************************************* */
void ntpupdate() {
  const time_t utcTime = time(nullptr);

  if (utcTime < static_cast<time_t>(1577836800UL)) {
    ntpTimeValid = false;
    snprintf(TimeString, sizeof(TimeString), "NTP nicht synchron");
    return;
  }

  if (!ntpTimeValid) {
    timeUpdated++;
  }
  ntpTimeValid = true;

  time_t localTime =
      utcTime + GMT_TIME_ZONE * utcOffsetInSeconds;
  struct tm calendar = {};
  if (gmtime_r(&localTime, &calendar) == nullptr) {
    return;
  }

  // Sommerzeit auf die Normalzeit anwenden, danach Datum neu berechnen.
  if (Sommerzeit_EinAus &&
      summertime_EU(calendar.tm_year + 1900,
                    calendar.tm_mon + 1,
                    calendar.tm_mday,
                    calendar.tm_hour,
                    GMT_TIME_ZONE)) {
    localTime += 3600;
    if (gmtime_r(&localTime, &calendar) == nullptr) {
      return;
    }
  }

  myyear = calendar.tm_year + 1900;
  mymonth = calendar.tm_mon + 1;
  myday = calendar.tm_mday;
  myweekday = calendar.tm_wday;
  myhours = calendar.tm_hour;
  myminutes = calendar.tm_min;
  mysecunds = calendar.tm_sec;

  const char dd = (mysecunds % 2) ? ' ' : ':';
  snprintf(TimeString, sizeof(TimeString),
           "%s, %02u:%02u%c%02u",
           daysOfTheWeek[myweekday],
           static_cast<unsigned>(myhours),
           static_cast<unsigned>(myminutes),
           dd,
           static_cast<unsigned>(mysecunds));

  if (!MenuPage) {
    lcd.setCursor(0, 3);
    lcd.print(TimeString);
  }
}
/* *******************************************************************************************************
                                         WiFiEvent
******************************************************************************************************* */
void WiFiEvent(WiFiEvent_t event) {
    Serial.printf("[WiFi-event] event: %d\n", event);
    switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        connectToMqtt();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("WiFi lost connection");
        wifiRecon=true;
        mqttRecon=false;
        previousTime_wifi=millis();
        break;
    case SYSTEM_EVENT_WIFI_READY: 
        break;
    case SYSTEM_EVENT_SCAN_DONE:
        break;
    case SYSTEM_EVENT_STA_START:
        break;
    case SYSTEM_EVENT_STA_STOP:
        break;
    case SYSTEM_EVENT_STA_CONNECTED:
        break;
    case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
        break;
    case SYSTEM_EVENT_STA_LOST_IP:
        wifiRecon=true;
        mqttRecon=false;
        previousTime_wifi=millis();
        break;
    case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:
        break;
    case SYSTEM_EVENT_STA_WPS_ER_FAILED:
        break;
    case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
        break;
    case SYSTEM_EVENT_STA_WPS_ER_PIN:
        break;
    case SYSTEM_EVENT_AP_START:
        break;
    case SYSTEM_EVENT_AP_STOP:
        break;
    case SYSTEM_EVENT_AP_STACONNECTED:
        break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
        break;
    case SYSTEM_EVENT_AP_STAIPASSIGNED:
        break;
    case SYSTEM_EVENT_AP_PROBEREQRECVED:
        break;
    case SYSTEM_EVENT_GOT_IP6:
        break;
    case SYSTEM_EVENT_ETH_START:
        break;
    case SYSTEM_EVENT_ETH_STOP:
        break;
    case SYSTEM_EVENT_ETH_CONNECTED:
        break;
    case SYSTEM_EVENT_ETH_DISCONNECTED:
        break;
    case SYSTEM_EVENT_ETH_GOT_IP:
        break;
    default: 
        break; 
    }
}
void connectToMqtt() {
 connect2mqtt=true; 
}
/* *******************************************************************************************************
                                         NTP update
******************************************************************************************************* */
void set2mqttupdate(){
  mqtt2update=true;
}
void mqttupdate() {
  if (!asyncMqttClient.connected()) {
    Serial.println("MQTT JSON: keine Verbindung");
    return;
  }
  String bez;
  char msg[50];
  char line0[48];
  char buffer[2048];
  JsonDocument doc;
  doc["S0Kessel"].set(tKessel);
  doc["S1Vorlauf"].set(tVorlauf);
  doc["S2Aussen"].set(tAussen);
  doc["S3Kueche"].set(tRoom);
  doc["S4Boiler"].set(tBoiler);
  doc["T0Room"].set(tmyRoomdest);
  doc["T1Boiler"].set(tBoilerDest);
  doc["T2Vorlauf"].set(vorlaufTemperatur);
  snprintf(line0, sizeof(line0), "%ld,%02d:%02d:%02d", uDay,uHour,uMinute,uSecond);
  doc["U0Uptime"].set(line0);
  snprintf(line0, sizeof(line0), "%ld:%02d:%02d", brhours,brminutes,brsecunds);
  doc["B0Brenner"].set(line0);
  snprintf(line0, sizeof(line0), "%02d:%02d", TagBeginHr,TagBeginMi);
  doc["Day"].set(line0);
  snprintf(line0, sizeof(line0), "%02d:%02d", NachtBeginHr,NachtBeginMi);
  doc["Night"].set(line0);
  snprintf(line0, sizeof(line0), "%s", BrennerRelais ? "1" : "0");
  doc["BrennerRelais"].set(line0);
  snprintf(line0, sizeof(line0), "%s", HeizungsRelais ? "1" : "0");
  doc["HeizungsRelais"].set(line0);
  snprintf(line0, sizeof(line0), "%s", BoilerRelais ? "1" : "0");
  doc["BoilerRelais"].set(line0);
  snprintf(line0, sizeof(line0), "%s", MischerAufRelais ? "1" : "0");
  doc["MischerAufRelais"].set(line0);
  snprintf(line0, sizeof(line0), "%s", MischerZuRelais ? "1" : "0");
  doc["MischerZuRelais"].set(line0);
  snprintf(line0, sizeof(line0), "%02d.%02d.%4d %02d:%02d", myday,mymonth,myyear,myhours,myminutes);
  doc["Uhrzeit"].set(line0);
  doc["WlanRetry"].set(wifi_retry);
  doc["WlanRSSI"].set(WiFi.RSSI());
  doc["atRegelung"].set(AussentemperaturRegelung);
  doc["PumpenNachlauf"].set(PumpenNachlauf);
  doc["tvmax"].set(tvmax);
  doc["taumin"].set(taumin);
  doc["n"].set(n);
  if(strcmp(mqtt_payload,"")==0){
    doc["answer"].set("---");
  }
    else{
    snprintf(line0, sizeof(line0), "%s", mqtt_payload);
    strcpy(mqtt_payload,"");
    doc["answer"].set(line0);
  }
  snprintf(line0, sizeof(line0), "%c" ,Betriebsart);
  doc["betriebsart"].set(line0);
  if (doc.overflowed() || measureJson(doc) >= sizeof(buffer)) {
    Serial.println("MQTT: Statusnachricht zu gross");
    return;
  }
  const size_t payloadSize = serializeJson(doc, buffer, sizeof(buffer));
//  serializeJsonPretty(doc, buffer);
  bez = bez + MQTT_TEXT + "JDATA";
  bez.toCharArray(msg,50);
  const uint16_t jsonPacketId = asyncMqttClient.publish(msg, 1, true, buffer, payloadSize);
  Serial.printf("MQTT JSON: %u Bytes, Versand %s\n", static_cast<unsigned>(payloadSize),
                jsonPacketId ? "eingereiht" : "abgelehnt");
}
/*
void mqttupdate() {
  String bez;
  char msg[50];
  char res[8];
  char line0[20];
    bez = bez + MQTT_TEXT + "S0" + "Kessel";
    snprintf(res, sizeof(res), "%6.2f", static_cast<double>(tKessel));
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1,true, res);
    bez="";
    bez = bez + MQTT_TEXT + "S1" + "Vorlauf";
    snprintf(res, sizeof(res), "%6.2f", static_cast<double>(tVorlauf));
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, res);
    bez="";
    bez = bez + MQTT_TEXT + "S2" + "Aussen";
    snprintf(res, sizeof(res), "%6.2f", static_cast<double>(tAussen));
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, res);
    bez="";
    bez = bez + MQTT_TEXT + "S3" + "Kueche";
    snprintf(res, sizeof(res), "%6.2f", static_cast<double>(tRoom));
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, res);
    bez="";
    bez = bez + MQTT_TEXT + "S4" + "Boiler";
    snprintf(res, sizeof(res), "%6.2f", static_cast<double>(tBoiler));
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, res);
    bez="";
    bez = bez + MQTT_TEXT + "T0" + "Room";
    snprintf(res, sizeof(res), "%6.2f", static_cast<double>(tmyRoomdest));
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, res);
    bez="";
    bez = bez + MQTT_TEXT + "U0" + "Uptime";
    snprintf(line0, sizeof(line0), "%ld,%02d:%02d:%02d", uDay,uHour,uMinute,uSecond);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "B0" + "Brenner";
    snprintf(line0, sizeof(line0), "%ld:%02d:%02d", brhours,brminutes,brsecunds);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "Day";
    snprintf(line0, sizeof(line0), "%02d:%02d", TagBeginHr,TagBeginMi);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "Night";
    snprintf(line0, sizeof(line0), "%02d:%02d", NachtBeginHr,NachtBeginMi);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "BrennerRelais";
    snprintf(line0, sizeof(line0), "%s", BrennerRelais ? "1" : "0");
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "HeizungsRelais";
    snprintf(line0, sizeof(line0), "%s", HeizungsRelais ? "1" : "0");
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "BoilerRelais";
    snprintf(line0, sizeof(line0), "%s", BoilerRelais ? "1" : "0");
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "Uhrzeit";
    snprintf(line0, sizeof(line0), "%02d.%02d.%4d %02d:%02d", myday,mymonth,myyear,myhours,myminutes);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "WlanRetryConnect";
    snprintf(line0, sizeof(line0), "%d", wifi_retry);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "WlanRSSI";
    snprintf(line0, sizeof(line0), "%d", WiFi.RSSI());
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "vorlaufTemperatur";
    snprintf(line0, sizeof(line0), "%.2f", vorlaufTemperatur);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "atRegelung";
    snprintf(line0, sizeof(line0), "%d", AussentemperaturRegelung);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
//    bez="";
//    bez = bez + MQTT_TEXT + "WlanAvgTimeMs";
//    snprintf(line0, sizeof(line0), "%d", avg_time_ms);
//    bez.toCharArray(msg,50);
//    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "PumpenNachlauf";
    snprintf(line0, sizeof(line0), "%d", PumpenNachlauf);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
//    bez="";
//    bez = bez + MQTT_TEXT + "MischerAuf";
//    snprintf(line0, sizeof(line0), "%d", MischerAufRelais);
//    bez.toCharArray(msg,50);
//    asyncMqttClient.publish(msg,1, true, line0);
//    bez="";
//    bez = bez + MQTT_TEXT + "MischerZu";
//    snprintf(line0, sizeof(line0), "%d", MischerZuRelais);
//    bez.toCharArray(msg,50);
//    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "tvmax";
    snprintf(line0, sizeof(line0), "%.2f", tvmax);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "taumin";
    snprintf(line0, sizeof(line0), "%.2f", taumin);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "n";
    snprintf(line0, sizeof(line0), "%.2f", n);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
//    bez="";
//    bez = bez + MQTT_TEXT + "dataToI2Cextender";
//    snprintf(line0, sizeof(line0), "%d", dataToI2C);
//    bez.toCharArray(msg,50);
//    asyncMqttClient.publish(msg,1, true, line0);
    if(strcmp(mqtt_payload,"")==0){
    }
    else{
      bez="";
      bez = bez + MQTT_TEXT + "answer"; //Antwort nach Subscribe Message
      snprintf(line0, sizeof(line0), "%s", mqtt_payload);
      strcpy(mqtt_payload,"");
      bez.toCharArray(msg,50);
      asyncMqttClient.publish(msg,1, true, line0);
    }
    bez="";
    bez = bez + MQTT_TEXT + "Betriebsart";
    snprintf(line0, sizeof(line0), "%c" ,Betriebsart);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    esp_task_wdt_reset(); //watchdog Zeit rücksetzen
}
*/
/* *******************************************************************************************************
                                         WLAN-MQTT Connecting
******************************************************************************************************* */
void connectToWifi() {
  //WiFi.begin(SECRET_SSID, SECRET_PASS);
  connect2wifi=true;
}

void connect() {
  if(wait_for_connect==0){
    if(WiFi.status() != WL_CONNECTED) {
      WiFi.begin(ssid, password);
     if(WiFi.status() == WL_CONNECTED) {
      if(!MenuPage){
        Serial.print("Wifi connection successful - IP-Address: ");
        Serial.println(WiFi.localIP());
      }
     }
     else {
      wait_for_connect=100;
    }
   }
  }//if waitForConnectResult
}
/* *******************************************************************************************************
                                         ds18b20
******************************************************************************************************* */
bool OneWireReset(int Pin) {
   // Preserve the long-line reset duration, sampling presence in the same
   // transaction rather than resetting once with the library and again here.
   digitalWrite(Pin, LOW);
   pinMode(Pin, OUTPUT);
   delayMicroseconds(550);
   noInterrupts();
   pinMode(Pin, INPUT);
   delayMicroseconds(70);
   const bool present = digitalRead(Pin) == LOW;
   interrupts();
   delayMicroseconds(430);
   return present;
}

void OneWireOutByte(int Pin, byte d) // output byte d (least sig bit first).
{
   byte n;
   for(n=8; n!=0; n--)
   {
      if ((d & 0x01) == 1)  // test least sig bit
      {
         digitalWrite(Pin, LOW);
         pinMode(Pin, OUTPUT);
         delayMicroseconds(5);
         pinMode(Pin, INPUT);
         delayMicroseconds(60);
      }
      else
      {
         digitalWrite(Pin, LOW);
         pinMode(Pin, OUTPUT);
         delayMicroseconds(60);
         pinMode(Pin, INPUT);
      }
      d=d>>1; // now the next bit is in
              // the least sig bit position.
   }
}//end OneWireOutByte

byte OneWireInByte(int Pin) // read byte, least sig byte first
{
  byte d, b;
  d=0;
/*This critical line added 04 Oct 16
    I hate to think how many derivatives of
      this code exist elsewhere on my web pages
      which have NOT HAD this. You may "get away"
      with not setting d to zero here... but it
      is A Very Bad Idea to trust to "hidden"
      initializations!
    The matter was brought to my attention by
      a kind reader who was THINKING OF YOU!!!
    If YOU spot an error, please write in, bring
      it to my attention, to save the next person
      grief.*/
// habe im internet das gefunden (chris) https://crazy-electronic.de/index.php/arduino/23-temperatur-messen-mit-ds18b20
   for (bLoopCounter=0; bLoopCounter<8; bLoopCounter++)
     {
      noInterrupts();//perni neu
      digitalWrite(Pin, LOW);
      pinMode(Pin, OUTPUT);
      delayMicroseconds(3);//perni war 5
      pinMode(Pin, INPUT);
      delayMicroseconds(10);//perni war 5
      b = digitalRead(Pin);
      interrupts();
      delayMicroseconds(53);//perni war 50
      d = (d >> 1) | (b << 7); // shift d to right and
         //insert b in most sig bit position
     }
   return(d);
}//end OneWireInByte()

void readTturePt1(byte Pin){
   if (!sensorDS1820[BOILER_NUMBER].reset()) {
     boilerDiagnostic = BOILER_NO_RESPONSE;
     markSensorFailure(BOILER_NUMBER);
     return;
   }
   sensorDS1820[BOILER_NUMBER].write(0xCC, POWER_MODE);
   sensorDS1820[BOILER_NUMBER].write(0x44, POWER_MODE);
}


void readTturePt2(byte Pin, const byte tmp_bWhichSensor){
   if (tmp_bWhichSensor != BOILER_NUMBER) return;
   OneWire& wire = sensorDS1820[tmp_bWhichSensor];
   if (!wire.reset()) {
     boilerDiagnostic = BOILER_NO_RESPONSE;
     markSensorFailure(tmp_bWhichSensor);
     return;
   }
   wire.write(0xCC, POWER_MODE);
   wire.write(0xBE, POWER_MODE);
   byte scratchpad[9];
   for (byte i = 0; i < 9; ++i) scratchpad[i] = wire.read();
   portENTER_CRITICAL(&boilerDiagnosticMux);
   memcpy(boilerRawBytes, scratchpad, sizeof(scratchpad));
   boilerRawAvailable = true;
   portEXIT_CRITICAL(&boilerDiagnosticMux);
   if (!validSensorScratchpad(scratchpad)) {
     bool allZero = true, allHigh = true;
     for (byte i = 0; i < 9; ++i) {
       allZero &= scratchpad[i] == 0;
       allHigh &= scratchpad[i] == 0xff;
     }
     boilerDiagnostic = allZero ? BOILER_ZERO_DATA : (allHigh ? BOILER_HIGH_DATA : BOILER_BAD_CRC);
     markSensorFailure(tmp_bWhichSensor);
     return;
   }
   int16_t raw = static_cast<int16_t>((scratchpad[1] << 8) | scratchpad[0]);
   const byte resolution = scratchpad[4] & 0x60;
   if (resolution == 0x00) raw &= ~7;
   else if (resolution == 0x20) raw &= ~3;
   else if (resolution == 0x40) raw &= ~1;
   const float measured = raw / 16.0f;
   if (!markSensorSuccess(tmp_bWhichSensor, measured)) return;
   if (tBoiler != measured + OS4) {
     tBoiler = measured + OS4;
     temp_update = true;
   }
}//end readTturePt2


void printTture(){//Uses values from global variables.
   if (bWhichSensor > kTtureSensorMaxIndex) return;
   String temp;
   if (Whole[bWhichSensor] < 10)
   /* To line up decimal points. This assumes that no tture will be < -99.9 or > +99.0 As these are in degrees C, that seems reasonable.
   And if a very high tture is measured, it will only slightly disturb the contents of the serial monitor.*/
   {
      temp = " ";
   }
   if (SignBit[bWhichSensor]) // If it is negative
     {
       temp = temp + "-";
     }//no ; here
   temp = temp + Whole[bWhichSensor];
   temp = temp + ".";
   if (Fract[bWhichSensor] < 10)
   {
      temp = temp + "0";
   }
   temp = temp + Fract[bWhichSensor];
   char buf[32];
   snprintf(buf, sizeof(buf), "%s", temp.c_str());

     switch (bWhichSensor) {
        case 0:
          //bez = bez + MQTT_TEXT + "S" + bWhichSensor + "Kessel";
          if(tKessel!=atof(buf)+OS0){
            temp_update=true;
            tKessel = atof(buf)+OS0;
          }
          break;
        case 1:
          //bez = bez + MQTT_TEXT + "S" + bWhichSensor + "Vorlauf";
          if(tVorlauf != atof(buf)+OS1){
            temp_update=true;
            tVorlauf = atof(buf)+OS1;
          }
          break;
        case 2:
          //bez = bez + MQTT_TEXT + "S" + bWhichSensor + "Aussen";
          if(tAussen != atof(buf)+OS2){
            tAussen = atof(buf)+OS2;
            temp_update=true;
          }
          break;
        case 3:
          //bez = bez + MQTT_TEXT + "S" + bWhichSensor + "Kueche";
          if(tRoom != atof(buf)+OS3){
            tRoom = atof(buf)+OS3;
            temp_update=true;
          }
          break;
        case 4:
          //bez = bez + MQTT_TEXT + "S" + bWhichSensor + "Boiler";
          if(tBoiler != atof(buf)+OS4){
            tBoiler = atof(buf)+OS4;
            temp_update=true;
          }
          break;
        default:
          break;
     }
}//end of printTture()



/* *******************************************************************************************************
                                         UPTIME
******************************************************************************************************* */
void uptime(){
  //** Making Note of an expected rollover *****//
  if(millis()>=3000000000){
  uHighMillis=1;
  }
  //** Making note of actual rollover **//
  if(millis()<=100000&&uHighMillis==1){
  uRollover++;
  uHighMillis=0;
  }
  long secsUp = millis()/1000;
  uSecond = secsUp%60;
  uMinute = (secsUp/60)%60;
  uHour = (secsUp/(60*60))%24;
  uDay = (uRollover*50)+(secsUp/(60*60*24));  //First portion takes care of a rollover [around 50 days]
}
/* *******************************************************************************************************
                                         Print UPTIME
******************************************************************************************************* */
void print_Uptime(){
  Serial.print(F("Uptime: ")); // The "F" Portion saves your SRam Space
  Serial.print(uDay);
  Serial.print(F("  Days  "));
  Serial.print(uHour);
  Serial.print(F("  Hours  "));
  Serial.print(uMinute);
  Serial.print(F("  Minutes  "));
  Serial.print(uSecond);
  Serial.println(F("  Seconds"));
};
/* *******************************************************************************************************
                                         AutomatikBetrieb
******************************************************************************************************* */
void Automatik(){
  RoomHeizen=true;
  KesselHeizen=true;
  BoilerHeizen=true;
  if (daynight=='D'){
    tmyRoomdest=tRoomTag; //Tag
  }
  if(daynight=='N'){
    tmyRoomdest=tRoomNacht; //Tag
  }
  RoomAnforderungf();
  BoilerAnforderungf();
  if (chckKessel()){
    chckRoom();
    chckBoiler();
  }
  SetOutPin();
}
/* *******************************************************************************************************
                                         Boilerbetrieb
******************************************************************************************************* */
void Boilerbetrieb(){
  RoomHeizen=false;
  KesselHeizen=true;
  BoilerHeizen=true;
  tmyRoomdest=0.0;
  BoilerAnforderungf();
  if (chckKessel()){
  chckBoiler();
  }
  MischerZu(); //2023-10-15 Mischer Zu - sonst heizt Raum mit.
  SetOutPin();
}
/* *******************************************************************************************************
                                         Heizungsbetrieb
******************************************************************************************************* */
void Heizungsbetrieb() {
  RoomHeizen=true;
  KesselHeizen=true;
  BoilerHeizen=false;
  if (daynight=='D'){
    tmyRoomdest=tRoomTag; //Tag
  }
  if(daynight=='N'){
    tmyRoomdest=tRoomNacht; //Tag
  }
  RoomAnforderungf();
  if (chckKessel()){
    chckRoom();
  }
  SetOutPin();
}
/* *******************************************************************************************************
                                         kein Betrieb
******************************************************************************************************* */
void kein_Betrieb() {
    RoomHeizen=false;
    BoilerHeizen=false;
    KesselHeizen=false;
    tmyRoomdest=0.0;
    BrennerRelais=false;//neu bis SetOutPin();
    if(tKessel<=kessel_min_temp){
        HeizungsRelais=false;//erst bei einer Mindesttemperatur des Kessels Pumpe ein
    }
    chckBoiler();//Boiler abschalten
  SetOutPin();
  //Serial.print("Alles ist ausgeschaltet!\n");
}
/* *******************************************************************************************************
                                         RoomAnforerung
******************************************************************************************************* */
bool updateHeatingCurve() {
  heizkurveGueltig = false;
  vorlaufTemperatur = 0.0f;
  if (AussentemperaturRegelung == 0) return false;

  // Snapshot the parameters and reject undefined mathematical domains.
  const double room = tmyRoomdest;
  const double outside = tAussen;
  const double maximum = tvmax;
  const double minimumOutside = taumin;
  const double exponent = n;
  if (!isfinite(room) || !isfinite(outside) || !isfinite(maximum) ||
      !isfinite(minimumOutside) || !isfinite(exponent) ||
      room < 5.0 || room > 40.0 || outside < -55.0 || outside > 125.0 ||
      maximum < room || maximum > 100.0 || minimumOutside < -50.0 ||
      minimumOutside > 0.0 || exponent < 0.1 || exponent > 10.0) return false;

  const double denominator = room - minimumOutside;
  if (denominator <= 0.0) return false;
  double ratio = (room - outside) / denominator;
  // Above the room target use the lower endpoint; below the design outside
  // temperature use the configured maximum, never a negative power base.
  if (ratio < 0.0) ratio = 0.0;
  if (ratio > 1.0) ratio = 1.0;
  const double target = room + (maximum - room) * pow(ratio, 1.0 / exponent);
  if (!isfinite(target) || target < room || target > maximum) return false;
  vorlaufTemperatur = static_cast<float>(floor(target * 100.0) / 100.0);
  heizkurveGueltig = true;
  return true;
}

void RoomAnforderungf(){
  if (heatingPumpSensorFault()) { RoomAnforderung = false; return; }
  if(AussentemperaturRegelung==0){
    if (tRoom < tmyRoomdest){
      RoomAnforderung=true;
    }else{
      RoomAnforderung=false;
    }
  }else{
    if (updateHeatingCurve() && tVorlauf < vorlaufTemperatur){
      RoomAnforderung=true;
    }else{
      RoomAnforderung=false;
    }
  }
}
/* *******************************************************************************************************
                                         Kachelofen Status
******************************************************************************************************* */
void updateKachelofenStatus(){
  const unsigned long now = millis();
  const bool mqttWertGueltig = (kachelofenLastUpdate != 0) &&
                               ((unsigned long)(now - kachelofenLastUpdate) <= kachelofenTimeout);

  if(!mqttWertGueltig){
    // Ohne frischen Kachelofenwert gilt der Ofen als inaktiv.
    kachelofenAktiv = false;
  }

  // Hysterese nur anwenden, wenn ein frischer MQTT-Wert vorhanden ist.
  if(mqttWertGueltig){
    if(!kachelofenAktiv && tKachelofen >= kachelofenEinTemp){
      kachelofenAktiv = true;
    }else if(kachelofenAktiv && tKachelofen <= kachelofenAusTemp){
      kachelofenAktiv = false;
    }
  }

  // AUTO folgt dem Kachelofen. RAUM/AUSSEN sind manuelle Service-Overrides.
  if(regelungsModus == REGELUNG_AUTO){
    AussentemperaturRegelung = kachelofenAktiv ? 1 : 0;
  }else if(regelungsModus == REGELUNG_RAUM){
    AussentemperaturRegelung = 0;
  }else{
    AussentemperaturRegelung = 1;
  }
}


/* *******************************************************************************************************
                                         BoilerAnforderung
******************************************************************************************************* */
void BoilerAnforderungf(){
  if (boilerPumpSensorFault()) { BoilerAnforderung = false; return; }
  if(tBoiler < tBoilerDest){
    BoilerAnforderung=1;
  }else{
    BoilerAnforderung=0;
  }
}


/* *******************************************************************************************************
                                         Ausgänge schalten
******************************************************************************************************* */
bool heatingPumpSensorFault() {
  return !sensorIsUsable(0) || !sensorIsUsable(1) ||
         !sensorIsUsable(AussentemperaturRegelung == 0 ? 3 : 2);
}

bool boilerPumpSensorFault() {
  return !sensorIsUsable(0) || !sensorIsUsable(4);
}

struct PumpFaultState {
  bool active = false;
  bool wasRunning = false;
  bool expiryReported = false;
  unsigned long started = 0;
};
PumpFaultState heatingPumpFault, boilerPumpFault;
constexpr unsigned long PUMP_FAULT_RUNON_MS = 10UL * 60UL * 1000UL;

bool pumpOutputWithFault(PumpFaultState& state, bool fault, bool requested,
                         uint8_t pin, const char* name) {
  if (!fault) {
    if (state.active) Serial.printf("%s: Sensoren wieder gueltig\n", name);
    state.active = false;
    return requested;
  }
  if (!state.active) {
    state.active = true;
    state.wasRunning = digitalRead(pin) == HIGH;
    state.started = millis();
    state.expiryReported = false;
    Serial.printf("%s: Sensorfehler, %s\n", name,
                  state.wasRunning ? "10 Minuten Nachlauf" : "bleibt aus");
  }
  const bool running = state.wasRunning &&
    (unsigned long)(millis() - state.started) < PUMP_FAULT_RUNON_MS;
  if (state.wasRunning && !running && !state.expiryReported) {
    Serial.printf("%s: Nachlauf beendet, gesperrt\n", name);
    state.expiryReported = true;
  }
  return running;
}

void SetOutPin(){
  enforceKesselSensorLock();
  const bool heatingFault = heatingPumpSensorFault();
  const bool boilerFault = boilerPumpSensorFault();
  const bool heatingOn = pumpOutputWithFault(heatingPumpFault, heatingFault,
      HeizungsRelais || Pumpenloesen, HeizungPin, "Heizungspumpe");
  const bool boilerOn = pumpOutputWithFault(boilerPumpFault, boilerFault,
      BoilerRelais || Pumpenloesen, BoilerPin, "Boilerpumpe");
  if (heatingFault) RoomAnforderung = false;
  if (boilerFault) { BoilerAnforderung = false; BoilerAufheizen = false; }
  HeizungsRelais = heatingOn;
  BoilerRelais = boilerOn;
  if (heatingOn){
    digitalWrite(HeizungPin, HIGH);
   // bitSet(ioextender0_indicate, exHeizung_Pin);
  }else{
    digitalWrite(HeizungPin, LOW);
   // bitClear(ioextender0_indicate, exHeizung_Pin);
  }
  if (boilerOn){
    digitalWrite(BoilerPin, HIGH);
   // bitSet(ioextender0_indicate, exBoiler_Pin);
  }else{
    digitalWrite(BoilerPin, LOW);
   // bitClear(ioextender0_indicate, exBoiler_Pin);
  }
  if (BrennerRelais==true){
    digitalWrite(BrennerPin, HIGH);
   // bitSet(ioextender0_indicate, exBrenner_Pin);
  }else{
    digitalWrite(BrennerPin, LOW);
   // bitClear(ioextender0_indicate, exBrenner_Pin);
  }
  //I2C_IO_BitWrite(ioextender0_addr,ioextender0_indicate);
}


/* *******************************************************************************************************
                                         Kessel Check
******************************************************************************************************* */
int chckKessel() {
  float nKesselDiff=tKesselDiff;
  if(BoilerAufheizen){
    nKesselDiff-=5;
  }else{
    nKesselDiff=tKesselDiff;
  }
  if (KesselHeizen==false){
    BrennerRelais=false;
  }else{
    if (tKessel >= (tKesselDest)) {
      BrennerRelais=false;//AUS
      //Serial.write("Brenner ausschalten\n");
    }
    if((tKessel < (tKesselDest-nKesselDiff)) && (BoilerAnforderung)) { //neu && BoilerAufheizen - um Kessel nicht immer aufzuheizen
      BrennerRelais=true;//EIN
      //Serial.write("Brenner einschalten\n");
    }
    if((tKessel < (tKesselDest-nKesselDiff)) && (RoomAnforderung)) { //neues if - um Kessel nicht immer aufzuheizen
      BrennerRelais=true;//EIN
      //Serial.write("Brenner einschalten\n");
    }
  }
  SetOutPin();
  if (tKessel < (tKesselDest-(tKesselDiff+10))){
    return 0;
  }else {
    return 1;
  }
}
/* *******************************************************************************************************
                                         Room Check
******************************************************************************************************* */
void chckRoom() {
  if (heatingPumpSensorFault()) {
    RoomAnforderung = false;
    if (!mischer_init_laeuft) MischerStop();
    SetOutPin();
    return;
  }

  updateHeatingCurve();
  if (AussentemperaturRegelung != 0 && !heizkurveGueltig) {
    RoomAnforderung = false;
    if (!mischer_init_laeuft) MischerStop();
    SetOutPin();
    return;
  }

  if(RoomHeizen==false){
    HeizungsRelais=false;
  }else{
    if(AussentemperaturRegelung == 0){
      if (tRoom >= tmyRoomdest)
      {
        HeizungsRelais=false;//AUS
        //Serial.write("Heizungspumpe ausschalten\n");
      }
      if(tRoom < (tmyRoomdest-tRoomDiff)) {
        HeizungsRelais=true;//EIN
        //Serial.write("Heizungspumpe einschalten\n");
      }
    }else { //AussentemperaturRegelung == 1
      //Serial.println("AussentemperaturRegelung == 1, HeizungsRelais ein");
      if(tKessel>=kessel_min_temp){
        HeizungsRelais=true;//erst bei einer Mindesttemperatur des Kessels Pumpe ein
      }
    if(!mischer_init_laeuft && mischer_init_auf && mischer_init_zu){
      if (tVorlauf < vorlaufTemperatur){
        if(mischer_wait==0){
          mischer_wait=MISCHER_WAIT;
          mischer_drive=MISCHER_DRIVE;
          MischerAuf(); 
        } //MischerAuf
      }else{
        if(mischer_wait==0){
          mischer_wait=MISCHER_WAIT;
          mischer_drive=MISCHER_DRIVE;
          MischerZu(); 
        } //MischerZu
      }
      if((MischerAufRelais || MischerZuRelais) && !mischer_drive){
        MischerStop();
      }
     }
    }
  }
  SetOutPin();
}
/* *******************************************************************************************************
                                         Boiler Check
******************************************************************************************************* */
void chckBoiler() {
  if (boilerPumpSensorFault()) {
    BoilerAnforderung = false;
    BoilerAufheizen = false;
    SetOutPin();
    return;
  }
  if(BoilerHeizen==false){
    BoilerRelais=false;
    BoilerAufheizen=false;
  }else{
  if (tBoiler >= tBoilerDest)
  {
    BoilerRelais=false;//AUS
    BoilerAufheizen=false;//Aufheizen stoppen
    Serial.write("Boiler aufgeheizt\n");
  }
//  if((tBoiler < (tBoilerDest-tBoilerDiff)) && tBoiler < tKessel) {
  if((tBoiler < (tBoilerDest-tBoilerDiff))) {
    //BoilerRelais=true;//EIN
    BoilerAufheizen=true;//Aufheizen starten
    Serial.write("Boiler aufheizen aktiv\n");
  }
  if(BoilerAufheizen){
    if(tBoiler < tKessel){
      BoilerRelais=true;
    }else{
      BoilerRelais=false;
      if(KesselHeizen){//zuerst noch schauen ob KesselHeizen überhaupt aktiv ist
        BrennerRelais=true;//Kessel muss erst Temperatur haben
      }
    }
  }
 }
  SetOutPin();
}

/* *******************************************************************************************************
                                         Umschaltung Regelung Aussen - Innen
******************************************************************************************************* */
void Regelungs_Switch_AI(){
if(AussentemperaturRegelung){AussentemperaturRegelung=0;}
else{AussentemperaturRegelung=1;}
}
/* *******************************************************************************************************
                                         PumpenNachlauf
******************************************************************************************************* */
void hpumpe_nachlauf(){
  PumpenNachlauf = 0;
}
/* *******************************************************************************************************
                                         Serial Read
******************************************************************************************************* */
void Serial_Read() {
/*
   Serial.println("Per Serial den Arbeitsmodus umschalten:");
   Serial.println("a ->AutomatikBetrieb");
   Serial.println("b ->BoilerBetrieb");
   Serial.println("h ->NurHeizungsBetrieb");
   Serial.println("r ->Aussen-/Raum-Regelung");
   Serial.println("s ->AUS");
*/
  if (Serial.available() > 0){
    int input = Serial.read();
    switch (input) {
      case 114:
        Regelungs_Switch_AI();
        break;
      case 97:
        Serial.print("Go to Automatik-Betrieb\n");
        Betriebsart='A';
        break;
      case 98:
        Serial.print("Go to Boiler-Betrieb\n");
        Betriebsart='B';
        break;
      case 104:
        Serial.print("Go to NurHeizungs-Betrieb\n");
        Betriebsart='H';
        break;
      case 115:
        Serial.print("Go to Disabled\n");
        Betriebsart='0';
        break;
      case 48:
        ESP.restart(); // bei 0 restart
        break;
      default:
        break;
    }
  }
}


/* *******************************************************************************************************
                                         mqtt callback
******************************************************************************************************* */
void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  mqtt_message++;
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void applyMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total, unsigned long receivedAt){
  if (!topic || !payload || index != 0 || len != total || len == 0 || len >= 64 ||
      memchr(payload, '\0', len) != nullptr) {
    Serial.println("MQTT: ungueltige oder aufgeteilte Nachricht verworfen");
    return;
  }
/*    /SmartHome/Keller/Heizung/setRaumTemp
in FHEM:   set MQTT_SERVER publish /SmartHome/Keller/Heizung/setRaumTemp up
           set MQTT_SERVER publish /SmartHome/Keller/Heizung/setRaumTemp down
*/
  if (strcmp(topic, GAS_MQTT_TOPIC) == 0) {
    char text[64];
    memcpy(text, payload, len);
    text[len] = '\0';
    char* end = nullptr;
    const double value = strtod(text, &end);
    if (end == text || *end != '\0' || !isfinite(value) || value < 0 || value > 1e12) {
      Serial.println("Gaszaehler: ungueltigen Stand verworfen");
      return;
    }
    portENTER_CRITICAL(&gasMux);
    pendingGasTotal = value;
    pendingGasRetained = properties.retain;
    pendingGas = true;
    portEXIT_CRITICAL(&gasMux);
    return;
  }
  // Kachelofen-Temperatur: eigenes Topic, Payload ist nur die Temperatur (z.B. 65.4).
  if(strcmp(topic, KACHELOFEN_MQTT_TOPIC) == 0){
    if(index == 0 && len == total && len > 0 && len < 16){
      char tempPayload[16];
      memcpy(tempPayload, payload, len);
      tempPayload[len] = '\0';
      char* endPtr = nullptr;
      float newTemp = strtof(tempPayload, &endPtr);
      if(endPtr != tempPayload && *endPtr == '\0' && isfinite(newTemp) && newTemp >= -40.0 && newTemp <= 200.0){
        tKachelofen = newTemp;
        kachelofenLastUpdate = receivedAt;
        updateKachelofenStatus();
        Serial.print("Kachelofen MQTT: ");
        Serial.print(tKachelofen, 1);
        Serial.print(" C, aktiv=");
        Serial.println(kachelofenAktiv ? "JA" : "NEIN");
      }else{
        Serial.println("Kachelofen MQTT: ungueltiger Temperaturwert verworfen");
      }
    }
    return;
  }

  mqtt_message++;
  const String controlTopic = String(MQTT_TEXT) + "set";
  if (strcmp(topic, controlTopic.c_str()) != 0) return;
  char new_payload[64];
  memcpy(new_payload, payload, len);
  new_payload[len] = '\0';

    Serial.print("\nReceived message [");
    Serial.print(topic);
    Serial.print("] ");
  //  payload[len] = '\0';

    if(strcmp(new_payload,"up")==0){
        tRoomTag += 0.2; //RaumTemperatur um 0.2 *C erhöhen
        tRoomNacht += 0.2;
        Serial.print("MQTT Message UP ->");
        Serial.print(tRoomTag);
        strcpy(mqtt_payload,"h_up");
        return;
    }
    if(strcmp(new_payload,"down")==0){
        tRoomTag -= 0.2; //RaumTemperatur um 0.2 *C senken
        tRoomNacht -= 0.1;
        Serial.print("MQTT Message DOWN ->");
        Serial.print(tRoomTag);
        strcpy(mqtt_payload,"h_down");
        return;
    }
    if(strcmp(new_payload,"save")==0){
        tRoomTag -= 2.0; //RaumTemperatur um 2.0 *C senken
        tRoomNacht -= 2.0;
        Serial.print("MQTT Message SAVE ->");
        Serial.print(tRoomTag);
        strcpy(mqtt_payload,"h_save");
        return;
    }
    if(strcmp(new_payload,"normal")==0){
        EEPROM.get( EEADDRESS_RAUM, tRoomTag );//RaumTemperatur vom eeprom holen
        EEPROM.get( EEADDRESS_RAUMNACHT, tRoomNacht );//RaumTemperatur vom eeprom holen
        Serial.print("MQTT Message NORMAL ->");
        Serial.print(tRoomTag);
        strcpy(mqtt_payload,"h_norm");
        return;
    }
    if(strcmp(new_payload,"party")==0){
        tRoomTag += 2.0; //RaumTemperatur um 0.2 *C senken
        tRoomNacht += 2.0;
        Serial.print("MQTT Message PARTY ->");
        Serial.print(tRoomTag);
        strcpy(mqtt_payload,"h_party");
        return;
    }
    if(strcmp(new_payload,"setoff")==0){
        BoilerBetrieb=0;
        NurHeizung=0;
        WinterBetrieb=0;
        EEPROM.put( EEADDRESS_BOILER_SOMMERBETRIEB, BoilerBetrieb );
        EEPROM.put( EEADDRESS_NUR_HEIZUNG, NurHeizung );
        EEPROM.put( EEADDRESS_WINTER, WinterBetrieb );
        strcpy(mqtt_payload,"h_aus");
        return;
    }
    if(strcmp(new_payload,"seton")==0){
        BoilerBetrieb=0;
        NurHeizung=0;
        WinterBetrieb=1;
        EEPROM.put( EEADDRESS_BOILER_SOMMERBETRIEB, BoilerBetrieb );
        EEPROM.put( EEADDRESS_NUR_HEIZUNG, NurHeizung );
        EEPROM.put( EEADDRESS_WINTER, WinterBetrieb );
        strcpy(mqtt_payload,"h_ein");
        return;
    }
    if(strcmp(new_payload,"setboiler")==0){
        Betriebsart='B';
        WinterBetrieb=0;
        BoilerBetrieb=1;
        NurHeizung=0;
        EEPROM.put( EEADDRESS_BOILER_SOMMERBETRIEB, BoilerBetrieb );
        EEPROM.put( EEADDRESS_NUR_HEIZUNG, NurHeizung );
        EEPROM.put( EEADDRESS_WINTER, WinterBetrieb );
        strcpy(mqtt_payload,"h_boiler");
        return;
    }
    if(strcmp(new_payload,"setwinter")==0){
        Betriebsart='A';
        WinterBetrieb=1;
        BoilerBetrieb=0;
        NurHeizung=0;
        EEPROM.put( EEADDRESS_BOILER_SOMMERBETRIEB, BoilerBetrieb );
        EEPROM.put( EEADDRESS_NUR_HEIZUNG, NurHeizung );
        EEPROM.put( EEADDRESS_WINTER, WinterBetrieb );
        strcpy(mqtt_payload,"h_winter");
        return;
    }
    if(strcmp(new_payload,"setheizung")==0){
        Betriebsart='H';
        WinterBetrieb=0;
        BoilerBetrieb=0;
        NurHeizung=1;
        EEPROM.put( EEADDRESS_BOILER_SOMMERBETRIEB, BoilerBetrieb );
        EEPROM.put( EEADDRESS_NUR_HEIZUNG, NurHeizung );
        EEPROM.put( EEADDRESS_WINTER, WinterBetrieb );
        strcpy(mqtt_payload,"h_heizung");
        return;
    }
    if(strcmp(new_payload,"ntpupdate")==0){
        ntpupdate();
        strcpy(mqtt_payload,"h_ntpup");
        return;
    }
    else if(strcmp(new_payload,"tin")==0){
        regelungsModus = REGELUNG_RAUM;
        updateKachelofenStatus();
        strcpy(mqtt_payload,"h_tin");
        return;
    }
    if(strcmp(new_payload,"tau")==0){
        regelungsModus = REGELUNG_AUSSEN;
        updateKachelofenStatus();
        strcpy(mqtt_payload,"h_tau");
        return;
    }
    if(strcmp(new_payload,"tauto")==0){
        regelungsModus = REGELUNG_AUTO;
        updateKachelofenStatus();
        strcpy(mqtt_payload,"h_tauto");
        return;
    }
    if(strstr(new_payload,"kachelofenEin:") == new_payload){
        const char* numeric = new_payload + 14;
        char* endPtr = nullptr;
        const float tempTemp = strtof(numeric, &endPtr);
        if (endPtr == numeric || *endPtr != '\0' || !isfinite(tempTemp) ||
            tempTemp < 10.0f || tempTemp > 150.0f) {
          Serial.println("MQTT: ungueltiger Zahlenwert verworfen");
          return;
        }
        if(isfinite(tempTemp) && tempTemp >= 10.0 && tempTemp <= 150.0 && tempTemp > kachelofenAusTemp){
          kachelofenEinTemp = tempTemp;
          EEPROM.put( EEADDRESS_KACHELOFEN_EIN, kachelofenEinTemp );
          EEPROM.commit();
          updateKachelofenStatus();
          strcpy(mqtt_payload,"h_kof_ein");
          Serial.print("Kachelofen EIN neu: ");
          Serial.println(kachelofenEinTemp, 1);
        }else{
          Serial.println("MQTT kachelofenEin: ungueltiger Wert");
        }
        return;
    }
    if(strstr(new_payload,"kachelofenAus:") == new_payload){
        const char* numeric = new_payload + 14;
        char* endPtr = nullptr;
        const float tempTemp = strtof(numeric, &endPtr);
        if (endPtr == numeric || *endPtr != '\0' || !isfinite(tempTemp) ||
            tempTemp < 0.0f || tempTemp > 140.0f) {
          Serial.println("MQTT: ungueltiger Zahlenwert verworfen");
          return;
        }
        if(isfinite(tempTemp) && tempTemp >= 0.0 && tempTemp <= 140.0 && tempTemp < kachelofenEinTemp){
          kachelofenAusTemp = tempTemp;
          EEPROM.put( EEADDRESS_KACHELOFEN_AUS, kachelofenAusTemp );
          EEPROM.commit();
          updateKachelofenStatus();
          strcpy(mqtt_payload,"h_kof_aus");
          Serial.print("Kachelofen AUS neu: ");
          Serial.println(kachelofenAusTemp, 1);
        }else{
          Serial.println("MQTT kachelofenAus: ungueltiger Wert");
        }
        return;
    }
    if(strncmp(new_payload,"newRoomTemp:", 12) == 0){ //Raumtemperatur per mqtt setzen
        const char* numeric = new_payload + 12;
        char* endPtr = nullptr;
        const float tempTemp = strtof(numeric, &endPtr);
        if (endPtr == numeric || *endPtr != '\0' || !isfinite(tempTemp) ||
            tempTemp < 5.0f || tempTemp > 40.0f) {
          Serial.println("MQTT: ungueltiger Zahlenwert verworfen");
          return;
        }
        tRoomTag = tempTemp;
        tRoomNacht = tempTemp;
        strcpy(mqtt_payload,"h_newRT");
        return;
    }
    if(strncmp(new_payload,"newBoilerTemp:", 14) == 0){ //Raumtemperatur per mqtt setzen
        const char* numeric = new_payload + 14;
        char* endPtr = nullptr;
        const float tempTemp = strtof(numeric, &endPtr);
        if (endPtr == numeric || *endPtr != '\0' || !isfinite(tempTemp) ||
            tempTemp < 0.0f || tempTemp > 100.0f) {
          Serial.println("MQTT: ungueltiger Zahlenwert verworfen");
          return;
        }
        tBoilerDest = tempTemp;
        EEPROM.put( EEADDRESS_BOILER, tBoilerDest );
        strcpy(mqtt_payload,"h_newBT");
        return;
    }
    if(strncmp(new_payload,"tvmax:", 6) == 0){ //max. Vorlauftemperatur  per mqtt setzen
        const char* numeric = new_payload + 6;
        char* endPtr = nullptr;
        const float tempTemp = strtof(numeric, &endPtr);
        if (endPtr == numeric || *endPtr != '\0' || !isfinite(tempTemp) ||
            tempTemp < 0.0f || tempTemp > 100.0f) {
          Serial.println("MQTT: ungueltiger Zahlenwert verworfen");
          return;
        }
        tvmax = tempTemp;
        strcpy(mqtt_payload,"h_tvmax");
        return;
    }
    if(strncmp(new_payload,"taumin:", 7) == 0){ //minimal Aussentemp per mqtt setzen
        const char* numeric = new_payload + 7;
        char* endPtr = nullptr;
        const float tempTemp = strtof(numeric, &endPtr);
        if (endPtr == numeric || *endPtr != '\0' || !isfinite(tempTemp) ||
            tempTemp < -50.0f || tempTemp > 0.0f) {
          Serial.println("MQTT: ungueltiger Zahlenwert verworfen");
          return;
        }
        taumin = tempTemp;
        strcpy(mqtt_payload,"h_tvmin");
        return;
    }
    if(strncmp(new_payload,"tn:", 3) == 0){ //Steigung per mqtt setzen
        const char* numeric = new_payload + 3;
        char* endPtr = nullptr;
        const float tempTemp = strtof(numeric, &endPtr);
        if (endPtr == numeric || *endPtr != '\0' || !isfinite(tempTemp) ||
            tempTemp < 0.1f || tempTemp > 10.0f) {
          Serial.println("MQTT: ungueltiger Zahlenwert verworfen");
          return;
        }
        n = tempTemp;
        strcpy(mqtt_payload,"h_tn");
        return;
    }
}


void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties,
                   size_t len, size_t index, size_t total) {
  if (!topic || !payload || index != 0 || len != total || len == 0 || len >= 64 ||
      memchr(payload, '\0', len) != nullptr) {
    Serial.println("MQTT: ungueltige oder aufgeteilte Nachricht verworfen");
    return;
  }
  if (strcmp(topic, MQTT_TEXT "set") != 0 &&
      strcmp(topic, KACHELOFEN_MQTT_TOPIC) != 0 && strcmp(topic, GAS_MQTT_TOPIC) != 0) return;
  const size_t topicLength = strnlen(topic, 128);
  if (topicLength >= 128) return;
  PendingMqttMessage message{};
  memcpy(message.topic, topic, topicLength + 1);
  memcpy(message.payload, payload, len);
  message.payload[len] = '\0';
  message.length = len;
  message.properties = properties;
  message.receivedAt = millis();
  if (!mqttMessageQueue || xQueueSend(mqttMessageQueue, &message, 0) != pdTRUE) {
    Serial.println("MQTT: Empfangswarteschlange voll oder nicht verfuegbar, Nachricht verworfen");
  }
}

void processMqttMessages() {
  if (!mqttMessageQueue) return;
  PendingMqttMessage message;
  // Bound each pass, so a stream of commands cannot starve the controller loop.
  for (unsigned i = 0; i < MQTT_QUEUE_LENGTH; ++i) {
    if (xQueueReceive(mqttMessageQueue, &message, 0) != pdTRUE) break;
    applyMqttMessage(message.topic, message.payload, message.properties,
                     message.length, 0, message.length, message.receivedAt);
  }
}

/* *******************************************************************************************************
                                         mqtt clearstring
******************************************************************************************************* */
/*void clearstring() {
  //Serial.flush(); // clears the buffer, you dont need this
  for (int r=0; r<7; r++){
  my_str[r] = '\0'; // deletes each block
  }
}
*/

/* *******************************************************************************************************
                                         mqtt reconnect
******************************************************************************************************* */
void onMqttConnect(bool sessionPresent) {
      const String subsc = String(MQTT_TEXT) + "set";
      Serial.println("connect to MQTT\n");
      Serial.print("Session present: ");
      Serial.println(sessionPresent);
      asyncMqttClient.subscribe(subsc.c_str(),1);
      asyncMqttClient.subscribe(KACHELOFEN_MQTT_TOPIC,1);
      asyncMqttClient.subscribe(GAS_MQTT_TOPIC,1);
      //uint16_t packetIdSub = asyncMqttClient.subscribe("/SmartHome/Keller/Heizung/setRaumTemp", 2);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.printf("MQTT-Abbruchgrund: %u\n",
              static_cast<unsigned>(reason));
  if (!WiFi.isConnected()){
    WiFi.disconnect();
    WiFi.begin(ssid,password);
  }
        mqttRecon=true;
        previousTime_mqtt=millis();
}
/* *******************************************************************************************************
                                         print LCD Values
******************************************************************************************************* */
void print_Main_LCD_Values(){
  char line0[21];
  char line1[21];
  char float_str0[32];
  char float_str1[32];

  lcd.setCursor(0, 0);
  lcd.print("                    ");
  snprintf(line1, sizeof(line1), "Br%sH%sB%sA%sZ%s", BrennerRelais ? "+" : "-", HeizungsRelais ? "+" : "-", BoilerRelais ? "+" : "-", MischerAufRelais ? "+" : "-", MischerZuRelais ? "+" : "-");
  lcd.setCursor(0, 0);
  lcd.print(line1);
   serial_go_home();
   Serial.println(line1);
  lcd.setCursor(12, 0);
  snprintf(float_str0, sizeof(float_str0), "%4.1f", static_cast<double>(tmyRoomdest));
  snprintf(line0, sizeof(line0), "%cR=%s",daynight,float_str0);
  lcd.print(line0);
   serial_newline(); 
   Serial.println(line0);
  lcd.setCursor(0, 1);
  lcd.print("                    ");
  lcd.setCursor(0, 1);
  snprintf(float_str0, sizeof(float_str0), "%4.1f", static_cast<double>(tKessel));
  snprintf(float_str1, sizeof(float_str1), "%4.1f", static_cast<double>(tVorlauf));
  snprintf(line0, sizeof(line0), "H:%-5s V:%-5s", float_str0, float_str1); // %6s right pads the string
  lcd.print(line0);
   serial_newline();
   Serial.println(line0);
  snprintf(line0, sizeof(line0), "A%d",AussentemperaturRegelung);
  lcd.setCursor(16, 1);
  lcd.print(line0);
   serial_newline();
   Serial.println(line0);
  snprintf(line0, sizeof(line0), "P%d",Pumpenloesen);
  lcd.setCursor(18, 1);
  lcd.print(line0);
   serial_newline();
   Serial.println(line0);
  memset(line0, 0, sizeof line0);//Der Buffer wird geloescht
  memset(float_str0, 0, sizeof float_str0);
  memset(float_str1, 0, sizeof float_str1);
  lcd.setCursor(0, 2);
  lcd.print("                    ");
  lcd.setCursor(0, 2);
  snprintf(float_str0, sizeof(float_str0), "%4.1f", static_cast<double>(tAussen));
  snprintf(float_str1, sizeof(float_str1), "%4.1f", static_cast<double>(tRoom));
  snprintf(line0, sizeof(line0), "A:%-5s R:%-5s", float_str0, float_str1); // %6s right pads the string
  lcd.print(line0);
   serial_newline();
   Serial.println(line0);
/*  if(asyncMqttClient.connected()){
    snprintf(line0, sizeof(line0), "M");
  }else{
    snprintf(line0, sizeof(line0), "-");
  }
  lcd.setCursor(15, 2);
  lcd.print(line0);
*/
  snprintf(line0, sizeof(line0), "SZ%d", Sommerzeit_EinAus);
  lcd.setCursor(17, 2);
  lcd.print(line0);
   serial_newline();
   Serial.println(line0);
  memset(line0, 0, sizeof line0);//Der Buffer wird geloescht
  memset(float_str0, 0, sizeof float_str0);
  memset(float_str1, 0, sizeof float_str1);
  snprintf(float_str0, sizeof(float_str0), "%4.1f", static_cast<double>(tBoiler));
  lcd.setCursor(0, 3);
  lcd.print("                    ");
  lcd.setCursor(0, 3);
  snprintf(line0, sizeof(line0), "B:%-5s ", float_str0); // %6s right pads the string
  lcd.print(line0);
   serial_newline();
   Serial.println(line0);
  snprintf(line0, sizeof(line0), "%02d.%02d. %02d%c%02d", myday, mymonth, myhours, doppelp, myminutes); // %6s right pads the string
  lcd.setCursor(8, 3);
  lcd.print(line0);
   serial_newline();
   Serial.println(line0);
  memset(line0, 0, sizeof line0);//Der Buffer wird geloescht
  snprintf(line0, sizeof(line0), "Mischer wait -> %d", mischer_wait);
  Serial.println(line0);
  esp_task_wdt_reset(); //watchdog Zeit wieder rücksetzen
}

boolean summertime_EU(int year, byte month, byte day, byte hour, byte tzHours)
// European Daylight Savings Time calculation by "jurs" for German Arduino Forum
// input parameters: "normal time" for year, month, day, hour and tzHours (0=UTC, 1=MEZ)
// return value: returns true during Daylight Saving Time, false otherwise
{
  if (month<3 || month>10) return false; // keine Sommerzeit in Jan, Feb, Nov, Dez
  if (month>3 && month<10) return true; // Sommerzeit in Apr, Mai, Jun, Jul, Aug, Sep
  if ((month==3 && ((hour + 24 * day)>=(1 + tzHours + 24*(31 - (5 * year /4 + 4) % 7)))) || ((month==10) && ((hour + 24 * day)<(1 + tzHours + 24*(31 - (5 * year /4 + 1) % 7))))){
    return true;
  }else{
    return false;
  }
}

//#################################################################################################################################
//                        I2C_IO_Extender
//#################################################################################################################################
void I2C_IO_Init(uint8_t address, uint8_t data){
  if (!ioExtenderPresent) return;
  Wire.beginTransmission(address);
  Wire.write(0xF);
  //Wire.write(data); //alle Ausgänge ausschalten
  Wire.endTransmission();
}
//#################################################################################################################################
void I2C_IO_BitWrite(uint8_t address, uint8_t data){
  if (!ioExtenderPresent) return;
  Wire.beginTransmission(address);
  Wire.write(data); //alle Ausgänge ausschalten
  Wire.endTransmission();
  dataToI2C = data;
//  dataToI2C=I2C_IO_ReadInputs(address);
}
uint8_t I2C_IO_ReadInputs(uint8_t address){
  if (!ioExtenderPresent) return 0xff;
  Wire.beginTransmission(address);
  Wire.requestFrom(address, uint8_t(2));
  uint8_t Data_In = Wire.read();
  Wire.endTransmission();
  return Data_In;
}
//#################################################################################################################################
bool vorlaufFaultOpening = false;
unsigned long vorlaufFaultStarted = 0;
constexpr unsigned long VORLAUF_FAULT_OPEN_MS = 20000UL;

bool enforceVorlaufSensorLock() {
  const bool locked = !sensorIsUsable(1);
  static bool faultActive = false;
  if (!locked) {
    if (faultActive) {
      vorlaufFaultOpening = false;
      MischerStop();
      faultActive = false;
      Serial.println("Mischer freigegeben: Vorlaufsensor gueltig");
    }
    return false;
  }
  if (!faultActive) {
    faultActive = true;
    vorlaufFaultStarted = millis();
    vorlaufFaultOpening = true;
    mischer_wait = 0;
    mischer_drive = 0;
    // The fault movement invalidates the previously referenced position.
    mischer_init_laeuft = false;
    mischer_init_auf = false;
    mischer_init_zu = false;
    previousTime_MischerInit = 0;
    Serial.println("Vorlaufsensor ungueltig: Mischer einmalig 20 Sekunden AUF");
  }
  if (vorlaufFaultOpening &&
      (unsigned long)(millis() - vorlaufFaultStarted) < VORLAUF_FAULT_OPEN_MS) {
    // Deliberate fault action, bypassing the normal movement guards.
    digitalWrite(MischerZu_Pin, LOW);
    MischerZuRelais = false;
    digitalWrite(MischerAuf_Pin, HIGH);
    MischerAufRelais = true;
  } else {
    if (vorlaufFaultOpening) {
      vorlaufFaultOpening = false;
      Serial.println("Mischer gesperrt: 20-Sekunden-Auffahrt beendet");
    }
    MischerStop();
  }
  return true;
}

void MischerInit(){
  if (enforceVorlaufSensorLock()) return;
  unsigned long currentTime0;
  if(!mischer_init_auf && !mischer_init_laeuft && !mischer_init_zu){
   mischer_init_laeuft=true;
   MischerAuf();
   previousTime_MischerInit=millis();
  }
  if(mischer_init_laeuft && !mischer_init_auf && !mischer_init_zu){
    currentTime0=millis();
   if(currentTime0 - previousTime_MischerInit >= mischer_init_time_auf){
    MischerStop();
    mischer_init_auf=true;
    previousTime_MischerInit=millis();
    delay(125);
    MischerZu();
   }
  }
  if(mischer_init_laeuft && mischer_init_auf && !mischer_init_zu){
    currentTime0=millis();
    if(currentTime0 - previousTime_MischerInit >= mischer_init_time_zu){
      MischerStop();
      mischer_init_zu=true;
      mischer_init_laeuft=false;
      previousTime_MischerInit=0;
    }
  }
}
//#################################################################################################################################
void MischerAuf(){
  if (enforceVorlaufSensorLock()) return;
  digitalWrite(MischerZu_Pin, LOW);
  MischerZuRelais=false;
  digitalWrite(MischerAuf_Pin, HIGH);
  MischerAufRelais=true;
//  #if defined(EXMISCHER)
//  bitClear(ioextender0_indicate, exMischerZu_Pin);
//  bitSet(ioextender0_indicate,exMischerAuf_Pin);
//  I2C_IO_BitWrite(ioextender0_addr,ioextender0_indicate);
//  #endif
//  timer6_mischer_nachlauf.update();
}
//#################################################################################################################################
void MischerZu(){
  if (enforceVorlaufSensorLock()) return;
  digitalWrite(MischerAuf_Pin, LOW);
  MischerAufRelais=false;
  digitalWrite(MischerZu_Pin, HIGH);
  MischerZuRelais=true;
//  #if defined(EXMISCHER)
//  bitClear(ioextender0_indicate, exMischerAuf_Pin);
//  bitSet(ioextender0_indicate,exMischerZu_Pin);
//  I2C_IO_BitWrite(ioextender0_addr,ioextender0_indicate);
//  #endif
//  timer6_mischer_nachlauf.update();
}
//#################################################################################################################################
void MischerStop(){
  // Normal regulation must not interrupt the explicitly requested fault run.
  // Also check its deadline here, in case another caller reaches us first.
  if (vorlaufFaultOpening &&
      (unsigned long)(millis() - vorlaufFaultStarted) < VORLAUF_FAULT_OPEN_MS) return;
  digitalWrite(MischerAuf_Pin, LOW);
  MischerAufRelais=false;
  digitalWrite(MischerZu_Pin, LOW);
  MischerZuRelais=false;
//  #if defined(EXMISCHER)
//  bitClear(ioextender0_indicate, exMischerZu_Pin);
//  bitClear(ioextender0_indicate, exMischerAuf_Pin);
//  I2C_IO_BitWrite(ioextender0_addr,ioextender0_indicate);
//  #endif
}
//#################################################################################################################################
void sensorDS1820_indicateChip(byte pin)
{
  if (pin > kTtureSensorMaxIndex) return;
  if ( !sensorDS1820[pin].search(addr)) {
    sensorDS1820[pin].reset_search();
    delay(250);
    return;
  }
  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return;
  }
  switch (addr[0]) {
    case 0x10:
      Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
      return;
  }
}
//#################################################################################################################################
void sensorDS1820_reset(byte pin)
{
  if (pin > kTtureSensorMaxIndex) return;
  sensorDS1820[pin].reset();
  sensorDS1820[pin].write(0xCC, POWER_MODE);
  sensorDS1820[pin].write(0x44, POWER_MODE);
}
//#################################################################################################################################
void sensorDS1820_read(byte pin)
{
  if (pin > kTtureSensorMaxIndex) return;
  float temperature;
  //byte bufData[9];
  if (!sensorDS1820[pin].reset()) { markSensorFailure(pin); return; }
  sensorDS1820[pin].write(0xCC, POWER_MODE);
  sensorDS1820[pin].write(0xBE, POWER_MODE);
  //sensorDS1820[pin].read_bytes(bufData, 9);
//  if(OneWire::crc8(bufData,8)==bufData[8]){
    //data is correct
/*****************************************/

byte data[12];
int16_t raw;
//byte type_s;
//type_s = 0;
for ( int i = 0; i < 9; i++)
    { data[i] = sensorDS1820[pin].read(); }
if(validSensorScratchpad(data)){

  raw = (data[1] << 8) | data[0];
  if (type_s)
    {
    raw = raw << 3;
    if (data[7] == 0x10)
      // Vorzeichen expandieren
      { raw = (raw & 0xFFF0) + 12 - data[6]; }
    }
  else
    {
    byte cfg = (data[4] & 0x60);
    // Aufloesung bestimmen, bei niedrigerer Aufloesung sind
    // die niederwertigen Bits undefiniert -> auf 0 setzen
    if (cfg == 0x00) raw = raw & ~7;      //  9 Bit Aufloesung,  93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 Bit Aufloesung, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 Bit Aufloesung, 375.0 ms
    // Default ist 12 Bit Aufloesung, 750 ms Wandlungszeit
    }
  temperature = ((float)raw / 16.0);
  if (!markSensorSuccess(pin, temperature)) return;

/*****************************************/
  //  temperature= ((float)((int)((unsigned int)bufData[0] | (((unsigned int)bufData[1]) << 8)))) * 0.0625 + 0.03125;
     switch (bWhichSensor) {
        case 0:
          //bez = bez + MQTT_TEXT + "S" + bWhichSensor + "Kessel";
          if(tKessel!=temperature+OS0){
            temp_update=true;
            tKessel = temperature+OS0;
          }
          break;
        case 1:
          //bez = bez + MQTT_TEXT + "S" + bWhichSensor + "Vorlauf";
          if(tVorlauf != temperature+OS1){
            temp_update=true;
            tVorlauf = temperature+OS1;
          }
          break;
        case 2:
          //bez = bez + MQTT_TEXT + "S" + bWhichSensor + "Aussen";
          if(tAussen != temperature+OS2){
            tAussen = temperature+OS2;
            temp_update=true;
          }
          break;
        case 3:
          //bez = bez + MQTT_TEXT + "S" + bWhichSensor + "Kueche";
          if(tRoom != temperature+OS3){
            tRoom = temperature+OS3;
            temp_update=true;
          }
          break;
        case 4:
          //bez = bez + MQTT_TEXT + "S" + bWhichSensor + "Boiler";
          if(tBoiler != temperature+OS4){
            tBoiler = temperature+OS4;
            temp_update=true;
          }
          break;
        default:
          break;
     }
  } else {
    markSensorFailure(pin);
  }
}
//#################################################################################################################################
void serial_go_home()
{
  Serial.write(27);
  Serial.print("[H");     // cursor to home command
}
//#################################################################################################################################
void serial_clear_screen()
{
  Serial.write(27);       // ESC command
  Serial.print("[2J");    // clear screen command
  Serial.write(27);
  Serial.print("[H");     // cursor to home command
}
//#################################################################################################################################
void serial_newline()
{
  Serial.write(27);
  Serial.print("\n");
}
//#################################################################################################################################

void notifyClients() {
  ws.textAll(String(HeizungsRelais));
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  if (arg == nullptr || data == nullptr) {
    return;
  }

  const AwsFrameInfo *info = static_cast<AwsFrameInfo*>(arg);
  if (!info->final || info->index != 0 ||
      info->len != len || info->opcode != WS_TEXT) {
    return;
  }

  // Empfangspuffer unveraendert lassen; exakt sechs Zeichen pruefen.
  if (len == 6 && memcmp(data, "toggle", 6) == 0) {
    HeizungsRelais = !HeizungsRelais;
    notifyClients();
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      //Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      //Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

String processor(const String& var){
  if (var != "WEBTIME") Serial.println(var);
  if(var == "MQTTUPDATE"){
    return asyncMqttClient.getClientId();
  }
  if(var == "TimeString"){
    return TimeString;
  }
  if(var == "HEIZUNGSPUMPE"){
    if (HeizungsRelais){
      return "ON";
    }
    else{
      return "OFF";
    }
  }else if(var == "BOILERPUMPE"){
    if (BoilerRelais){
      return "ON";
    }
    else{
      return "OFF";
    }
  }else if(var == "TROOM"){
    return readTemperature(tRoom);
  }else if(var == "GASDAY"){
    return gasDisplay(true);
  }else if(var == "GASTOTAL"){
    return gasDisplay(false);
  }else if(var == "WEBTIME"){
    const time_t utc = time(nullptr);
    if (utc < static_cast<time_t>(1577836800UL)) return "NTP nicht synchron";
    time_t local = utc + GMT_TIME_ZONE * utcOffsetInSeconds;
    struct tm calendar = {};
    if (!gmtime_r(&local, &calendar)) return "Uhrzeit nicht verfuegbar";
    if (Sommerzeit_EinAus && summertime_EU(calendar.tm_year + 1900,
        calendar.tm_mon + 1, calendar.tm_mday, calendar.tm_hour, GMT_TIME_ZONE)) {
      local += 3600;
      if (!gmtime_r(&local, &calendar)) return "Uhrzeit nicht verfuegbar";
    }
    char text[48];
    snprintf(text, sizeof(text), "%s, %02d.%02d.%04d %02d:%02d:%02d",
             daysOfTheWeek[calendar.tm_wday], calendar.tm_mday, calendar.tm_mon + 1,
             calendar.tm_year + 1900, calendar.tm_hour, calendar.tm_min, calendar.tm_sec);
    return String(text);
  }else if(var == "BRENNERSPERRE"){
    return sensorIsUsable(0) ? "Kesselsensor OK - keine Sensorsperre" :
                              "BRENNER GESPERRT - Kesselsensor ungueltig";
  }else if(var == "TKESSEL"){
    return readTemperature(tKessel);
  }else if(var == "TVORLAUF"){
    return readTemperature(tVorlauf);
  }else if(var == "TBOILER"){
    return readTemperature(tBoiler);
  }else if(var == "TAUSSEN"){
    return readTemperature(tAussen);
  }else if(var == "TKACHELOFEN"){
    return readTemperature(tKachelofen);
  }else if(var == "KACHELOFENSTATUS"){
    return kachelofenAktiv ? "AKTIV" : "INAKTIV";
  }else if(var == "BETRIEBSART"){
    return WinterBetrieb ? "AUTOMATIKBETRIEB" : BoilerBetrieb ? "NUR BOILER" : NurHeizung ? "NUR HEIZUNG" : "AUS";
  }else if(var == "BETRIEBAUTO"){
    return WinterBetrieb ? "selected" : "";
  }else if(var == "BETRIEBBOILER"){
    return !WinterBetrieb && BoilerBetrieb ? "selected" : "";
  }else if(var == "BETRIEBHEIZUNG"){
    return !WinterBetrieb && !BoilerBetrieb && NurHeizung ? "selected" : "";
  }else if(var == "BETRIEBAUS"){
    return !WinterBetrieb && !BoilerBetrieb && !NurHeizung ? "selected" : "";
  }else if(var == "RAUMTAG"){
    return String(tRoomTag, 1);
  }else if(var == "RAUMNACHT"){
    return String(tRoomNacht, 1);
  }else if(var == "MODEAUTO"){
    return regelungsModus == REGELUNG_AUTO ? "selected" : "";
  }else if(var == "MODERAUM"){
    return regelungsModus == REGELUNG_RAUM ? "selected" : "";
  }else if(var == "MODEAUSSEN"){
    return regelungsModus == REGELUNG_AUSSEN ? "selected" : "";
  }else if(var == "REGELUNGSART"){
    if(regelungsModus == REGELUNG_AUTO){
      return AussentemperaturRegelung ? "AUTO / AUSSENTEMPERATUR" : "AUTO / RAUMTEMPERATUR";
    }
    return regelungsModus == REGELUNG_RAUM ? "MANUELL / RAUMTEMPERATUR" : "MANUELL / AUSSENTEMPERATUR";
  }else if(var == "KACHELOFENEIN"){
    return String(kachelofenEinTemp, 1);
  }else if(var == "KACHELOFENAUS"){
    return String(kachelofenAusTemp, 1);
  }else if(var == "WIFIRSSI"){
    return readValue(WiFi.RSSI());
  }else if(var == "WIFISSID"){
    return ssid; 
  }
  return String();
}
//#################################################################################################################################

String readTemperature(const float& var){
  const float* readings[5] = {&tKessel, &tVorlauf, &tAussen, &tRoom, &tBoiler};
  for (byte i = 0; i < 5; ++i) {
    if (&var == readings[i] && !sensorIsUsable(i)) return "Sensorfehler";
  }
    if (!isfinite(var)) {
    return "--";
  }
  else {
    return String(var);
  }
}

//#################################################################################################################################

String readValue(const int& var){
    if (isnan(var)) {    
    return "--";
  }
  else {
    return String(var);
  }
}

