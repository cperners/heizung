//updates
//2023-10-15 Mischer Zu - sonst heizt Raum mit. -> BoilerBetrieb();

//ende updates

/*
https://github.com/radames/NTP_RTC_Sync/blob/master/NTP_RTC_Sync.ino
https://github.com/radames/NTP_RTC_Sync
https://github.com/JChristensen/Timezone/blob/master/examples/HardwareRTC/HardwareRTC.ino
https://wiki.ta.co.at/Heizkreisregelung_(Funktion)
2020-01-24 chckBoiler() geändert - Kesseltemperatur war kleiner als Boilertemperatur
           chckKessel() geändert - nKesselDiff um beim BoilerAufheizen eine höhere Temperatur zu haben
https://github.com/fedorweems/YouTube/blob/Arduino-Game-V1/ESP8266%20Home%20Automation%20MQTT%20-%20Arduino
*/
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <math.h>
#include <WiFi.h>
#include <ESP32Ping.h>
#include <esp_task_wdt.h>
#include <RTClib.h> //RTC Clock einbinden
#include <AsyncTCP.h>
#include <AsyncMqttClient.h>
#include <LiquidCrystal_I2C.h> //This library you can add via Include Library > Manage Library >
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <OneWire.h> //fuer DS18B20
#include <EEPROM.h>
#include <Keypad_I2C.h>
#include <Ticker.h>
#include <Timezone.h>
#include <Update.h>
#include <ArduinoJson.h>
//define externe Mischersteuerung per I2c
#define EXMISCHER

//3 seconds WDT
#define WDT_TIMEOUT 200

#define SECRET_SSID "bigheat"
#define SECRET_PASS "password"
//#define SECRET_SSID "MyAP4Me"
//#define SECRET_PASS "dasisteintest"

#define MQTT_CLIENT_ID "ESP32_GasHeizung_V0.01"
#define MQTT_USERNAME "heizung"
#define MQTT_PASSWORD "password"
#define MQTT_HOST IPAddress(192, 168, 0, 1)
#define MQTT_PORT 1883

//define PIN für DS18B20
#define VORLAUFTEMP 25
#define AUSSENTEMP 26
#define KUECHENTEMP 27
#define KESSELTEMP 32
#define BOILERTEMP 33
//https://github.com/JChristensen/Timezone/tree/master/examples/Change_TZ_1
#define MYSERIAL 0  //Serial mit 1 einschalten
#define BAUD_RATE 115200
#define MISCHER_WAIT 30
#define MISCHER_DRIVE 5
#define MISCHER_INIT_TIME_ZU 6000
#define MISCHER_INIT_TIME_AUF 10000
//define Ausgänge
#define MISCHER_AUF_PIN 18
#define MISCHER_ZU_PIN 2
#define BRENNER_PIN 16
#define HEIZUNG_PIN 17
#define BOILER_PIN 4

#define lcd_addr 0x27
#define keypad_addr 0x20
#define ioextender0_addr 0x22
//A0-A1-A2 dip switch to off position

#define MQTT_TEXT "/SmartHome/Keller/Heizung/"
/*  https://haus-automatisierung.com/nodered/2017/12/13/node-red-tutorial-reihe-part-4-verbindung-fhem.html */

#define MENUPAGE_TEMPERATUR 1 ... 6
#define MENUPAGE_TEMPERATUR1 14 ... 17

//zwischen Menupage 1 unt 5 soll *C angezeigt werden
#define MENUPAGE_NUM 7 ... 9
//zwischen Menaupage 6 und 8 soll nur die Zahl angezeigt werden
#define MENUPAGE_TIME 10 ... 12
//zwischen Menupage 9 und 11 soll HH.M angezeigt werden /float
#define MENUPAGE_SZ 13
#define MENUPAGE_NUM_AT 18
#define MENUPAGE_MAX 18
/* *******************************************************************************************************
                                         EEPROM
******************************************************************************************************* */
#define EEADDRESS_BOILER 0
#define EEADDRESS_RAUM 8
#define EEADDRESS_KESSEL 16
#define EEADDRESS_DIFFRAUM 24
#define EEADDRESS_DIFFKESSEL 32
#define EEADDRESS_RAUMNACHT 40
#define EEADDRESS_TAG 48
#define EEADDRESS_NACHT 56
#define EEADDRESS_DIFFBOILER 64
#define EEADDRESS_BOILER_SOMMERBETRIEB 72
#define EEADDRESS_NUR_HEIZUNG 80
#define EEADDRESS_BR_LAUFZEIT 88
#define EEADDRESS_SOMMERZEIT_EINAUS 96
#define EEADDRESS_WINTER 104
#define EEADDRESS_TVMAX 112
#define EEADDRESS_TAUMIN 120
#define EEADDRESS_NN 128
#define EEADDRESS_AUSSENTEMPREGELUNG 136
#define EE_SIZE EEADDRESS_AUSSENTEMPREGELUNG+8


#define KESSEL_MIN_TEMP 30.0
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
                                         Zeit via RTC
******************************************************************************************************* */
RTC_DS1307 rtc;
byte dayticker_hr=0;

/* *******************************************************************************************************
                                         Netzwerk
******************************************************************************************************* */
//https://github.com/micw/ArduinoProjekte/blob/master/HeizungsSteuerung/HeizungsSteuerung.ino
#define WIFI_RECON_TIMER 2000
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
float tmyRoomdest;//Zieltemperature je nach Tageszeit
byte WinterBetrieb,BoilerBetrieb,NurHeizung;
int AussentemperaturRegelung,AussentemperaturRegelungAlt = 0;
byte Pumpenloesen = 0;
byte PumpenNachlauf = 0;
//Pumpenloesen von 18:25 bis 18:27
#define PUMPENL_HR 18
#define PUMPENL_MIN_B 25
#define PUMPENL_MIN_E 27
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

unsigned int jumptoDefault = 0;
char jump = '0';
#define OS0 0.00           // Offset Temp Sensor 1 (alle Offsets bitte mit allen Temp.Sensoren abgleichen!)
#define OS1 0.00           // Offset Temp Sensor 1 (alle Offsets bitte mit allen Temp.Sensoren abgleichen!)
#define OS2 0.00           // Offset Temp Sensor 2
#define OS3 0.00          // Offset Temp Sensor 3
#define OS4 0.00            // Offset Temp Sensor 4
/* *******************************************************************************************************
                                              Zeit
******************************************************************************************************* */
float TagBegin,NachtBegin;
volatile int TagBeginHr, TagBeginMi, NachtBeginHr, NachtBeginMi;
/* *******************************************************************************************************
                                              mqtt
******************************************************************************************************* */
#define MQTT_RECON_TIMER 2000
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
char mqtt_payload[10];
bool mqtt2update=false;
int mqtt_message=0;
/* *******************************************************************************************************
                                               OTA
******************************************************************************************************* */
char TimeString[20];
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>ESP Web Server</title>
  <meta http-equiv="Refresh" content="5">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="icon" href="data:,">
  <style>
  html {
    font-family: Arial, Helvetica, sans-serif;
    text-align: center;
  }
  h1 {
    font-size: 1.8rem;
    color: white;
    font-weight: bold;
  }
  h2{
    font-size: 1.5rem;
    font-weight: normal;
    color: white;
  }
  h3{
    font-size: 1.2rem;
    font-weight: lighter;
    color: #e7e7e7;
  }
  .topnav {
    overflow: hidden;
    background-color: #143642;
  }
  body {
    margin: 0;
  }
  .content {
    padding: 30px;
    max-width: 600px;
    min-width: 300px;
    margin: 0 auto;
  }
  .card {
    background-color: #F8F7F9;;
    box-shadow: 2px 2px 12px 1px rgba(140,140,140,.5);
    padding-top:10px;
    padding-bottom:20px;
  }
  .button {
    padding: 13px 40px;
    font-size: 22px;
    text-align: center;
    outline: none;
    color: #fff;
    background-color: #0f8b8d;
    border: none;
    border-radius: 5px;
    -webkit-touch-callout: none;
    -webkit-user-select: none;
    -khtml-user-select: none;
    -moz-user-select: none;
    -ms-user-select: none;
    user-select: none;
    -webkit-tap-highlight-color: rgba(0,0,0,0);
   }
   /*.button:hover {background-color: #0f8b8d}*/
   .button:active {
     background-color: #0f8b8d;
     box-shadow: 2 2px #CDCDCD;
     transform: translateY(2px);
   }
   .state {
     font-size: 1.5rem;
     color:#8c8c8c;
     font-weight: bold;
   }
   p { font-size: 2.6rem; }
    .units { font-size: 1.2rem; }
    .dht-labels{
      font-size: 1.5rem;
      vertical-align:middle;
      padding-bottom: 15px;
   }
   .layout {
  display: grid;
  grid-template-rows: repeat(auto-fit, 2fr);
  grid-template-columns: repeat(5, 2fr);
  gap: 12px 8px;
  }
  </style>
<title>ESP32 Heizungs-Server</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<link rel="stylesheet" href="https://use.fontawesome.com/releases/v5.7.2/css/all.css" 
        integrity="sha384-fnmOCqbTlWIlj8LyTjo7mOUStjsKC4pOpQbqyi7RrhN7udi9RwhKkMHpvLbHG9Sr" crossorigin="anonymous">
<link rel="icon" href="data:,">
</head>
<body>
  <div class="topnav">
    <h1>ESP32 Heizung-Steuerung<hr>%TimeString%<hr>%MQTTUPDATE%</h1>
  </div>
  <section class="layout">
  <div class="content">
    <div class="card">
      <h3>Heizungspumpe</h3>
      <p class="state">state: <span id="state">%HEIZUNGSPUMPE%</span></p>
      <p><button id="button" class="button">Toggle</button></p>
    </div>
  </div>
  <div class="content">
    <div class="card">
      <h3>Boilerpumpe</h3>
      <p class="state">state: <span id="state">%BOILERPUMPE%</span></p>
      <p><button id="button" class="button">Toggle</button></p>
    </div>
  </div>
  </section>
  <p>
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
  <div>WLAN-SSID: <span id="wifi_rssi">%WIFISSID%</span></div>
  <div>WLAN-Signal: <span id="wifi_rssi">%WIFIRSSI%</span></div>
  <div><input type="range" onchange="updateSliderTimer(this)" id="wifi_rssis" min="-100" max="-10" value="%WIFIRSSI%" step="1" class="slider2"></div>
  </section>
  <p>
<script>
  var gateway = `ws://${window.location.hostname}/ws`;
  var websocket;
  window.addEventListener('load', onLoad);
  function initWebSocket() {
    console.log('Trying to open a WebSocket connection...');
    websocket = new WebSocket(gateway);
    websocket.onopen    = onOpen;
    websocket.onclose   = onClose;
    websocket.onmessage = onMessage; // <-- add this line
  }
  function onOpen(event) {
    console.log('Connection opened');
  }
  function onClose(event) {
    console.log('Connection closed');
    setTimeout(initWebSocket, 2000);
  }
  function onMessage(event) {
    var state;
    if (event.data == "1"){
      state = "ON";
    }
    else{
      state = "OFF";
    }
    document.getElementById('state').innerHTML = state;
  }
  function onLoad(event) {
    initWebSocket();
    initButton();
  }
  function initButton() {
    document.getElementById('button').addEventListener('click', toggle);
  }
  function toggle(){
    websocket.send('toggle');
  }

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
</script>
</body>
</html>)rawliteral";

/* *******************************************************************************************************
                                               NTP
******************************************************************************************************* */
//NTPClient
#define Z_DIFF 2 //bei Zeitdifferenz von 2 Sekunden NTP zu RTC updaten
//GMT Time Zone with sign
#define GMT_TIME_ZONE +1
//Force RTC update and store on EEPROM
//change this to a random number between 0-255 to force time update
#define FORCE_RTC_UPDATE 27
#define NTP_UPDATE 30000
#define NTP_UPDATE_HOUR 3

//closest NTP Server
//#define NTP_SERVER "0.at.pool.ntp.org"
#define NTP_SERVER "192.168.0.1"
const long utcOffsetInSeconds = 3600;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTP_SERVER, GMT_TIME_ZONE*utcOffsetInSeconds, 60000);
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
#define ANSWER_TIME   1100UL
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
LiquidCrystal_I2C lcd(lcd_addr, 20, 4); //0x3F wird ersetzt - Davor den i2c scanner laufen lassen!!! 16 Zeichen, 2 Zeilen
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

#define POWER_MODE 0 //power mode: 0 - external, 1 - parasitic
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
#define BOILER_NUMBER 4
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

int TReading[kTtureSensorMaxIndex], SignBit[kTtureSensorMaxIndex],
        Whole[kTtureSensorMaxIndex],Fract[kTtureSensorMaxIndex];
//Whole[] holds the ABSOLUTE VALUE of the integer part
//  of the reading. E.g. for either 12.7 r -12.7, Whole
//  holds 12. (SignBit[] tells you if it is +ve or -ve.)

float fTc_100[kTtureSensorMaxIndex];


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
void connectToWifi();
void rtcconnect();
void OneWireReset(int Pin);
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
void BoilerAnforderungf();
void MenuBetriebBoiler();
void MenuBetriebNurHeizung();
void MenuHeizbeginnTag();
void MenuHeizEndeNacht();
void Time2LCD();
void SetOutPin();
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
void setup() {
  Serial.begin(BAUD_RATE);
  esp_task_wdt_init(WDT_TIMEOUT, true); //disable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch

  pinMode(BrennerPin, OUTPUT);
  pinMode(HeizungPin, OUTPUT);
  pinMode(BoilerPin, OUTPUT);
  pinMode(MischerAuf_Pin, OUTPUT);
  pinMode(MischerZu_Pin, OUTPUT);

  EEPROM.begin(EE_SIZE);
//EEPROM.put() nur 1x beim 1. initialisieren der Variablen auskommentieren
 //hier anfang
 /*
  EEPROM.put(EEADDRESS_BOILER, eeBoiler);
  EEPROM.put(EEADDRESS_RAUM, eeRaum);
  EEPROM.put(EEADDRESS_KESSEL, eeKessel);
  EEPROM.put(EEADDRESS_DIFFRAUM, eeDiffRaum);
  EEPROM.put(EEADDRESS_DIFFKESSEL, eeDiffKessel);
  EEPROM.put(EEADDRESS_DIFFBOILER, eeDiffBoiler);
  EEPROM.put(EEADDRESS_RAUMNACHT, eeRaumNacht);
  EEPROM.put(EEADDRESS_TAG, eeTag);
  EEPROM.put(EEADDRESS_NACHT, eeNacht);
  EEPROM.put(EEADDRESS_WINTER, eeWinter);
  EEPROM.put(EEADDRESS_BOILER_SOMMERBETRIEB, eeBoilerBetrieb);
  EEPROM.put(EEADDRESS_NUR_HEIZUNG, eeNurHeizung);
  EEPROM.put(EEADDRESS_BR_LAUFZEIT, eeBrennerLaufzeit);
  EEPROM.put(EEADDRESS_SOMMERZEIT_EINAUS, eeSommerzeit_EinAus);
  EEPROM.put(EEADDRESS_TVMAX, eetvmax);
  EEPROM.put(EEADDRESS_TAUMIN, eetaumin);
  EEPROM.put(EEADDRESS_NN, een);
  EEPROM.put(EEADDRESS_AUSSENTEMPREGELUNG, eeAuTempRegel);
  EEPROM.commit();
*/
 // hier ende
  delay(6000);//Wait for newly restarted system to stabilize

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
  EEPROM.get( EEADDRESS_TAG, TagBegin );
  TagBeginHr = (int)(TagBegin);
  TagBeginMi = (int)((TagBegin - TagBeginHr)*100);
  Serial.println( TagBegin, 2 );
  Serial.print("Tag - \n");
  Serial.println( TagBeginHr );
  Serial.print(":");
  Serial.println( TagBeginMi );
  Serial.print("...\n");
  EEPROM.get( EEADDRESS_NACHT, NachtBegin );
  NachtBeginHr = (int)(NachtBegin);
  NachtBeginMi = (int)((NachtBegin - NachtBeginHr)*100);
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
  delay(6000);
  Wire.begin(); //I2C ESP32 -> SDA (default is GPIO 21), SCL (default is GPIO 22)
// i2c ioextender
  ioextender0_indicate = 0b11111111;
  I2C_IO_Init(ioextender0_addr,ioextender0_indicate);
//
  lcd.begin();
  lcd.setBacklight(HIGH);
  //MenueBooting();
  I2C_Keypad.begin();
  delay(1000);
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
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }
   Serial.print("Connecting to :");
   Serial.println(ssid);
   WiFi.onEvent(WiFiEvent);
   WiFi.mode(WIFI_STA);
   WiFi.begin(ssid, password);
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
   
   timeClient.begin();
   timer0_time.start();
   timer1_2lcd.start();
   timer2_ntp.start();
   timer3_mqttupdate.start();
  serial_clear_screen();
  initWebSocket();
  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });
  // Start ElegantOTA
  AsyncElegantOTA.begin(&server);
  server.begin();
  esp_task_wdt_reset(); //watchdog Zeit rücksetzen
  serial_clear_screen();
}//end of setup()


/* *******************************************************************************************************
                                         MAIN LOOP
******************************************************************************************************* */
void loop(){
  unsigned long currentTime;
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
  if (!rtc.isrunning()){
    rtcconnect(); 
  }
  currentTime = millis();
  esp_task_wdt_reset(); //watchdog Zeit rücksetzen
  if(readSensor == false){
    //ds_alt    readTturePt1(tture[bWhichSensor]);//N.B.: Values passed back in globals
    if (bWhichSensor==BOILER_NUMBER){readTturePt1(tture[bWhichSensor]);}
    else{
      if(setup_sensorDS1820){sensorDS1820_indicateChip(bWhichSensor); setup_sensorDS1820--;}//Setup nur einmal beim Start ausführen
      sensorDS1820_reset(bWhichSensor);
   } //ds_neu
    readSensor = true;
  }
  if (currentTime - previousTime > ANSWER_TIME){ //delay 1900
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
printTture();}
else {sensorDS1820_read(bWhichSensor);} //ds_neu
      delay(5);//war 50
      bWhichSensor++;
      readSensor = false;
      if (wait_for_connect>0){
        wait_for_connect--;
        if(!MenuPage)Serial.println((String)timeClient.getEpochTime()+", "+timeClient.getFormattedTime());
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
          if(Sommerzeit_EinAus && !i_char){
              if (summertime_EU( myyear, mymonth, myday, myhours, GMT_TIME_ZONE)==true){
                myhours--;
              }
          }
          if(!Sommerzeit_EinAus && i_char){
              if (summertime_EU( myyear, mymonth, myday, myhours, GMT_TIME_ZONE)==true){
                myhours++;
              }
          }
          Sommerzeit_EinAus=i_char;
          EEPROM.put(EEADDRESS_SOMMERZEIT_EINAUS, i_char);
          lcd.print(" ->");
          lcd.print(i_char);
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
            TagBeginMi = (int)((TagBegin - TagBeginHr)*100);
            EEPROM.put(EEADDRESS_TAG, f_char);
            lcd.print(" ->");
            lcd.print(f_char);
            break;
          case 11:
            NachtBegin=f_char;
            NachtBeginHr = (int)(NachtBegin);
            NachtBeginMi = (int)((NachtBegin - NachtBeginHr)*100);
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
  char float_str[8];
  char line0[21];
  dtostrf(tRoomTag,4,2,float_str);
  sprintf(line0, "TempNow: %-9sC", float_str); // %6s right pads the string
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
  char float_str[8];
  char line0[21];
  dtostrf(tBoilerDest,4,2,float_str);
  sprintf(line0, "TempNow: %-9sC", float_str);
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
  char float_str[8];
  char line0[21];
  dtostrf(tKesselDest,4,2,float_str);
  sprintf(line0, "TempNow: %-9sC", float_str);
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
  char float_str[8];
  char line0[21];
  dtostrf(tRoomDiff,4,2,float_str);
  sprintf(line0, "TempNow: %-9sC", float_str);
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
  char float_str[8];
  char line0[21];
  dtostrf(tKesselDiff,4,2,float_str);
  sprintf(line0, "TempNow: %-9sC", float_str);
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
  char float_str[8];
  char line0[21];
  dtostrf(tRoomNacht,4,2,float_str);
  sprintf(line0, "TempNow: %-9sC", float_str);
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
  sprintf(line0, "WinterBetr: %d", WinterBetrieb);
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
  sprintf(line0, "BoilerBetrieb: %d", BoilerBetrieb);
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
  sprintf(line0, "NurHeizBetr: %d", NurHeizung);
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
  char float_str[8];
  char line0[21];
  dtostrf(TagBegin,4,2,float_str);
  sprintf(line0, "ZeitNow: %-5s", float_str);
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
  char float_str[8];
  char line0[21];
  dtostrf(NachtBegin,4,2,float_str);
  sprintf(line0, "ZeitNow: %-5s", float_str);
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
  char float_str[8];
  char line0[21];
  dtostrf(tBoilerDiff,4,2,float_str);
  sprintf(line0, "TempNow: %-9sC", float_str);
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
  char float_str[8];
  char line0[21];
  dtostrf(tvmax,4,2,float_str);
  sprintf(line0, "TempNow: %-9sC", float_str);
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
  char float_str[8];
  char line0[21];
  dtostrf(taumin,4,2,float_str);
  sprintf(line0, "TempNow: %-9sC", float_str);
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
  char float_str[8];
  char line0[21];
  dtostrf(n,4,2,float_str);
  sprintf(line0, "FaktorNow: %-9s", float_str);
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
  char float_str[8];
  char line0[21];
  dtostrf(n,4,2,float_str);
  sprintf(line0, "Ja(1)/Nein(0): %d", AussentemperaturRegelung);
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
    sprintf(line0, "UP: %ld, %02d:%02d%c%02d", uDay,uHour,uMinute,doppelp,uSecond);
    lcd.print(line0);
    lcd.setCursor(0, 1);
    lcd.printf("H%ld A%c %s %d",BrennerLaufzeit,Betriebsart,mqtt_payload,mqtt_message );
    lcd.setCursor(0, 2);
    lcd.printf("UpdTime:%ld, wr:%d",timeUpdated,wifi_retry);
    if(rtc.isrunning()){sprintf(line0, "RTC OK");}else{sprintf(line0, "NO RTC");}
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
  sprintf(line0, "Sommerzeit: %d", Sommerzeit_EinAus);
  lcd.print(line0);
  memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
  jumptoDefault=10;
}
/* *******************************************************************************************************
                                         Schaltuhr
******************************************************************************************************* */
void Schaltuhr(){
  //Tag-Nacht Betrieb)
  long mytime = (100*myhours)+myminutes;
  long t_begin = (100*TagBeginHr)+TagBeginMi;
  long t_ende = (100*NachtBeginHr)+NachtBeginMi;
  if (mytime > t_begin && mytime < t_ende){
    daynight='D';
  }else{
    daynight='N';
  }
}
/* *******************************************************************************************************
                                         mytime update - every seconds
******************************************************************************************************* */
void setmyuhr(){
  set2myuhr=true;
}
void myuhr(){
  mysecunds++;
  if (mysecunds==60){
    myminutes++;
    mysecunds=0;
    ntpupdate();
    Schaltuhr();
  }
  if (myminutes==60){myhours++; myminutes=0;}
  if (myhours==24){myday++; myweekday++;myhours=0;}
  if (myweekday==7){ntpupdate(); Schaltuhr();}
  if(BrennerRelais==true){brsecunds++;}
  if (brsecunds==60){brminutes++; brsecunds=0;}
  if (brminutes==60){brhours++; brminutes=0;}
  if (brhours%2 == 0){
  //  EEPROM.put(EEADDRESS_BR_LAUFZEIT, brhours);
  //  EEPROM.commit();
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
                                         NTP update + RTC check if connected
******************************************************************************************************* */
void ntpupdate() {
  long actualTime=0;
  unsigned int differenz = Z_DIFF;
  DateTime now;
   if (!rtc.isrunning()) {
     rtc.begin();
   } 
   now = rtc.now();
  //}
  if(timeClient.update()){//if 20200204
    actualTime = timeClient.getEpochTime();
    if ((now.unixtime()<(actualTime-differenz)) || (now.unixtime()>(actualTime+differenz))){
       rtc.adjust(DateTime(actualTime));
      timeUpdated++;
    }
  }
  myhours = now.hour();
  myminutes = now.minute();
  mysecunds = now.second();
  myday = now.day();
  mymonth = now.month();
  myyear = now.year();
  myweekday = now.dayOfTheWeek();
  if(summertime_EU( myyear, mymonth, myday, myhours, GMT_TIME_ZONE) && Sommerzeit_EinAus)myhours++;
  char tempString[20];
  char dd=':';
  if(now.second()%2){dd=' ';}
  sprintf(tempString, "%s, %02d:%02d%c%02d", daysOfTheWeek[myweekday],now.hour(), now.minute(), dd, now.second());
  sprintf(TimeString, "%s, %02d:%02d%c%02d", daysOfTheWeek[myweekday],now.hour(), now.minute(), dd, now.second());
  if(!MenuPage){
    lcd.setCursor(0, 3);
    lcd.print(tempString);
    Serial.print(tempString);
  }
  if(!MenuPage){
    Serial.println((String)"\nTimeDate: "+daysOfTheWeek[timeClient.getDay()]+", "+timeClient.getHours()+":"+timeClient.getMinutes()+":"+timeClient.getSeconds()+"\n");
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
        delay(2000);
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
                                         NTP update + RTC check if connected
******************************************************************************************************* */
void set2mqttupdate(){
  mqtt2update=true;
}
void mqttupdate() {
  String bez;
  char msg[50];
  char line0[20];
  char buffer[2048];
  DynamicJsonDocument doc(2048);
  doc["S0Kessel"].set(tKessel);
  doc["S1Vorlauf"].set(tVorlauf);
  doc["S2Aussen"].set(tAussen);
  doc["S3Kueche"].set(tRoom);
  doc["S4Boiler"].set(tBoiler);
  doc["T0Room"].set(tmyRoomdest);
  doc["T1Boiler"].set(tBoilerDest);
  doc["T2Vorlauf"].set(vorlaufTemperatur);
  sprintf(line0, "%ld,%02d:%02d:%02d", uDay,uHour,uMinute,uSecond);
  doc["U0Uptime"].set(line0);
  sprintf(line0, "%ld:%02d:%02d", brhours,brminutes,brsecunds);
  doc["B0Brenner"].set(line0);
  sprintf(line0, "%02d:%02d", TagBeginHr,TagBeginMi);
  doc["Day"].set(line0);
  sprintf(line0, "%02d:%02d", NachtBeginHr,NachtBeginMi);
  doc["Night"].set(line0);
  sprintf(line0, "%s", BrennerRelais ? "1" : "0");
  doc["BrennerRelais"].set(line0);
  sprintf(line0, "%s", HeizungsRelais ? "1" : "0");
  doc["HeizungsRelais"].set(line0);
  sprintf(line0, "%s", BoilerRelais ? "1" : "0");
  doc["BoilerRelais"].set(line0);
  sprintf(line0, "%s", MischerAufRelais ? "1" : "0");
  doc["MischerAufRelais"].set(line0);
  sprintf(line0, "%s", MischerZuRelais ? "1" : "0");
  doc["MischerZuRelais"].set(line0);
  sprintf(line0, "%02d.%02d.%4d %02d:%02d", myday,mymonth,myyear,myhours,myminutes);
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
    sprintf(line0, mqtt_payload);
    strcpy(mqtt_payload,"");
    doc["answer"].set(line0);
  }
  sprintf(line0, "%c" ,Betriebsart);
  doc["betriebsart"].set(line0);
  serializeJson(doc, buffer);
//  serializeJsonPretty(doc, buffer);
  bez = bez + MQTT_TEXT + "JDATA";
  bez.toCharArray(msg,50);
  asyncMqttClient.publish(msg, 1, true, buffer);
}
/*
void mqttupdate() {
  String bez;
  char msg[50];
  char res[8];
  char line0[20];
    bez = bez + MQTT_TEXT + "S0" + "Kessel";
    dtostrf(tKessel, 6, 2, res);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1,true, res);
    bez="";
    bez = bez + MQTT_TEXT + "S1" + "Vorlauf";
    dtostrf(tVorlauf, 6, 2, res);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, res);
    bez="";
    bez = bez + MQTT_TEXT + "S2" + "Aussen";
    dtostrf(tAussen, 6, 2, res);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, res);
    bez="";
    bez = bez + MQTT_TEXT + "S3" + "Kueche";
    dtostrf(tRoom, 6, 2, res);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, res);
    bez="";
    bez = bez + MQTT_TEXT + "S4" + "Boiler";
    dtostrf(tBoiler, 6, 2, res);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, res);
    bez="";
    bez = bez + MQTT_TEXT + "T0" + "Room";
    dtostrf(tmyRoomdest, 6, 2, res);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, res);
    bez="";
    bez = bez + MQTT_TEXT + "U0" + "Uptime";
    sprintf(line0, "%ld,%02d:%02d:%02d", uDay,uHour,uMinute,uSecond);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "B0" + "Brenner";
    sprintf(line0, "%ld:%02d:%02d", brhours,brminutes,brsecunds);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "Day";
    sprintf(line0, "%02d:%02d", TagBeginHr,TagBeginMi);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "Night";
    sprintf(line0, "%02d:%02d", NachtBeginHr,NachtBeginMi);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "BrennerRelais";
    sprintf(line0, "%s", BrennerRelais ? "1" : "0");
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "HeizungsRelais";
    sprintf(line0, "%s", HeizungsRelais ? "1" : "0");
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "BoilerRelais";
    sprintf(line0, "%s", BoilerRelais ? "1" : "0");
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "Uhrzeit";
    sprintf(line0, "%02d.%02d.%4d %02d:%02d", myday,mymonth,myyear,myhours,myminutes);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "WlanRetryConnect";
    sprintf(line0, "%d", wifi_retry);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "WlanRSSI";
    sprintf(line0, "%d", WiFi.RSSI());
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "vorlaufTemperatur";
    sprintf(line0, "%.2f", vorlaufTemperatur);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "atRegelung";
    sprintf(line0, "%d", AussentemperaturRegelung);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
//    bez="";
//    bez = bez + MQTT_TEXT + "WlanAvgTimeMs";
//    sprintf(line0, "%d", avg_time_ms);
//    bez.toCharArray(msg,50);
//    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "PumpenNachlauf";
    sprintf(line0, "%d", PumpenNachlauf);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
//    bez="";
//    bez = bez + MQTT_TEXT + "MischerAuf";
//    sprintf(line0, "%d", MischerAufRelais);
//    bez.toCharArray(msg,50);
//    asyncMqttClient.publish(msg,1, true, line0);
//    bez="";
//    bez = bez + MQTT_TEXT + "MischerZu";
//    sprintf(line0, "%d", MischerZuRelais);
//    bez.toCharArray(msg,50);
//    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "tvmax";
    sprintf(line0, "%.2f", tvmax);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "taumin";
    sprintf(line0, "%.2f", taumin);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "n";
    sprintf(line0, "%.2f", n);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
//    bez="";
//    bez = bez + MQTT_TEXT + "dataToI2Cextender";
//    sprintf(line0, "%d", dataToI2C);
//    bez.toCharArray(msg,50);
//    asyncMqttClient.publish(msg,1, true, line0);
    if(strcmp(mqtt_payload,"")==0){
    }
    else{
      bez="";
      bez = bez + MQTT_TEXT + "answer"; //Antwort nach Subscribe Message
      sprintf(line0, mqtt_payload);
      strcpy(mqtt_payload,"");
      bez.toCharArray(msg,50);
      asyncMqttClient.publish(msg,1, true, line0);
    }
    bez="";
    bez = bez + MQTT_TEXT + "Betriebsart";
    sprintf(line0, "%c" ,Betriebsart);
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
void OneWireReset(int Pin) // reset.  Should improve to act as a presence pulse
{
   digitalWrite(Pin, LOW);
   pinMode(Pin, OUTPUT); // bring low for 500 us
   delayMicroseconds(550);//500 original
   pinMode(Pin, INPUT);
   delayMicroseconds(500);
}//end  OneWireReset()

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
   /*This part starts the sensor doing a conversion,
       i.e. taking a reading, and storing result inside
       itself.
     This must be given time to complete. The time necessary
       is affected by how sensor is powered, and exact
       sensor time. 1.8 seconds (yes, nearly 1/30th of a minute!)
       should, off the top of my head, be enough, worst case.
     Other things can be happening while this is being
       "given time"... just don't disturb the chip.

     Pass WHICH pin you want to read in "Pin"
     Returns values in... (See global declarations)*/
   OneWireReset(Pin);
   OneWireOutByte(Pin, 0xcc);
   OneWireOutByte(Pin, 0x44); // request temperature conversion,
        //  maintain strong pullup while that is done.
}//end readTturePt1


void readTturePt2(byte Pin, const byte tmp_bWhichSensor){
   /*This part starts asks the sensor for the reading
       it took "a moment ago", arising from the call
       of readTturePt1
     (This should have a "check that CheckSum isn't
       reporting a problem" added to it... or else
       a readTturePt3 should be added to do that.
       Not "necessary", but A Very Good Idea!)
     Pass the pin connected to the sensor you want
       to read in "Pin"
     Returns values in... (See global declarations)*/
   int HighByte,LowByte;
   OneWireReset(Pin);
   OneWireOutByte(Pin, 0xcc);
   OneWireOutByte(Pin, 0xbe);
   LowByte = OneWireInByte(Pin);
   HighByte = OneWireInByte(Pin);
   //At this point, we SHOULD have the two bytes returned by the sensor safely
   //  captured in LowByte and HighByte.
   //Now we turn to combining them into a temperature reading in
   //  more usual units.
   //My work got confused... does this return the tture in
   //  TReading in TENTHS of degree C or HUNDRETHS of degree C?
   //The code down to "end of dubious section"... is quite suspect.
   //Remember, if you have to work on it, that some DS18xx chips encode
   //  the temperatures slightly differently than others. Sigh.
   //I'm not SURE which chip this encoding is right for...
   //I think it is the DS18B20 (and others using the same
   //data format.)
   TReading[tmp_bWhichSensor] = (HighByte << 8) + LowByte;
   SignBit[tmp_bWhichSensor] = TReading[tmp_bWhichSensor] & 0x8000;  // test most sig bit

   if (SignBit[tmp_bWhichSensor]) // negative

   {
      TReading[tmp_bWhichSensor] = (TReading[tmp_bWhichSensor] ^ 0xffff) + 1; // 2's comp
   }

   //At this point, we SHOULD have the two bytes returned by the sensor safely
   fTc_100[tmp_bWhichSensor] = (6.0 * TReading[tmp_bWhichSensor]) + TReading[tmp_bWhichSensor] / 4.0;
   //multiply by (100 * 0.0625) or 6.25
   Whole[tmp_bWhichSensor] = fTc_100[tmp_bWhichSensor] / 100.0;  // separate off the whole
   //number and fractional portions
   //Now remove the negative sign from negative values in Whole...
   if  (Whole[tmp_bWhichSensor] < 0) {Whole[tmp_bWhichSensor]*= -1;};
   Fract[tmp_bWhichSensor] = round((fTc_100[tmp_bWhichSensor]-Whole[tmp_bWhichSensor])* 100);
   bWhichSensor=tmp_bWhichSensor;//durch irgendwas ändert sich der bWhichSensor - Wert. Deshalb setze ich ihn hier wieder richtig.
}//end readTturePt2


void printTture(){//Uses values from global variables.
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
   char buf[temp.length()];
   temp.toCharArray(buf,temp.length());

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
void RoomAnforderungf(){
  if(AussentemperaturRegelung==0){
    if (tRoom < tmyRoomdest){
      RoomAnforderung=true;
    }else{
      RoomAnforderung=false;
    }
  }else{
    if (tVorlauf < vorlaufTemperatur){
      RoomAnforderung=true;
    }else{
      RoomAnforderung=false;
    }
  }
}
/* *******************************************************************************************************
                                         BoilerAnforderung
******************************************************************************************************* */
void BoilerAnforderungf(){
  if(tBoiler < tBoilerDest){
    BoilerAnforderung=1;
  }else{
    BoilerAnforderung=0;
  }
}


/* *******************************************************************************************************
                                         Ausgänge schalten
******************************************************************************************************* */
void SetOutPin(){
  if (HeizungsRelais==true || Pumpenloesen){
    digitalWrite(HeizungPin, HIGH);
   // bitSet(ioextender0_indicate, exHeizung_Pin);
  }else{
    digitalWrite(HeizungPin, LOW);
   // bitClear(ioextender0_indicate, exHeizung_Pin);
  }
  if (BoilerRelais==true || Pumpenloesen){
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

// Begin Berechnung für AT-Regelung
  float ti = tmyRoomdest;
  float tau = tAussen;

  vorlaufTemperatur = ti + (tvmax -ti)*pow(((ti-tau)/(ti-taumin)),(1/n));
  vorlaufTemperatur = ((int)(vorlaufTemperatur*100)) / 100.0;
// Ende Berechnung für AT-Regelung

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

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total){
/*    /SmartHome/Keller/Heizung/setRaumTemp
in FHEM:   set MQTT_SERVER publish /SmartHome/Keller/Heizung/setRaumTemp up
           set MQTT_SERVER publish /SmartHome/Keller/Heizung/setRaumTemp down
*/
  mqtt_message++;
  char new_payload[len+1];
  new_payload[len] = '\0';
  strncpy(new_payload, payload, len);

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
        AussentemperaturRegelung = 0;
        EEPROM.put( EEADDRESS_AUSSENTEMPREGELUNG, AussentemperaturRegelung );
        strcpy(mqtt_payload,"h_tin");
    }
    if(strcmp(new_payload,"tau")==0){
        AussentemperaturRegelung = 1;
        EEPROM.put( EEADDRESS_AUSSENTEMPREGELUNG, AussentemperaturRegelung );
        strcpy(mqtt_payload,"h_tau");
        return;
    }
    if(strstr(new_payload,"newRoomTemp")){ //Raumtemperatur per mqtt setzen
        char* teilstr = strchr((char *)new_payload, ':');
        float tempTemp = atof(teilstr+1);
        tRoomTag = tempTemp;
        tRoomNacht = tempTemp;
        strcpy(mqtt_payload,"h_newRT");
        return;
    }
    if(strstr(new_payload,"newBoilerTemp")){ //Raumtemperatur per mqtt setzen
        char* teilstr = strchr((char *)new_payload, ':');
        float tempTemp = atof(teilstr+1);
        tBoilerDest = tempTemp;
        EEPROM.put( EEADDRESS_BOILER, tBoilerDest );
        strcpy(mqtt_payload,"h_newBT");
        return;
    }
    if(strstr(new_payload,"tvmax")){ //max. Vorlauftemperatur  per mqtt setzen
        char* teilstr = strchr((char *)new_payload, ':');
        float tempTemp = atof(teilstr+1);
        tvmax = tempTemp;
        strcpy(mqtt_payload,"h_tvmax");
        return;
    }
    if(strstr(new_payload,"taumin")){ //minimal Aussentemp per mqtt setzen
        char* teilstr = strchr((char *)new_payload, ':');
        float tempTemp = atof(teilstr+1);
        taumin = tempTemp;
        strcpy(mqtt_payload,"h_tvmin");
        return;
    }
    if(strstr(new_payload,"tn")){ //Steigung per mqtt setzen
        char* teilstr = strchr((char *)new_payload, ':');
        float tempTemp = atof(teilstr+1);
        n = tempTemp;
        strcpy(mqtt_payload,"h_tn");
        return;
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
                                         rtc reconnect
******************************************************************************************************* */
void rtcconnect(){
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    lcd.setCursor(0,2);
    lcd.print("Couldn't find RTC");
  }else{
    lcd.setCursor(0,2);
    lcd.print(" RTC connected!  ");
  }
}
/* *******************************************************************************************************
                                         mqtt reconnect
******************************************************************************************************* */
void onMqttConnect(bool sessionPresent) {
      char subsc[50]=MQTT_TEXT;
      Serial.println("connect to MQTT\n");
      Serial.print("Session present: ");
      Serial.println(sessionPresent);
      strcat(subsc,"set");//nun /SmartHome/Keller/Heizung/setRaumTemp
      asyncMqttClient.subscribe(subsc,1);
      //uint16_t packetIdSub = asyncMqttClient.subscribe("/SmartHome/Keller/Heizung/setRaumTemp", 2);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
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
  char float_str0[8];
  char float_str1[8];

  lcd.setCursor(0, 0);
  lcd.print("                    ");
  sprintf(line1, "Br%sH%sB%sA%sZ%s", BrennerRelais ? "+" : "-", HeizungsRelais ? "+" : "-", BoilerRelais ? "+" : "-", MischerAufRelais ? "+" : "-", MischerZuRelais ? "+" : "-");
  lcd.setCursor(0, 0);
  lcd.print(line1);
   serial_go_home();
   Serial.println(line1);
  lcd.setCursor(12, 0);
  dtostrf(tmyRoomdest,4,1,float_str0);
  sprintf(line0, "%cR=%s",daynight,float_str0);
  lcd.print(line0);
   serial_newline(); 
   Serial.println(line0);
  lcd.setCursor(0, 1);
  lcd.print("                    ");
  lcd.setCursor(0, 1);
  dtostrf(tKessel,4,1,float_str0);
  dtostrf(tVorlauf,4,1,float_str1);
  sprintf(line0, "H:%-5s V:%-5s", float_str0, float_str1); // %6s right pads the string
  lcd.print(line0);
   serial_newline();
   Serial.println(line0);
  sprintf(line0,"A%d",AussentemperaturRegelung);
  lcd.setCursor(16, 1);
  lcd.print(line0);
   serial_newline();
   Serial.println(line0);
  sprintf(line0,"P%d",Pumpenloesen);
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
  dtostrf(tAussen,4,1,float_str0);
  dtostrf(tRoom,4,1,float_str1);
  sprintf(line0, "A:%-5s R:%-5s", float_str0, float_str1); // %6s right pads the string
  lcd.print(line0);
   serial_newline();
   Serial.println(line0);
/*  if(asyncMqttClient.connected()){
    sprintf(line0, "M");
  }else{
    sprintf(line0, "-");
  }
  lcd.setCursor(15, 2);
  lcd.print(line0);
*/
  sprintf(line0, "SZ%d", Sommerzeit_EinAus);
  lcd.setCursor(17, 2);
  lcd.print(line0);
   serial_newline();
   Serial.println(line0);
  memset(line0, 0, sizeof line0);//Der Buffer wird geloescht
  memset(float_str0, 0, sizeof float_str0);
  memset(float_str1, 0, sizeof float_str1);
  dtostrf(tBoiler,4,1,float_str0);
  lcd.setCursor(0, 3);
  lcd.print("                    ");
  lcd.setCursor(0, 3);
  sprintf(line0, "B:%-5s ", float_str0); // %6s right pads the string
  lcd.print(line0);
   serial_newline();
   Serial.println(line0);
  sprintf(line0, "%02d.%02d. %02d%c%02d", myday, mymonth, myhours, doppelp, myminutes); // %6s right pads the string
  lcd.setCursor(8, 3);
  lcd.print(line0);
   serial_newline();
   Serial.println(line0);
  memset(line0, 0, sizeof line0);//Der Buffer wird geloescht
  sprintf(line0, "Mischer wait -> %d", mischer_wait);
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
  Wire.beginTransmission(address);
  Wire.write(0xF);
  //Wire.write(data); //alle Ausgänge ausschalten
  Wire.endTransmission();
}
//#################################################################################################################################
void I2C_IO_BitWrite(uint8_t address, uint8_t data){
  Wire.beginTransmission(address);
  Wire.write(data); //alle Ausgänge ausschalten
  Wire.endTransmission();
  dataToI2C = data;
//  dataToI2C=I2C_IO_ReadInputs(address);
}
uint8_t I2C_IO_ReadInputs(uint8_t address){
  Wire.beginTransmission(address);
  Wire.requestFrom(address, uint8_t(2));
  uint8_t Data_In = Wire.read();
  Wire.endTransmission();
  return Data_In;
}
//#################################################################################################################################
void MischerInit(){
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
  sensorDS1820[pin].reset();
  sensorDS1820[pin].write(0xCC, POWER_MODE);
  sensorDS1820[pin].write(0x44, POWER_MODE);
}
//#################################################################################################################################
void sensorDS1820_read(byte pin)
{
  float temperature;
  //byte bufData[9];
  sensorDS1820[pin].reset();
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
if(OneWire::crc8(data, 8)==data[8]){

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
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    if (strcmp((char*)data, "toggle") == 0) {
      HeizungsRelais = !HeizungsRelais;
      notifyClients();
    }
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
  Serial.println(var);
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
  }else if(var == "TKESSEL"){
    return readTemperature(tKessel);
  }else if(var == "TVORLAUF"){
    return readTemperature(tVorlauf);
  }else if(var == "TBOILER"){
    return readTemperature(tBoiler);
  }else if(var == "TAUSSEN"){
    return readTemperature(tAussen);
  }else if(var == "WIFIRSSI"){
    return readValue(WiFi.RSSI());
  }else if(var == "WIFISSID"){
    return ssid; 
  }
  return String();
}
//#################################################################################################################################

String readTemperature(const float& var){
    if (isnan(var)) {    
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

