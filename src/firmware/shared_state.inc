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
int requestedWebMode = -1;
portMUX_TYPE webSettingsMux = portMUX_INITIALIZER_UNLOCKED;
struct KachelofenThresholdRequest { bool pending; float ein; float aus; };
KachelofenThresholdRequest requestedKachelofenThresholds = {};
int requestedOperatingMode = -1;
struct RoomSetpointRequest { bool pending; float day; float night; uint32_t id; bool hasSchedule; uint16_t dayMinutes; uint16_t nightMinutes; };
struct RoomSaveResult { uint32_t id; byte state; };
RoomSaveResult roomSaveResults[8] = {};
uint32_t nextRoomSaveId = 0;
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
uint32_t lcdClearGeneration = 0;
bool keypadPresent = false;
bool ioExtenderPresent = false;

// Auch print/printf laufen ueber write: ohne Display keine Buszugriffe.
class OptionalLCD : public LiquidCrystal_I2C {
public:
  OptionalLCD(uint8_t address, uint8_t columns, uint8_t rows)
    : LiquidCrystal_I2C(address, columns, rows) {}
  void begin() { if (lcdPresent) LiquidCrystal_I2C::begin(); }
  void clear() { ++lcdClearGeneration; if (lcdPresent) LiquidCrystal_I2C::clear(); }
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
byte sensorFamily[5] = {};
unsigned long sensorFamilyLastAttempt[5] = {};
bool sensorFamilyAttempted[5] = {};
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

