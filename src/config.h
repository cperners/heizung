#pragma once

// Zentrale Konfiguration der Heizungssteuerung.
// Zugangsdaten stehen separat in secrets.h.
#include <IPAddress.h>
#include "secrets.h"

#define EXMISCHER
#define WDT_TIMEOUT 200



#define VORLAUFTEMP 25
#define AUSSENTEMP 26
#define KUECHENTEMP 27
#define KESSELTEMP 32
#define BOILERTEMP 33
#define MYSERIAL 0  //Serial mit 1 einschalten
#define BAUD_RATE 115200
#define MISCHER_WAIT 30
#define MISCHER_DRIVE 5
#define MISCHER_INIT_TIME_ZU 6000
#define MISCHER_INIT_TIME_AUF 10000
#define MISCHER_AUF_PIN 18
#define MISCHER_ZU_PIN 2
#define BRENNER_PIN 16
#define HEIZUNG_PIN 17
#define BOILER_PIN 4
#define lcd_addr 0x27
#define keypad_addr 0x20
#define ioextender0_addr 0x22

#define MENUPAGE_TEMPERATUR 1 ... 6
#define MENUPAGE_TEMPERATUR1 14 ... 17
#define MENUPAGE_NUM 7 ... 9
#define MENUPAGE_TIME 10 ... 12
#define MENUPAGE_SZ 13
#define MENUPAGE_NUM_AT 18
#define MENUPAGE_MAX 18
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
#define EEADDRESS_KACHELOFEN_EIN 144
#define EEADDRESS_KACHELOFEN_AUS 152
#define EE_SIZE EEADDRESS_KACHELOFEN_AUS+8
#define KESSEL_MIN_TEMP 30.0
#define WIFI_RECON_TIMER 2000
#define PUMPENL_HR 18
#define PUMPENL_MIN_B 25
#define PUMPENL_MIN_E 27
#define OS0 0.00           // Offset Temp Sensor 1 (alle Offsets bitte mit allen Temp.Sensoren abgleichen!)
#define OS1 0.00           // Offset Temp Sensor 1 (alle Offsets bitte mit allen Temp.Sensoren abgleichen!)
#define OS2 0.00           // Offset Temp Sensor 2
#define OS3 0.00          // Offset Temp Sensor 3
#define OS4 0.00            // Offset Temp Sensor 4
#define MQTT_RECON_TIMER 2000
#define GMT_TIME_ZONE +1
#define NTP_UPDATE 30000
#define NTP_UPDATE_HOUR 3
#define NTP_SERVER1 "192.168.0.1"
#define NTP_SERVER2 "0.at.pool.ntp.org"
#define NTP_SERVER3 "1.at.pool.ntp.org"
#define ANSWER_TIME   1100UL
#define POWER_MODE 0 //power mode: 0 - external, 1 - parasitic
#define BOILER_NUMBER 4

#define GAS_MQTT_TOPIC MQTT_TEXT "gaszaehler"
