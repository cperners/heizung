"""Run the actual reconnect service/event handler with a small host-side driver stub."""
from pathlib import Path
import re, subprocess, tempfile
root = Path(__file__).resolve().parents[2]
network = (root/'src/firmware/network.inc').read_text()
support = (root/'src/firmware/runtime_support.inc').read_text()
event = network[:network.index('void serviceMqttReconnect()')]
constants = list(dict.fromkeys(re.findall(r'case (SYSTEM_EVENT_\w+):', event)))
head = '''#include <cassert>
#include <cstdint>
#define WIFI_RECON_TIMER 2000UL
#define MQTT_RECON_TIMER 2000UL
#define WL_CONNECTED 3
#define portENTER_CRITICAL(x) ((void)0)
#define portEXIT_CRITICAL(x) ((void)0)
using WiFiEvent_t=int;
struct WiFiEventInfo_t { struct {uint8_t reason;} wifi_sta_disconnected; };
unsigned long now=0,previousTime_wifi=0;
unsigned long millis(){return now;}
bool wifiRecon=false;
uint32_t wifiLinkGeneration=0;
int wifiReconnectMux=0;
uint32_t wifiDisconnectCount=0,wifiReconnectCount=0;
uint8_t wifiLastDisconnectReason=0;
struct Driver {int state=0; uint32_t ip=0; int reconnects=0;
int status(){return state;} uint32_t localIP(){return ip;}
bool reconnect(){++reconnects; return true;} } WiFi;
struct Log { template<class... T> void printf(T...){} template<class T> void println(T){} } Serial;
int mqttSettingsMux=0;
bool mqttTransportIdle=true, settingsChanging=false, wifiTrial=false;
bool mqttSettingsAreChanging(){return settingsChanging;}
bool wifiTrialInProgress(){return wifiTrial;}
enum class AsyncMqttClientDisconnectReason { TCP_DISCONNECTED };
void onMqttDisconnect(AsyncMqttClientDisconnectReason){mqttTransportIdle=true;}
struct Mqtt {
 bool live=false, reportsDisconnect=true;
 int attempts=0, closes=0;
 bool connected(){return live;}
 void connect(){++attempts;}
 void disconnect(bool force){
  assert(force);++closes;live=false;
  if(reportsDisconnect)onMqttDisconnect(AsyncMqttClientDisconnectReason::TCP_DISCONNECTED);
 }
} asyncMqttClient;
void connectToWifi();
'''
head += '\n'.join(f'const int {name}={i};' for i,name in enumerate(constants))+'\n'
connect = network[network.index('void connectToWifi() {'):network.index('\nvoid connect()')]
service = support[:support.index('bool saveBoilerSetpoint')]
mqtt = network[len(event):network.index('/*',len(event))]
tests='''int main(){
WiFiEventInfo_t info={};info.wifi_sta_disconnected.reason=202;
WiFiEvent(SYSTEM_EVENT_STA_DISCONNECTED,info);assert(wifiRecon && wifiDisconnectCount==1 && wifiLastDisconnectReason==202);
now=1999;serviceWifiReconnect();assert(WiFi.reconnects==0);
now=2000;serviceWifiReconnect();assert(WiFi.reconnects==1 && wifiReconnectCount==1 && wifiRecon);
now=3999;serviceWifiReconnect();assert(WiFi.reconnects==1);
// Successful connection clears the pending retry and schedules MQTT.
WiFi.state=WL_CONNECTED;WiFi.ip=123;
WiFiEvent(SYSTEM_EVENT_STA_GOT_IP,info);assert(!wifiRecon);
serviceMqttReconnect();assert(asyncMqttClient.attempts==1);
now=5000;serviceWifiReconnect();assert(WiFi.reconnects==1);
// Even a late queued retry cannot disconnect an already recovered connection.
connectToWifi();now+=15000;serviceWifiReconnect();assert(WiFi.reconnects==1 && !wifiRecon);
// If an attempt emits no driver event, another timed retry remains possible.
WiFi.state=0;WiFi.ip=0;connectToWifi();now+=2000;serviceWifiReconnect();assert(WiFi.reconnects==2);
now+=2000;serviceWifiReconnect();assert(WiFi.reconnects==2);
now+=13000;serviceWifiReconnect();assert(WiFi.reconnects==3);
// Lost IP is treated as a reconnect request, with no immediate driver call.
WiFiEvent(SYSTEM_EVENT_STA_LOST_IP,info);assert(wifiRecon && WiFi.reconnects==3);
serviceMqttReconnect();assert(asyncMqttClient.closes==1);
now+=2000;serviceMqttReconnect();assert(asyncMqttClient.attempts==1);
WiFi.state=WL_CONNECTED;WiFi.ip=123;
WiFiEvent(SYSTEM_EVENT_STA_GOT_IP,info);
serviceMqttReconnect();assert(asyncMqttClient.attempts==2);
// Broker starts later than Wi-Fi. A failed attempt must be retried.
onMqttDisconnect(AsyncMqttClientDisconnectReason::TCP_DISCONNECTED);
now+=1999;serviceMqttReconnect();assert(asyncMqttClient.attempts==2);
++now;serviceMqttReconnect();assert(asyncMqttClient.attempts==3);
// A connect that never calls back must also be retried.
asyncMqttClient.reportsDisconnect=false;
now+=29999;serviceMqttReconnect();assert(asyncMqttClient.closes==1);
++now;serviceMqttReconnect();assert(asyncMqttClient.closes==2 && mqttTransportIdle);
now+=2000;serviceMqttReconnect();assert(asyncMqttClient.attempts==4);
// Fast router recovery between loop passes still discards the old socket.
asyncMqttClient.live=true;
WiFiEvent(SYSTEM_EVENT_STA_DISCONNECTED,info);
WiFiEvent(SYSTEM_EVENT_STA_GOT_IP,info);
serviceMqttReconnect();assert(asyncMqttClient.closes==3);
now+=2000;settingsChanging=true;
serviceMqttReconnect();assert(asyncMqttClient.attempts==4);
settingsChanging=false;wifiTrial=true;
serviceMqttReconnect();assert(asyncMqttClient.attempts==4);
wifiTrial=false;serviceMqttReconnect();assert(asyncMqttClient.attempts==5);
asyncMqttClient.live=true;
now+=60000;serviceMqttReconnect();assert(asyncMqttClient.closes==3);
}
'''
with tempfile.TemporaryDirectory() as directory:
 source=Path(directory)/'test.cpp';binary=Path(directory)/'test';source.write_text(head+connect+event+service+mqtt+tests)
 subprocess.run(['clang++','-std=c++11',str(source),'-o',str(binary)],check=True)
 subprocess.run([str(binary)],check=True)
print('WLAN/MQTT: Router-Neustart, spaeter Broker, fehlende Callbacks und Einstellungswechsel geprueft.')
