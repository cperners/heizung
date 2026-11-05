"""Run the actual reconnect service/event handler with a small host-side driver stub."""
from pathlib import Path
import re, subprocess, tempfile
root = Path(__file__).resolve().parents[2]
network = (root/'src/firmware/network.inc').read_text()
support = (root/'src/firmware/runtime_support.inc').read_text()
event = network[:network.index('void connectToMqtt()')]
constants = list(dict.fromkeys(re.findall(r'case (SYSTEM_EVENT_\w+):', event)))
head = '''#include <cassert>
#include <cstdint>
#define WIFI_RECON_TIMER 2000UL
#define WL_CONNECTED 3
#define portENTER_CRITICAL(x) ((void)0)
#define portEXIT_CRITICAL(x) ((void)0)
using WiFiEvent_t=int;
struct WiFiEventInfo_t { struct {uint8_t reason;} wifi_sta_disconnected; };
unsigned long now=0,previousTime_wifi=0;
unsigned long millis(){return now;}
bool wifiRecon=false,mqttRecon=false;
int wifiReconnectMux=0;
uint32_t wifiDisconnectCount=0,wifiReconnectCount=0;
uint8_t wifiLastDisconnectReason=0;
struct Driver {int state=0; uint32_t ip=0; int reconnects=0;
int status(){return state;} uint32_t localIP(){return ip;}
bool reconnect(){++reconnects; return true;} } WiFi;
struct Log { template<class... T> void printf(T...){} template<class T> void println(T){} } Serial;
int mqttRequests=0; void connectToMqtt(){++mqttRequests;}
void connectToWifi();
'''
head += '\n'.join(f'const int {name}={i};' for i,name in enumerate(constants))+'\n'
connect = network[network.index('void connectToWifi() {'):network.index('\nvoid connect()')]
service = support[:support.index('bool saveBoilerSetpoint')]
tests='''int main(){
WiFiEventInfo_t info={};info.wifi_sta_disconnected.reason=202;
WiFiEvent(SYSTEM_EVENT_STA_DISCONNECTED,info);assert(wifiRecon && wifiDisconnectCount==1 && wifiLastDisconnectReason==202);
now=1999;serviceWifiReconnect();assert(WiFi.reconnects==0);
now=2000;serviceWifiReconnect();assert(WiFi.reconnects==1 && wifiReconnectCount==1 && wifiRecon);
now=3999;serviceWifiReconnect();assert(WiFi.reconnects==1);
// Successful connection clears the pending retry and schedules MQTT.
WiFi.state=WL_CONNECTED;WiFi.ip=123;
WiFiEvent(SYSTEM_EVENT_STA_GOT_IP,info);assert(!wifiRecon && mqttRequests==1);
now=5000;serviceWifiReconnect();assert(WiFi.reconnects==1);
// Even a late queued retry cannot disconnect an already recovered connection.
connectToWifi();now+=15000;serviceWifiReconnect();assert(WiFi.reconnects==1 && !wifiRecon);
// If an attempt emits no driver event, another timed retry remains possible.
WiFi.state=0;WiFi.ip=0;connectToWifi();now+=2000;serviceWifiReconnect();assert(WiFi.reconnects==2);
now+=2000;serviceWifiReconnect();assert(WiFi.reconnects==2);
now+=13000;serviceWifiReconnect();assert(WiFi.reconnects==3);
// Lost IP is treated as a reconnect request, with no immediate driver call.
WiFiEvent(SYSTEM_EVENT_STA_LOST_IP,info);assert(wifiRecon && WiFi.reconnects==3);
}
'''
with tempfile.TemporaryDirectory() as directory:
 source=Path(directory)/'test.cpp';binary=Path(directory)/'test';source.write_text(head+connect+event+service+tests)
 subprocess.run(['clang++','-std=c++11',str(source),'-o',str(binary)],check=True)
 subprocess.run([str(binary)],check=True)
print('WLAN-Reconnect: Erfolgsabbruch, spaeter Auftrag, Wartezeit und fehlende Treiberevents geprueft.')
