"""Exercise actual GPIO event capture, bounded queue, ACK and replay behavior."""
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[2]
head=r'''
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>
#include <sys/time.h>
using std::string;
using portMUX_TYPE=int;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) ((void)0)
#define portEXIT_CRITICAL(x) ((void)0)
constexpr uint8_t BrennerPin=16,HeizungPin=4,BoilerPin=5,MischerAuf_Pin=18,MischerZu_Pin=2;
int pins[64]={};void digitalWrite(uint8_t p,uint8_t v){pins[p]=v;}int digitalRead(uint8_t p){return pins[p];}
uint64_t now=0;int64_t esp_timer_get_time(){return now*1000;}uint32_t esp_random(){return 1234;}
bool clockValid=false;
int fakeTime(timeval* t,void*){t->tv_sec=clockValid?1800000000:0;t->tv_usec=123000;return 0;}
#define gettimeofday fakeTime
bool changing=false;bool mqttSettingsAreChanging(){return changing;}
#define MQTT_TEXT "/SmartHome/Keller/Heizung/"
void onRelayEventPublished(uint16_t);
struct Client {
 bool live=false,reject=false,early=false;uint16_t next=1;
 std::vector<string> sent;std::vector<uint16_t> ids;
 bool connected(){return live;}
 uint16_t publish(const char* topic,int qos,bool retained,const char* text,int len){
  assert(string(topic)=="/SmartHome/Keller/Heizung/schaltereignis");assert(qos==1&&!retained);
  if(reject)return 0;
  uint16_t id=next++;sent.emplace_back(text,len);ids.push_back(id);
  if(early)onRelayEventPublished(id);
  return id;
 }
} asyncMqttClient;
'''
tests=r'''
void reset(){
 for(auto& p:pins)p=0;relayEventHead=relayEventCount=0;relaySequence=relayLost=0;
 relayStates=0;relayEventsReady=relayResyncNeeded=false;relayTrackedPacket=relayAckedPacket=relayInFlight=0;
 relayConnectionEpoch=relaySentEpoch=0;relaySeenEpoch=UINT32_MAX;relayRetryAfter=0;
 now=0;clockValid=changing=false;asyncMqttClient=Client{};beginRelayEvents();
}
void ack(){onRelayEventPublished(relayInFlight);serviceRelayEvents();}
int main(){
 reset();assert(relayEventCount==1&&relayEvents[0].kind==0&&relayEvents[0].unixMs==0);
 writeRelayOutput(MischerAuf_Pin,1);now=5;writeRelayOutput(MischerAuf_Pin,0);
 assert(relayEventCount==3);assert(relayEvents[1].changed==8&&relayEvents[1].states==8);
 assert(relayEvents[2].changed==8&&relayEvents[2].states==0&&relayEvents[2].uptimeMs==5);
 writeRelayOutput(MischerAuf_Pin,0);assert(relayEventCount==3); // no duplicate steady state
 clockValid=true;now=10;writeRelayOutput(BrennerPin,1);assert(relayEvents[3].unixMs==1800000000123LL);
 writeRelayOutput(HeizungPin,1);writeRelayOutput(BoilerPin,1);writeRelayOutput(MischerZu_Pin,1);
 assert(relayStates==23&&pins[BrennerPin]&&pins[MischerZu_Pin]);
 asyncMqttClient.live=true;serviceRelayEvents();assert(relayInFlight&&asyncMqttClient.sent.size()==1);
 assert(asyncMqttClient.sent.back().find("\"typ\":\"startup\"")!=string::npos);
 for(int i=0;i<50;i++)serviceRelayEvents();assert(asyncMqttClient.sent.size()==1);
 onRelayEventPublished(999);serviceRelayEvents();assert(asyncMqttClient.sent.size()==1);
 ack();assert(relayEventHead==1&&asyncMqttClient.sent.size()==2);
 // Disconnect before ACK keeps the same event identity and original timestamp.
 const string original=asyncMqttClient.sent.back();relayEventsDisconnected();asyncMqttClient.live=false;
 serviceRelayEvents();assert(!relayInFlight&&relayEventHead==1);
 now=10000;asyncMqttClient.live=true;serviceRelayEvents();assert(asyncMqttClient.sent.back()==original);
 while(relayEventCount)ack();assert(!relayInFlight);
 // Rejected enqueue preserves the event and retries at most once a second.
 reset();asyncMqttClient.live=true;asyncMqttClient.reject=true;serviceRelayEvents();assert(relayEventCount==2&&!relayInFlight);
 asyncMqttClient.reject=false;serviceRelayEvents();assert(asyncMqttClient.sent.empty());
 now=1000;serviceRelayEvents();assert(asyncMqttClient.sent.size()==1);while(relayEventCount)ack();
 // An ACK delivered before publish() returns must not wedge the queue.
 reset();asyncMqttClient.live=true;asyncMqttClient.early=true;
 for(int i=0;i<5;i++)serviceRelayEvents();assert(relayEventCount==0);
 // Offline buffer is bounded and records lost transitions; resync has current states.
 reset();for(int i=0;i<130;i++){now++;writeRelayOutput(HeizungPin,(i+1)%2);}
 assert(relayEventCount==128&&relayLost==3&&relayResyncNeeded);
 asyncMqttClient.live=true;serviceRelayEvents();ack();assert(!relayResyncNeeded);
 bool found=false;for(int i=0;i<relayEventCount;i++){
  const auto& e=relayEvents[(relayEventHead+i)%128];if(e.kind==3){found=true;assert(e.states==relayStates);}
 }assert(found);
 while(relayEventCount)ack();assert(asyncMqttClient.sent.back().find("\"verlorenSeitStart\":3")!=string::npos);
 // Configuration test must never send events to the broker under test.
 reset();asyncMqttClient.live=true;changing=true;serviceRelayEvents();assert(asyncMqttClient.sent.empty());
 changing=false;serviceRelayEvents();assert(!asyncMqttClient.sent.empty());
 // GPIO capture keeps a monotonic 64-bit timestamp beyond millis() wraparound.
 now=0x100000005ULL;writeRelayOutput(HeizungPin,1);
 assert(relayEvents[(relayEventHead+relayEventCount-1)%128].uptimeMs==now);
}
'''
with tempfile.TemporaryDirectory() as d:
 p=Path(d)/'test.cpp';b=Path(d)/'test'
 p.write_text(head+(root/'src/firmware/relay_events.inc').read_text()+tests)
 subprocess.run(['clang++','-std=c++11','-fsanitize=address,undefined',str(p),'-o',str(b)],check=True)
 subprocess.run([str(b)],check=True)
# Ensure every actuator write uses the capture path, including the central safety stop.
import re
for f in ['heating','mixer','runtime_support']:
 assert not re.search(r'digitalWrite\((BrennerPin|HeizungPin|BoilerPin|MischerAuf_Pin|MischerZu_Pin),',(root/f'src/firmware/{f}.inc').read_text())
print('MQTT-Schaltereignisse: kurze Impulse, alle Ausgaenge, ACK, Wiederholung, Ueberlauf, Zeitstempel und Brokerwechsel geprueft.')
