"""Test real pulse control and special-mode ownership independently of LCD ticks."""
from pathlib import Path
import re,subprocess,tempfile
root=Path(__file__).resolve().parents[2]
heat=(root/'src/firmware/heating.inc').read_text();mix=(root/'src/firmware/mixer.inc').read_text()
def function(s,name):
 a=re.search(r'^void '+name+r'\(',s,re.M).start();b=s.index('{',a)+1;depth=1
 while depth:
  depth+=(s[b]=='{')-(s[b]=='}');b+=1
 return s[a:b]+'\n'
head='''#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
using std::isfinite;using byte=uint8_t;
#define HIGH 1
#define LOW 0
#define MISCHER_WAIT 30
#define MISCHER_DRIVE 5
#define MISCHER_RUHEZONE_GRAD 0.5f
#define MISCHER_AUS_KESSEL_MAX_TEMP 35.f
#define MISCHER_AUS_ZUFAHRT_MINUTEN 2UL
#define MISCHER_BOILER_ZUFAHRT_MINUTEN 2UL
#define MISCHER_BOILER_NACHPRUEFUNG_MINUTEN 10UL
int pins[64]={};const int BrennerPin=16,MischerAuf_Pin=18,MischerZu_Pin=2;
void digitalWrite(int p,int v){pins[p]=v;}
uint32_t now=0;unsigned long millis(){return now;}void delay(int ms){now+=ms;}
bool valid[5]={true,true,true,true,true};bool sensorIsUsable(int i){return valid[i];}
bool UrlaubBetrieb=false,WinterBetrieb=true,BoilerBetrieb=false,NurHeizung=false;
bool HeizungsRelais=true;
bool offHeatRecoveryActive=false,BrennerRelais=false,MischerAufRelais=false,MischerZuRelais=false;
bool mischerPulseActive=false;uint32_t mischerPulseStarted=0;
bool mischer_init_laeuft=false,mischer_init_auf=true,mischer_init_zu=true;
unsigned long previousTime_MischerInit=0,mischer_init_time_auf=10000,mischer_init_time_zu=6000;
int mischer_wait=0,mischer_drive=0,AussentemperaturRegelung=1;
float tVorlauf=25,tKessel=60,restwaermeZielTemp=35;
struct Log{template<class T>void println(T){}}Serial;
void MischerStop();void MischerAuf();void MischerZu();
'''
source=head+mix[mix.index('bool vorlaufFaultOpening'):]+function(heat,'regulateMischerToTarget')
source+='''
void reset(){
 valid[0]=valid[1]=true;WinterBetrieb=true;BoilerBetrieb=NurHeizung=UrlaubBetrieb=false;
 updateMischerOffMode();updateMischerBoilerMode();enforceVorlaufSensorLock();MischerStop();
 now=1000;HeizungsRelais=true;tKessel=60;tVorlauf=25;AussentemperaturRegelung=1;offHeatRecoveryActive=false;
 mischer_wait=mischer_drive=0;mischer_init_laeuft=false;mischer_init_auf=mischer_init_zu=true;
}
int main(){
 reset();regulateMischerToTarget(30);assert(pins[MischerAuf_Pin] && mischerPulseActive);
 now=5999;serviceRegulatedMischerPulse();assert(pins[MischerAuf_Pin]);
 now=6000;serviceRegulatedMischerPulse();assert(!pins[MischerAuf_Pin] && !mischerPulseActive);
 // An inactive heating circuit stops immediately, without another regulator call.
 reset();regulateMischerToTarget(30);AussentemperaturRegelung=0;HeizungsRelais=false;serviceRegulatedMischerPulse();assert(!pins[MischerAuf_Pin]);
 // Combined room control may keep a pulse while its heating pump is active.
 reset();AussentemperaturRegelung=0;regulateMischerToTarget(30);serviceRegulatedMischerPulse();assert(pins[MischerAuf_Pin]);
 now+=5000;serviceRegulatedMischerPulse();assert(!pins[MischerAuf_Pin]);
 // Rest heat keeps curve control even with normal RAUM selected.
 reset();offHeatRecoveryActive=true;AussentemperaturRegelung=0;regulateMischerToTarget(30);
 serviceRegulatedMischerPulse();assert(pins[MischerAuf_Pin]);now+=5000;serviceRegulatedMischerPulse();assert(!pins[MischerAuf_Pin]);
 // Both tolerance boundaries, exact target, and a tiny deviation mean STOP.
 for(float value:{29.5f,30.f,30.5f,30.01f}){reset();tVorlauf=value;regulateMischerToTarget(30);assert(!pins[MischerAuf_Pin] && !pins[MischerZu_Pin]);}
 reset();tVorlauf=30.6f;regulateMischerToTarget(30);assert(pins[MischerZu_Pin]);
 // A new measurement entering the band stops a pulse before its deadline.
 tVorlauf=30.2f;regulateMischerToTarget(30);assert(!pins[MischerZu_Pin]);
 reset();regulateMischerToTarget(30);tVorlauf=31;regulateMischerToTarget(30);assert(!pins[MischerAuf_Pin] && !pins[MischerZu_Pin]);
 // 32-bit millisecond wraparound still gives exactly the normal duration.
 reset();now=0xfffffff0U;regulateMischerToTarget(30);now+=4999;serviceRegulatedMischerPulse();assert(pins[MischerAuf_Pin]);
 ++now;serviceRegulatedMischerPulse();assert(!pins[MischerAuf_Pin]);
 // Initialization is deliberately longer than a normal five-second pulse.
 reset();mischer_init_auf=mischer_init_zu=false;MischerInit();assert(pins[MischerAuf_Pin] && !mischerPulseActive);
 now+=6000;serviceRegulatedMischerPulse();assert(pins[MischerAuf_Pin]);
 // Direct special movements take ownership from an earlier normal pulse.
 reset();regulateMischerToTarget(30);MischerZu();assert(!mischerPulseActive);now+=5000;serviceRegulatedMischerPulse();assert(pins[MischerZu_Pin]);
 // Sensor-fault opening lasts its own 20 seconds, never only five seconds.
 reset();regulateMischerToTarget(30);valid[1]=false;enforceVorlaufSensorLock();assert(pins[MischerAuf_Pin] && !mischerPulseActive);
 now+=6000;serviceRegulatedMischerPulse();enforceVorlaufSensorLock();assert(pins[MischerAuf_Pin]);
 now+=14000;enforceVorlaufSensorLock();assert(!pins[MischerAuf_Pin]);valid[1]=true;enforceVorlaufSensorLock();
 // AUS closing and boiler-only closing retain their own two-minute duration.
 reset();regulateMischerToTarget(30);WinterBetrieb=false;tKessel=35;updateMischerOffMode();assert(pins[MischerZu_Pin] && !mischerPulseActive);
 now+=6000;serviceRegulatedMischerPulse();assert(pins[MischerZu_Pin]);
 reset();WinterBetrieb=false;BoilerBetrieb=true;updateMischerBoilerMode();assert(pins[MischerZu_Pin] && !mischerPulseActive);
 now+=6000;serviceRegulatedMischerPulse();assert(pins[MischerZu_Pin]);
}
'''
with tempfile.TemporaryDirectory() as d:
 p=Path(d)/'test.cpp';b=Path(d)/'test';p.write_text(source)
 subprocess.run(['clang++','-std=c++11','-include','initializer_list','-fsanitize=address,undefined',str(p),'-o',str(b)],check=True)
 subprocess.run([str(b)],check=True)
print('Mischer: unabhängige Frist, Moduswechsel, Ruhezone, Timerüberlauf und Vorrang der Sonderfahrten geprüft.')
