"""Exercise actual off-state, curve, pulse, output and sensor-lock functions."""
from pathlib import Path
import re, subprocess, tempfile
root=Path(__file__).resolve().parents[2]
heat=(root/'src/firmware/heating.inc').read_text()
mix=(root/'src/firmware/mixer.inc').read_text()
runtime=(root/'src/firmware/runtime_support.inc').read_text()
def function(source,name):
 a=re.search(r'^(?:void|bool|int) '+re.escape(name)+r'\(',source,re.M).start()
 start=source.index('{',a);depth=1;b=start+1
 while depth:
  depth+=(source[b]=='{')-(source[b]=='}');b+=1
 return source[a:b]+'\n'
head='''#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
using std::isfinite;
using byte=uint8_t;
#define HIGH 1
#define LOW 0
#define MISCHER_AUS_KESSEL_MAX_TEMP 35.0f
#define MISCHER_AUS_ZUFAHRT_MINUTEN 2UL
#define MISCHER_WAIT 30
#define MISCHER_DRIVE 5
#define MISCHER_RUHEZONE_GRAD 0.5f
constexpr uint32_t MISCHER_PULSE_MS = 5000;
constexpr int BrennerPin=16,HeizungPin=4,BoilerPin=5,MischerAuf_Pin=18,MischerZu_Pin=2;
int pins[64]={};void digitalWrite(int p,int v){pins[p]=v;}int digitalRead(int p){return pins[p];}
unsigned long now=0;unsigned long millis(){return now;}
bool valid[5]={true,true,true,true,true};bool sensorIsUsable(byte i){return i<5 && valid[i];}
bool UrlaubBetrieb=false,WinterBetrieb=true,BoilerBetrieb=false,NurHeizung=false;
bool mischerPulseActive=false;uint32_t mischerPulseStarted=0;
bool offHeatRecoveryActive=false,RoomHeizen=true,BoilerHeizen=true,KesselHeizen=true;
bool BrennerRelais=true,HeizungsRelais=false,BoilerRelais=false;
bool MischerAufRelais=false,MischerZuRelais=false,RoomAnforderung=false,BoilerAnforderung=false,BoilerAufheizen=false;
bool mischer_init_laeuft=false,mischer_init_auf=true,mischer_init_zu=true,mischerBoilerMode=false;
unsigned long previousTime_MischerInit=0,mischer_init_time_auf=10000,mischer_init_time_zu=6000;
int mischer_wait=0,mischer_drive=0,Pumpenloesen=0;
float kessel_min_temp=30;
float tBoiler=45.58f,tKessel=60,tVorlauf=25,tAussen=12,tRoom=24,tRoomTag=22.5,tRoomNacht=21.5,tRoomDiff=0.1f;
float tmyRoomdest=22.5,vorlaufTemperatur=0,tvmax=40,taumin=-15,n=1.9,restwaermeZielTemp=35;
bool heizkurveGueltig=false;int AussentemperaturRegelung=0;char daynight='D';
struct Logger{template<class... T>void printf(T...){}template<class T>void println(T){}}Serial;
void delay(int){}
void MischerStop();void MischerAuf();void MischerZu();void SetOutPin();
bool heatingPumpSensorFault();bool boilerPumpSensorFault();bool enforceVorlaufSensorLock();
void runOffHeatRecovery();void regulateMischerToTarget(float);void startRegulatedMischerPulse(bool);
'''
# Include real state declarations for off mode and fault run-on.
a=mix.index('bool vorlaufFaultOpening');b=mix.index('void updateMischerOffMode()',a)
source=head+mix[a:b]
a=heat.index('struct PumpFaultState');b=heat.index('void SetOutPin()',a)
source+=heat[a:b]
for text,name in [(runtime,'enforceKesselSensorLock'),(mix,'updateMischerOffMode'),(mix,'enforceVorlaufSensorLock'),(mix,'MischerAuf'),(mix,'MischerZu'),(mix,'MischerStop'),(mix,'MischerInit'),(mix,'startRegulatedMischerPulse'),(mix,'serviceRegulatedMischerPulse'),(heat,'heatingPumpSensorFault'),(heat,'boilerPumpSensorFault'),(heat,'SetOutPin'),(heat,'updateHeatingCurve'),(heat,'regulateMischerToTarget'),(heat,'runOffHeatRecovery'),(heat,'kein_Betrieb'),(heat,'chckRoom')]:
 source+=function(text,name)
source+='''
void offLoop(){updateMischerOffMode();enforceVorlaufSensorLock();serviceRegulatedMischerPulse();MischerInit();kein_Betrieb();assert(!BrennerRelais && pins[BrennerPin]==LOW);}
void reset(){
 WinterBetrieb=true;BoilerBetrieb=NurHeizung=UrlaubBetrieb=false;
 for(int i=0;i<5;++i)valid[i]=true;
 updateMischerOffMode();enforceVorlaufSensorLock();
 heatingPumpFault={};boilerPumpFault={};
 for(int &p:pins)p=LOW;
 now=1000;restwaermeZielTemp=35;tKessel=60;tVorlauf=25;tAussen=12;tvmax=40;taumin=-15;n=1.9;
 tRoomTag=22.5;tRoomNacht=21.5;daynight='D';AussentemperaturRegelung=0;
 BrennerRelais=true;pins[BrennerPin]=HIGH;HeizungsRelais=BoilerRelais=false;
 mischer_init_laeuft=false;mischer_init_auf=mischer_init_zu=true;
 mischer_wait=mischer_drive=0;Pumpenloesen=0;
 WinterBetrieb=false;
}
int main(){
 reset();offLoop();
 assert(offHeatRecoveryActive && pins[HeizungPin]==HIGH && pins[BoilerPin]==LOW);
 assert(AussentemperaturRegelung==0 && heizkurveGueltig && vorlaufTemperatur>22.5 && vorlaufTemperatur<=40);
 assert(pins[MischerAuf_Pin]==HIGH && pins[MischerZu_Pin]==LOW && !mischer_init_laeuft);
 // Normal pulse ends after the independent one-second countdown ticks.
 now+=5000;mischer_wait=25;offLoop();assert(!pins[MischerAuf_Pin] && !pins[MischerZu_Pin]);
 tVorlauf=45;mischer_wait=0;offLoop();assert(pins[MischerZu_Pin] && !pins[MischerAuf_Pin]);
 // Cold outside still cannot raise the target above the configured maximum.
 tAussen=-50;offLoop();assert(vorlaufTemperatur<=tvmax);
 // At exactly 35 the pump stops and the two-minute close starts.
 tKessel=35;offLoop();assert(!offHeatRecoveryActive && !pins[HeizungPin] && pins[MischerZu_Pin]);
 now+=119999;offLoop();assert(pins[MischerZu_Pin]);
 ++now;offLoop();assert(!pins[MischerZu_Pin] && !pins[MischerAuf_Pin]);
 now+=1000000;offLoop();assert(!pins[MischerZu_Pin] && !pins[HeizungPin]);
 // Renewed heat resumes regulated removal rather than blind closing.
 tKessel=36;tVorlauf=24;offLoop();assert(offHeatRecoveryActive && pins[HeizungPin] && pins[MischerAuf_Pin]);
 // Leaving AUS releases the state without completing a pending close.
 WinterBetrieb=true;updateMischerOffMode();assert(!offHeatRecoveryActive && !mischerOffMode && !pins[MischerAuf_Pin]);
 // A sensor fault never starts a stopped pump.
 reset();valid[2]=false;offLoop();assert(!pins[HeizungPin] && !pins[MischerAuf_Pin]);
 // A running pump retains the existing bounded ten-minute sensor-fault run-on.
 reset();offLoop();valid[2]=false;offLoop();assert(pins[HeizungPin]);
 now+=600000;offLoop();assert(!pins[HeizungPin]);
 // Vorlauf fault keeps the existing 20-second opening, then locks both outputs.
 reset();offLoop();valid[1]=false;offLoop();assert(pins[MischerAuf_Pin]);
 now+=20000;offLoop();assert(!pins[MischerAuf_Pin] && !pins[MischerZu_Pin]);
 // Missing room sensor does not block outside-based rest-heat removal.
 reset();valid[3]=false;offLoop();assert(pins[HeizungPin]);
 // Invalid curve must not drive the mixer or falsely claim a valid target.
 reset();n=0;offLoop();assert(!pins[HeizungPin] && !pins[MischerAuf_Pin] && !heizkurveGueltig);
 // No new burner request can escape the central AUS output guard.
 reset();offLoop();BrennerRelais=true;SetOutPin();assert(!BrennerRelais && !pins[BrennerPin]);
 // Starting cold in AUS closes once without starting the heating pump.
 reset();tKessel=20;offLoop();assert(!offHeatRecoveryActive && !pins[HeizungPin] && pins[MischerZu_Pin]);
 // Saved limits at both ends of the configurable range are honored.
 reset();restwaermeZielTemp=30;tKessel=30.1f;offLoop();assert(offHeatRecoveryActive);
 tKessel=30;offLoop();assert(!offHeatRecoveryActive && !pins[HeizungPin]);
 reset();restwaermeZielTemp=40;tKessel=40.1f;offLoop();assert(offHeatRecoveryActive);
 tKessel=40;offLoop();assert(!offHeatRecoveryActive && !pins[HeizungPin]);
 // Night target is used without switching to room control.
 reset();daynight='N';offLoop();assert(tmyRoomdest==tRoomNacht && pins[HeizungPin]);
 // Boiler heat-flow guard works even if the charging routine was skipped.
 reset();WinterBetrieb=true;tKessel=43;tBoiler=45.58f;BoilerRelais=true;BoilerAufheizen=true;
 SetOutPin();assert(!pins[BoilerPin] && !BoilerRelais && BoilerAufheizen);
 tBoiler=42.9f;BoilerRelais=true;SetOutPin();assert(pins[BoilerPin]);
 tBoiler=43;SetOutPin();assert(!pins[BoilerPin]);
 // Preserve the already agreed bounded sensor-fault run-on.
 tBoiler=42;BoilerRelais=true;SetOutPin();assert(pins[BoilerPin]);
 valid[4]=false;tBoiler=46;SetOutPin();assert(pins[BoilerPin]);
 now+=600000;SetOutPin();assert(!pins[BoilerPin]);
 // A stopped pump still cannot start on an invalid boiler reading.
 reset();WinterBetrieb=true;valid[4]=false;BoilerRelais=true;SetOutPin();assert(!pins[BoilerPin]);
 // Normal room mode needs both room and outside sensors, unlike rest heat.
 reset();WinterBetrieb=true;RoomHeizen=true;tRoom=20;tmyRoomdest=22.5;
 valid[2]=false;assert(heatingPumpSensorFault());
 valid[2]=true;valid[3]=false;assert(heatingPumpSensorFault());
 // A valid room call now controls the mixer on a curve while its pump runs.
 valid[3]=true;mischer_init_auf=mischer_init_zu=true;
 chckRoom(true);assert(pins[HeizungPin] && pins[MischerAuf_Pin]);
 tRoom=tmyRoomdest;chckRoom(true);assert(!pins[HeizungPin] && !pins[MischerAuf_Pin]);
 // Non-finite boiler temperature is never considered cooled.
 reset();valid[0]=false;tKessel=std::numeric_limits<float>::quiet_NaN();offLoop();
 assert(offHeatRecoveryActive && !pins[HeizungPin] && !pins[MischerZu_Pin]);
}
'''
with tempfile.TemporaryDirectory() as folder:
 test=Path(folder)/'test.cpp';binary=Path(folder)/'test';test.write_text(source)
 subprocess.run(['clang++','-std=c++11','-fsanitize=address,undefined',str(test),'-o',str(binary)],check=True)
 subprocess.run([str(binary)],check=True)
print('AUS-Restwaerme: Brennersperre, Heizkurve, 35-Grad-Grenze, Schliessfahrt, Wiedererwaermung und Sensorfehler geprueft.')
