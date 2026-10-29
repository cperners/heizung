"""Check real mode orchestration at the independent heating/boiler boundaries."""
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[2]
s=(root/'src/firmware/heating.inc').read_text()
helper=s[:s.index('/* *******************************************************************************************************\n                                         Boilerbetrieb')]
a=s.index('void Heizungsbetrieb()');b=s.index('/* *******************************************************************************************************\n                                         kein Betrieb',a)
heating=s[a:b]
head='''#include <cassert>
#include <cmath>
#include <limits>
using std::isfinite;
bool RoomHeizen=false,KesselHeizen=false,BoilerHeizen=false,HeizungsRelais=false,mischer_init_laeuft=false;
int AussentemperaturRegelung=1;char daynight='D';float tmyRoomdest=0,tRoomTag=22.5,tRoomNacht=21.5,tKessel=20,kessel_min_temp=30;
float tRoom=20;bool roomValid=true;bool sensorIsUsable(int){return roomValid;}
bool fault=false;int rooms=0,boilers=0,stops=0,kesselCalls=0;
bool heatingPumpSensorFault(){return fault;}
void MischerStop(){++stops;}
void SetOutPin(){}
void chckRoom(){++rooms;if(!fault)HeizungsRelais=true;}
void chckBoiler(){++boilers;}
int chckKessel(){++kesselCalls;return tKessel>=48;}
void RoomAnforderungf(){} void BoilerAnforderungf(){}
'''
tests='''int main(){
for(float t : {29.9f,30.f,47.9f,48.f,90.f}){
 tKessel=t; rooms=boilers=0;HeizungsRelais=true;Automatik();
 assert(rooms==(t>=30?1:0));assert(boilers==(t>=48?1:0));assert(HeizungsRelais==(t>=30));
 rooms=0;Heizungsbetrieb();assert(rooms==(t>=30?1:0));
}
// Room mode retains the old gate; boiler calls remain unchanged.
AussentemperaturRegelung=0;tKessel=35;rooms=boilers=0;Automatik();assert(rooms==0 && boilers==0);
tKessel=48;Automatik();assert(rooms==1 && boilers==1);
// Cold boiler does not stop a running pump while room heat is still needed.
tKessel=35;tRoom=20;HeizungsRelais=true;rooms=0;Automatik();assert(HeizungsRelais && rooms==0);
// The same cold boiler cannot start an idle pump.
HeizungsRelais=false;Automatik();assert(!HeizungsRelais && rooms==0);
// Room target remains effective below the old boiler threshold.
HeizungsRelais=true;tRoom=tRoomTag;Automatik();assert(!HeizungsRelais);
HeizungsRelais=true;tRoom=tRoomTag+1;Heizungsbetrieb();assert(!HeizungsRelais);
// Inside the room hysteresis band, keep the previous state.
tRoom=tRoomTag-0.05f;HeizungsRelais=true;Automatik();assert(HeizungsRelais);
// An invalid or non-finite reading must not trigger a new room-target decision.
roomValid=false;tRoom=30;HeizungsRelais=true;Automatik();assert(HeizungsRelais);
roomValid=true;tRoom=std::numeric_limits<float>::quiet_NaN();Automatik();assert(HeizungsRelais);
// Night target applies as well.
daynight='N';tRoom=tRoomNacht;HeizungsRelais=true;Automatik();assert(!HeizungsRelais);daynight='D';
// Turning off heat explicitly can still clear the request below the gate.
RoomHeizen=false;HeizungsRelais=true;tRoom=20;controlSpaceHeating(false);assert(!HeizungsRelais);
// Faults reach the existing sensor/run-on path rather than bypassing it.
AussentemperaturRegelung=1;fault=true;tKessel=20;rooms=0;Automatik();assert(rooms==1);
// Do not interrupt the independent reference movement.
fault=false;mischer_init_laeuft=true;stops=0;Automatik();assert(stops==0);
}
'''
with tempfile.TemporaryDirectory() as folder:
 source=Path(folder)/'test.cpp';binary=Path(folder)/'test'
 source.write_text(head+helper+heating+tests.replace('#unused',''))
 subprocess.run(['clang++','-std=c++11','-include','initializer_list',str(source),'-o',str(binary)],check=True)
 subprocess.run([str(binary)],check=True)
print('Heizkreisfreigabe unter/ab 30 Grad, separate Boilergrenze, Raumregelung und Fehlerrouting geprueft.')
