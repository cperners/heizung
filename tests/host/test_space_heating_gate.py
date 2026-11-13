"""Exercise real mode orchestration and room/curve decisions at the boiler gate."""
from pathlib import Path
import re,subprocess,tempfile
root=Path(__file__).resolve().parents[2]
s=(root/'src/firmware/heating.inc').read_text()
def function(name):
 a=re.search(r'^void '+name+r'\(',s,re.M).start();b=s.index('{',a)+1;depth=1
 while depth:
  depth+=(s[b]=='{')-(s[b]=='}');b+=1
 return s[a:b]+'\n'
head='''#include <cassert>
#include <cmath>
using std::isfinite;
bool UrlaubBetrieb=false,RoomHeizen=false,KesselHeizen=false,BoilerHeizen=false,HeizungsRelais=false;
bool mischer_init_laeuft=false,mischer_init_auf=true,mischer_init_zu=true;
int AussentemperaturRegelung=1;char daynight='D';
float tmyRoomdest=0,tRoomTag=22.5,tRoomNacht=21.5,tKessel=20,kessel_min_temp=30,tRoom=20,tRoomDiff=.1,tVorlauf=25,vorlaufTemperatur=30;
bool fault=false,curveValid=true,heizkurveGueltig=true,RoomAnforderung=false;
int moves=0,boilers=0,stops=0;
bool heatingPumpSensorFault(){return fault;}
bool updateHeatingCurve(bool=false){heizkurveGueltig=curveValid;vorlaufTemperatur=curveValid?30:0;return curveValid;}
void regulateMischerToTarget(float){++moves;}
void MischerStop(){++stops;}void SetOutPin(){}
void chckBoiler(){++boilers;}
int chckKessel(){return tKessel>=48;}
void RoomAnforderungf(){}void BoilerAnforderungf(){}
void chckRoom(bool allowPumpStart=true);
'''
tests='''int main(){
for(float t:{29.9f,30.f,47.9f,48.f,90.f}){
 tKessel=t;moves=boilers=0;HeizungsRelais=false;Automatik();
 assert(HeizungsRelais==(t>=30));assert(moves==(t>=30?1:0));assert(boilers==(t>=48?1:0));
 moves=0;HeizungsRelais=false;Heizungsbetrieb();assert(HeizungsRelais==(t>=30));
}
// In room mode the old boiler gate still controls a new pump start.
AussentemperaturRegelung=0;tKessel=35;tRoom=20;HeizungsRelais=false;moves=boilers=0;
Automatik();assert(!HeizungsRelais && moves==0 && boilers==0);
// A running circuit continues, with mixer regulation, even below that gate.
HeizungsRelais=true;Automatik();assert(HeizungsRelais && moves==1);
// Reaching the room target stops the request and normal mixer movement.
tRoom=tRoomTag;stops=0;Automatik();assert(!HeizungsRelais && stops>0);
tRoom=20;Automatik();assert(!HeizungsRelais);
// Once warm enough, normal room hysteresis can start the pump again.
tKessel=48;moves=0;Automatik();assert(HeizungsRelais && moves==1 && boilers==1);
tRoom=tRoomTag-.05f;HeizungsRelais=true;Automatik();assert(HeizungsRelais);
HeizungsRelais=false;Automatik();assert(!HeizungsRelais);
// The night target is used too.
daynight='N';tRoom=20;Automatik();assert(HeizungsRelais);tRoom=tRoomNacht;Automatik();assert(!HeizungsRelais);daynight='D';
// Invalid curve inhibits normal pumping rather than freezing an old command.
tRoom=20;curveValid=false;HeizungsRelais=true;Automatik();assert(!HeizungsRelais);curveValid=true;
// Fault handling reaches the central output layer and stops normal mixing.
fault=true;stops=0;Automatik();assert(stops>0);fault=false;
// Initialization remains a separate movement and is not stopped here.
mischer_init_laeuft=true;tRoom=tRoomTag;stops=0;Automatik();assert(stops==0);
}
'''
source=head+function('controlSpaceHeating')+function('Automatik')+function('Heizungsbetrieb')+function('chckRoom')+tests
with tempfile.TemporaryDirectory() as d:
 p=Path(d)/'test.cpp';b=Path(d)/'test';p.write_text(source)
 subprocess.run(['clang++','-std=c++11','-include','initializer_list','-fsanitize=address,undefined',str(p),'-o',str(b)],check=True)
 subprocess.run([str(b)],check=True)
print('Raum-Heizbedarf, Heizkurven-Mischer und separate Kesselfreigabe geprüft.')
