"""Validate target/difference display from the actual source, including faults."""
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[2]
s=(root/'src/firmware/runtime_support.inc').read_text();a=s.index('String vorlaufCalculationDisplay(');b=s.index('String boilerTemperatureDifferenceDisplay()',a)
head='''#include <cassert>
#include <cmath>
#include <cstdio>
#include <string>
#include <limits>
using std::isfinite;using String=std::string;
bool offHeatRecoveryActive=false,UrlaubBetrieb=false,WinterBetrieb=true,NurHeizung=false;
int AussentemperaturRegelung=1;bool heizkurveGueltig=true;
float vorlaufTemperatur=32.5,tVorlauf=34,tvmax=40;
bool valid[4]={true,true,true,true};bool sensorIsUsable(int i){return valid[i];}
'''
test='''int main(){
assert(vorlaufCalculationDisplay(false)=="32.50 °C");assert(vorlaufCalculationDisplay(true)=="+1.50 °C");
tVorlauf=30;assert(vorlaufCalculationDisplay(true)=="-2.50 °C");
tVorlauf=32.5;assert(vorlaufCalculationDisplay(true)=="+0.00 °C");
AussentemperaturRegelung=0;assert(vorlaufCalculationDisplay(false)=="32.50 °C");
offHeatRecoveryActive=true;assert(vorlaufCalculationDisplay(false)=="32.50 °C");
valid[2]=false;assert(vorlaufCalculationDisplay(false)=="Nicht verfuegbar - Sensorfehler");valid[2]=true;
heizkurveGueltig=false;assert(vorlaufCalculationDisplay(true)=="Keine gueltige Berechnung");heizkurveGueltig=true;
vorlaufTemperatur=41;assert(vorlaufCalculationDisplay(false)=="Keine gueltige Berechnung");
vorlaufTemperatur=std::numeric_limits<float>::quiet_NaN();assert(vorlaufCalculationDisplay(false)=="Keine gueltige Berechnung");
offHeatRecoveryActive=false;AussentemperaturRegelung=1;UrlaubBetrieb=true;assert(vorlaufCalculationDisplay(false)=="Nicht aktiv");
UrlaubBetrieb=false;WinterBetrieb=false;assert(vorlaufCalculationDisplay(true)=="Nicht aktiv");
}
'''
with tempfile.TemporaryDirectory() as d:
 source=Path(d)/'test.cpp';binary=Path(d)/'test';source.write_text(head+s[a:b]+test)
 subprocess.run(['clang++','-std=c++11',str(source),'-o',str(binary)],check=True)
 subprocess.run([str(binary)],check=True)
print('Vorlauf-Sollwert, Differenzvorzeichen, Restwaerme und ungueltige Werte geprueft.')
