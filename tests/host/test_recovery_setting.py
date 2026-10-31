"""Verify actual transactional save and new-field validation with a host store."""
from pathlib import Path
import re, subprocess, tempfile
root=Path(__file__).resolve().parents[2]
s=(root/'src/firmware/runtime_support.inc').read_text()
def function(name):
 a=re.search(r'^bool '+name+r'\(',s,re.M).start();start=s.index('{',a);b=start+1;depth=1
 while depth:
  depth+=(s[b]=='{')-(s[b]=='}');b+=1
 return s[a:b]+'\n'
config=(root/'src/config.h').read_text()
assert '#define EEADDRESS_RESTWAERME_ZIEL 172' in config
assert '#define EE_SIZE 176' in config
assert 'repairSetting(EEADDRESS_RESTWAERME_ZIEL, float(MISCHER_AUS_KESSEL_MAX_TEMP), 30, 40)' in s
head='''#include <cassert>
#include <cmath>
#include <map>
#include <limits>
using std::isfinite;
const int EEADDRESS_TVMAX=0, EEADDRESS_NN=4, EEADDRESS_RESTWAERME_ZIEL=172;
float tvmax=40,n=1.9,restwaermeZielTemp=35;
struct Store{std::map<int,float> buffer{{0,40},{4,1.9},{172,35}},persisted=buffer;bool ok=true;int commits=0;
void put(int a,float v){buffer[a]=v;}
void get(int a,float &v){v=buffer.count(a)?buffer[a]:std::numeric_limits<float>::quiet_NaN();}
bool commit(){++commits;if(ok)persisted=buffer;return ok;}}EEPROM;
struct Log{template<class...T>void printf(T...){}}Serial;
'''
tests='''int main(){
assert(saveFlowLimit(40,true,n,true,35));assert(EEPROM.commits==0 && restwaermeZielTemp==35);
assert(saveFlowLimit(40,true,n,true,30));assert(restwaermeZielTemp==30 && EEPROM.persisted[172]==30);
assert(saveFlowLimit(42,true,2,true,40));assert(tvmax==42 && n==2 && restwaermeZielTemp==40);
const auto before=EEPROM.buffer;const int count=EEPROM.commits;
float bad[]={29.9f,40.1f,std::numeric_limits<float>::infinity(),std::numeric_limits<float>::quiet_NaN()};
for(float v:bad)assert(!saveFlowLimit(50,true,3,true,v));
assert(EEPROM.buffer==before && EEPROM.commits==count && restwaermeZielTemp==40);
EEPROM.ok=false;assert(!saveFlowLimit(44,true,2.5,true,35));
assert(tvmax==42 && n==2 && restwaermeZielTemp==40 && EEPROM.buffer==before && EEPROM.persisted==before);
EEPROM.ok=true;assert(saveFlowLimit(45));assert(restwaermeZielTemp==40 && n==2);
EEPROM.buffer.erase(172);assert(repairSetting(172,35.f,30,40));assert(EEPROM.buffer[172]==35);
assert(!repairSetting(172,35.f,30,40));
EEPROM.buffer[172]=40;assert(!repairSetting(172,35.f,30,40));
EEPROM.buffer[172]=99;assert(repairSetting(172,35.f,30,40));
}
'''
source=head+function('saveFlowLimit')+'template<typename T>\n'+function('repairSetting')+tests
with tempfile.TemporaryDirectory() as folder:
 p=Path(folder)/'test.cpp';binary=Path(folder)/'test';p.write_text(source)
 subprocess.run(['clang++','-std=c++11','-fsanitize=address,undefined',str(p),'-o',str(binary)],check=True)
 subprocess.run([str(binary)],check=True)
print('Restwaerme-Einstellung: 30/40-Grad-Grenzen, Standard 35, Speicherung, Altclient und Fehler-Rollback geprueft.')
