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
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <map>
#include <limits>
using std::isfinite;
#define MISCHER_DRIVE_MIN_SECONDS 1
#define MISCHER_DRIVE_MAX_SECONDS 10
#define MISCHER_OPEN_INTERVAL_MIN 5
#define MISCHER_OPEN_INTERVAL_MAX 120
constexpr uint32_t MISCHER_INTERVALS_TAG=0x4D490000UL, MISCHER_DRIVE_LEGACY_TAG=0x4D440000UL, MISCHER_DRIVE_TIMES_TAG=0x4D450000UL;
struct MischerIntervals{uint8_t minimum,maximum;};
constexpr MischerIntervals DEFAULT_MISCHER_DRIVE_HALF_SECONDS={4,20};
MischerIntervals mischerIntervals={15,30},mischerDriveHalfSeconds=DEFAULT_MISCHER_DRIVE_HALF_SECONDS;
const int EEADDRESS_MISCHER_DRIVE_TIMES=12,EEADDRESS_MISCHER_INTERVALS=8,EEADDRESS_TVMAX=0, EEADDRESS_NN=4, EEADDRESS_RESTWAERME_ZIEL=172;
float tvmax=40,n=1.9,restwaermeZielTemp=35;
struct Store{std::map<int,double> buffer{{0,40},{4,1.9},{172,35}},persisted=buffer;bool ok=true;int commits=0;
void put(int a,double v){buffer[a]=v;}
void get(int a,float &v){v=buffer.count(a)?buffer[a]:std::numeric_limits<float>::quiet_NaN();}
void get(int a,uint32_t &v){v=buffer.count(a)?static_cast<uint32_t>(buffer[a]):0xffffffffUL;}
bool commit(){++commits;if(ok)persisted=buffer;return ok;}}EEPROM;
struct Log{template<class...T>void printf(T...){}}Serial;
'''
mqtt=(root/'src/firmware/mqtt.inc').read_text()
start=mqtt.index('    const char mixerPrefix')
end=mqtt.index('    const char vacationPrefix',start)
mqtt_command='''
char mqtt_payload[32];
bool mqtt2update=false;
void mixerCommand(char* new_payload, bool retained) {
  struct {bool retain;} properties={retained};
'''+mqtt[start:end]+'}\n'
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
const MischerIntervals invalid={31,30}, updated={12,55}, second={20,60};
int parsed=0;
assert(parseMischerInterval("120",parsed)&&parsed==120);
assert(validMischerIntervals(5,120) && !validMischerIntervals(4,120));
for(const char* text:{"","-1","10.5","10,30","121","999999999999"})assert(!parseMischerInterval(text,parsed));
assert(!saveFlowLimit(45,false,0,false,0,&invalid));
assert(saveFlowLimit(45,false,0,false,0,&updated));
assert(mischerIntervals.minimum==12 && mischerIntervals.maximum==55);
assert(EEPROM.persisted[8]==(MISCHER_INTERVALS_TAG | (55UL<<8) | 12));
EEPROM.ok=false;
assert(!saveFlowLimit(46,false,0,false,0,&second));
assert(mischerIntervals.minimum==12 && mischerIntervals.maximum==55 && tvmax==45);
assert(EEPROM.buffer[8]==EEPROM.persisted[8]);
EEPROM.ok=true;
char validCommand[]="mischerPause:17,46";
mixerCommand(validCommand,false);
assert(mqtt2update && strcmp(mqtt_payload,"h_mischerpause")==0);
assert(mischerIntervals.minimum==17 && mischerIntervals.maximum==46);
char retainedCommand[]="mischerPause:10,30";
mixerCommand(retainedCommand,true);
assert(strcmp(mqtt_payload,"h_mischerpause_invalid")==0 && mischerIntervals.minimum==17);
char malformedCommand[]="mischerPause:12,45,60";
mixerCommand(malformedCommand,false);
assert(strcmp(mqtt_payload,"h_mischerpause_invalid")==0 && mischerIntervals.maximum==46);
char failedCommand[]="mischerPause:15,30";
EEPROM.ok=false;
mixerCommand(failedCommand,false);
assert(strcmp(mqtt_payload,"h_save_error")==0 && mischerIntervals.maximum==46);
EEPROM.ok=true;
const auto legacyDrive = decodeMischerDriveTimes(MISCHER_DRIVE_LEGACY_TAG | (8UL<<8) | 3);
assert(legacyDrive.minimum==6 && legacyDrive.maximum==16);
assert(decodeMischerDriveTimes(0xffffffffUL).maximum==20);
assert(decodeMischerDriveTimes(MISCHER_DRIVE_LEGACY_TAG | (130UL<<8) | 129).minimum==4);
char driveCommand[]="mischerFahrt:3.5,8.5";
mixerCommand(driveCommand,false);
assert(strcmp(mqtt_payload,"h_mischerfahrt")==0 && mischerDriveHalfSeconds.minimum==7 && mischerDriveHalfSeconds.maximum==17);
assert(mischerIntervals.minimum==17 && mischerIntervals.maximum==46);
assert(EEPROM.persisted[12]==(MISCHER_DRIVE_TIMES_TAG | (17UL<<8) | 7));
const auto savedDrive = decodeMischerDriveTimes(static_cast<uint32_t>(EEPROM.persisted[12]));
assert(savedDrive.minimum==7 && savedDrive.maximum==17);
char invalidDrive[]="mischerFahrt:2.25,10";
mixerCommand(invalidDrive,false);
assert(strcmp(mqtt_payload,"h_mischerfahrt_invalid")==0 && mischerDriveHalfSeconds.maximum==17);
char failedDrive[]="mischerFahrt:2,10";
EEPROM.ok=false;
mixerCommand(failedDrive,false);
assert(strcmp(mqtt_payload,"h_save_error")==0 && mischerDriveHalfSeconds.maximum==17);
assert(EEPROM.buffer[12]==EEPROM.persisted[12]);
EEPROM.ok=true;
mqtt2update=false;
char recoveryCommand[]="restwaermeZieltemperatur:36.5";
mixerCommand(recoveryCommand,false);
assert(mqtt2update && strcmp(mqtt_payload,"h_restwaerme")==0);
assert(restwaermeZielTemp==36.5f && EEPROM.persisted[172]==36.5);
assert(tvmax==45 && n==2 && mischerDriveHalfSeconds.maximum==17);
const int commitsAfterRecovery=EEPROM.commits;
mixerCommand(recoveryCommand,false);
assert(EEPROM.commits==commitsAfterRecovery);
for (const char* text : {"", "29.9", "40.1", "35,5", "35x", "nan", "inf"}) {
  char command[64]="restwaermeZieltemperatur:";
  strcat(command,text);
  mixerCommand(command,false);
  assert(strcmp(mqtt_payload,"h_restwaerme_invalid")==0);
}
char otherRecovery[]="restwaermeZieltemperatur:35";
mixerCommand(otherRecovery,true);
assert(strcmp(mqtt_payload,"h_restwaerme_invalid")==0 && EEPROM.commits==commitsAfterRecovery);
EEPROM.ok=false;
mixerCommand(otherRecovery,false);
assert(strcmp(mqtt_payload,"h_save_error")==0);
assert(restwaermeZielTemp==36.5f && EEPROM.buffer[172]==36.5 && EEPROM.persisted[172]==36.5);
EEPROM.buffer[172]=40;assert(!repairSetting(172,35.f,30,40));

EEPROM.buffer[172]=99;assert(repairSetting(172,35.f,30,40));
}
'''
source=head+function('validMischerIntervals')+function('validMischerDriveTimes')+function('parseMischerDriveTime')+function('parseMischerInterval')+s[s.index('MischerIntervals decodeMischerDriveTimes'):s.index('bool parseMischerDriveTime')]+function('validRecoveryTarget')+function('parseRecoveryTarget')+function('saveFlowLimit')+'template<typename T>\n'+function('repairSetting')+mqtt_command+tests
with tempfile.TemporaryDirectory() as folder:
 p=Path(folder)/'test.cpp';binary=Path(folder)/'test';p.write_text(source)
 subprocess.run(['clang++','-std=c++11','-fsanitize=address,undefined',str(p),'-o',str(binary)],check=True)
 subprocess.run([str(binary)],check=True)
print('Vorlaufeinstellungen: Validierung, Speicherung, Altclient, MQTT-Befehl und Fehler-Rollback geprueft.')
