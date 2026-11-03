#include "config.h"
#include "heating.h"
#include "mixer.h"
#include "buffer.h"
#include "ds18b20.h"
#include "ntp.h"
#include <time.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static sensor_snapshot_t sensors(int64_t now) {
 sensor_snapshot_t v={0};float t[]={0,21,40,30,55,62,48,31};
 for(int i=0;i<8;i++)v.value[i]=(sensor_value_t){.celsius=t[i],.valid=true,.sampled_ms=now};return v;
}
static void config_tests(void) {
 heating_config_t c;config_defaults(&c);assert(config_validate(&c));
 c.curve_slope=NAN;assert(!config_validate(&c));config_defaults(&c);
 c.kessel_max=65;assert(!config_validate(&c));config_defaults(&c);
 c.periods[2].start_minute=510;assert(!config_validate(&c));config_defaults(&c);
 c.pv_enabled=true;assert(!config_validate(&c));config_defaults(&c);
 c.mixer_dead_ms=0;assert(!config_validate(&c));
}
static void control_tests(void) {
 heating_config_t c;config_defaults(&c);heating_state_t h;heating_init(&h,0);heating_status_t o;
 int64_t now=180000;sensor_snapshot_t v=sensors(now);
 heating_step(&h,&c,&v,now,360,true,true,&o);
 assert(o.outputs.burner&&o.outputs.heating_pump);assert(o.flow_target==45);
 /* Sicherheitsgrenzen umgehen noch laufende Mindestlaufzeit. */
 v.value[KESSEL].celsius=85;heating_step(&h,&c,&v,now+100,360,true,true,&o);
 assert(!o.outputs.burner&&(o.faults&FAULT_KESSEL_HOT)&&o.outputs.mixer==MIX_CLOSE);
 v=sensors(now+200);heating_step(&h,&c,&v,now+200,360,true,true,&o);assert(!o.outputs.burner);
 v=sensors(now+200000);heating_step(&h,&c,&v,now+200000,360,true,true,&o);assert(o.outputs.burner);
 v.value[FLOW].valid=false;heating_step(&h,&c,&v,now+200100,360,true,true,&o);
 assert(!o.outputs.burner&&!o.outputs.heating_pump&&(o.faults&FAULT_FLOW));
 /* Jeder kritische Sensor unabhängig, NaN und veraltete Messung fail closed. */
 for(int i=KESSEL;i<=FLOW;i++){
  heating_init(&h,0);v=sensors(now);v.value[i].celsius=NAN;
  heating_step(&h,&c,&v,now,360,true,true,&o);assert(!o.outputs.burner);
  v=sensors(now);v.value[i].sampled_ms=now-5000;
  heating_step(&h,&c,&v,now,360,true,true,&o);assert(!o.outputs.burner);
 }
 /* Raumfehler ohne Abschaltung: Kurve ohne Korrektur. */
 heating_init(&h,0);v=sensors(now);v.value[ROOM].valid=false;
 heating_step(&h,&c,&v,now,360,true,true,&o);assert(o.outputs.burner&&o.flow_target==45);
 /* Außenfehler: keine normale Heizung; Frost über lokale Sensoren möglich. */
 heating_init(&h,0);v=sensors(now);v.value[OUTSIDE].valid=false;
 heating_step(&h,&c,&v,now,360,true,true,&o);assert(!o.outputs.burner);
 v.value[FLOW].celsius=4;v.value[KESSEL].celsius=20;heating_step(&h,&c,&v,now+100,360,true,true,&o);assert(o.frost&&o.outputs.burner);
 /* Boilerfehler darf Heizkreis weiterlaufen lassen. */
 heating_init(&h,0);v=sensors(now);v.value[DHW].valid=false;
 heating_step(&h,&c,&v,now,360,true,true,&o);assert(!o.outputs.boiler_pump&&o.outputs.heating_pump);
 /* Warmwasservorrang und Ende der Ladeanforderung. */
 heating_init(&h,0);v=sensors(now);v.value[DHW].celsius=40;v.value[KESSEL].celsius=55;
 heating_step(&h,&c,&v,now,360,true,true,&o);assert(o.dhw_demand&&o.outputs.boiler_pump&&!o.outputs.heating_pump);
 v.value[DHW].celsius=50;heating_step(&h,&c,&v,now+100,360,true,true,&o);
 assert(!o.dhw_demand&&!o.outputs.boiler_pump);
 /* Solltemperatur gilt vor und nach Mitternacht und bei NTP-Ausfall. */
 heating_init(&h,0);v=sensors(now);
 heating_step(&h,&c,&v,now,0,true,true,&o);assert(o.room_target==17.5f);
 heating_step(&h,&c,&v,now,510,true,true,&o);assert(o.room_target==18);
 heating_step(&h,&c,&v,now,990,true,true,&o);assert(o.room_target==21);
 heating_step(&h,&c,&v,now,1350,true,true,&o);assert(o.room_target==17.5f);
 heating_step(&h,&c,&v,now,360,false,false,&o);assert(o.room_target==17.5f&&(o.faults&FAULT_TIME)&&(o.faults&FAULT_NETWORK));
 /* Aus und Frostschutz sind jetzt getrennt. */
 c.mode=MODE_OFF;heating_init(&h,0);v=sensors(now);v.value[OUTSIDE].celsius=10;
 heating_step(&h,&c,&v,now,360,true,true,&o);assert(!o.outputs.burner&&!o.outputs.heating_pump);
 v.value[OUTSIDE].celsius=-5;heating_step(&h,&c,&v,now+100,360,true,true,&o);assert(!o.frost&&!o.outputs.burner);
 c.mode=MODE_FROST;v.value[KESSEL].celsius=20;heating_step(&h,&c,&v,now+200,360,true,true,&o);assert(o.frost&&o.outputs.burner);
 c.mode=MODE_AUTO;heating_init(&h,0);v=sensors(now);v.value[OUTSIDE].celsius=25;
 heating_step(&h,&c,&v,now,360,true,true,&o);assert(!o.outputs.burner);
 /* Brenner normale Mindestlaufzeit; Pumpennachlauf. */
 heating_init(&h,0);v=sensors(now);heating_step(&h,&c,&v,now,360,true,true,&o);
 v.value[KESSEL].celsius=60;heating_step(&h,&c,&v,now+100,360,true,true,&o);assert(o.outputs.burner);
 v=sensors(now+120000);v.value[KESSEL].celsius=60;
 heating_step(&h,&c,&v,now+120000,360,true,true,&o);assert(!o.outputs.burner);
 c.mode=MODE_OFF;v=sensors(now+120100);v.value[OUTSIDE].celsius=10;
 heating_step(&h,&c,&v,now+120100,360,true,true,&o);assert(o.outputs.heating_pump);
 v=sensors(now+241000);v.value[OUTSIDE].celsius=10;
 heating_step(&h,&c,&v,now+241000,360,true,true,&o);assert(!o.outputs.heating_pump);
}
static void mixer_tests(void) {
 mixer_state_t m;mixer_init(&m,0);
 assert(mixer_step(&m,MIX_OPEN,0,1000,120)==MIX_CLOSE); /* Referenzfahrt */
 assert(mixer_step(&m,MIX_CLOSE,120000,1000,120)==MIX_STOP);assert(m.position==0);
 assert(mixer_step(&m,MIX_OPEN,120100,1000,120)==MIX_STOP);
 assert(mixer_step(&m,MIX_OPEN,121000,1000,120)==MIX_OPEN);
 assert(mixer_step(&m,MIX_CLOSE,121100,1000,120)==MIX_STOP);
 assert(mixer_step(&m,MIX_CLOSE,121999,1000,120)==MIX_STOP);
 assert(mixer_step(&m,MIX_CLOSE,122100,1000,120)==MIX_CLOSE);
 /* STOP dazwischen darf die Richtungswechseltotzeit nicht umgehen. */
 mixer_step(&m,MIX_STOP,122200,1000,120);
 assert(mixer_step(&m,MIX_OPEN,122300,1000,120)==MIX_STOP);
 for(int n=0;n<10000;n++) {int64_t t=123000+n*100; mixer_dir_t d=mixer_step(&m,n%3,t,1000,120);
 assert(d>=MIX_STOP&&d<=MIX_CLOSE);assert(m.position>=0&&m.position<=1);}
}
static void sensor_tests(void) {
 uint8_t b[]={0x50,0x05,0x4b,0x46,0x7f,0xff,0x0c,0x10,0};float t;
 b[8]=ds18b20_crc(b,8);assert(ds18b20_decode(b,&t)&&t==85);
 b[0]^=1;assert(!ds18b20_decode(b,&t));
 memset(b,0,9);assert(!ds18b20_decode(b,&t));memset(b,255,9);assert(!ds18b20_decode(b,&t));
 /* Exact CRC-valid scratchpads from the two user sensors, not fixed reserved bytes. */
 uint8_t outside[]={0x8e,0x01,0x7f,0x80,0x7f,0xff,0x02,0x10,0x8d};
 uint8_t room[]={0x9c,0x01,0x7f,0x80,0x7f,0xff,0x04,0x10,0xf5};
 assert(ds18b20_crc(outside,8)==outside[8]&&ds18b20_decode(outside,&t)&&t==24.875f);
 assert(ds18b20_crc(room,8)==room[8]&&ds18b20_decode(room,&t)&&t==25.75f);
 outside[6]^=1;assert(!ds18b20_decode(outside,&t));outside[6]^=1;
 outside[4]=0;outside[8]=ds18b20_crc(outside,8);assert(!ds18b20_decode(outside,&t));
 outside[4]=0x7f;outside[0]=0xff;outside[1]=0x7f;outside[8]=ds18b20_crc(outside,8);assert(!ds18b20_decode(outside,&t));
 uint8_t negative[]={0x60,0xff,0x4b,0x46,0x7f,0xff,0x0c,0x10,0};negative[8]=ds18b20_crc(negative,8);
 assert(ds18b20_decode(negative,&t)&&t==-10);
 sensor_snapshot_t v=sensors(100);assert(!buffer_evaluate(&v,false,100,5).available);
 assert(buffer_evaluate(&v,true,100,5).mean==47);v.value[BUFFER_MID].valid=false;
 assert(!buffer_evaluate(&v,true,100,5).available);
}

static void timezone_tests(void) {
 assert(setenv("TZ",HEATING_TIMEZONE,1)==0);tzset();
 /* UTC 2026-03-29 00:30 / 01:30, danach Oktober-Rückstellung. */
 time_t times[]={1774744200,1774747800,1792888200,1792891800};
 int hour[]={1,3,2,2};int dst[]={0,1,1,0};
 for(int i=0;i<4;i++){struct tm local;assert(localtime_r(&times[i],&local));
 assert(local.tm_hour==hour[i]&&local.tm_min==30&&local.tm_isdst==dst[i]);}
}
static void safety_sequence_tests(void) {
 heating_config_t c;config_defaults(&c);heating_state_t h;heating_init(&h,0);
 unsigned random=17;
 for(int i=0;i<20000;i++){
  int64_t now=(int64_t)i*1000;sensor_snapshot_t v=sensors(now);
  for(int j=0;j<5;j++) {random=random*1664525u+1013904223u;
   v.value[j].celsius=(float)(random%110)-10;
   if(random%17==0)v.value[j].valid=false;}
  heating_status_t o;heating_step(&h,&c,&v,now,i%1440,true,true,&o);
  bool unsafe=!v.value[KESSEL].valid||!v.value[FLOW].valid||v.value[KESSEL].celsius>=c.kessel_max||v.value[FLOW].celsius>=c.flow_max;
  if(unsafe)assert(!o.outputs.burner&&o.outputs.mixer==MIX_CLOSE);
  assert(!o.outputs.heater);assert(isfinite(o.flow_target));
  assert(o.flow_target>=c.flow_min&&o.flow_target<=c.flow_max);
  if(!v.value[DHW].valid||v.value[DHW].celsius>=c.boiler_target)assert(!o.outputs.boiler_pump);
 }
}
int main(void){config_tests();control_tests();mixer_tests();sensor_tests();timezone_tests();safety_sequence_tests();puts("PASS: Konfiguration, Regelung, Sicherheit, Zeitprogramm/DST, Mischer, CRC, Puffer; 20.000 Sicherheitszyklen");}
