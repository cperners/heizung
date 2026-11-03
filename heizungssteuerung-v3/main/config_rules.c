#include "config.h"
#include <math.h>
#include <string.h>
void config_defaults(heating_config_t *c) {
 memset(c,0,sizeof(*c));
 *c=(heating_config_t){ .version=2,.mode=MODE_AUTO,
 .holiday_room_target=15,.curve_zero=45,.curve_slope=1,.room_gain=3,.room_target=21,.unsynced_room=17.5,
 .flow_min=25,.flow_max=60,.summer_temp=20,.boiler_target=50,.boiler_hysteresis=5,
 .kessel_max=85,.kessel_margin=10,.burner_hysteresis=5,.dhw_lift=10,
 .frost_outside=3,.frost_water=5,.frost_flow=30,.mixer_band=1,
 .burner_min_on_s=120,.burner_min_off_s=180,.pump_overrun_s=120,
 .mixer_travel_s=240,.mixer_dead_ms=1000,.sensor_stale_s=5,.dhw_priority=true,
 .periods={{360,21},{510,18},{990,21},{1350,17.5}} };
}
static bool range(float v,float lo,float hi) { return isfinite(v)&&v>=lo&&v<=hi; }
bool config_validate(const heating_config_t *c) {
 if(c->version!=2||c->mode<MODE_OFF||c->mode>MODE_FROST||c->pv_enabled) return false;
 if(!range(c->curve_zero,20,65)||!range(c->curve_slope,0,3)||!range(c->room_gain,0,10)
 ||!range(c->holiday_room_target,5,25)||!range(c->room_target,5,30)||!range(c->unsynced_room,5,30)
 ||!range(c->flow_min,10,40)||!range(c->flow_max,30,75)||c->flow_min>=c->flow_max
 ||!range(c->summer_temp,10,30)||!range(c->boiler_target,35,65)
 ||!range(c->boiler_hysteresis,1,15)||!range(c->kessel_max,60,95)
 ||!range(c->kessel_margin,3,20)||!range(c->burner_hysteresis,1,15)
 ||!range(c->dhw_lift,5,20)||!range(c->frost_outside,-5,10)
 ||!range(c->frost_water,2,10)||!range(c->frost_flow,15,40)
 ||c->frost_flow>c->flow_max||!range(c->mixer_band,0.3,5)) return false;
 if(c->flow_max+c->kessel_margin>c->kessel_max-3
 ||c->boiler_target+c->dhw_lift>c->kessel_max-3
 ||c->burner_hysteresis>=c->kessel_margin) return false;
 if(c->burner_min_on_s<10||c->burner_min_on_s>1800||c->burner_min_off_s<10
 ||c->burner_min_off_s>1800||c->pump_overrun_s>900||c->mixer_travel_s<30
 ||c->mixer_travel_s>600||c->mixer_dead_ms<500||c->mixer_dead_ms>10000
 ||c->sensor_stale_s<3||c->sensor_stale_s>10) return false;
 for(int i=0;i<PERIOD_COUNT;i++) if(c->periods[i].start_minute>=1440
 ||!range(c->periods[i].room_target,5,30)
 ||(i&&c->periods[i].start_minute<=c->periods[i-1].start_minute)) return false;
 return true;
}

const char *config_mode_name(heating_mode_t mode) {
 static const char *names[]={"off","auto","manual","heating","dhw","holiday","frost"};
 return mode>=MODE_OFF&&mode<=MODE_FROST?names[mode]:"off";
}
bool config_mode_parse(const char *text,heating_mode_t *mode) {
 for(int i=MODE_OFF;i<=MODE_FROST;i++)if(!strcmp(text,config_mode_name(i))){*mode=(heating_mode_t)i;return true;}
 return false;
}
bool config_migrate_v1(const void *blob,size_t size,heating_config_t *out) {
 /* V1 entspricht exakt dem unveraenderten Praefix vor holiday_room_target. */
 if(size!=offsetof(heating_config_t,holiday_room_target))return false;
 heating_config_t migrated;config_defaults(&migrated);memcpy(&migrated,blob,size);
 if(migrated.version!=1||migrated.mode<MODE_OFF||migrated.mode>MODE_MANUAL)return false;
 if(migrated.mode==MODE_OFF)migrated.mode=MODE_FROST; /* Alt-Aus enthielt Frostschutz. */
 migrated.version=2;migrated.holiday_room_target=15;
 if(!config_validate(&migrated))return false;
 *out=migrated;return true;
}
