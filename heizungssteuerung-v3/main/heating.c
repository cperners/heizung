#include "heating.h"
#include <math.h>
#include <string.h>
static float clamp(float x,float a,float b) { return fminf(b,fmaxf(a,x)); }
static bool good(const sensor_value_t *v,int64_t now,const heating_config_t *c) {
 return v->valid&&isfinite(v->celsius)&&v->celsius>=-55&&v->celsius<=125
 &&v->sampled_ms<=now&&now-v->sampled_ms<(int64_t)c->sensor_stale_s*1000;
}
void heating_init(heating_state_t *s,int64_t now) {
 memset(s,0,sizeof(*s)); s->burner_changed_ms=now;
}
void heating_step_with_rest_heat(heating_state_t *s,const heating_config_t *c,const sensor_snapshot_t *v,
 int64_t now,int minute,bool time_valid,bool network_ok,bool room_feedback,const rest_heat_config_t *rest,heating_status_t *o) {
 memset(o,0,sizeof(*o));
 bool ok[SENSOR_COUNT]; for(int i=0;i<SENSOR_COUNT;i++) ok[i]=good(&v->value[i],now,c);
 float k=v->value[KESSEL].celsius, f=v->value[FLOW].celsius;
 if(!ok[KESSEL]) o->faults|=FAULT_KESSEL;
 if(!ok[FLOW]) o->faults|=FAULT_FLOW;
 if(!ok[DHW]) o->faults|=FAULT_DHW;
 if(!ok[OUTSIDE]) o->faults|=FAULT_OUTSIDE;
 if(!ok[ROOM]) o->faults|=FAULT_ROOM;
 if(!time_valid) o->faults|=FAULT_TIME;
 if(!network_ok) o->faults|=FAULT_NETWORK;
 if(ok[KESSEL]&&k>=c->kessel_max) o->faults|=FAULT_KESSEL_HOT;
 if(ok[FLOW]&&f>=c->flow_max) o->faults|=FAULT_FLOW_HOT;
 o->room_target=c->room_target;
 if(c->mode==MODE_AUTO||c->mode==MODE_HEATING_ONLY) {
  o->room_target=c->unsynced_room;
  if(time_valid) { o->room_target=c->periods[PERIOD_COUNT-1].room_target;
   for(int i=0;i<PERIOD_COUNT;i++) if(minute>=c->periods[i].start_minute) o->room_target=c->periods[i].room_target; }
 }
 if(c->mode==MODE_HOLIDAY)o->room_target=c->holiday_room_target;
 bool heating_mode=c->mode==MODE_AUTO||c->mode==MODE_HEATING_ONLY||c->mode==MODE_HOLIDAY||c->mode==MODE_MANUAL;
 bool dhw_mode=c->mode==MODE_AUTO||c->mode==MODE_DHW_ONLY||c->mode==MODE_MANUAL;
 o->frost=c->mode!=MODE_OFF&&((ok[OUTSIDE]&&v->value[OUTSIDE].celsius<c->frost_outside)
 ||(ok[KESSEL]&&k<c->frost_water)||(ok[FLOW]&&f<c->frost_water)
 ||(dhw_mode&&ok[DHW]&&v->value[DHW].celsius<c->frost_water));
 bool critical=(o->faults&(FAULT_KESSEL|FAULT_FLOW|FAULT_KESSEL_HOT|FAULT_FLOW_HOT))!=0;
 bool off_transition=s->mode_seen&&s->previous_mode!=MODE_OFF&&c->mode==MODE_OFF;
 bool previous_heating=s->previous_mode==MODE_AUTO||s->previous_mode==MODE_HEATING_ONLY||s->previous_mode==MODE_HOLIDAY||s->previous_mode==MODE_MANUAL;
 bool was_rest=s->rest_active;
 if(off_transition&&previous_heating&&rest&&rest->enabled&&!critical&&ok[KESSEL]&&ok[FLOW]&&k>rest->stop_c&&(s->heat_pump||s->burner)){
  s->rest_active=true;s->rest_started_ms=now;
 }
 if(s->rest_active&&(!rest||!rest->enabled||c->mode!=MODE_OFF||critical||!ok[KESSEL]||!ok[FLOW]||k<=rest->stop_c||now<s->rest_started_ms||now-s->rest_started_ms>=(int64_t)rest->max_s*1000))s->rest_active=false;
 if((was_rest&&!s->rest_active)||s->rest_active)s->heat_until_ms=0;
 bool heat=heating_mode&&ok[OUTSIDE]&&v->value[OUTSIDE].celsius<c->summer_temp;
 float target=ok[OUTSIDE]?c->curve_zero-c->curve_slope*v->value[OUTSIDE].celsius:c->frost_flow;
 if(room_feedback&&ok[ROOM]) target+=c->room_gain*(o->room_target-v->value[ROOM].celsius);
 else target+=c->room_gain*(o->room_target-c->room_target);
 o->room_correction_suppressed=!room_feedback;
 o->flow_target=clamp(target,c->flow_min,c->flow_max-1);
 if(o->frost) {heat=true; o->flow_target=heating_mode?fmaxf(c->frost_flow,o->flow_target):c->frost_flow;}
 bool dhw=boiler_step(&s->boiler,c,ok[DHW],v->value[DHW].celsius,dhw_mode);
 if(!dhw_mode)s->dhw_until_ms=0;
 o->dhw_demand=dhw;
 if(s->rest_active){heat=true;o->flow_target=clamp(s->last_flow_target,c->flow_min,c->flow_max-1);o->rest_heat_active=true;o->rest_heat_remaining_s=(uint32_t)(((int64_t)rest->max_s*1000-(now-s->rest_started_ms)+999)/1000);}
 bool hp=heat&&(!dhw||!c->dhw_priority), bp=dhw;
 /* Pumpen nur laden, wenn Kessel wärmer als Boiler ist (kein Rückkühlen). */
 bp=bp&&ok[KESSEL]&&k>v->value[DHW].celsius+(s->dhw_pump?0:2);
 o->kessel_target=clamp(dhw?c->boiler_target+c->dhw_lift:o->flow_target+c->kessel_margin,20,c->kessel_max-3);
 if(s->rest_active)o->kessel_target=rest->stop_c;
 bool demand=c->mode!=MODE_OFF&&(heat||dhw);
 if(!demand||k>=o->kessel_target) s->burner_demand=false;
 else if(k<=o->kessel_target-c->burner_hysteresis) s->burner_demand=true;
 bool burner=s->burner;
 if(critical||c->mode==MODE_OFF) burner=false; /* Vorrang vor jeder Mindestlaufzeit. */
 else if(burner&&(!s->burner_demand)&&now-s->burner_changed_ms>=(int64_t)c->burner_min_on_s*1000) burner=false;
 else if(!burner&&s->burner_demand&&now-s->burner_changed_ms>=(int64_t)c->burner_min_off_s*1000) burner=true;
 /* Rest-Mindestlaufzeit ohne aktuelle Anforderung: Heizkreispumpe,
    Mischer bleibt ZU. Bei DHW-Anforderung erwärmt der Kessel zunächst sich
    selbst; eine erforderliche kesselinterne Umwälzung ist anlagenseitig nötig. */
 if(burner&&!heat&&!dhw) hp=true;
 if(hp&&!s->rest_active) s->heat_until_ms=now+(int64_t)c->pump_overrun_s*1000;
 if(bp) s->dhw_until_ms=now+(int64_t)c->pump_overrun_s*1000;
 hp=hp||now<s->heat_until_ms;
 bp=(bp||now<s->dhw_until_ms)&&ok[DHW]&&ok[KESSEL];
 /* Sensorfehler: unbestimmten Wärmeweg sperren. Übertemperatur: bekannte
    Heizkreispumpe zur Wärmeabfuhr an, Mischer ZU. Anlagenhydraulik prüfen! */
 if(!ok[FLOW]||!ok[KESSEL]) { hp=false; bp=false; s->heat_until_ms=s->dhw_until_ms=0; }
 else if(critical) { hp=true; bp=false; }
 else if(dhw&&c->dhw_priority) hp=false;
 if(bp&&(k<=v->value[DHW].celsius||v->value[DHW].celsius>=c->boiler_target)) bp=false;
 if(burner!=s->burner) {s->burner=burner; s->burner_changed_ms=now;}
 s->heat_pump=hp; s->dhw_pump=bp;
 o->outputs=(output_request_t){.burner=burner,.heating_pump=hp,.boiler_pump=bp,.mixer=MIX_STOP};
 if(critical||!hp||!heat||(dhw&&c->dhw_priority)) o->outputs.mixer=MIX_CLOSE;
 else if(f<o->flow_target-c->mixer_band) o->outputs.mixer=MIX_OPEN;
 else if(f>o->flow_target+c->mixer_band) o->outputs.mixer=MIX_CLOSE;
 if(heating_mode)s->last_flow_target=o->flow_target;
 s->previous_mode=c->mode;s->mode_seen=true;
 /* Bei fehlendem Vorlauffühler: ZU für höchstens geschätzte Vollfahrzeit
    wird zentral im mixer-Modul begrenzt. */
}

void heating_step(heating_state_t *s,const heating_config_t *c,const sensor_snapshot_t *v,
 int64_t now,int minute,bool time_valid,bool network_ok,heating_status_t *out) {
 heating_step_with_room_control(s,c,v,now,minute,time_valid,network_ok,true,out);
}

void heating_step_with_room_control(heating_state_t *s,const heating_config_t *c,const sensor_snapshot_t *v,
 int64_t now,int minute,bool time_valid,bool network_ok,bool room_feedback,heating_status_t *o){
 heating_step_with_rest_heat(s,c,v,now,minute,time_valid,network_ok,room_feedback,NULL,o);
}
