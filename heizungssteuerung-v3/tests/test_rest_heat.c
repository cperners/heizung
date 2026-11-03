#include "config.h"
#include "heating.h"
#include "rest_heat_rules.h"
#include <assert.h>
#include <stdio.h>
static sensor_snapshot_t sample(int64_t now,float k,float f){
 sensor_snapshot_t v={0};float t[]={5,21,k,f,60};
 for(int i=0;i<5;i++)v.value[i]=(sensor_value_t){.valid=true,.celsius=t[i],.sampled_ms=now};return v;
}
static void step(heating_state_t *h,heating_config_t *c,rest_heat_config_t *r,int64_t now,float k,float f,heating_status_t *o){sensor_snapshot_t v=sample(now,k,f);heating_step_with_rest_heat(h,c,&v,now,360,true,true,true,r,o);}
static void start(heating_state_t *h,heating_config_t *c,rest_heat_config_t *r,int64_t now,heating_status_t *o){heating_init(h,0);c->mode=MODE_HEATING_ONLY;step(h,c,r,now,60,30,o);assert(o->outputs.heating_pump);c->mode=MODE_OFF;step(h,c,r,now+100,60,30,o);assert(o->rest_heat_active&&o->outputs.heating_pump&&!o->outputs.burner);}
int main(void){
 heating_config_t c;config_defaults(&c);rest_heat_config_t r;rest_heat_defaults(&r);assert(rest_heat_validate(&r)&&r.enabled&&r.stop_c==30&&r.max_s==1800);
 rest_heat_config_t bad=r;bad.stop_c=19;assert(!rest_heat_validate(&bad));bad=r;bad.max_s=7201;assert(!rest_heat_validate(&bad));
 heating_state_t h;heating_status_t o;int64_t now=200000;
 c.mode=MODE_OFF;heating_init(&h,0);step(&h,&c,&r,now,60,30,&o);assert(!o.rest_heat_active&&!o.outputs.heating_pump);
 start(&h,&c,&r,now,&o);assert(o.outputs.mixer==MIX_OPEN&&o.flow_target==40&&o.kessel_target==30&&o.rest_heat_remaining_s==1800);
 step(&h,&c,&r,now+200,55,44,&o);assert(o.rest_heat_active&&o.outputs.mixer==MIX_CLOSE&&!o.outputs.burner);
 step(&h,&c,&r,now+300,30,29,&o);assert(!o.rest_heat_active&&!o.outputs.heating_pump&&o.outputs.mixer==MIX_CLOSE&&h.heat_until_ms==0);
 step(&h,&c,&r,now+400,60,30,&o);assert(!o.rest_heat_active&&!o.outputs.heating_pump);
 start(&h,&c,&r,now,&o);step(&h,&c,&r,now+100+(int64_t)r.max_s*1000,60,30,&o);assert(!o.rest_heat_active&&!o.outputs.heating_pump&&!o.outputs.burner);
 start(&h,&c,&r,now,&o);sensor_snapshot_t v=sample(now+200,60,30);v.value[FLOW].valid=false;
 heating_step_with_rest_heat(&h,&c,&v,now+200,360,true,true,true,&r,&o);assert(!o.rest_heat_active&&!o.outputs.heating_pump&&!o.outputs.burner&&o.outputs.mixer==MIX_CLOSE);
 start(&h,&c,&r,now,&o);step(&h,&c,&r,now+200,c.kessel_max,30,&o);assert(!o.rest_heat_active&&o.outputs.heating_pump&&!o.outputs.burner&&o.outputs.mixer==MIX_CLOSE);
 start(&h,&c,&r,now,&o);r.enabled=false;step(&h,&c,&r,now+200,60,30,&o);assert(!o.rest_heat_active&&!o.outputs.heating_pump);r.enabled=true;
 start(&h,&c,&r,now,&o);c.mode=MODE_HEATING_ONLY;step(&h,&c,&r,now+200,60,30,&o);assert(!o.rest_heat_active&&o.outputs.heating_pump);
 c.mode=MODE_DHW_ONLY;heating_init(&h,0);step(&h,&c,&r,now,60,30,&o);c.mode=MODE_OFF;step(&h,&c,&r,now+100,60,30,&o);assert(!o.rest_heat_active);
 r.enabled=false;heating_init(&h,0);c.mode=MODE_HEATING_ONLY;step(&h,&c,&r,now,60,30,&o);c.mode=MODE_OFF;step(&h,&c,&r,now+100,60,30,&o);assert(!o.rest_heat_active&&o.outputs.heating_pump&&o.outputs.mixer==MIX_CLOSE);
 /* Mixed mode/temperature/sensor faults through the production entry point. */
 unsigned seed=12345,active_count=0;rest_heat_defaults(&r);heating_init(&h,0);
 for(int i=0;i<10000;i++){
  seed=seed*1664525u+1013904223u;c.mode=(heating_mode_t)(seed%7);
  int64_t tick=200000+(int64_t)i*2000;
  float k=20+(seed>>8)%75,f=20+(seed>>16)%50;sensor_snapshot_t v2=sample(tick,k,f);
  if((seed&31)==0)v2.value[KESSEL].valid=false;
  if((seed&63)==1)v2.value[FLOW].valid=false;
  heating_step_with_rest_heat(&h,&c,&v2,tick,360,true,true,true,&r,&o);
  bool critical=(o.faults&(FAULT_KESSEL|FAULT_FLOW|FAULT_KESSEL_HOT|FAULT_FLOW_HOT))!=0;
  if(c.mode==MODE_OFF)assert(!o.outputs.burner&&!o.outputs.boiler_pump&&!o.frost);
  if(critical)assert(!o.outputs.burner&&o.outputs.mixer==MIX_CLOSE&&!o.rest_heat_active);
  if(o.rest_heat_active){active_count++;assert(c.mode==MODE_OFF&&o.outputs.heating_pump&&!o.outputs.burner&&k>r.stop_c&&!critical);}
 }
 assert(active_count>0);
 puts("PASS: Restwaerme 30 Grad, geregelter Mischer, Brennersperre, Zeitlimit, Sensorfehler, Sicherheitsvorrang, Abbruch, Neustart und klassischer Nachlauf.");
}
