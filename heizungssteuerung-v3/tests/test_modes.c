#include "config.h"
#include "heating.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
static sensor_snapshot_t sensors(int64_t now) {
 sensor_snapshot_t v={0};float t[]={5,21,55,30,40};
 for(int i=0;i<5;i++)v.value[i]=(sensor_value_t){.valid=true,.celsius=t[i],.sampled_ms=now};
 return v;
}
int main(void) {
 heating_config_t c;config_defaults(&c);assert(c.version==2&&c.holiday_room_target==15);
 heating_config_t legacy=c,migrated;legacy.version=1;legacy.mode=MODE_OFF;legacy.curve_slope=1.7f;
 size_t n=offsetof(heating_config_t,holiday_room_target);assert(n==140);
 assert(config_migrate_v1(&legacy,n,&migrated));assert(migrated.mode==MODE_FROST&&migrated.curve_slope==1.7f&&migrated.holiday_room_target==15);
 assert(!config_migrate_v1(&legacy,n-1,&migrated));legacy.version=9;assert(!config_migrate_v1(&legacy,n,&migrated));
 for(int i=MODE_OFF;i<=MODE_FROST;i++){heating_mode_t mode;assert(config_mode_parse(config_mode_name(i),&mode)&&mode==(heating_mode_t)i);}
 heating_mode_t mode;assert(!config_mode_parse("nonsense",&mode));
 const int64_t now=180000;heating_state_t h;heating_status_t o;sensor_snapshot_t v;
 c.mode=MODE_AUTO;heating_init(&h,0);v=sensors(now);heating_step(&h,&c,&v,now,360,true,true,&o);
 assert(o.dhw_demand&&o.outputs.boiler_pump&&!o.outputs.heating_pump);
 /* Bereits laufende Ladung und ihr Nachlauf werden bei Nur Heizung beendet. */
 c.mode=MODE_HEATING_ONLY;heating_step(&h,&c,&v,now+100,360,true,true,&o);
 assert(!o.dhw_demand&&!o.outputs.boiler_pump&&o.outputs.heating_pump&&h.dhw_until_ms==0);
 assert(o.room_target==21);heating_step(&h,&c,&v,now+200,510,true,true,&o);assert(o.room_target==18);
 c.mode=MODE_DHW_ONLY;heating_init(&h,0);v=sensors(now);heating_step(&h,&c,&v,now,360,true,true,&o);
 assert(o.dhw_demand&&o.outputs.boiler_pump&&!o.outputs.heating_pump);
 c.mode=MODE_HOLIDAY;heating_init(&h,0);v=sensors(now);heating_step(&h,&c,&v,now,360,true,true,&o);
 assert(o.room_target==15&&!o.dhw_demand&&!o.outputs.boiler_pump&&o.outputs.heating_pump);
 /* Urlaub bleibt bei aktivem Kachelofen abgesenkt, ohne Raumrueckfuehrung. */
 heating_step_with_room_control(&h,&c,&v,now,360,true,true,false,&o);assert(o.flow_target==25&&o.room_correction_suppressed);
 c.mode=MODE_FROST;heating_init(&h,0);v=sensors(now);heating_step(&h,&c,&v,now,360,true,true,&o);
 assert(!o.frost&&!o.outputs.burner&&!o.dhw_demand&&!o.outputs.heating_pump&&!o.outputs.boiler_pump);
 v.value[OUTSIDE].celsius=-5;v.value[KESSEL].celsius=20;
 heating_step(&h,&c,&v,now+100,360,true,true,&o);assert(o.frost&&o.outputs.burner&&o.outputs.heating_pump&&!o.outputs.boiler_pump&&o.flow_target==c.frost_flow);
 c.mode=MODE_OFF;heating_step(&h,&c,&v,now+200,360,true,true,&o);
 assert(!o.frost&&!o.outputs.burner&&!o.dhw_demand); /* Mindestlaufzeit umgangen. */
 assert(o.outputs.heating_pump); /* Laufender Nachlauf bleibt. */
 v=sensors(now+200000);v.value[OUTSIDE].celsius=-5;heating_step(&h,&c,&v,now+200000,360,true,true,&o);
 assert(!o.outputs.burner&&!o.outputs.heating_pump&&!o.outputs.boiler_pump);
 v.value[KESSEL].celsius=c.kessel_max;heating_step(&h,&c,&v,now+200100,360,true,true,&o);
 assert(!o.outputs.burner&&o.outputs.heating_pump&&(o.faults&FAULT_KESSEL_HOT));
 for(int i=MODE_OFF;i<=MODE_FROST;i++) {
  c.mode=i;heating_init(&h,0);v=sensors(now);v.value[FLOW].valid=false;
  heating_step(&h,&c,&v,now,360,true,true,&o);assert(!o.outputs.burner&&!o.outputs.boiler_pump);
 }
 puts("PASS: sechs Betriebsarten, Urlaubsabsenkung mit Kachelofen, Boiler aus, Nachlauf, Sicherheitsvorrang und V1-Migration.");
}
