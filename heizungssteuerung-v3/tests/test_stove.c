#include "stove_rules.h"
#include "heating.h"
#include "config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static void valid_config(stove_config_t *c){stove_defaults(c);c->enabled=true;strcpy(c->topic,"haus/kachelofen/temperatur");assert(stove_validate(c));}
int main(void) {
    stove_config_t c;stove_defaults(&c);assert(stove_validate(&c));assert(c.on_c==45&&c.off_c==32&&!c.enabled);
    c.enabled=true;assert(!stove_validate(&c));valid_config(&c);
    const char *bad_topics[]={"house/#","house/+","bad topic","heizung/anlage/stove","heizung/pv/power"};
    for(unsigned i=0;i<sizeof(bad_topics)/sizeof(bad_topics[0]);i++){strcpy(c.topic,bad_topics[i]);assert(!stove_validate(&c));}
    valid_config(&c);c.off_c=45;assert(!stove_validate(&c));valid_config(&c);c.on_c=NAN;assert(!stove_validate(&c));
    valid_config(&c);c.stale_s=9;assert(!stove_validate(&c));valid_config(&c);
    float t;assert(stove_parse_temperature(" 4.65e1 \r\n",&t)&&t==46.5f);
    const char *bad[]={""," ","nan","inf","0x2d","45,5","45 C","{\"temp\":45}","501","-56","45 50","1e999"};
    for(unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);i++)assert(!stove_parse_temperature(bad[i],&t));
    stove_state_t s={0};stove_sample(&s,&c,"50",true,0);assert(!s.valid&&!s.active);
    stove_sample(&s,&c,"40",false,1000);assert(s.valid&&!s.active&&stove_room_gain(&s,3)==3);
    stove_sample(&s,&c,"45",false,2000);assert(s.valid&&s.active&&stove_room_gain(&s,3)==0);
    stove_sample(&s,&c,"40",false,3000);assert(s.active);
    stove_sample(&s,&c,"32.1",false,4000);assert(s.active);
    stove_sample(&s,&c,"32",false,5000);assert(!s.active);
    stove_sample(&s,&c,"40",false,6000);assert(!s.active);
    stove_sample(&s,&c,"50",false,7000);assert(s.active);
    stove_expire(&s,&c,306999);assert(s.valid&&s.active);
    stove_sample(&s,&c,"55",true,306999);assert(s.updated_ms==7000);
    stove_expire(&s,&c,307000);assert(!s.valid&&!s.active&&stove_room_gain(&s,3)==3);
    stove_sample(&s,&c,"40",false,308000);assert(s.valid&&!s.active);
    stove_sample(&s,&c,"50",false,309000);stove_sample(&s,&c,"NaN",false,310000);assert(!s.valid&&!s.active&&s.errors==1);
    stove_sample(&s,&c,"50",false,311000);c.enabled=false;stove_expire(&s,&c,311000);assert(!s.valid&&!s.active);
    valid_config(&c);stove_sample(&s,&c,"50",false,312000);stove_expire(&s,&c,1);assert(!s.valid&&!s.active);
    /* Dieselbe effektive Konfiguration wie im echten Regler: warmer Raum darf
       bei aktivem Ofen die restliche Heizkurve nicht herunterregeln. */
    heating_config_t config;config_defaults(&config);heating_state_t h;heating_status_t out;
    sensor_snapshot_t sensors={0};float temps[]={5,28,40,30,55};
    for(int i=0;i<5;i++)sensors.value[i]=(sensor_value_t){.celsius=temps[i],.valid=true,.sampled_ms=180000};
    heating_init(&h,0);heating_step(&h,&config,&sensors,180000,360,true,true,&out);float corrected=out.flow_target;
    stove_sample(&s,&c,"45",false,180000);heating_config_t effective=config;effective.room_gain=stove_room_gain(&s,config.room_gain);
    heating_init(&h,0);heating_step(&h,&effective,&sensors,180000,360,true,true,&out);
    assert(out.flow_target==40&&out.flow_target>corrected&&config.room_gain==3);
    sensors.value[KESSEL].celsius=config.kessel_max;
    heating_step(&h,&effective,&sensors,180001,360,true,true,&out);assert(!out.outputs.burner&&(out.faults&FAULT_KESSEL_HOT));
    puts("PASS: Kachelofen-Parser, Schwellen 45/32, Hysterese, Retained, Ablauf, Fallback und Heizungsintegration.");
}
