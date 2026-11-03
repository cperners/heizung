#include "stove_rules.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>
#include <errno.h>
void stove_defaults(stove_config_t *c) {
    memset(c,0,sizeof(*c));c->version=1;c->on_c=45;c->off_c=32;c->stale_s=300;
}
bool stove_validate(const stove_config_t *c) {
    if(c->version!=1||!memchr(c->topic,0,sizeof(c->topic)))return false;
    if(c->enabled&&!c->topic[0])return false;
    for(const unsigned char *p=(const unsigned char*)c->topic;*p;p++)
        if(*p<=32||*p>=127||*p=='+'||*p=='#')return false;
    /* Eigene Ausgangstopics duerfen nicht als Eingang zurueckgefuehrt werden. */
    if(c->topic[0]&&(!strncmp(c->topic,"heizung/anlage/",15)||!strncmp(c->topic,"heizung/pv/",11)))return false;
    return isfinite(c->on_c)&&isfinite(c->off_c)&&c->off_c>=-20&&c->on_c<=250&&c->off_c<c->on_c&&c->stale_s>=10&&c->stale_s<=3600;
}
bool stove_parse_temperature(const char *text,float *value) {
    bool digit=false;
    for(const unsigned char *p=(const unsigned char*)text;*p;p++) {
        if(isdigit(*p)){digit=true;continue;}
        if(!isspace(*p)&&*p!='.'&&*p!='-'&&*p!='+'&&*p!='e'&&*p!='E')return false;
    }
    if(!digit)return false;
    char *end;errno=0;float t=strtof(text,&end);
    if(end==text||errno==ERANGE||!isfinite(t)||t < -55||t>500)return false;
    while(isspace((unsigned char)*end))end++;
    if(*end)return false;
    *value=t;return true;
}
void stove_expire(stove_state_t *s,const stove_config_t *c,int64_t now) {
    if(!c->enabled||!s->valid||now<s->updated_ms||now-s->updated_ms>=(int64_t)c->stale_s*1000){s->valid=false;s->active=false;}
}
void stove_sample(stove_state_t *s,const stove_config_t *c,const char *payload,bool retained,int64_t now) {
    stove_expire(s,c,now);
    /* Retained-Snapshot hat kein belastbares Messalter und steuert nicht. */
    if(!c->enabled||retained)return;
    float t;
    if(!stove_parse_temperature(payload,&t)){
        s->valid=false;s->active=false;if(s->errors<UINT32_MAX)s->errors++;return;
    }
    bool active=s->valid&&s->active;
    if(t>=c->on_c)active=true;
    else if(t<=c->off_c)active=false;
    s->celsius=t;s->updated_ms=now;s->valid=true;s->active=active;
}
float stove_room_gain(const stove_state_t *s,float normal_gain) {
    return s->valid&&s->active?0:normal_gain;
}
