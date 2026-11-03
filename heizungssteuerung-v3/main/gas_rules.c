#include "gas_rules.h"
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <stdlib.h>
#include <math.h>
void gas_defaults(gas_config_t *c){memset(c,0,sizeof(*c));c->version=1;c->stale_s=3600;}
bool gas_validate(const gas_config_t *c){
    if(c->version!=1||!memchr(c->topic,0,sizeof(c->topic))||(c->enabled&&!c->topic[0])||c->stale_s<10||c->stale_s>86400)return false;
    for(const unsigned char *p=(const unsigned char*)c->topic;*p;p++)if(*p<=32||*p>=127||*p=='#'||*p=='+')return false;
    return !c->topic[0]||(strncmp(c->topic,"heizung/anlage/",15)&&strncmp(c->topic,"heizung/pv/",11));
}
bool gas_parse_total(const char *text,double *total){
    bool digit=false;
    for(const unsigned char *p=(const unsigned char*)text;*p;p++){
        if(isdigit(*p)){digit=true;continue;}
        if(!isspace(*p)&&*p!='.'&&*p!='-'&&*p!='+'&&*p!='e'&&*p!='E')return false;
    }
    char *end;errno=0;double v=strtod(text,&end);
    if(!digit||end==text||errno==ERANGE||!isfinite(v)||v<0||v>1e9)return false;
    while(isspace((unsigned char)*end))end++;
    if(*end)return false;
    *total=v;return true;
}
bool gas_meter_validate(const gas_meter_t *m){
    return m->version==1&&m->day>=20200101&&m->day<=99991231&&isfinite(m->baseline)&&isfinite(m->total)
      &&m->baseline>=0&&m->total>=m->baseline&&m->total<=1e9&&m->baseline_epoch>0&&m->last_epoch>=m->baseline_epoch;
}
bool gas_meter_sample(gas_meter_t *m,double total,uint32_t day,int64_t epoch){
    bool had=gas_meter_validate(m),reset=had&&total<m->total;
    bool fresh=!had||m->day!=day||reset;
    uint32_t resets=had?m->resets:0;
    if(reset&&resets<UINT32_MAX)resets++;
    if(fresh){memset(m,0,sizeof(*m));m->version=1;m->day=day;m->baseline=total;m->baseline_epoch=epoch;m->resets=resets;}
    m->total=total;m->last_epoch=epoch;
    return fresh;
}
