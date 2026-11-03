#include "mixer.h"
#include <string.h>
void mixer_init(mixer_state_t *s,int64_t now) {
 memset(s,0,sizeof(*s)); s->position=-1;s->last_ms=now;s->stopped_ms=now;
}
mixer_dir_t mixer_step(mixer_state_t *s,mixer_dir_t want,int64_t now,uint32_t dead,uint32_t travel) {
 int64_t dt=now-s->last_ms;s->last_ms=now;
 if(dt<0) dt=0;
 if(s->direction!=MIX_STOP) {
  s->run_ms+=(uint32_t)dt;
  if(s->position>=0) { s->position+=(s->direction==MIX_OPEN?1:-1)*(float)dt/(travel*1000.0f);
   if(s->position<0)s->position=0;
   if(s->position>1)s->position=1; }
  if(s->run_ms>=travel*1000) {s->position=s->direction==MIX_OPEN?1:0;want=MIX_STOP;}
 }
 if(want<MIX_STOP||want>MIX_CLOSE) want=MIX_STOP;
 if(s->position<0&&want==MIX_OPEN) want=MIX_CLOSE; /* Referenzfahrt zuerst ZU. */
 if((s->position==0&&want==MIX_CLOSE)||(s->position==1&&want==MIX_OPEN))want=MIX_STOP;
 if(s->direction!=MIX_STOP&&want!=s->direction) {
  s->last_direction=s->direction;s->direction=MIX_STOP;s->stopped_ms=now;s->run_ms=0;
  return MIX_STOP;
 }
 if(want==MIX_STOP)return s->direction=MIX_STOP;
 if(s->direction==MIX_STOP) {
  if(s->last_direction!=MIX_STOP&&want!=s->last_direction&&now-s->stopped_ms<dead)return MIX_STOP;
  s->direction=want; s->run_ms=0;
 }
 return s->direction;
}
