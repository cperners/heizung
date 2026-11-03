#include "display_idle.h"
void display_idle_init(display_idle_t *s,uint32_t now){*s=(display_idle_t){.last_activity=now};}
display_idle_action_t display_idle_step(display_idle_t *s,uint32_t now,bool contact,bool released){
 if(contact){s->last_activity=now;if(s->blanked)return DISPLAY_IDLE_WAKE;}
 if(released&&s->suppress_touch){s->suppress_touch=false;s->last_activity=now;}
 if(!s->blanked&&!s->suppress_touch&&(uint32_t)(now-s->last_activity)>=DISPLAY_IDLE_TIMEOUT_MS)return DISPLAY_IDLE_BLANK;
 return DISPLAY_IDLE_NONE;
}
void display_idle_commit(display_idle_t *s,display_idle_action_t action,bool success,uint32_t now){
 if(!success)return;
 if(action==DISPLAY_IDLE_BLANK)s->blanked=true;
 if(action==DISPLAY_IDLE_WAKE){s->blanked=false;s->suppress_touch=true;s->last_activity=now;}
}
bool display_idle_accept_touch(const display_idle_t *s){return !s->blanked&&!s->suppress_touch;}
