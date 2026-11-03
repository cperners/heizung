#include "boiler.h"
bool boiler_step(boiler_state_t *s,const heating_config_t *c,bool valid,float t,bool enabled) {
 if(!valid||!enabled||t>=c->boiler_target) s->demand=false;
 else if(t<=c->boiler_target-c->boiler_hysteresis) s->demand=true;
 return s->demand;
}
