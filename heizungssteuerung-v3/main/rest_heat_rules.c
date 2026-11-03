#include "rest_heat_rules.h"
#include <math.h>
#include <string.h>
void rest_heat_defaults(rest_heat_config_t *c){memset(c,0,sizeof(*c));c->version=1;c->enabled=true;c->stop_c=30;c->max_s=1800;}
bool rest_heat_validate(const rest_heat_config_t *c){return c->version==1&&isfinite(c->stop_c)&&c->stop_c>=20&&c->stop_c<=60&&c->max_s>=60&&c->max_s<=7200;}
