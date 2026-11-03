#pragma once
#include "model.h"
typedef struct { bool demand; } boiler_state_t;
bool boiler_step(boiler_state_t *s,const heating_config_t *c,bool valid,float temp,bool enabled);
