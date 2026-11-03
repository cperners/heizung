#pragma once
#include "model.h"
#include "esp_err.h"
esp_err_t outputs_init(void);
void outputs_apply(const output_request_t *r,int64_t now,const heating_config_t *c);
void outputs_get(output_request_t *r,float *position,bool *timed_out);
void outputs_emergency_stop(void);
bool outputs_enabled(void);
