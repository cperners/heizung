#pragma once
#include "rest_heat_rules.h"
#include "esp_err.h"
esp_err_t rest_heat_init(void);
void rest_heat_get(rest_heat_config_t *c);
char *rest_heat_json(void);
esp_err_t rest_heat_update(const char *text,char *reason,unsigned size);
esp_err_t rest_heat_update_validate(const char *text,char *reason,unsigned size);
