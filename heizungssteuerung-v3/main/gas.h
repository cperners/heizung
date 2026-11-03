#pragma once
#include "gas_rules.h"
#include "esp_err.h"
#include "cJSON.h"
esp_err_t gas_init(void);
void gas_config_get(gas_config_t *c);
void gas_set_connected(bool connected);
bool gas_receive(const char *topic,const char *payload,bool retained,int64_t received_ms);
char *gas_config_json(void);
esp_err_t gas_config_update(const char *json,char *reason,unsigned size);
cJSON *gas_status_json(int64_t now_ms);

esp_err_t gas_config_update_validate(const char *json,char *reason,unsigned size);
