#pragma once
#include "stove_rules.h"
#include "esp_err.h"
esp_err_t stove_init(void);
void stove_get(stove_config_t *config,stove_state_t *state,int64_t now);
void stove_set_connected(bool connected);
bool stove_receive(const char *topic,const char *payload,bool retained,int64_t now);
char *stove_config_json(void);
esp_err_t stove_config_update(const char *json,char *reason,unsigned size);

esp_err_t stove_config_update_validate(const char *json,char *reason,unsigned size);

/* Bis zu 288 Fuenf-Minuten-Messpunkte seit dem Geraetestart. */
char *stove_history_json(int64_t now);
