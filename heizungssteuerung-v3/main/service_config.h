#pragma once
#include "esp_err.h"
#include <stdbool.h>
#include "mqtt_topics.h"
typedef struct { unsigned version; char uri[192], username[64], password[128]; char ntp_primary[128], ntp_secondary[128], ntp_tertiary[128]; char base_topic[MQTT_BASE_SIZE]; } service_config_t;
esp_err_t service_config_init(void);
void service_config_get(service_config_t *s);
char *service_config_json(void); /* Passwort wird niemals ausgegeben. */
esp_err_t service_config_update(const char *json,char *reason,unsigned size);

esp_err_t service_config_update_validate(const char *json,char *reason,unsigned size);

bool service_config_topic_is_own(const char *topic);
