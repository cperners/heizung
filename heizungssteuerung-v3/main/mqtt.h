#pragma once
#include "model.h"
#include "esp_err.h"
esp_err_t mqtt_init(void);
bool mqtt_connected(void);
void mqtt_publish_status(const sensor_snapshot_t *v,const heating_config_t *c,const heating_status_t *s);
