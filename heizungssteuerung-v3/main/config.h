#pragma once
#include "model.h"
#include <stddef.h>
void config_defaults(heating_config_t *c);
bool config_validate(const heating_config_t *c);
bool config_migrate_v1(const void *blob,size_t size,heating_config_t *out);
const char *config_mode_name(heating_mode_t mode);
bool config_mode_parse(const char *text,heating_mode_t *mode);
#ifndef HOST_TEST
#include "esp_err.h"
esp_err_t config_init(void);
void config_get(heating_config_t *c);
/* Atomarer vollständiger JSON-Update; NVS-Commit vor Übernahme. */
esp_err_t config_update_json(const char *json, char *reason, unsigned size);
char *config_json(void); /* Aufrufer gibt mit free() frei. */

/* Oeffentliche Betriebsartwahl: ausschliesslich eines der sechs Modi. */
esp_err_t config_mode_update_json(const char *json,char *reason,unsigned size);

esp_err_t config_update_json_validate(const char *json,char *reason,unsigned size);
#endif
