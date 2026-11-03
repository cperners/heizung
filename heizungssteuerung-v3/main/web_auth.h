#pragma once
#include "esp_err.h"
#include <stdbool.h>
esp_err_t web_auth_init(void);
bool web_auth_configured(void);
bool web_auth_verify(const char *header);
esp_err_t web_auth_update(const char *json,char *reason,unsigned size);
