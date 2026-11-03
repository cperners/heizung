#pragma once
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "esp_http_server.h"
esp_err_t touch_access_init(void);
unsigned touch_access_epoch(void);
bool touch_access_unlock(const char *pin,unsigned epoch);
void touch_access_lock(void);
bool touch_access_unlocked(void);
esp_err_t touch_access_change(const char *pin);
bool touch_access_token(char *out,size_t size);
bool touch_access_authorized(httpd_req_t *request);
