#pragma once
#include "esp_err.h"
#include "esp_http_server.h"
#include <stdint.h>
#include <stdbool.h>
esp_err_t ota_boot_start(void);
void ota_control_tick(int64_t now_ms);
void ota_boot_ready(void);
esp_err_t ota_upload(httpd_req_t *request,char *reason,unsigned size);
bool ota_restart_required(void);
