#pragma once
#include "model.h"
#include "esp_err.h"
typedef void (*web_status_reader_t)(heating_status_t *status);
esp_err_t web_init(web_status_reader_t reader);
