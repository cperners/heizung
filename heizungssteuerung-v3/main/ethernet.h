#pragma once
#include <stdbool.h>
#include "esp_err.h"
esp_err_t ethernet_init(void);
bool ethernet_has_ip(void);
