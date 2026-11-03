#pragma once
#include <stdint.h>
#include "esp_err.h"
typedef struct {uint64_t pin_bit_mask;int mode,pull_up_en,pull_down_en;} gpio_config_t;
#define GPIO_MODE_INPUT_OUTPUT_OD 1
#define GPIO_PULLUP_DISABLE 0
#define GPIO_PULLDOWN_DISABLE 0
int gpio_get_level(int);
esp_err_t gpio_set_level(int,int);
esp_err_t gpio_config(const gpio_config_t*);
