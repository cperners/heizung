#pragma once
#include <stdbool.h>
#include <stdint.h>
typedef struct {unsigned version;bool enabled;float stop_c;uint32_t max_s;} rest_heat_config_t;
void rest_heat_defaults(rest_heat_config_t *c);
bool rest_heat_validate(const rest_heat_config_t *c);
