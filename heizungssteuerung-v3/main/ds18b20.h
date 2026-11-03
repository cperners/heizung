#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
uint8_t ds18b20_crc(const uint8_t *data,size_t len);
bool ds18b20_decode(const uint8_t scratch[9],float *celsius);
