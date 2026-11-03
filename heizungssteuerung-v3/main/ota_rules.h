#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define OTA_PREFIX_SIZE 288
bool ota_prefix_valid(const uint8_t *bytes,size_t length,unsigned chip_revision,const char *project);
