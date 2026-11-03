#pragma once
#include "esp_err.h"
char *backup_export(void);
esp_err_t backup_stage(const char *text,char *reason,unsigned size);
esp_err_t backup_apply_pending(void);
