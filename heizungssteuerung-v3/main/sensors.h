#pragma once
#include "model.h"
#include "esp_err.h"
esp_err_t sensors_init(void);
void sensors_get(sensor_snapshot_t *s);

typedef enum {
 SENSOR_DIAG_WAITING,SENSOR_DIAG_DISABLED,SENSOR_DIAG_OK,SENSOR_DIAG_IDLE_LOW,
 SENSOR_DIAG_DRIVE_HIGH,SENSOR_DIAG_NO_PRESENCE,SENSOR_DIAG_RELEASE_LOW,
 SENSOR_DIAG_ROM_CRC,SENSOR_DIAG_FAMILY,SENSOR_DIAG_PARASITE,
 SENSOR_DIAG_CONVERSION,SENSOR_DIAG_SCRATCH_CRC,SENSOR_DIAG_SCRATCH_INVALID
} sensor_diag_code_t;
typedef struct {
 sensor_diag_code_t code;
 int gpio;
 bool presence,have_rom,have_scratch;
 uint8_t rom[8],scratch[9];
 uint32_t attempts,successes;
} sensor_diagnostic_t;
void sensors_get_diagnostics(sensor_diagnostic_t out[SENSOR_COUNT]);
const char *sensor_diagnostic_text(sensor_diag_code_t code);

#include "cJSON.h"
cJSON *sensors_history_json(int64_t now,unsigned span_s);
