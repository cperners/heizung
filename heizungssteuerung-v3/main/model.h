#pragma once
#include <stdbool.h>
#include <stdint.h>
#define SENSOR_COUNT 8
#define PERIOD_COUNT 4
typedef enum { OUTSIDE, ROOM, KESSEL, FLOW, DHW, BUFFER_TOP, BUFFER_MID, BUFFER_BOTTOM } sensor_id_t;
typedef struct { float celsius; bool valid; uint32_t failures; int64_t sampled_ms; } sensor_value_t;
typedef struct { sensor_value_t value[SENSOR_COUNT]; } sensor_snapshot_t;
typedef enum { MODE_OFF, MODE_AUTO, MODE_MANUAL, MODE_HEATING_ONLY, MODE_DHW_ONLY, MODE_HOLIDAY, MODE_FROST } heating_mode_t;
typedef struct { uint16_t start_minute; float room_target; } period_t;
typedef struct {
 uint32_t version;
 heating_mode_t mode;
 float curve_zero, curve_slope, room_gain, room_target, unsynced_room;
 float flow_min, flow_max, summer_temp, boiler_target, boiler_hysteresis;
 float kessel_max, kessel_margin, burner_hysteresis, dhw_lift;
 float frost_outside, frost_water, frost_flow, mixer_band;
 uint32_t burner_min_on_s, burner_min_off_s, pump_overrun_s;
 uint32_t mixer_travel_s, mixer_dead_ms, sensor_stale_s;
 bool dhw_priority, buffer_enabled, pv_enabled;
 period_t periods[PERIOD_COUNT];
 float holiday_room_target;
} heating_config_t;
typedef enum { MIX_STOP, MIX_OPEN, MIX_CLOSE } mixer_dir_t;
typedef struct {
 bool burner, heating_pump, boiler_pump;
 mixer_dir_t mixer;
 bool heater; /* Immer false: Leistungselektronik noch nicht implementiert. */
} output_request_t;
enum { FAULT_KESSEL=1, FAULT_FLOW=2, FAULT_KESSEL_HOT=4, FAULT_FLOW_HOT=8,
 FAULT_DHW=16, FAULT_OUTSIDE=32, FAULT_ROOM=64, FAULT_TIME=128, FAULT_NETWORK=256 };
typedef struct {
 output_request_t outputs;
 float room_target, flow_target, kessel_target;
 uint32_t faults;
 bool frost, dhw_demand;
 bool rest_heat_active;uint32_t rest_heat_remaining_s;
 bool room_correction_suppressed;
} heating_status_t;
