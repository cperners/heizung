#pragma once
#include "model.h"
#include "boiler.h"
#include "rest_heat_rules.h"
typedef struct {
 boiler_state_t boiler;
 bool burner, heat_pump, dhw_pump, burner_demand;
 int64_t burner_changed_ms, heat_until_ms, dhw_until_ms;
 bool mode_seen,rest_active;heating_mode_t previous_mode;int64_t rest_started_ms;float last_flow_target;
} heating_state_t;
void heating_init(heating_state_t *s,int64_t now);
void heating_step(heating_state_t *s,const heating_config_t *c,const sensor_snapshot_t *v,
 int64_t now,int minute,bool time_valid,bool network_ok,heating_status_t *out);

/* Bei abgeschalteter Raumrueckfuehrung bleiben Sollwertabsenkungen wirksam. */
void heating_step_with_room_control(heating_state_t *s,const heating_config_t *c,const sensor_snapshot_t *v,
 int64_t now,int minute,bool time_valid,bool network_ok,bool room_feedback,heating_status_t *out);

void heating_step_with_rest_heat(heating_state_t *s,const heating_config_t *c,const sensor_snapshot_t *v,
 int64_t now,int minute,bool time_valid,bool network_ok,bool room_feedback,
 const rest_heat_config_t *rest,heating_status_t *out);
