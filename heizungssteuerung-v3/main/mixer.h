#pragma once
#include "model.h"
typedef struct {
 mixer_dir_t direction,last_direction;
 int64_t last_ms, stopped_ms;
 float position; /* 0..1 nur geschätzt; -1 vor Referenzfahrt. */
 uint32_t run_ms;
} mixer_state_t;
void mixer_init(mixer_state_t *s,int64_t now);
mixer_dir_t mixer_step(mixer_state_t *s,mixer_dir_t wanted,int64_t now,uint32_t dead_ms,uint32_t travel_s);
