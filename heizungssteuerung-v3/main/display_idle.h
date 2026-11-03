#pragma once
#include <stdbool.h>
#include <stdint.h>
#define DISPLAY_IDLE_TIMEOUT_MS 60000u
typedef enum { DISPLAY_IDLE_NONE, DISPLAY_IDLE_BLANK, DISPLAY_IDLE_WAKE } display_idle_action_t;
typedef struct { uint32_t last_activity; bool blanked, suppress_touch; } display_idle_t;
void display_idle_init(display_idle_t *s,uint32_t now);
display_idle_action_t display_idle_step(display_idle_t *s,uint32_t now,bool contact,bool released);
void display_idle_commit(display_idle_t *s,display_idle_action_t action,bool success,uint32_t now);
bool display_idle_accept_touch(const display_idle_t *s);
