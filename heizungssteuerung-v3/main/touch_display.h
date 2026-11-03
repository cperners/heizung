#pragma once
#include "model.h"
/* Eigene niedrige Taskprioritaet; ein Displayfehler stoppt den Regler nicht. */
void touch_display_start(void (*get_status)(heating_status_t *));
