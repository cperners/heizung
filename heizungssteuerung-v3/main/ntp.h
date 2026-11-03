#pragma once
#include <stdbool.h>
#define HEATING_TIMEZONE "CET-1CEST,M3.5.0/2,M10.5.0/3"
void ntp_init(void);
bool ntp_local_minute(int *minute);

/* Sekunden seit letzter erfolgreicher Synchronisation; -1 vor dem ersten Sync. */
long ntp_sync_age_s(void);
