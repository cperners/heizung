#pragma once
#include <stdbool.h>
#include <stdint.h>
typedef struct {float watts[4];int64_t updated_ms[4];bool valid[4];} pv_status_t;
void pv_init(void);
bool pv_receive(const char *topic,const char *payload,int64_t now);
void pv_get(pv_status_t *s,int64_t now);
/* Reserviert: Hausverbrauch -> Warmwasser -> Puffer -> Verbraucher -> Netz.
   Kein Schaltpfad zum Heizstab, unabhängig von MQTT-Eingaben. */
