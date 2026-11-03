#pragma once
#include <stdbool.h>
#include <stdint.h>
#define GAS_TOPIC_SIZE 192
typedef struct {unsigned version;bool enabled;char topic[GAS_TOPIC_SIZE];uint32_t stale_s;} gas_config_t;
typedef struct {
    unsigned version;
    uint32_t day,resets;
    double baseline,total;
    int64_t baseline_epoch,last_epoch;
} gas_meter_t;
void gas_defaults(gas_config_t *c);
bool gas_validate(const gas_config_t *c);
bool gas_parse_total(const char *text,double *total);
bool gas_meter_validate(const gas_meter_t *m);
/* true: Tagesbasis/Zaehlerreset neu; deshalb sofort sichern. */
bool gas_meter_sample(gas_meter_t *m,double total,uint32_t local_day,int64_t epoch);
