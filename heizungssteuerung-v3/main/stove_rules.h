#pragma once
#include <stdbool.h>
#include <stdint.h>
#define STOVE_TOPIC_SIZE 192
typedef struct {
    unsigned version;
    bool enabled;
    char topic[STOVE_TOPIC_SIZE];
    float on_c,off_c;
    uint32_t stale_s;
} stove_config_t;
typedef struct {
    bool valid,active;
    float celsius;
    int64_t updated_ms;
    uint32_t errors;
} stove_state_t;
void stove_defaults(stove_config_t *c);
bool stove_validate(const stove_config_t *c);
bool stove_parse_temperature(const char *payload,float *celsius);
void stove_sample(stove_state_t *s,const stove_config_t *c,const char *payload,bool retained,int64_t now);
void stove_expire(stove_state_t *s,const stove_config_t *c,int64_t now);
float stove_room_gain(const stove_state_t *s,float normal_gain);
