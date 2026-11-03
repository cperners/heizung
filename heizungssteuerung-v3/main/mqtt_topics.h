#pragma once
#include <stdbool.h>
#include <stddef.h>
#define MQTT_BASE_SIZE 160
bool mqtt_base_valid(const char *base);
bool mqtt_topic_under(const char *base,const char *topic);
bool mqtt_topic_join(char *out,size_t size,const char *base,const char *suffix);
