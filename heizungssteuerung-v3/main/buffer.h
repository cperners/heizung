#pragma once
#include "model.h"
typedef struct { bool available; float top,mid,bottom,mean; } buffer_status_t;
buffer_status_t buffer_evaluate(const sensor_snapshot_t *s,bool enabled,int64_t now,uint32_t stale_s);
