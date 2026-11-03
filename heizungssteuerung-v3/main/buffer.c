#include "buffer.h"
#include <math.h>
buffer_status_t buffer_evaluate(const sensor_snapshot_t *s,bool enabled,int64_t now,uint32_t stale_s) {
 buffer_status_t b={0};if(!enabled)return b;
 for(int i=BUFFER_TOP;i<=BUFFER_BOTTOM;i++)if(!s->value[i].valid||!isfinite(s->value[i].celsius)
 ||now-s->value[i].sampled_ms>=(int64_t)stale_s*1000)return b;
 b.available=true;b.top=s->value[BUFFER_TOP].celsius;b.mid=s->value[BUFFER_MID].celsius;
 b.bottom=s->value[BUFFER_BOTTOM].celsius;b.mean=(b.top+b.mid+b.bottom)/3;return b;
}
