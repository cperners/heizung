#include "pv.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
static SemaphoreHandle_t lock;
static pv_status_t values;
static const char *topics[]={"heizung/pv/power","heizung/pv/surplus","heizung/pv/grid_import","heizung/pv/grid_export"};
void pv_init(void){lock=xSemaphoreCreateMutex();configASSERT(lock);}
bool pv_receive(const char *topic,const char *text,int64_t now){
 char *end;float v=strtof(text,&end);if(end==text||*end||!isfinite(v)||v<0||v>1000000)return false;
 for(int i=0;i<4;i++)if(!strcmp(topic,topics[i])){xSemaphoreTake(lock,portMAX_DELAY);
 values.watts[i]=v;values.updated_ms[i]=now;values.valid[i]=true;xSemaphoreGive(lock);return true;}return false;
}
void pv_get(pv_status_t *s,int64_t now){xSemaphoreTake(lock,portMAX_DELAY);*s=values;xSemaphoreGive(lock);
 for(int i=0;i<4;i++)if(now-s->updated_ms[i]>30000)s->valid[i]=false;
}
