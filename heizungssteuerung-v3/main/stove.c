#include "stove.h"
#include "service_config.h"
#include "nvs.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
static stove_config_t current;
static stove_state_t state;
static SemaphoreHandle_t lock;
static bool connected;
#define HISTORY_POINTS 288
typedef struct {int64_t ms;float temperature;bool active;} history_point_t;
static history_point_t history[HISTORY_POINTS];
static unsigned history_count,history_next;
static void history_add(int64_t now){
 if(history_count){unsigned last=(history_next+HISTORY_POINTS-1)%HISTORY_POINTS;if(history[last].ms/300000==now/300000){history[last]=(history_point_t){now,state.celsius,state.active};return;}}
 history[history_next]=(history_point_t){now,state.celsius,state.active};history_next=(history_next+1)%HISTORY_POINTS;if(history_count<HISTORY_POINTS)history_count++;
}
esp_err_t stove_init(void) {
    lock=xSemaphoreCreateMutex();if(!lock)return ESP_ERR_NO_MEM;
    stove_defaults(&current);memset(&state,0,sizeof(state));connected=false;history_count=history_next=0;
    nvs_handle_t h;esp_err_t e=nvs_open("stove",NVS_READONLY,&h);
    if(e==ESP_ERR_NVS_NOT_FOUND)return ESP_OK;
    if(e!=ESP_OK)return e;
    stove_config_t saved;size_t n=sizeof(saved);e=nvs_get_blob(h,"config",&saved,&n);nvs_close(h);
    if(e==ESP_ERR_NVS_NOT_FOUND)return ESP_OK;
    if(e!=ESP_OK||n!=sizeof(saved)||!stove_validate(&saved))return ESP_ERR_INVALID_STATE;
    current=saved;return ESP_OK;
}
void stove_get(stove_config_t *c,stove_state_t *s,int64_t now) {
    xSemaphoreTake(lock,portMAX_DELAY);stove_expire(&state,&current,now);
    if(c)*c=current;
    if(s)*s=state;
    xSemaphoreGive(lock);
}
void stove_set_connected(bool online) {
    xSemaphoreTake(lock,portMAX_DELAY);connected=online;state.valid=false;state.active=false;xSemaphoreGive(lock);
}
bool stove_receive(const char *topic,const char *payload,bool retained,int64_t now) {
    xSemaphoreTake(lock,portMAX_DELAY);
    bool match=current.enabled&&!strcmp(topic,current.topic);
    if(match&&connected){stove_sample(&state,&current,payload,retained,now);if(!retained&&state.valid&&state.updated_ms==now)history_add(now);}
    xSemaphoreGive(lock);return match;
}
char *stove_config_json(void) {
    stove_config_t c;xSemaphoreTake(lock,portMAX_DELAY);c=current;xSemaphoreGive(lock);
    cJSON *j=cJSON_CreateObject();if(!j)return NULL;
    cJSON_AddBoolToObject(j,"enabled",c.enabled);cJSON_AddStringToObject(j,"topic",c.topic);
    cJSON_AddNumberToObject(j,"on_c",c.on_c);cJSON_AddNumberToObject(j,"off_c",c.off_c);
    cJSON_AddNumberToObject(j,"stale_s",c.stale_s);
    char *text=cJSON_PrintUnformatted(j);cJSON_Delete(j);return text;
}
static esp_err_t stove_config_update_impl(const char *text,char *reason,unsigned size,bool dry) {
    cJSON *j=cJSON_ParseWithOpts(text,NULL,true);
    if(!cJSON_IsObject(j)||!j->child){cJSON_Delete(j);snprintf(reason,size,"Ungueltiges JSON");return ESP_ERR_INVALID_ARG;}
    stove_config_t next;xSemaphoreTake(lock,portMAX_DELAY);next=current;xSemaphoreGive(lock);
    bool good=true;
    for(const cJSON *v=j->child;v;v=v->next) {
        for(const cJSON *p=j->child;p!=v;p=p->next)if(!strcmp(p->string,v->string))good=false;
        if(!strcmp(v->string,"enabled")) {if(!cJSON_IsBool(v)){good=false;break;}next.enabled=cJSON_IsTrue(v);}
        else if(!strcmp(v->string,"topic")) {
            if(!cJSON_IsString(v)||strlen(v->valuestring)>=sizeof(next.topic)){good=false;break;}
            snprintf(next.topic,sizeof(next.topic),"%s",v->valuestring);
        } else if(!strcmp(v->string,"on_c")||!strcmp(v->string,"off_c")) {
            if(!cJSON_IsNumber(v)||!isfinite(v->valuedouble)){good=false;break;}
            if(!strcmp(v->string,"on_c"))next.on_c=(float)v->valuedouble;else next.off_c=(float)v->valuedouble;
        } else if(!strcmp(v->string,"stale_s")) {
            if(!cJSON_IsNumber(v)||!isfinite(v->valuedouble)||v->valuedouble<10||v->valuedouble>3600||floor(v->valuedouble)!=v->valuedouble){good=false;break;}
            next.stale_s=(uint32_t)v->valuedouble;
        } else {good=false;break;}
    }
    cJSON_Delete(j);esp_err_t e=ESP_ERR_INVALID_ARG;
    if(good&&stove_validate(&next)&&(dry||!service_config_topic_is_own(next.topic))) {
        if(dry)return ESP_OK;
        nvs_handle_t h;e=nvs_open("stove",NVS_READWRITE,&h);
        if(e==ESP_OK){e=nvs_set_blob(h,"config",&next,sizeof(next));if(e==ESP_OK)e=nvs_commit(h);nvs_close(h);}
        if(e==ESP_OK){xSemaphoreTake(lock,portMAX_DELAY);current=next;state.valid=false;state.active=false;xSemaphoreGive(lock);}
    }
    snprintf(reason,size,"%s",e==ESP_OK?"Kachelofen gespeichert. Neustart in drei Sekunden.":e==ESP_ERR_INVALID_ARG?"Topic ungueltig/reserviert, Schwellen ungueltig (inaktiv < aktiv) oder Messalter ausserhalb 10-3600 s":"NVS konnte nicht speichern");
    return e;
}

esp_err_t stove_config_update(const char *text,char *reason,unsigned size){return stove_config_update_impl(text,reason,size,false);}
esp_err_t stove_config_update_validate(const char *text,char *reason,unsigned size){return stove_config_update_impl(text,reason,size,true);}

char *stove_history_json(int64_t now){
 cJSON *j=cJSON_CreateObject();if(!j)return NULL;
 cJSON *a=cJSON_AddArrayToObject(j,"points");if(!a){cJSON_Delete(j);return NULL;}
 xSemaphoreTake(lock,portMAX_DELAY);
 cJSON_AddNumberToObject(j,"stale_s",current.stale_s);cJSON_AddNumberToObject(j,"on_c",current.on_c);cJSON_AddNumberToObject(j,"off_c",current.off_c);
 for(unsigned i=0;i<history_count;i++){
  const history_point_t *p=&history[(history_next+HISTORY_POINTS-history_count+i)%HISTORY_POINTS];
  if(p->ms>now||now-p->ms>86400000)continue;
  cJSON *v=cJSON_CreateObject();if(!v)continue;
  cJSON_AddNumberToObject(v,"age_s",(now-p->ms)/1000);cJSON_AddNumberToObject(v,"temperature",p->temperature);cJSON_AddBoolToObject(v,"active",p->active);cJSON_AddItemToArray(a,v);
 }
 xSemaphoreGive(lock);char *text=cJSON_PrintUnformatted(j);cJSON_Delete(j);return text;
}
