#include "gas.h"
#include "service_config.h"
#include "ntp.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
typedef struct {unsigned version;char topic[GAS_TOPIC_SIZE];gas_meter_t meter;} gas_saved_t;
static SemaphoreHandle_t lock;
static gas_config_t config;
static gas_meter_t meter;
static bool connected,received,dated,known,storage_error;
static double total;
static int64_t updated_ms,last_checkpoint;
static uint32_t errors;
static uint32_t day_at(time_t t){struct tm local;localtime_r(&t,&local);return (local.tm_year+1900)*10000+(local.tm_mon+1)*100+local.tm_mday;}
esp_err_t gas_init(void){
    if(!lock)lock=xSemaphoreCreateMutex();
    if(!lock)return ESP_ERR_NO_MEM;
    gas_defaults(&config);
    memset(&meter,0,sizeof(meter));connected=received=dated=known=storage_error=false;total=0;updated_ms=last_checkpoint=0;errors=0;
    nvs_handle_t h;esp_err_t e=nvs_open("gas",NVS_READONLY,&h);
    if(e==ESP_ERR_NVS_NOT_FOUND)return ESP_OK;
    if(e!=ESP_OK){storage_error=true;return ESP_OK;}
    gas_config_t saved;size_t n=sizeof(saved);e=nvs_get_blob(h,"config",&saved,&n);
    if(e==ESP_OK&&n==sizeof(saved)&&gas_validate(&saved))config=saved;
    else if(e!=ESP_ERR_NVS_NOT_FOUND)storage_error=true;
    gas_saved_t record;n=sizeof(record);e=nvs_get_blob(h,"meter",&record,&n);nvs_close(h);
    if(e==ESP_OK&&n==sizeof(record)&&record.version==1&&memchr(record.topic,0,sizeof(record.topic))&&gas_meter_validate(&record.meter)){
        if(!strcmp(record.topic,config.topic)){meter=record.meter;total=meter.total;known=true;}
    }else if(e!=ESP_ERR_NVS_NOT_FOUND)storage_error=true;
    return ESP_OK;
}
void gas_config_get(gas_config_t *c){xSemaphoreTake(lock,portMAX_DELAY);*c=config;xSemaphoreGive(lock);}
void gas_set_connected(bool online){xSemaphoreTake(lock,portMAX_DELAY);connected=online;received=false;xSemaphoreGive(lock);}
bool gas_receive(const char *topic,const char *payload,bool retained,int64_t received_ms){
    gas_saved_t snapshot={.version=1};bool checkpoint=false;
    int minute;bool clock=ntp_local_minute(&minute);int64_t now=esp_timer_get_time()/1000;
    int64_t epoch=(int64_t)time(NULL)-(now>=received_ms?(now-received_ms)/1000:0);
    xSemaphoreTake(lock,portMAX_DELAY);
    bool match=config.enabled&&!strcmp(topic,config.topic);
    if(!match||!connected||retained){xSemaphoreGive(lock);return match;}
    double value;
    if(!gas_parse_total(payload,&value)){
        if(errors<UINT32_MAX)errors++;
        received=false;xSemaphoreGive(lock);return true;
    }
    total=value;known=true;received=true;updated_ms=received_ms;dated=clock;
    if(clock){
        checkpoint=gas_meter_sample(&meter,value,day_at((time_t)epoch),epoch)||now-last_checkpoint>=900000||storage_error;
        if(checkpoint){snapshot.meter=meter;snprintf(snapshot.topic,sizeof(snapshot.topic),"%s",config.topic);}
    }
    xSemaphoreGive(lock);
    /* Nur MQTT-Task schreibt Messdaten; keine NVS-Sperre im Heizungsregler. */
    if(checkpoint){
        nvs_handle_t h;esp_err_t e=nvs_open("gas",NVS_READWRITE,&h);
        if(e==ESP_OK){e=nvs_set_blob(h,"meter",&snapshot,sizeof(snapshot));if(e==ESP_OK)e=nvs_commit(h);nvs_close(h);}
        xSemaphoreTake(lock,portMAX_DELAY);
        if(!strcmp(snapshot.topic,config.topic)){storage_error=e!=ESP_OK;if(e==ESP_OK)last_checkpoint=now;}
        xSemaphoreGive(lock);
        if(e!=ESP_OK)ESP_LOGW("gas","Zaehlerstand konnte nicht gesichert werden");
    }
    return true;
}
char *gas_config_json(void){
    gas_config_t c;gas_config_get(&c);cJSON *j=cJSON_CreateObject();if(!j)return NULL;
    cJSON_AddBoolToObject(j,"enabled",c.enabled);cJSON_AddStringToObject(j,"topic",c.topic);cJSON_AddNumberToObject(j,"stale_s",c.stale_s);
    char *s=cJSON_PrintUnformatted(j);cJSON_Delete(j);return s;
}
static esp_err_t gas_config_update_impl(const char *text,char *reason,unsigned size,bool dry){
    cJSON *j=cJSON_ParseWithOpts(text,NULL,true);
    if(!cJSON_IsObject(j)||!j->child){cJSON_Delete(j);snprintf(reason,size,"Ungueltiges JSON");return ESP_ERR_INVALID_ARG;}
    const cJSON *basis=cJSON_GetObjectItemCaseSensitive(j,"baseline_m3");
    if(basis){
        if(dry){cJSON_Delete(j);return ESP_ERR_INVALID_ARG;}
        int minute;bool clock=ntp_local_minute(&minute);uint32_t today=clock?day_at(time(NULL)):0;
        xSemaphoreTake(lock,portMAX_DELAY);
        bool valid=j->child==basis&&!basis->next&&cJSON_IsNumber(basis)&&isfinite(basis->valuedouble)
          &&config.enabled&&clock&&gas_meter_validate(&meter)&&meter.day==today
          &&basis->valuedouble>=0&&basis->valuedouble<=meter.total;
        esp_err_t e=ESP_ERR_INVALID_ARG;
        if(valid){
            gas_saved_t save={.version=1,.meter=meter};save.meter.baseline=basis->valuedouble;
            snprintf(save.topic,sizeof(save.topic),"%s",config.topic);
            nvs_handle_t h;e=nvs_open("gas",NVS_READWRITE,&h);
            if(e==ESP_OK){e=nvs_set_blob(h,"meter",&save,sizeof(save));if(e==ESP_OK)e=nvs_commit(h);nvs_close(h);}
            if(e==ESP_OK){meter=save.meter;storage_error=false;last_checkpoint=esp_timer_get_time()/1000;}
        }
        xSemaphoreGive(lock);cJSON_Delete(j);
        snprintf(reason,size,"%s",e==ESP_OK?"Tagesbasis gespeichert. Neustart erfolgt.":e==ESP_ERR_INVALID_ARG?"Aktuelle Tagesbasis erforderlich; Korrektur muss zwischen null und Gesamtstand liegen":"NVS konnte nicht speichern");return e;
    }
    gas_config_t next;gas_config_get(&next);bool okay=true;
    for(const cJSON *v=j->child;v;v=v->next){
        for(const cJSON *p=j->child;p!=v;p=p->next)if(!strcmp(p->string,v->string))okay=false;
        if(!strcmp(v->string,"enabled")){if(!cJSON_IsBool(v)){okay=false;break;}next.enabled=cJSON_IsTrue(v);}
        else if(!strcmp(v->string,"topic")){
            if(!cJSON_IsString(v)||strlen(v->valuestring)>=sizeof(next.topic)){okay=false;break;}
            snprintf(next.topic,sizeof(next.topic),"%s",v->valuestring);
        }else if(!strcmp(v->string,"stale_s")){
            if(!cJSON_IsNumber(v)||!isfinite(v->valuedouble)||v->valuedouble<10||v->valuedouble>86400||floor(v->valuedouble)!=v->valuedouble){okay=false;break;}
            next.stale_s=(uint32_t)v->valuedouble;
        }else{okay=false;break;}
    }
    cJSON_Delete(j);esp_err_t e=ESP_ERR_INVALID_ARG;
    if(okay&&gas_validate(&next)&&(dry||!service_config_topic_is_own(next.topic))){
        if(dry)return ESP_OK;
        nvs_handle_t h;e=nvs_open("gas",NVS_READWRITE,&h);
        if(e==ESP_OK){e=nvs_set_blob(h,"config",&next,sizeof(next));if(e==ESP_OK)e=nvs_commit(h);nvs_close(h);}
        if(e==ESP_OK){xSemaphoreTake(lock,portMAX_DELAY);
            if(strcmp(config.topic,next.topic)){memset(&meter,0,sizeof(meter));known=false;dated=false;last_checkpoint=0;}
            config=next;received=false;xSemaphoreGive(lock);
        }
    }
    snprintf(reason,size,"%s",e==ESP_OK?"Gaszaehler gespeichert. Neustart erfolgt.":e==ESP_ERR_INVALID_ARG?"Topic ungueltig/reserviert oder Messalter ausserhalb 10-86400 s":"NVS konnte nicht speichern");return e;
}
cJSON *gas_status_json(int64_t now){
    int minute;bool clock=ntp_local_minute(&minute);uint32_t today=clock?day_at(time(NULL)):0;
    gas_config_t c;gas_meter_t m;bool fresh,have,day_valid,store;double value;uint32_t errs;int64_t age;
    xSemaphoreTake(lock,portMAX_DELAY);c=config;m=meter;value=total;have=known;
    age=received&&updated_ms<=now?(now-updated_ms)/1000:-1;
    fresh=c.enabled&&connected&&received&&age>=0&&age<c.stale_s;
    day_valid=c.enabled&&have&&gas_meter_validate(&m)&&clock&&m.day==today&&(dated||!received);
    store=storage_error;errs=errors;xSemaphoreGive(lock);
    cJSON *j=cJSON_CreateObject();if(!j)return NULL;
    cJSON_AddBoolToObject(j,"enabled",c.enabled);cJSON_AddBoolToObject(j,"fresh",fresh);
    cJSON_AddItemToObject(j,"total_m3",c.enabled&&have?cJSON_CreateNumber(value):cJSON_CreateNull());
    cJSON_AddItemToObject(j,"today_m3",day_valid?cJSON_CreateNumber(m.total-m.baseline):cJSON_CreateNull());
    cJSON_AddItemToObject(j,"age_s",age>=0?cJSON_CreateNumber(age):cJSON_CreateNull());
    cJSON_AddBoolToObject(j,"time_synced",clock);cJSON_AddBoolToObject(j,"storage_error",store);
    cJSON_AddNumberToObject(j,"errors",errs);cJSON_AddNumberToObject(j,"resets",m.resets);
    char since[48]="";if(day_valid){time_t t=(time_t)m.baseline_epoch;struct tm local;localtime_r(&t,&local);strftime(since,sizeof(since),"%d.%m.%Y %H:%M",&local);}
    cJSON_AddStringToObject(j,"since",since);return j;
}

esp_err_t gas_config_update(const char *text,char *reason,unsigned size){return gas_config_update_impl(text,reason,size,false);}
esp_err_t gas_config_update_validate(const char *text,char *reason,unsigned size){return gas_config_update_impl(text,reason,size,true);}
