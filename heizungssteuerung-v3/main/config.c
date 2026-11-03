#include "config.h"
#include "cJSON.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stddef.h>
static heating_config_t current;
static SemaphoreHandle_t lock;
typedef enum { F_FLOAT,F_INT,F_BOOL } field_type_t;
typedef struct {const char *name;size_t offset;field_type_t type;} field_t;
#define F(n,t) {#n,offsetof(heating_config_t,n),t}
static const field_t fields[]={
 F(curve_zero,F_FLOAT),F(curve_slope,F_FLOAT),F(room_gain,F_FLOAT),F(room_target,F_FLOAT),
 F(holiday_room_target,F_FLOAT),F(unsynced_room,F_FLOAT),F(flow_min,F_FLOAT),F(flow_max,F_FLOAT),F(summer_temp,F_FLOAT),
 F(boiler_target,F_FLOAT),F(boiler_hysteresis,F_FLOAT),F(kessel_max,F_FLOAT),
 F(kessel_margin,F_FLOAT),F(burner_hysteresis,F_FLOAT),F(dhw_lift,F_FLOAT),
 F(frost_outside,F_FLOAT),F(frost_water,F_FLOAT),F(frost_flow,F_FLOAT),F(mixer_band,F_FLOAT),
 F(burner_min_on_s,F_INT),F(burner_min_off_s,F_INT),F(pump_overrun_s,F_INT),
 F(mixer_travel_s,F_INT),F(mixer_dead_ms,F_INT),F(sensor_stale_s,F_INT),
 F(dhw_priority,F_BOOL),F(buffer_enabled,F_BOOL),F(pv_enabled,F_BOOL)};
static esp_err_t save(const heating_config_t *c) {
 nvs_handle_t h;esp_err_t e=nvs_open("heating",NVS_READWRITE,&h);if(e!=ESP_OK)return e;
 e=nvs_set_blob(h,"config_v2",c,sizeof(*c));if(e==ESP_OK)e=nvs_commit(h);nvs_close(h);return e;
}
esp_err_t config_init(void) {
 lock=xSemaphoreCreateMutex();if(!lock)return ESP_ERR_NO_MEM;config_defaults(&current);
 nvs_handle_t h;esp_err_t e=nvs_open("heating",NVS_READONLY,&h);
 if(e==ESP_ERR_NVS_NOT_FOUND)return ESP_OK;
 if(e!=ESP_OK)return e;
 heating_config_t saved;size_t n=sizeof(saved);e=nvs_get_blob(h,"config_v2",&saved,&n);
 if(e==ESP_ERR_NVS_NOT_FOUND) {
  unsigned char legacy[sizeof(heating_config_t)];n=sizeof(legacy);
  e=nvs_get_blob(h,"config_v1",legacy,&n);nvs_close(h);
  if(e==ESP_ERR_NVS_NOT_FOUND)return ESP_OK;
  if(e!=ESP_OK||!config_migrate_v1(legacy,n,&saved))return ESP_ERR_INVALID_STATE;
  e=save(&saved);if(e!=ESP_OK)return e;
 } else {
  nvs_close(h);
  if(e!=ESP_OK||n!=sizeof(saved)||!config_validate(&saved))return ESP_ERR_INVALID_STATE;
 }
 current=saved;return ESP_OK;
}
void config_get(heating_config_t *c) {xSemaphoreTake(lock,portMAX_DELAY);*c=current;xSemaphoreGive(lock);}
static bool parse(heating_config_t *c,const cJSON *j) {
 if(!cJSON_IsObject(j)||!j->child)return false;
 for(const cJSON *v=j->child;v;v=v->next) {
  for(const cJSON *prev=j->child;prev!=v;prev=prev->next)if(!strcmp(prev->string,v->string))return false;
  if(!strcmp(v->string,"mode")) {
   if(!cJSON_IsString(v))return false;
   if(!config_mode_parse(v->valuestring,&c->mode))return false;
   continue;
  }
  if(!strcmp(v->string,"periods")) {
   if(!cJSON_IsArray(v)||cJSON_GetArraySize(v)!=PERIOD_COUNT)return false;
   for(int i=0;i<PERIOD_COUNT;i++) {
    const cJSON *p=cJSON_GetArrayItem(v,i),*start=cJSON_GetObjectItemCaseSensitive(p,"start_minute"),
     *target=cJSON_GetObjectItemCaseSensitive(p,"room_target");
    if(!cJSON_IsObject(p)||cJSON_GetArraySize(p)!=2||!cJSON_IsNumber(start)||!cJSON_IsNumber(target)
     ||!isfinite(start->valuedouble)||start->valuedouble<0||start->valuedouble>=1440
     ||floor(start->valuedouble)!=start->valuedouble)return false;
    c->periods[i]=(period_t){(uint16_t)start->valuedouble,(float)target->valuedouble};
   }continue;
  }
  bool found=false;
  for(size_t i=0;i<sizeof(fields)/sizeof(fields[0]);i++)if(!strcmp(v->string,fields[i].name)) {
   void *dest=(char*)c+fields[i].offset;found=true;
   if(fields[i].type==F_BOOL) {if(!cJSON_IsBool(v))return false;*(bool*)dest=cJSON_IsTrue(v);}
   else {if(!cJSON_IsNumber(v)||!isfinite(v->valuedouble))return false;
    if(fields[i].type==F_FLOAT)*(float*)dest=(float)v->valuedouble;
    else {if(v->valuedouble<0||v->valuedouble>UINT32_MAX||floor(v->valuedouble)!=v->valuedouble)return false;
     *(uint32_t*)dest=(uint32_t)v->valuedouble;}}
   break;
  }
  if(!found)return false;
 }return config_validate(c);
}
static esp_err_t config_update_json_impl(const char *text,char *reason,unsigned size,bool dry) {
 const char *end=NULL;cJSON *j=cJSON_ParseWithOpts(text,&end,true);
 if(!j) {snprintf(reason,size,"ungueltiges JSON");return ESP_ERR_INVALID_ARG;}
 xSemaphoreTake(lock,portMAX_DELAY);heating_config_t next=current;
 esp_err_t e=ESP_ERR_INVALID_ARG;
 if(parse(&next,j)) {e=dry?ESP_OK:save(&next);if(e==ESP_OK&&!dry)current=next;}
 xSemaphoreGive(lock);cJSON_Delete(j);
 snprintf(reason,size,"%s",e==ESP_OK?"ok":e==ESP_ERR_INVALID_ARG?"unbekanntes Feld, Typ oder Grenzwert":"NVS Commit fehlgeschlagen");return e;
}
char *config_json(void) {
 heating_config_t c;config_get(&c);cJSON *j=cJSON_CreateObject();if(!j)return NULL;
 cJSON_AddStringToObject(j,"mode",config_mode_name(c.mode));
 for(size_t i=0;i<sizeof(fields)/sizeof(fields[0]);i++) {
  const void *p=(const char*)&c+fields[i].offset;
  if(fields[i].type==F_BOOL)cJSON_AddBoolToObject(j,fields[i].name,*(const bool*)p);
  else cJSON_AddNumberToObject(j,fields[i].name,fields[i].type==F_FLOAT?*(const float*)p:*(const uint32_t*)p);
 }
 cJSON *a=cJSON_AddArrayToObject(j,"periods");
 for(int i=0;i<PERIOD_COUNT;i++){cJSON *p=cJSON_CreateObject();cJSON_AddNumberToObject(p,"start_minute",c.periods[i].start_minute);
 cJSON_AddNumberToObject(p,"room_target",c.periods[i].room_target);cJSON_AddItemToArray(a,p);}
 char *s=cJSON_PrintUnformatted(j);cJSON_Delete(j);return s;
}

esp_err_t config_mode_update_json(const char *text,char *reason,unsigned size){
 cJSON *j=cJSON_ParseWithOpts(text,NULL,true);
 const cJSON *v=cJSON_GetObjectItemCaseSensitive(j,"mode");
 bool valid=cJSON_IsObject(j)&&v&&j->child==v&&!v->next&&cJSON_IsString(v);
 if(valid){const char *m=v->valuestring;valid=!strcmp(m,"off")||!strcmp(m,"frost")||!strcmp(m,"auto")||!strcmp(m,"heating")||!strcmp(m,"dhw")||!strcmp(m,"holiday");}
 cJSON_Delete(j);
 if(!valid){snprintf(reason,size,"Nur die sechs Betriebsarten sind erlaubt");return ESP_ERR_INVALID_ARG;}
 return config_update_json(text,reason,size);
}

esp_err_t config_update_json(const char *text,char *reason,unsigned size){return config_update_json_impl(text,reason,size,false);}
esp_err_t config_update_json_validate(const char *text,char *reason,unsigned size){return config_update_json_impl(text,reason,size,true);}
