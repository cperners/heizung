#include "rest_heat.h"
#include "nvs.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
static rest_heat_config_t current;
static SemaphoreHandle_t lock;
esp_err_t rest_heat_init(void){
 lock=xSemaphoreCreateMutex();if(!lock)return ESP_ERR_NO_MEM;rest_heat_defaults(&current);
 nvs_handle_t h;esp_err_t e=nvs_open("rest_heat",NVS_READONLY,&h);if(e==ESP_ERR_NVS_NOT_FOUND)return ESP_OK;if(e!=ESP_OK)return e;
 rest_heat_config_t saved;size_t n=sizeof(saved);e=nvs_get_blob(h,"config",&saved,&n);nvs_close(h);
 if(e==ESP_ERR_NVS_NOT_FOUND)return ESP_OK;
 if(e!=ESP_OK||n!=sizeof(saved)||!rest_heat_validate(&saved))return ESP_ERR_INVALID_STATE;
 current=saved;return ESP_OK;
}
void rest_heat_get(rest_heat_config_t *c){xSemaphoreTake(lock,portMAX_DELAY);*c=current;xSemaphoreGive(lock);}
char *rest_heat_json(void){
 rest_heat_config_t c;rest_heat_get(&c);cJSON *j=cJSON_CreateObject();if(!j)return NULL;
 cJSON_AddBoolToObject(j,"enabled",c.enabled);cJSON_AddNumberToObject(j,"stop_c",c.stop_c);cJSON_AddNumberToObject(j,"max_s",c.max_s);
 char *s=cJSON_PrintUnformatted(j);cJSON_Delete(j);return s;
}
static esp_err_t update(const char *text,char *reason,unsigned size,bool dry){
 cJSON *j=cJSON_ParseWithOpts(text,NULL,true);rest_heat_config_t next;rest_heat_get(&next);bool okay=cJSON_IsObject(j)&&j->child;
 if(okay)for(const cJSON *v=j->child;v;v=v->next){
  for(const cJSON *p=j->child;p!=v;p=p->next)if(!strcmp(p->string,v->string))okay=false;
  if(!strcmp(v->string,"enabled")){if(!cJSON_IsBool(v))okay=false;else next.enabled=cJSON_IsTrue(v);}
  else if(!strcmp(v->string,"stop_c")){if(!cJSON_IsNumber(v))okay=false;else next.stop_c=(float)v->valuedouble;}
  else if(!strcmp(v->string,"max_s")){if(!cJSON_IsNumber(v)||!isfinite(v->valuedouble)||v->valuedouble<60||v->valuedouble>7200||floor(v->valuedouble)!=v->valuedouble)okay=false;else next.max_s=(uint32_t)v->valuedouble;}
  else okay=false;
 }
 cJSON_Delete(j);esp_err_t e=ESP_ERR_INVALID_ARG;
 if(okay&&rest_heat_validate(&next)){
  if(dry)return ESP_OK;
  nvs_handle_t h;e=nvs_open("rest_heat",NVS_READWRITE,&h);
  if(e==ESP_OK){e=nvs_set_blob(h,"config",&next,sizeof(next));if(e==ESP_OK)e=nvs_commit(h);nvs_close(h);}
  if(e==ESP_OK){xSemaphoreTake(lock,portMAX_DELAY);current=next;xSemaphoreGive(lock);}
 }
 snprintf(reason,size,"%s",e==ESP_OK?"Restwaermenutzung gespeichert":e==ESP_ERR_INVALID_ARG?"Abschaltung 20-60 Grad, maximale Laufzeit 60-7200 Sekunden":"Speichern fehlgeschlagen");return e;
}
esp_err_t rest_heat_update(const char *s,char *r,unsigned n){return update(s,r,n,false);}
esp_err_t rest_heat_update_validate(const char *s,char *r,unsigned n){return update(s,r,n,true);}
