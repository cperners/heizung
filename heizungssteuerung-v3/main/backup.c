#include "backup.h"
#include "config.h"
#include "service_config.h"
#include "stove.h"
#include "gas.h"
#include "rest_heat.h"
#include "cJSON.h"
#include "nvs.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#define BACKUP_MAX 8192
typedef esp_err_t (*updater_t)(const char*,char*,unsigned);
static const char *sections[]={"heating","services","stove","gas","rest_heat"};
static const updater_t validators[]={config_update_json_validate,service_config_update_validate,stove_config_update_validate,gas_config_update_validate,rest_heat_update_validate};
static const updater_t writers[]={config_update_json,service_config_update,stove_config_update,gas_config_update,rest_heat_update};
static bool unique(const cJSON *j){
 if(cJSON_IsObject(j))for(const cJSON *v=j->child;v;v=v->next){for(const cJSON *p=j->child;p!=v;p=p->next)if(!strcmp(p->string,v->string))return false;if(!unique(v))return false;}
 if(cJSON_IsArray(j))for(const cJSON *v=j->child;v;v=v->next)if(!unique(v))return false;
 return true;
}
static bool exact(const cJSON *j,const char *keys){
 if(!cJSON_IsObject(j)||!j->child)return false;
 for(const cJSON *v=j->child;v;v=v->next){char token[80];snprintf(token,sizeof(token),"|%s|",v->string);if(!strstr(keys,token))return false;}
 return true;
}
static esp_err_t inspect(const char *text,cJSON **out,char *reason,unsigned size){
 cJSON *j=cJSON_ParseWithOpts(text,NULL,true);*out=NULL;
 const cJSON *format=cJSON_GetObjectItemCaseSensitive(j,"format"),*version=cJSON_GetObjectItemCaseSensitive(j,"version");
 if(!exact(j,"|format||version||heating||services||stove||gas||rest_heat|")||!unique(j)||!cJSON_IsString(format)||strcmp(format->valuestring,"heizung-settings")||!cJSON_IsNumber(version)||(version->valuedouble!=1&&version->valuedouble!=2&&version->valuedouble!=3))goto invalid;
 cJSON *rest=cJSON_GetObjectItemCaseSensitive(j,"rest_heat");
 if(!rest&&version->valuedouble<3){rest=cJSON_AddObjectToObject(j,"rest_heat");cJSON_AddBoolToObject(rest,"enabled",true);cJSON_AddNumberToObject(rest,"stop_c",30);cJSON_AddNumberToObject(rest,"max_s",1800);}
 if(!exact(rest,"|enabled||stop_c||max_s|")||cJSON_GetArraySize(rest)!=3)goto invalid;
 const cJSON *services=cJSON_GetObjectItemCaseSensitive(j,"services");
 if(!exact(services,"|uri||username||ntp_primary||ntp_secondary||ntp_tertiary||base_topic|"))goto invalid;
 if(version->valuedouble==1&&cJSON_GetArraySize(services)==5&&!cJSON_GetObjectItemCaseSensitive(services,"base_topic"))cJSON_AddStringToObject((cJSON*)services,"base_topic","heizung/anlage");
 if(cJSON_GetArraySize(services)!=6||!cJSON_GetObjectItemCaseSensitive(services,"base_topic"))goto invalid;
 const cJSON *gas=cJSON_GetObjectItemCaseSensitive(j,"gas"),*stove=cJSON_GetObjectItemCaseSensitive(j,"stove");
 if(!exact(gas,"|enabled||topic||stale_s|")||cJSON_GetArraySize(gas)!=3||!exact(stove,"|enabled||topic||on_c||off_c||stale_s|")||cJSON_GetArraySize(stove)!=5)goto invalid;
 /* Require the complete heating field set from this schema, not a patch. */
 char *current=config_json();cJSON *reference=current?cJSON_Parse(current):NULL;free(current);
 const cJSON *heat=cJSON_GetObjectItemCaseSensitive(j,"heating");
 bool complete=reference&&cJSON_IsObject(heat)&&cJSON_GetArraySize(heat)==cJSON_GetArraySize(reference);
 if(complete)for(const cJSON *v=reference->child;v;v=v->next)if(!cJSON_GetObjectItemCaseSensitive(heat,v->string))complete=false;
 cJSON_Delete(reference);if(!complete)goto invalid;
 for(unsigned i=0;i<5;i++){
  const cJSON *section=cJSON_GetObjectItemCaseSensitive(j,sections[i]);char *s=cJSON_PrintUnformatted(section);
  if(!s){cJSON_Delete(j);return ESP_ERR_NO_MEM;}
  esp_err_t e=validators[i](s,reason,size);free(s);if(e!=ESP_OK){cJSON_Delete(j);return e;}
 }
 const char *base=cJSON_GetObjectItemCaseSensitive(services,"base_topic")->valuestring;
 if(mqtt_topic_under(base,cJSON_GetObjectItemCaseSensitive(stove,"topic")->valuestring)||mqtt_topic_under(base,cJSON_GetObjectItemCaseSensitive(gas,"topic")->valuestring))goto invalid;
 *out=j;return ESP_OK;
invalid:
 cJSON_Delete(j);snprintf(reason,size,"Keine vollstaendige kompatible Einstellungssicherung");return ESP_ERR_INVALID_ARG;
}
char *backup_export(void){
 cJSON *j=cJSON_CreateObject();if(!j)return NULL;
 cJSON_AddStringToObject(j,"format","heizung-settings");cJSON_AddNumberToObject(j,"version",3);
 char *texts[]={config_json(),service_config_json(),stove_config_json(),gas_config_json(),rest_heat_json()};
 bool okay=true;
 for(unsigned i=0;i<5;i++){cJSON *section=texts[i]?cJSON_Parse(texts[i]):NULL;free(texts[i]);if(!section){okay=false;continue;}if(i==1)cJSON_DeleteItemFromObjectCaseSensitive(section,"password_set");cJSON_AddItemToObject(j,sections[i],section);}
 char *text=okay?cJSON_PrintUnformatted(j):NULL;cJSON_Delete(j);return text;
}
esp_err_t backup_stage(const char *text,char *reason,unsigned size){
 if(strlen(text)>=BACKUP_MAX){snprintf(reason,size,"Sicherung ist zu gross");return ESP_ERR_INVALID_ARG;}
 cJSON *j;esp_err_t e=inspect(text,&j,reason,size);if(e!=ESP_OK)return e;cJSON_Delete(j);
 nvs_handle_t h;e=nvs_open("restore",NVS_READWRITE,&h);
 if(e==ESP_OK){e=nvs_set_blob(h,"pending",text,strlen(text)+1);if(e==ESP_OK)e=nvs_commit(h);nvs_close(h);}
 snprintf(reason,size,"%s",e==ESP_OK?"Sicherung geprueft. Wiederherstellung beim Neustart.":"Sicherung konnte nicht vorgemerkt werden");return e;
}
esp_err_t backup_apply_pending(void){
 nvs_handle_t h;esp_err_t e=nvs_open("restore",NVS_READWRITE,&h);if(e!=ESP_OK)return e;
 size_t n=0;e=nvs_get_blob(h,"pending",NULL,&n);
 if(e==ESP_ERR_NVS_NOT_FOUND){nvs_close(h);return ESP_OK;}
 if(e!=ESP_OK||n<2||n>BACKUP_MAX){nvs_close(h);return ESP_ERR_INVALID_STATE;}
 char *text=malloc(n);if(!text){nvs_close(h);return ESP_ERR_NO_MEM;}
 e=nvs_get_blob(h,"pending",text,&n);
 if(e!=ESP_OK||text[n-1]||memchr(text,0,n-1)){free(text);nvs_close(h);return ESP_ERR_INVALID_STATE;}
 char reason[160];cJSON *j=NULL;e=inspect(text,&j,reason,sizeof(reason));free(text);
 if(e==ESP_OK)for(unsigned i=0;i<5;i++){char *s=cJSON_PrintUnformatted(cJSON_GetObjectItemCaseSensitive(j,sections[i]));if(!s){e=ESP_ERR_NO_MEM;break;}e=writers[i](s,reason,sizeof(reason));free(s);if(e!=ESP_OK)break;}
 cJSON_Delete(j);
 /* Keep the journal until every section is stored; a power cut replays it. */
 if(e==ESP_OK){e=nvs_erase_key(h,"pending");if(e==ESP_OK)e=nvs_commit(h);}
 nvs_close(h);return e;
}
