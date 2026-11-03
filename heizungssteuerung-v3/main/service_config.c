#include "service_config.h"
#include "sdkconfig.h"
#include "cJSON.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stddef.h>
static service_config_t current;
static SemaphoreHandle_t lock;
static bool host_valid(const char *host,size_t cap,bool optional) {
    if(!memchr(host,0,cap))return false;
    if(!*host)return optional;
    for(const unsigned char *p=(const unsigned char*)host;*p;p++)if(!isalnum(*p)&&*p!='.'&&*p!='-'&&*p!=':')return false;
    return true;
}
static bool valid(const service_config_t *s) {
    if(s->version!=2 || !memchr(s->base_topic,0,sizeof(s->base_topic)) || !mqtt_base_valid(s->base_topic) || !memchr(s->uri,0,sizeof(s->uri)) ||
       !memchr(s->username,0,sizeof(s->username)) || !memchr(s->password,0,sizeof(s->password))) return false;
    const char *host=NULL;
    if(!strncmp(s->uri,"mqtt://",7)) host=s->uri+7;
    if(!strncmp(s->uri,"mqtts://",8)) host=s->uri+8;
    if(!host || !*host || strchr(host,'@') || strchr(host,'/') || strchr(host,'?') || strchr(host,'#')) return false;
    const char *port=NULL;
    if(*host=='[') {
        const char *closing=strchr(host,']');if(!closing||closing==host+1)return false;
        for(const char *p=host+1;p<closing;p++)if(!isxdigit((unsigned char)*p)&&*p!=':'&&*p!='.')return false;
        if(closing[1]){if(closing[1]!=':')return false;port=closing+2;}
    } else {
        const char *colon=strchr(host,':');const char *end=colon?colon:host+strlen(host);
        if(end==host)return false;
        for(const char *p=host;p<end;p++)if(!isalnum((unsigned char)*p)&&*p!='.'&&*p!='-')return false;
        if(colon)port=colon+1;
    }
    if(port){unsigned long value=0;if(!*port)return false;
        for(const char *p=port;*p;p++){if(!isdigit((unsigned char)*p))return false;value=value*10+(unsigned)(*p-'0');if(value>65535)return false;}
        if(!value)return false;
    }
    return host_valid(s->ntp_primary,sizeof(s->ntp_primary),false) && host_valid(s->ntp_secondary,sizeof(s->ntp_secondary),true) && host_valid(s->ntp_tertiary,sizeof(s->ntp_tertiary),true);
}
esp_err_t service_config_init(void) {
    lock=xSemaphoreCreateMutex();if(!lock)return ESP_ERR_NO_MEM;
    memset(&current,0,sizeof(current));current.version=2;
    snprintf(current.base_topic,sizeof(current.base_topic),"heizung/anlage");
    snprintf(current.uri,sizeof(current.uri),"%s",CONFIG_HEATING_MQTT_URI);
    snprintf(current.username,sizeof(current.username),"%s",CONFIG_HEATING_MQTT_USER);
    snprintf(current.password,sizeof(current.password),"%s",CONFIG_HEATING_MQTT_PASSWORD);
    snprintf(current.ntp_primary,sizeof(current.ntp_primary),"192.168.0.1");
    snprintf(current.ntp_secondary,sizeof(current.ntp_secondary),"0.at.pool.ntp.org");
    snprintf(current.ntp_tertiary,sizeof(current.ntp_tertiary),"1.at.pool.ntp.org");
    if(!valid(&current)) return ESP_ERR_INVALID_ARG;
    nvs_handle_t h;esp_err_t e=nvs_open("services_v2",NVS_READONLY,&h);
    service_config_t saved={0};size_t n=sizeof(saved);
    if(e==ESP_OK){e=nvs_get_blob(h,"connection",&saved,&n);nvs_close(h);}
    if(e==ESP_OK){if(n!=sizeof(saved)||!valid(&saved))return ESP_ERR_INVALID_STATE;current=saved;return ESP_OK;}
    if(e!=ESP_ERR_NVS_NOT_FOUND)return e;
    e=nvs_open("services",NVS_READONLY,&h);if(e==ESP_ERR_NVS_NOT_FOUND)return ESP_OK;if(e!=ESP_OK)return e;
    n=sizeof(saved);e=nvs_get_blob(h,"connection",&saved,&n);nvs_close(h);
    if(e==ESP_ERR_NVS_NOT_FOUND)return ESP_OK;
    if(e!=ESP_OK||n!=offsetof(service_config_t,base_topic)||saved.version!=1)return ESP_ERR_INVALID_STATE;
    saved.version=2;snprintf(saved.base_topic,sizeof(saved.base_topic),"heizung/anlage");
    if(!valid(&saved))return ESP_ERR_INVALID_STATE;
    e=nvs_open("services_v2",NVS_READWRITE,&h);
    if(e==ESP_OK){e=nvs_set_blob(h,"connection",&saved,sizeof(saved));if(e==ESP_OK)e=nvs_commit(h);nvs_close(h);}
    if(e==ESP_OK)current=saved;
    return e;
}

void service_config_get(service_config_t *s) {
    xSemaphoreTake(lock,portMAX_DELAY);*s=current;xSemaphoreGive(lock);
}
char *service_config_json(void) {
    service_config_t s;service_config_get(&s);cJSON *j=cJSON_CreateObject();if(!j)return NULL;
    cJSON_AddStringToObject(j,"base_topic",s.base_topic);cJSON_AddStringToObject(j,"uri",s.uri);cJSON_AddStringToObject(j,"username",s.username);
    cJSON_AddBoolToObject(j,"password_set",s.password[0]!=0);
    cJSON_AddStringToObject(j,"ntp_primary",s.ntp_primary);
    cJSON_AddStringToObject(j,"ntp_secondary",s.ntp_secondary);
    cJSON_AddStringToObject(j,"ntp_tertiary",s.ntp_tertiary);
    char *text=cJSON_PrintUnformatted(j);cJSON_Delete(j);return text;
}
static esp_err_t service_config_update_impl(const char *text,char *reason,unsigned size,bool dry) {
    cJSON *j=cJSON_ParseWithOpts(text,NULL,true);
    if(!cJSON_IsObject(j) || !j->child) {cJSON_Delete(j);snprintf(reason,size,"Ungueltiges JSON");return ESP_ERR_INVALID_ARG;}
    xSemaphoreTake(lock,portMAX_DELAY);service_config_t next=current;bool okay=true;
    for(const cJSON *v=j->child;v;v=v->next) {
        for(const cJSON *a=j->child;a!=v;a=a->next) if(!strcmp(a->string,v->string))okay=false;
        char *dest=NULL;size_t cap=0;
        if(!strcmp(v->string,"base_topic")){dest=next.base_topic;cap=sizeof(next.base_topic);}
        else if(!strcmp(v->string,"uri")){dest=next.uri;cap=sizeof(next.uri);}
        else if(!strcmp(v->string,"username")){dest=next.username;cap=sizeof(next.username);}
        else if(!strcmp(v->string,"password")){dest=next.password;cap=sizeof(next.password);}
        else if(!strcmp(v->string,"ntp_primary")){dest=next.ntp_primary;cap=sizeof(next.ntp_primary);}
        else if(!strcmp(v->string,"ntp_secondary")){dest=next.ntp_secondary;cap=sizeof(next.ntp_secondary);}
        else if(!strcmp(v->string,"ntp_tertiary")){dest=next.ntp_tertiary;cap=sizeof(next.ntp_tertiary);}
        if(!dest || !cJSON_IsString(v) || strlen(v->valuestring)>=cap){okay=false;break;}
        memcpy(dest,v->valuestring,strlen(v->valuestring)+1);
    }
    esp_err_t e=ESP_ERR_INVALID_ARG;
    if(okay && valid(&next)) {
        if(dry){xSemaphoreGive(lock);cJSON_Delete(j);return ESP_OK;}
        nvs_handle_t h;e=nvs_open("services_v2",NVS_READWRITE,&h);
        if(e==ESP_OK){e=nvs_set_blob(h,"connection",&next,sizeof(next));if(e==ESP_OK)e=nvs_commit(h);nvs_close(h);}
        if(e==ESP_OK)current=next;
    }
    xSemaphoreGive(lock);cJSON_Delete(j);
    snprintf(reason,size,"%s",e==ESP_OK?"Gespeichert. Neustart erfolgt.":e==ESP_ERR_INVALID_ARG?"Broker-Adresse oder Eingabe ungueltig":"NVS konnte nicht speichern");
    return e;
}

esp_err_t service_config_update(const char *text,char *reason,unsigned size){return service_config_update_impl(text,reason,size,false);}
esp_err_t service_config_update_validate(const char *text,char *reason,unsigned size){return service_config_update_impl(text,reason,size,true);}

bool service_config_topic_is_own(const char *topic){xSemaphoreTake(lock,portMAX_DELAY);bool own=mqtt_topic_under(current.base_topic,topic);xSemaphoreGive(lock);return own;}
