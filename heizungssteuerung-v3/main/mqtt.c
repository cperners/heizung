#include "mqtt.h"
#include "service_config.h"
#include "mqtt_topics.h"
#include "config.h"
#include "outputs.h"
#include "pv.h"
#include "stove.h"
#include "gas.h"
#include "buffer.h"
#include "mqtt_client.h"
#include "sdkconfig.h"
#include "esp_crt_bundle.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "cJSON.h"
#include <stdatomic.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#define PAYLOAD_MAX 2048
static esp_mqtt_client_handle_t client;
static service_config_t connection;
static char status_topic[256],config_set_topic[256];
static atomic_bool connected;
static _Atomic unsigned connection_epoch;
typedef struct {char topic[256];char payload[PAYLOAD_MAX+1];bool retained;unsigned epoch;int64_t received_ms;} command_t;
static QueueHandle_t commands;
static command_t incoming;
static int received,total;
static bool collecting;
static int64_t last_config_ms=-10000;
static void publish(const char *suffix,const char *data,bool retain){
 char topic[256];if(mqtt_topic_join(topic,sizeof(topic),connection.base_topic,suffix)&&atomic_load(&connected))esp_mqtt_client_enqueue(client,topic,data,0,1,retain,false);
}
bool mqtt_connected(void){return atomic_load(&connected);}
static void config_report(const char *reason){
 cJSON *j=cJSON_CreateObject();if(!j)return;cJSON_AddStringToObject(j,"result",reason);
 char *text=cJSON_PrintUnformatted(j);if(text){publish("config/result",text,false);free(text);}cJSON_Delete(j);
}
static void command_worker(void *unused){
 (void)unused;command_t cmd;
 for(;;){if(xQueueReceive(commands,&cmd,portMAX_DELAY)!=pdTRUE)continue;
  if(!strcmp(cmd.topic,config_set_topic)){
   int64_t now=esp_timer_get_time()/1000;
   if(cmd.retained){config_report("retained Befehle werden abgelehnt");continue;}
   if(now-last_config_ms<2000){config_report("zu viele Konfigurationsaenderungen");continue;}
   last_config_ms=now;char reason[96];config_update_json(cmd.payload,reason,sizeof(reason));config_report(reason);
   char *json=config_json();if(json){publish("config",json,true);free(json);}
  }else if(mqtt_topic_under(connection.base_topic,cmd.topic)){
   /* Eigene Ausgaben nie als externe Fuehlerdaten behandeln. */
  }else if(cmd.epoch==atomic_load(&connection_epoch)&&gas_receive(cmd.topic,cmd.payload,cmd.retained,cmd.received_ms)) {
   /* Gasdaten beeinflussen keine Heizungsanforderung. */
  }else if(cmd.epoch==atomic_load(&connection_epoch)&&stove_receive(cmd.topic,cmd.payload,cmd.retained,cmd.received_ms)) {
   /* Separater Fühler: niemals persistente room_gain-Konfiguration verändern. */
  }else if(!cmd.retained)pv_receive(cmd.topic,cmd.payload,esp_timer_get_time()/1000);
 }
}
static void event(void *arg,esp_event_base_t base,int32_t id,void *event_data){
 (void)arg;(void)base;esp_mqtt_event_handle_t e=event_data;
 switch(id){
 case MQTT_EVENT_CONNECTED:
  atomic_fetch_add(&connection_epoch,1);stove_set_connected(true);gas_set_connected(true);
  atomic_store(&connected,true);publish("status","online",true);
  esp_mqtt_client_subscribe(client,config_set_topic,1);
  esp_mqtt_client_subscribe(client,"heizung/pv/+",1);
  {stove_config_t c;stove_get(&c,NULL,esp_timer_get_time()/1000);
   if(c.enabled&&c.topic[0])esp_mqtt_client_subscribe(client,c.topic,1); }
  {gas_config_t c;gas_config_get(&c);if(c.enabled&&c.topic[0])esp_mqtt_client_subscribe(client,c.topic,1); }
  {char *j=config_json();if(j){publish("config",j,true);free(j);}}break;
 case MQTT_EVENT_DISCONNECTED:atomic_fetch_add(&connection_epoch,1);stove_set_connected(false);gas_set_connected(false);atomic_store(&connected,false);collecting=false;break;
 case MQTT_EVENT_DATA:
  if(e->current_data_offset==0){
   collecting=false;received=0;total=e->total_data_len;
   if(e->topic_len<=0||e->topic_len>=(int)sizeof(incoming.topic)||total<=0||total>PAYLOAD_MAX)break;
   memset(&incoming,0,sizeof(incoming));memcpy(incoming.topic,e->topic,e->topic_len);
   incoming.retained=e->retain;incoming.epoch=atomic_load(&connection_epoch);incoming.received_ms=esp_timer_get_time()/1000;collecting=true;
  }
  if(!collecting||e->current_data_offset!=received||e->data_len<0||received+e->data_len>total){collecting=false;break;}
  memcpy(incoming.payload+received,e->data,e->data_len);received+=e->data_len;
  if(received==total){incoming.payload[total]=0;collecting=false;
   if(memchr(incoming.payload,0,total)){config_report("NUL im Payload");break;}
   if(xQueueSend(commands,&incoming,0)!=pdTRUE)config_report("Befehlsqueue voll");
  }break;
 default:break;
 }
}
esp_err_t mqtt_init(void){
 service_config_get(&connection);
 if(!mqtt_topic_join(status_topic,sizeof(status_topic),connection.base_topic,"status")||!mqtt_topic_join(config_set_topic,sizeof(config_set_topic),connection.base_topic,"config/set"))return ESP_ERR_INVALID_ARG;
 commands=xQueueCreate(4,sizeof(command_t));if(!commands)return ESP_ERR_NO_MEM;
 esp_mqtt_client_config_t cfg={
 .broker.address.uri=connection.uri,
 .broker.verification.crt_bundle_attach=esp_crt_bundle_attach,
 .credentials.username=connection.username,
 .credentials.authentication.password=connection.password,
 .session.last_will={.topic=status_topic,.msg="offline",.qos=1,.retain=1},
 .session.keepalive=30,.network.reconnect_timeout_ms=5000,.buffer.size=1024,
 .outbox.limit=16384};
 client=esp_mqtt_client_init(&cfg);if(!client)return ESP_ERR_NO_MEM;
 ESP_ERROR_CHECK(esp_mqtt_client_register_event(client,ESP_EVENT_ANY_ID,event,NULL));
 if(xTaskCreate(command_worker,"mqtt_commands",6144,NULL,3,NULL)!=pdPASS)return ESP_ERR_NO_MEM;
 return esp_mqtt_client_start(client);
}
static void number(const char *topic,float value){char buf[32];snprintf(buf,sizeof(buf),"%.2f",(double)value);publish(topic,buf,true);}
static void boolean(const char *topic,bool value){publish(topic,value?"1":"0",true);}
void mqtt_publish_status(const sensor_snapshot_t *v,const heating_config_t *c,const heating_status_t *s){
 if(!mqtt_connected())return;
 static const char *names[]={"aussen","raum","kessel","vorlauf","boiler","puffer/oben","puffer/mitte","puffer/unten"};
 int64_t now=esp_timer_get_time()/1000;
 cJSON *j=cJSON_CreateObject();if(!j)return;
 output_request_t actual;float pos;bool timeout;outputs_get(&actual,&pos,&timeout);
 cJSON_AddNumberToObject(j,"faults",s->faults);cJSON_AddBoolToObject(j,"output_timeout",timeout);
 cJSON_AddBoolToObject(j,"actuators_enabled",outputs_enabled());cJSON_AddBoolToObject(j,"frost",s->frost);
 cJSON_AddBoolToObject(j,"rest_heat_active",s->rest_heat_active);cJSON_AddNumberToObject(j,"rest_heat_remaining_s",s->rest_heat_remaining_s);
 cJSON_AddBoolToObject(j,"dhw_demand",s->dhw_demand);cJSON_AddNumberToObject(j,"mixer_position",pos);
 stove_config_t sc;stove_state_t ss;stove_get(&sc,&ss,now);
 cJSON *st=cJSON_AddObjectToObject(j,"stove");cJSON_AddBoolToObject(st,"enabled",sc.enabled);
 cJSON_AddBoolToObject(st,"valid",ss.valid);cJSON_AddBoolToObject(st,"active",ss.valid&&ss.active);
 cJSON_AddItemToObject(st,"temperature",ss.valid?cJSON_CreateNumber(ss.celsius):cJSON_CreateNull());
 cJSON_AddNumberToObject(st,"errors",ss.errors);
 cJSON_AddStringToObject(st,"status",!sc.enabled?"disabled":!ss.valid?"unavailable":ss.active?"active":"inactive");
 cJSON_AddBoolToObject(j,"room_correction_suppressed",s->room_correction_suppressed);
 cJSON_AddItemToObject(j,"gas",gas_status_json(now));
 cJSON_AddNumberToObject(j,"uptime_s",now/1000);cJSON *a=cJSON_AddObjectToObject(j,"sensors");
 for(int i=0;i<SENSOR_COUNT;i++){
  char topic[256];snprintf(topic,sizeof(topic),"%s",names[i]);
  bool valid=v->value[i].valid&&isfinite(v->value[i].celsius)&&now-v->value[i].sampled_ms<(int64_t)c->sensor_stale_s*1000;
  if(valid)number(topic,v->value[i].celsius);else publish(topic,"null",true);
  cJSON *entry=cJSON_AddObjectToObject(a,names[i]);cJSON_AddBoolToObject(entry,"valid",valid);
  cJSON_AddNumberToObject(entry,"failures",v->value[i].failures);
 }
 boolean("brenner",actual.burner);boolean("heizungspumpe",actual.heating_pump);
 boolean("boilerpumpe",actual.boiler_pump);boolean("mischer_auf",actual.mixer==MIX_OPEN);
 boolean("mischer_zu",actual.mixer==MIX_CLOSE);
 number("soll/raum",s->room_target);number("soll/vorlauf",s->flow_target);
 number("soll/boiler",c->boiler_target);
 publish("mode",config_mode_name(c->mode),true);
 buffer_status_t b=buffer_evaluate(v,c->buffer_enabled,now,c->sensor_stale_s);
 cJSON_AddBoolToObject(j,"buffer_available",b.available);if(b.available)cJSON_AddNumberToObject(j,"buffer_mean",b.mean);
 pv_status_t pv;pv_get(&pv,now);cJSON *p=cJSON_AddArrayToObject(j,"pv_watts");
 for(int i=0;i<4;i++)cJSON_AddItemToArray(p,pv.valid[i]?cJSON_CreateNumber(pv.watts[i]):cJSON_CreateNull());
 char *json=cJSON_PrintUnformatted(j);if(json){publish("diagnostics",json,true);free(json);}cJSON_Delete(j);
}
