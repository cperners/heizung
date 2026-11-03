#include "sensors.h"
#include "board.h"
#include "config.h"
#include "ds18b20.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <math.h>
#include <string.h>
static const int pins[SENSOR_COUNT]={PIN_SENSOR_OUTSIDE,PIN_SENSOR_ROOM,PIN_SENSOR_KESSEL,PIN_SENSOR_FLOW,PIN_SENSOR_DHW,PIN_SENSOR_BUFFER_TOP,PIN_SENSOR_BUFFER_MID,PIN_SENSOR_BUFFER_BOTTOM};
static sensor_snapshot_t values;
static sensor_diagnostic_t diagnostics[SENSOR_COUNT];
static SemaphoreHandle_t lock,history_lock;
#define SENSOR_HISTORY_POINTS 1440
typedef struct {int64_t ms;float temperature[5];uint8_t valid;} sensor_history_point_t;
static sensor_history_point_t history[SENSOR_HISTORY_POINTS];
static unsigned history_count,history_next;
static void record_history(int64_t now){
 sensor_snapshot_t snapshot;sensors_get(&snapshot);sensor_history_point_t p={.ms=now};
 for(int i=0;i<5;i++)if(snapshot.value[i].valid&&isfinite(snapshot.value[i].celsius)){p.valid|=1u<<i;p.temperature[i]=snapshot.value[i].celsius;}
 xSemaphoreTake(history_lock,portMAX_DELAY);
 if(history_count){unsigned last=(history_next+SENSOR_HISTORY_POINTS-1)%SENSOR_HISTORY_POINTS;if(history[last].ms/60000==now/60000){history[last]=p;xSemaphoreGive(history_lock);return;}}
 history[history_next]=p;history_next=(history_next+1)%SENSOR_HISTORY_POINTS;if(history_count<SENSOR_HISTORY_POINTS)history_count++;
 xSemaphoreGive(history_lock);
}
static void history_json_point(cJSON *a,const sensor_history_point_t *p,int64_t now){
 cJSON *j=cJSON_CreateObject();if(!j)return;
 cJSON_AddNumberToObject(j,"age_s",(now-p->ms)/1000);cJSON *v=cJSON_AddArrayToObject(j,"temperatures");
 if(!v){cJSON_Delete(j);return;}
 for(int i=0;i<5;i++)cJSON_AddItemToArray(v,p->valid&(1u<<i)?cJSON_CreateNumber(p->temperature[i]):cJSON_CreateNull());
 cJSON_AddItemToArray(a,j);
}
cJSON *sensors_history_json(int64_t now,unsigned span_s){
 if(span_s!=3600&&span_s!=21600&&span_s!=86400)span_s=3600;
 unsigned interval=((span_s+287*60)/(288*60))*60;
 cJSON *j=cJSON_CreateObject();if(!j)return NULL;
 cJSON_AddNumberToObject(j,"interval_s",interval);cJSON_AddNumberToObject(j,"span_s",span_s);
 cJSON *a=cJSON_AddArrayToObject(j,"points");if(!a){cJSON_Delete(j);return NULL;}
 xSemaphoreTake(history_lock,portMAX_DELAY);const sensor_history_point_t *pending=NULL;int64_t bucket=-1;
 for(unsigned i=0;i<history_count;i++){
  const sensor_history_point_t *p=&history[(history_next+SENSOR_HISTORY_POINTS-history_count+i)%SENSOR_HISTORY_POINTS];
  if(p->ms>now||now-p->ms>(int64_t)span_s*1000)continue;
  int64_t next_bucket=p->ms/(interval*1000LL);
  if(pending&&next_bucket!=bucket)history_json_point(a,pending,now);
  pending=p;bucket=next_bucket;
 }
 if(pending)history_json_point(a,pending,now);
 xSemaphoreGive(history_lock);return j;
}
static portMUX_TYPE timing=portMUX_INITIALIZER_UNLOCKED;
/* Ein Sensor je Bus, externe Versorgung: Parasite Power wird nicht unterstützt.
   Zeitkritische Slots sperren Interrupts nur für <=70 us, nie für 750 ms. */
static bool reset(int pin,sensor_diagnostic_t *d) {
 if(!gpio_get_level(pin)){d->code=SENSOR_DIAG_IDLE_LOW;return false;}
 gpio_set_level(pin,0);esp_rom_delay_us(3);
 if(gpio_get_level(pin)){gpio_set_level(pin,1);d->code=SENSOR_DIAG_DRIVE_HIGH;return false;}
 esp_rom_delay_us(477);
 portENTER_CRITICAL(&timing);gpio_set_level(pin,1);esp_rom_delay_us(70);
 bool presence=!gpio_get_level(pin);portEXIT_CRITICAL(&timing);
 esp_rom_delay_us(410);
 if(!gpio_get_level(pin)){d->code=SENSOR_DIAG_RELEASE_LOW;return false;}
 if(!presence){d->code=SENSOR_DIAG_NO_PRESENCE;return false;}
 d->presence=true;return true;
}
static void bit_write(int p,bool b) {
 portENTER_CRITICAL(&timing);gpio_set_level(p,0);esp_rom_delay_us(b?6:60);
 gpio_set_level(p,1);esp_rom_delay_us(b?64:10);portEXIT_CRITICAL(&timing);
}
static bool bit_read(int p) {
 portENTER_CRITICAL(&timing);gpio_set_level(p,0);esp_rom_delay_us(3);
 gpio_set_level(p,1);esp_rom_delay_us(10);bool b=gpio_get_level(p);
 esp_rom_delay_us(57);portEXIT_CRITICAL(&timing);return b;
}
static void write_byte(int p,uint8_t b) {for(int i=0;i<8;i++){bit_write(p,b&1);b>>=1;}}
static uint8_t read_byte(int p) {uint8_t b=0;for(int i=0;i<8;i++)if(bit_read(p))b|=1<<i;return b;}
static bool begin(int p,sensor_diagnostic_t *d) {
 if(!reset(p,d))return false;
 write_byte(p,0x33);
 for(int i=0;i<8;i++)d->rom[i]=read_byte(p);
 d->have_rom=true;
 if(ds18b20_crc(d->rom,7)!=d->rom[7]){d->code=SENSOR_DIAG_ROM_CRC;return false;}
 if(d->rom[0]!=0x28){d->code=SENSOR_DIAG_FAMILY;return false;}
 if(!reset(p,d))return false;
 write_byte(p,0xcc);write_byte(p,0xb4);
 if(!bit_read(p)){d->code=SENSOR_DIAG_PARASITE;return false;}
 if(!reset(p,d))return false;
 write_byte(p,0xcc);write_byte(p,0x44);return true;
}
static bool read_result(int p,bool started,sensor_diagnostic_t *d,float *t){
 if(!started)return false;
 if(!bit_read(p)){d->code=SENSOR_DIAG_CONVERSION;return false;}
 if(!reset(p,d))return false;
 write_byte(p,0xcc);write_byte(p,0xbe);
 for(int n=0;n<9;n++)d->scratch[n]=read_byte(p);
 d->have_scratch=true;
 if(ds18b20_crc(d->scratch,8)!=d->scratch[8]){d->code=SENSOR_DIAG_SCRATCH_CRC;return false;}
 if(!ds18b20_decode(d->scratch,t)){d->code=SENSOR_DIAG_SCRATCH_INVALID;return false;}
 d->code=SENSOR_DIAG_OK;return true;
}
const char *sensor_diagnostic_text(sensor_diag_code_t code){
 switch(code){
 case SENSOR_DIAG_DISABLED:return "Nicht aktiviert";
 case SENSOR_DIAG_OK:return "Messung erfolgreich";
 case SENSOR_DIAG_IDLE_LOW:return "Datenleitung vor Reset LOW: Versorgung, Pull-up und Verdrahtung pruefen";
 case SENSOR_DIAG_DRIVE_HIGH:return "Datenleitung folgt LOW-Ansteuerung nicht: Anschluss/GPIO pruefen";
 case SENSOR_DIAG_NO_PRESENCE:return "Kein Praesenzimpuls: Fuehler antwortet nicht";
 case SENSOR_DIAG_RELEASE_LOW:return "Datenleitung nach Reset weiterhin LOW";
 case SENSOR_DIAG_ROM_CRC:return "Fuehlerantwort vorhanden, ROM-Pruefsumme fehlerhaft";
 case SENSOR_DIAG_FAMILY:return "ROM gueltig, aber kein DS18B20 (Familie 0x28)";
 case SENSOR_DIAG_PARASITE:return "Fuehler meldet Parasitaerversorgung: Plus/GND pruefen";
 case SENSOR_DIAG_CONVERSION:return "Temperaturwandlung nach 800 ms nicht fertig";
 case SENSOR_DIAG_SCRATCH_CRC:return "Temperaturdaten empfangen, Pruefsumme fehlerhaft";
 case SENSOR_DIAG_SCRATCH_INVALID:return "Pruefsumme gueltig, Temperaturdaten unplausibel";
 default:return "Warte auf erste Messung";
 }
}
static void worker(void *unused) {
 (void)unused;
 for(;;) {
  heating_config_t c;config_get(&c);bool started[SENSOR_COUNT]={0};sensor_diagnostic_t next[SENSOR_COUNT]={0};
  int count=c.buffer_enabled?8:5;
  /* Vor Beginn alten Wert nicht invalidieren: Alter wird im Regler geprüft. */
  for(int i=0;i<count;i++)started[i]=begin(pins[i],&next[i]);
  vTaskDelay(pdMS_TO_TICKS(800));
  for(int i=0;i<SENSOR_COUNT;i++) {
   float t=NAN;bool good=false;
   if(i<count)good=read_result(pins[i],started[i],&next[i],&t);
   else next[i].code=SENSOR_DIAG_DISABLED;
   next[i].gpio=pins[i];
   xSemaphoreTake(lock,portMAX_DELAY);sensor_value_t *v=&values.value[i];
   v->valid=good;v->celsius=good?t:NAN;v->sampled_ms=esp_timer_get_time()/1000;
   if(!good&&i<count&&v->failures<UINT32_MAX)v->failures++;
   sensor_diag_code_t previous=diagnostics[i].code;
   next[i].attempts=diagnostics[i].attempts;next[i].successes=diagnostics[i].successes;
   if(i<count&&next[i].attempts<UINT32_MAX)next[i].attempts++;
   if(good&&next[i].successes<UINT32_MAX)next[i].successes++;
   diagnostics[i]=next[i];
   xSemaphoreGive(lock);
   if(i<count&&(next[i].attempts==1||previous!=next[i].code))ESP_LOGI("sensors","GPIO %d: %s",pins[i],sensor_diagnostic_text(next[i].code));
  }
  record_history(esp_timer_get_time()/1000);
  vTaskDelay(pdMS_TO_TICKS(200));
 }
}
esp_err_t sensors_init(void) {
 lock=xSemaphoreCreateMutex();history_lock=xSemaphoreCreateMutex();if(!lock||!history_lock)return ESP_ERR_NO_MEM;
 history_count=history_next=0;
 for(int i=0;i<SENSOR_COUNT;i++){
  values.value[i].celsius=NAN;diagnostics[i].gpio=pins[i];
  gpio_config_t g={.pin_bit_mask=1ULL<<pins[i],.mode=GPIO_MODE_INPUT_OUTPUT_OD,
    .pull_up_en=GPIO_PULLUP_DISABLE,.pull_down_en=GPIO_PULLDOWN_DISABLE};
  esp_err_t e=gpio_config(&g);if(e!=ESP_OK)return e;gpio_set_level(pins[i],1);
 }
 return xTaskCreate(worker,"ds18b20",4096,NULL,4,NULL)==pdPASS?ESP_OK:ESP_ERR_NO_MEM;
}
void sensors_get(sensor_snapshot_t *s) {xSemaphoreTake(lock,portMAX_DELAY);*s=values;xSemaphoreGive(lock);}

void sensors_get_diagnostics(sensor_diagnostic_t out[SENSOR_COUNT]){xSemaphoreTake(lock,portMAX_DELAY);memcpy(out,diagnostics,sizeof(diagnostics));xSemaphoreGive(lock);}
