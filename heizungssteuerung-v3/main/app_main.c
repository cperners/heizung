#include "sensors.h"
#include "touch_display.h"
#include "outputs.h"
#include "heating.h"
#include "ethernet.h"
#include "mqtt.h"
#include "service_config.h"
#include "web.h"
#include "ntp.h"
#include "pv.h"
#include "stove.h"
#include "gas.h"
#include "backup.h"
#include "rest_heat.h"
#include "ota.h"
#include "config.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
static SemaphoreHandle_t status_lock;
static heating_status_t latest_status;
static void status_get(heating_status_t *s) {
 xSemaphoreTake(status_lock,portMAX_DELAY);*s=latest_status;xSemaphoreGive(status_lock);
}
static void controller(void *unused){
 (void)unused;heating_state_t state;heating_init(&state,esp_timer_get_time()/1000);
 ESP_ERROR_CHECK(esp_task_wdt_add(NULL));TickType_t wake=xTaskGetTickCount();
 for(;;){
  int64_t now=esp_timer_get_time()/1000;heating_config_t c;config_get(&c);
  sensor_snapshot_t v;sensors_get(&v);int minute=0;bool valid_time=ntp_local_minute(&minute);
  stove_state_t stove;stove_get(NULL,&stove,now);
  rest_heat_config_t rest;rest_heat_get(&rest);
  heating_status_t s;heating_step_with_rest_heat(&state,&c,&v,now,minute,valid_time,ethernet_has_ip()&&mqtt_connected(),!(stove.valid&&stove.active),&rest,&s);
  s.room_correction_suppressed=stove.valid&&stove.active;
  outputs_apply(&s.outputs,now,&c);
  xSemaphoreTake(status_lock,portMAX_DELAY);latest_status=s;xSemaphoreGive(status_lock);
  ota_control_tick(now);
  ESP_ERROR_CHECK(esp_task_wdt_reset());
  /* Kommunikation ist eigene Task; kein blockierendes Publish im Regler. */
  vTaskDelayUntil(&wake,pdMS_TO_TICKS(100));
 }
}
static void telemetry(void *unused){
 (void)unused;for(;;){vTaskDelay(pdMS_TO_TICKS(5000));heating_config_t c;config_get(&c);
 sensor_snapshot_t v;sensors_get(&v);
 heating_status_t s;xSemaphoreTake(status_lock,portMAX_DELAY);s=latest_status;xSemaphoreGive(status_lock);
 mqtt_publish_status(&v,&c,&s);}
}
void app_main(void){
 ESP_ERROR_CHECK(outputs_init());
 ESP_ERROR_CHECK(ota_boot_start());
 /* NVS niemals automatisch löschen: beschädigte Konfiguration verlangt Eingriff. */
 esp_err_t e=nvs_flash_init();if(e==ESP_OK)e=config_init();
 if(e!=ESP_OK){outputs_emergency_stop();ESP_LOGE("main","Konfiguration/NVS defekt: %s; Lastausgaenge gesperrt",esp_err_to_name(e));return;}
 ESP_ERROR_CHECK(service_config_init());
 ESP_ERROR_CHECK(stove_init());
 ESP_ERROR_CHECK(gas_init());
 ESP_ERROR_CHECK(rest_heat_init());
 ESP_ERROR_CHECK(backup_apply_pending());
 pv_init();ESP_ERROR_CHECK(sensors_init());
 status_lock=xSemaphoreCreateMutex();if(!status_lock){outputs_emergency_stop();return;}
 /* Regler startet vor Netzwerk. Netzfehler lassen den lokalen Regler weiterlaufen. */
 if(xTaskCreate(controller,"heating",6144,NULL,8,NULL)!=pdPASS){outputs_emergency_stop();return;}
 e=ethernet_init();if(e!=ESP_OK){ESP_LOGE("main","Ethernet: %s",esp_err_to_name(e));return;}
 e=web_init(status_get);if(e!=ESP_OK){ESP_LOGE("main","Webserver: %s",esp_err_to_name(e));return;}
 ntp_init();e=mqtt_init();if(e!=ESP_OK)ESP_LOGE("main","MQTT: %s",esp_err_to_name(e));
 if(xTaskCreate(telemetry,"telemetry",6144,NULL,2,NULL)!=pdPASS)ESP_LOGE("main","Keine Telemetrie-Task");
 touch_display_start(status_get);
 ota_boot_ready();
 ESP_LOGI("main","Start; Lastausgaenge %s",outputs_enabled()?"FREIGEGEBEN":"GESPERRT (Inbetriebnahme)");
}
