#include "outputs.h"
#include "board.h"
#include "mixer.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"
static const int pins[]={PIN_BURNER,PIN_HEATING_PUMP,PIN_BOILER_PUMP,PIN_MIX_OPEN,PIN_MIX_CLOSE,PIN_HEATER};
static SemaphoreHandle_t lock;
static mixer_state_t mixer;
static output_request_t actual;
static int64_t last_apply;
static bool timeout_fault;
static bool ready;
bool outputs_enabled(void) {
#ifdef CONFIG_HEATING_ACTUATORS_ENABLED
 return true;
#else
 return false;
#endif
}
static void set(int pin,bool on) {
#ifdef CONFIG_HEATING_RELAY_ACTIVE_LOW
 gpio_set_level(pin,!on);
#else
 gpio_set_level(pin,on);
#endif
}
static void all_off(void) {for(unsigned i=0;i<sizeof(pins)/sizeof(pins[0]);i++)set(pins[i],false);actual=(output_request_t){0};}
static void guard(void *unused) {
 (void)unused;for(;;){vTaskDelay(pdMS_TO_TICKS(100));xSemaphoreTake(lock,portMAX_DELAY);
 if(esp_timer_get_time()/1000-last_apply>500){all_off();timeout_fault=true;mixer_init(&mixer,esp_timer_get_time()/1000);}
 xSemaphoreGive(lock);}
}
esp_err_t outputs_init(void) {
 lock=xSemaphoreCreateMutex();if(!lock)return ESP_ERR_NO_MEM;
 /* AUS-Latch VOR Umschalten auf OUTPUT; externe Pull-Widerstände bleiben nötig. */
 for(unsigned i=0;i<sizeof(pins)/sizeof(pins[0]);i++){
  set(pins[i],false);gpio_config_t g={.pin_bit_mask=1ULL<<pins[i],.mode=GPIO_MODE_OUTPUT};
  esp_err_t e=gpio_config(&g);if(e!=ESP_OK)return e;set(pins[i],false);
 }
 last_apply=esp_timer_get_time()/1000;mixer_init(&mixer,last_apply);ready=true;
 return xTaskCreate(guard,"output_guard",3072,NULL,10,NULL)==pdPASS?ESP_OK:ESP_ERR_NO_MEM;
}
void outputs_emergency_stop(void) {
 if(!ready)return;
 xSemaphoreTake(lock,portMAX_DELAY);all_off();timeout_fault=true;
 mixer_init(&mixer,esp_timer_get_time()/1000);xSemaphoreGive(lock);
}
void outputs_apply(const output_request_t *r,int64_t now,const heating_config_t *c) {
 xSemaphoreTake(lock,portMAX_DELAY);last_apply=now;
 /* Timeout wird verriegelt: Neustart und Ursache prüfen. */
 if(!outputs_enabled()||timeout_fault){all_off();xSemaphoreGive(lock);return;}
 mixer_dir_t d=mixer_step(&mixer,r->mixer,now,c->mixer_dead_ms,c->mixer_travel_s);
 set(PIN_BURNER,r->burner);set(PIN_HEATING_PUMP,r->heating_pump);set(PIN_BOILER_PUMP,r->boiler_pump);
 /* Immer beide AUS vor EIN: eine einzige zentrale Schreibstelle. */
 set(PIN_MIX_OPEN,false);set(PIN_MIX_CLOSE,false);
 if(d==MIX_OPEN)set(PIN_MIX_OPEN,true);else if(d==MIX_CLOSE)set(PIN_MIX_CLOSE,true);
 set(PIN_HEATER,false);actual=*r;actual.mixer=d;actual.heater=false;xSemaphoreGive(lock);
}
void outputs_get(output_request_t *r,float *position,bool *timed_out) {
 xSemaphoreTake(lock,portMAX_DELAY);*r=actual;*position=mixer.position;*timed_out=timeout_fault;xSemaphoreGive(lock);
}
