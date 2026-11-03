#include "ota.h"
#include "ota_rules.h"
#include "outputs.h"
#include "config.h"
#include "esp_ota_ops.h"
#include "esp_app_desc.h"
#include "esp_app_format.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <stdatomic.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
_Static_assert(sizeof(esp_image_header_t)+sizeof(esp_image_segment_header_t)==32,"App descriptor offset");
_Static_assert(offsetof(esp_image_header_t,min_chip_rev_full)==15,"Revision offset");
_Static_assert(offsetof(esp_app_desc_t,project_name)==48,"Project offset");
static atomic_bool ready,restart_required;
static _Atomic uint32_t control_tick_s;
static void confirm_boot(void *unused){
 (void)unused;uint32_t now=(uint32_t)(esp_timer_get_time()/1000000);
 if(atomic_load(&ready)&&(uint32_t)(now-atomic_load(&control_tick_s))<2){
  output_request_t r;float position;bool timeout;outputs_get(&r,&position,&timeout);
  if(!timeout&&esp_ota_mark_app_valid_cancel_rollback()==ESP_OK){ESP_LOGI("ota","Teststart bestaetigt");return;}
 }
 ESP_LOGE("ota","Teststart nicht bestaetigt; Neustart fuer Rueckfall");esp_restart();
}
esp_err_t ota_boot_start(void){
 esp_ota_img_states_t state;const esp_partition_t *p=esp_ota_get_running_partition();
 if(!p||esp_ota_get_state_partition(p,&state)!=ESP_OK||state!=ESP_OTA_IMG_PENDING_VERIFY)return ESP_OK;
 esp_timer_handle_t timer;esp_timer_create_args_t args={.callback=confirm_boot,.name="ota_boot_check"};
 esp_err_t e=esp_timer_create(&args,&timer);return e==ESP_OK?esp_timer_start_once(timer,45000000):e;
}
void ota_control_tick(int64_t now){atomic_store(&control_tick_s,(uint32_t)(now/1000));}
void ota_boot_ready(void){atomic_store(&ready,true);}
bool ota_restart_required(void){return atomic_load(&restart_required);}
esp_err_t ota_upload(httpd_req_t *r,char *reason,unsigned size){
 const esp_partition_t *target=esp_ota_get_next_update_partition(NULL),*running=esp_ota_get_running_partition();
 esp_ota_img_states_t state;
 if(running&&esp_ota_get_state_partition(running,&state)==ESP_OK&&state==ESP_OTA_IMG_PENDING_VERIFY){snprintf(reason,size,"Teststart noch nicht bestaetigt; bitte 45 Sekunden warten");return ESP_ERR_INVALID_STATE;}
 if(!target||r->content_len<OTA_PREFIX_SIZE||r->content_len>target->size){snprintf(reason,size,"Keine passende OTA-Partition oder Datei zu gross/zu klein");return ESP_ERR_INVALID_ARG;}
 heating_config_t c;config_get(&c);output_request_t actual;float position;bool timeout;outputs_get(&actual,&position,&timeout);
 if(c.mode!=MODE_OFF||actual.burner||actual.heating_pump||actual.boiler_pump||actual.mixer!=MIX_STOP){snprintf(reason,size,"Vor Update Betriebsart Aus waehlen und Pumpennachlauf abwarten");return ESP_ERR_INVALID_STATE;}
 uint8_t *buffer=malloc(4096);if(!buffer)return ESP_ERR_NO_MEM;
 size_t got=0;esp_err_t e=ESP_ERR_INVALID_ARG;esp_ota_handle_t handle=0;bool opened=false;
 int64_t started=esp_timer_get_time();
 while(got<OTA_PREFIX_SIZE){if(esp_timer_get_time()-started>10000000)goto failed;int n=httpd_req_recv(r,(char*)buffer+got,OTA_PREFIX_SIZE-got);if(n<=0)goto failed;got+=n;}
 esp_chip_info_t chip;esp_chip_info(&chip);
 if(!ota_prefix_valid(buffer,got,chip.revision,esp_app_get_description()->project_name)){snprintf(reason,size,"Keine passende Heizungs-Firmware fuer diesen Chip");goto done;}
 /* All further output writes remain latched off until the scheduled restart. */
 outputs_emergency_stop();atomic_store(&restart_required,true);
 e=esp_ota_begin(target,OTA_WITH_SEQUENTIAL_WRITES,&handle);if(e!=ESP_OK)goto failed;opened=true;
 e=esp_ota_write(handle,buffer,got);if(e!=ESP_OK)goto failed;
 while(got<r->content_len){
  if(esp_timer_get_time()-started>180000000){e=ESP_ERR_TIMEOUT;goto failed;}
  size_t want=r->content_len-got;if(want>4096)want=4096;
  int n=httpd_req_recv(r,(char*)buffer,want);if(n<=0){e=ESP_FAIL;goto failed;}
  e=esp_ota_write(handle,buffer,n);if(e!=ESP_OK)goto failed;got+=n;
 }
 e=esp_ota_end(handle);opened=false;if(e!=ESP_OK)goto failed;
 e=esp_ota_set_boot_partition(target);if(e!=ESP_OK)goto failed;
 snprintf(reason,size,"Firmware geprueft. Neustart und Teststart folgen.");goto done;
failed:
 if(opened)esp_ota_abort(handle);
 snprintf(reason,size,"Update fehlgeschlagen; bisherige Firmware bleibt ausgewaehlt. Neustart erforderlich.");
done:
 free(buffer);return e;
}
