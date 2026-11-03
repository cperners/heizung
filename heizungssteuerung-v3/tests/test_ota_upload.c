/* Production OTA orchestration with explicit host-only transport/flash doubles.
   The firmware build separately checks the real ESP-IDF APIs. */
#include "ota.h"
#include "ota_rules.h"
#include "config.h"
#include "outputs.h"
#include "ota_fake.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static const char *scenario;
static unsigned char image[1024];
static size_t cursor,written;
static bool begun,ended,aborted,selected,stopped,restarted,marked;
static void(*timer_callback)(void*);
static int64_t now_us=1000000;
static const esp_partition_t partition={2097152,"ota_0"};
static const esp_app_desc_t desc={.project_name="heizungssteuerung"};
static bool is(const char *s){return !strcmp(scenario,s);}
const esp_app_desc_t *esp_app_get_description(void){return &desc;}
void esp_chip_info(esp_chip_info_t *c){c->revision=302;}
void config_get(heating_config_t *c){memset(c,0,sizeof(*c));c->mode=is("mode")?MODE_AUTO:MODE_OFF;}
void outputs_get(output_request_t *r,float *position,bool *timeout){memset(r,0,sizeof(*r));r->heating_pump=is("pump");*position=0;*timeout=stopped;}
void outputs_emergency_stop(void){stopped=true;}
const esp_partition_t *esp_ota_get_running_partition(void){return &partition;}
const esp_partition_t *esp_ota_get_next_update_partition(const void *unused){(void)unused;return &partition;}
esp_err_t esp_ota_get_state_partition(const esp_partition_t *p,esp_ota_img_states_t *s){(void)p;*s=(is("pending")||is("health")||is("healthfail"))?ESP_OTA_IMG_PENDING_VERIFY:ESP_OTA_IMG_VALID;return ESP_OK;}
esp_err_t esp_ota_mark_app_valid_cancel_rollback(void){marked=true;return ESP_OK;}
esp_err_t esp_ota_begin(const esp_partition_t *p,size_t n,esp_ota_handle_t *h){assert(p==&partition&&n==OTA_WITH_SEQUENTIAL_WRITES);begun=true;*h=1;return is("begin")?ESP_FAIL:ESP_OK;}
esp_err_t esp_ota_write(esp_ota_handle_t h,const void *p,size_t n){assert(h==1&&p);if(is("write"))return ESP_FAIL;written+=n;return ESP_OK;}
esp_err_t esp_ota_end(esp_ota_handle_t h){assert(h==1);ended=true;return is("checksum")?ESP_FAIL:ESP_OK;}
esp_err_t esp_ota_abort(esp_ota_handle_t h){assert(h==1);aborted=true;return ESP_OK;}
esp_err_t esp_ota_set_boot_partition(const esp_partition_t *p){assert(p==&partition&&ended&&written==sizeof(image));if(is("select"))return ESP_FAIL;selected=true;return ESP_OK;}
esp_err_t esp_timer_create(const esp_timer_create_args_t *a,esp_timer_handle_t *h){timer_callback=a->callback;*h=1;return ESP_OK;}
esp_err_t esp_timer_start_once(esp_timer_handle_t h,uint64_t us){assert(h==1&&us==45000000);return ESP_OK;}
int64_t esp_timer_get_time(void){return now_us;}
void esp_restart(void){restarted=true;}
int httpd_req_recv(httpd_req_t *r,char *out,size_t n){(void)r;if(is("disconnect")&&cursor>=400)return -1;if(n>17)n=17;if(n>sizeof(image)-cursor)n=sizeof(image)-cursor;memcpy(out,image+cursor,n);cursor+=n;return (int)n;}
int main(int argc,char **argv){
 assert(argc==2);scenario=argv[1];
 if(is("health")||is("healthfail")){assert(ota_boot_start()==ESP_OK&&timer_callback);now_us=45000000;ota_control_tick(45000);if(is("health"))ota_boot_ready();timer_callback(NULL);assert(marked==is("health")&&restarted==is("healthfail"));return 0;}
 image[0]=0xe9;image[1]=7;image[12]=0x12;image[15]=301&255;image[16]=301>>8;image[17]=399&255;image[18]=399>>8;image[23]=1;image[32]=0x32;image[33]=0x54;image[34]=0xcd;image[35]=0xab;strcpy((char*)image+80,"heizungssteuerung");
 if(is("chip"))image[12]=9;
 httpd_req_t request={.content_len=is("size")?2097153:sizeof(image)};char reason[192];
 esp_err_t e=ota_upload(&request,reason,sizeof(reason));
 if(is("success")){assert(e==ESP_OK&&selected&&begun&&ended&&stopped&&!aborted&&ota_restart_required());}
 else{assert(e!=ESP_OK&&!selected);if(is("chip")||is("size")||is("mode")||is("pump")||is("pending"))assert(!begun&&!stopped);if(is("disconnect")||is("write"))assert(aborted);if(is("checksum"))assert(ended&&!aborted);}
 return 0;
}
