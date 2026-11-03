#include "touch_access.h"
#include "sdkconfig.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "mbedtls/pkcs5.h"
#include "mbedtls/platform_util.h"
#include "lwip/sockets.h"
#include <string.h>
#include <ctype.h>
#include <stdio.h>
typedef struct {unsigned version;unsigned char salt[16],hash[32];} pin_record_t;
static pin_record_t record;
static SemaphoreHandle_t lock;
static bool unlocked;
static unsigned epoch;
static char token[65];
static int64_t last_try=-2000;
static bool valid(const char *p){size_t n=strlen(p);if(n<4||n>8)return false;for(;*p;p++)if(*p<'0'||*p>'9')return false;return true;}
static bool derive(const char *p,const unsigned char *salt,unsigned char *hash){return mbedtls_pkcs5_pbkdf2_hmac_ext(MBEDTLS_MD_SHA256,(const unsigned char*)p,strlen(p),salt,16,10000,32,hash)==0;}
esp_err_t touch_access_init(void){
 unlocked=false;epoch++;last_try=-2000;memset(&record,0,sizeof(record));
 lock=xSemaphoreCreateMutex();if(!lock)return ESP_ERR_NO_MEM;
 unsigned char random[32];esp_fill_random(random,sizeof(random));for(unsigned i=0;i<32;i++)snprintf(token+2*i,3,"%02x",random[i]);mbedtls_platform_zeroize(random,sizeof(random));
 nvs_handle_t h;esp_err_t e=nvs_open("touch_access",NVS_READONLY,&h);
 if(e==ESP_OK){size_t n=sizeof(record);e=nvs_get_blob(h,"pin",&record,&n);nvs_close(h);if(e==ESP_OK)return n==sizeof(record)&&record.version==1?ESP_OK:ESP_ERR_INVALID_STATE;}
 if(e!=ESP_ERR_NVS_NOT_FOUND)return e;
 record.version=1;esp_fill_random(record.salt,sizeof(record.salt));
 if(!valid(CONFIG_HEATING_TOUCH_PIN))return ESP_ERR_INVALID_ARG;
 return derive(CONFIG_HEATING_TOUCH_PIN,record.salt,record.hash)?ESP_OK:ESP_FAIL;
}
unsigned touch_access_epoch(void){if(!lock)return 0;xSemaphoreTake(lock,portMAX_DELAY);unsigned n=epoch;xSemaphoreGive(lock);return n;}
bool touch_access_unlock(const char *pin,unsigned expected_epoch){
 if(!lock||!valid(pin))return false;
 xSemaphoreTake(lock,portMAX_DELAY);int64_t now=esp_timer_get_time()/1000;
 if(expected_epoch!=epoch||now-last_try<2000){xSemaphoreGive(lock);return false;}last_try=now;
 unsigned char actual[32];bool ok=derive(pin,record.salt,actual);unsigned char difference=0;
 if(ok)for(unsigned i=0;i<32;i++)difference|=actual[i]^record.hash[i];
 mbedtls_platform_zeroize(actual,sizeof(actual));
 unlocked=ok&&!difference;xSemaphoreGive(lock);return ok&&!difference;
}
void touch_access_lock(void){if(lock){xSemaphoreTake(lock,portMAX_DELAY);unlocked=false;epoch++;xSemaphoreGive(lock);}}
bool touch_access_unlocked(void){if(!lock)return false;xSemaphoreTake(lock,portMAX_DELAY);bool u=unlocked;xSemaphoreGive(lock);return u;}
esp_err_t touch_access_change(const char *pin){
 if(!lock||!valid(pin))return ESP_ERR_INVALID_ARG;
 xSemaphoreTake(lock,portMAX_DELAY);if(!unlocked){xSemaphoreGive(lock);return ESP_ERR_INVALID_STATE;}
 pin_record_t next={.version=1};esp_fill_random(next.salt,sizeof(next.salt));esp_err_t e=derive(pin,next.salt,next.hash)?ESP_OK:ESP_FAIL;
 if(e==ESP_OK){nvs_handle_t h;e=nvs_open("touch_access",NVS_READWRITE,&h);if(e==ESP_OK){e=nvs_set_blob(h,"pin",&next,sizeof(next));if(e==ESP_OK)e=nvs_commit(h);nvs_close(h);}}
 if(e==ESP_OK){record=next;unlocked=false;epoch++;}mbedtls_platform_zeroize(&next,sizeof(next));xSemaphoreGive(lock);return e;
}
bool touch_access_token(char *out,size_t size){if(size<sizeof(token)||!touch_access_unlocked())return false;memcpy(out,token,sizeof(token));return true;}
bool touch_access_authorized(httpd_req_t *r){
 if(!touch_access_unlocked())return false;
 struct sockaddr_storage peer;socklen_t size=sizeof(peer);
 if(getpeername(httpd_req_to_sockfd(r),(struct sockaddr*)&peer,&size)!=0)return false;
 bool local=false;
 if(peer.ss_family==AF_INET)local=ntohl(((struct sockaddr_in*)&peer)->sin_addr.s_addr)==INADDR_LOOPBACK;
 else if(peer.ss_family==AF_INET6){
  static const unsigned char v4_loop[16]={0,0,0,0,0,0,0,0,0,0,255,255,127,0,0,1};
  static const unsigned char v6_loop[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1};
  const void *a=&((struct sockaddr_in6*)&peer)->sin6_addr;
  local=!memcmp(a,v4_loop,sizeof(v4_loop))||!memcmp(a,v6_loop,sizeof(v6_loop));
 }
 if(!local)return false;
 char supplied[65]={0};bool ok=httpd_req_get_hdr_value_str(r,"X-Touch-Session",supplied,sizeof(supplied))==ESP_OK;
 unsigned char diff=0;for(unsigned i=0;i<sizeof(token);i++)diff|=supplied[i]^token[i];mbedtls_platform_zeroize(supplied,sizeof(supplied));return ok&&!diff;
}
