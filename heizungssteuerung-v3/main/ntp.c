#include "ntp.h"
#include "service_config.h"
static service_config_t servers;
#include "esp_sntp.h"
#include "esp_timer.h"
#include <stdlib.h>
#include <time.h>
#include <stdatomic.h>
#include <stdint.h>
static _Atomic uint32_t last_sync_s;
static atomic_bool has_sync;
static void synced(struct timeval *tv){(void)tv;atomic_store(&last_sync_s,(uint32_t)(esp_timer_get_time()/1000000));atomic_store(&has_sync,true);}
void ntp_init(void){
 setenv("TZ",HEATING_TIMEZONE,1);tzset();
 service_config_get(&servers);
 esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);esp_sntp_setservername(0,servers.ntp_primary);
 if(servers.ntp_secondary[0])esp_sntp_setservername(1,servers.ntp_secondary);
 if(servers.ntp_tertiary[0])esp_sntp_setservername(2,servers.ntp_tertiary);
 esp_sntp_set_time_sync_notification_cb(synced);esp_sntp_init();
}
bool ntp_local_minute(int *minute){
 uint32_t last=atomic_load(&last_sync_s);
 if(!atomic_load(&has_sync)||(uint32_t)(esp_timer_get_time()/1000000)-last>86400)return false;
 time_t now=time(NULL);struct tm local;localtime_r(&now,&local);*minute=local.tm_hour*60+local.tm_min;return true;
}

long ntp_sync_age_s(void){return atomic_load(&has_sync)?(long)((uint32_t)(esp_timer_get_time()/1000000)-atomic_load(&last_sync_s)):-1;}
