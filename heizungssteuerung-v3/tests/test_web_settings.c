/* Produktionsparser mit simulierter NVS-Persistenz. Kryptoadapter nutzt OpenSSL
   statt MbedTLS auf dem Host; die echte MbedTLS-Integration wird cross-kompiliert. */
#include "service_config.h"
#include "web_auth.h"
#include "stove.h"
#include "config.h"
#include "gas.h"
#include "backup.h"
#include "rest_heat.h"
#include <time.h>
#include <math.h>
#include "nvs.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <openssl/evp.h>
#include <openssl/rand.h>
#include <openssl/crypto.h>
static unsigned char stored[11][8192],pending[11][8192];
static size_t sizes[11],pending_sizes[11];
static bool commit_failure;
static int fail_after=-1;
static bool erased[11];
static unsigned slot(nvs_handle_t h,const char *key){return h==4&&!strcmp(key,"config_v2")?5:h==6&&!strcmp(key,"meter")?7:h;}
esp_err_t nvs_open(const char *name,int mode,nvs_handle_t *h){*h=!strcmp(name,"rest_heat")?10:!strcmp(name,"services_v2")?9:!strcmp(name,"restore")?8:!strcmp(name,"gas")?6:!strcmp(name,"services")?1:!strcmp(name,"stove")?3:!strcmp(name,"heating")?4:2;return mode==NVS_READONLY&&!sizes[*h]&&!((*h==4||*h==6)&&sizes[*h+1])?ESP_ERR_NVS_NOT_FOUND:ESP_OK;}
esp_err_t nvs_get_blob(nvs_handle_t h,const char *key,void *out,size_t *n){h=slot(h,key);if(!sizes[h])return ESP_ERR_NVS_NOT_FOUND;if(!out){*n=sizes[h];return ESP_OK;}if(*n<sizes[h])return ESP_ERR_INVALID_STATE;*n=sizes[h];memcpy(out,stored[h],*n);return ESP_OK;}
esp_err_t nvs_set_blob(nvs_handle_t h,const char *key,const void *v,size_t n){h=slot(h,key);assert(n<=8192);memcpy(pending[h],v,n);pending_sizes[h]=n;return ESP_OK;}
esp_err_t nvs_commit(nvs_handle_t h){if(commit_failure||fail_after==0)return ESP_FAIL;if(fail_after>0)fail_after--;for(unsigned i=h;i<=(h==4||h==6?h+1:h);i++){if(erased[i]){sizes[i]=0;pending_sizes[i]=0;erased[i]=false;}else if(pending_sizes[i]){memcpy(stored[i],pending[i],pending_sizes[i]);sizes[i]=pending_sizes[i];}}return ESP_OK;}
esp_err_t nvs_erase_key(nvs_handle_t h,const char *key){h=slot(h,key);erased[h]=true;return ESP_OK;}
void nvs_close(nvs_handle_t h){for(unsigned i=h;i<=(h==4||h==6?h+1:h);i++){pending_sizes[i]=0;erased[i]=false;}}
void esp_fill_random(void *p,size_t n){assert(RAND_bytes(p,(int)n)==1);}
void mbedtls_platform_zeroize(void *p,size_t n){OPENSSL_cleanse(p,n);}
int mbedtls_pkcs5_pbkdf2_hmac_ext(int md,const unsigned char *p,size_t n,const unsigned char *salt,size_t sn,unsigned rounds,unsigned len,unsigned char *out){(void)md;return PKCS5_PBKDF2_HMAC((const char*)p,(int)n,salt,(int)sn,(int)rounds,EVP_sha256(),(int)len,out)?0:-1;}
int mbedtls_base64_decode(unsigned char *out,size_t cap,size_t *n,const unsigned char *text,size_t len){if(len%4||len/4*3>cap)return -1;int v=EVP_DecodeBlock(out,text,(int)len);if(v<0)return -1;if(len&&text[len-1]=='=')v--;if(len>1&&text[len-2]=='=')v--;*n=(size_t)v;return 0;}
static void auth_header(char *out,const char *user,const char *pass){unsigned char raw[256];int n=snprintf((char*)raw,sizeof(raw),"%s:%s",user,pass);strcpy(out,"Basic ");EVP_EncodeBlock((unsigned char*)out+6,raw,n);}
static int64_t fake_ms=1000;
static time_t fake_epoch=1793102400;
static bool clock_valid=true;
int64_t esp_timer_get_time(void){return fake_ms*1000;}
time_t time(time_t *p){if(p)*p=fake_epoch;return fake_epoch;}
bool ntp_local_minute(int *m){*m=0;return clock_valid;}
static double gas_number(const char *key){cJSON *j=gas_status_json(fake_ms);cJSON *v=cJSON_GetObjectItemCaseSensitive(j,key);assert(cJSON_IsNumber(v));double n=v->valuedouble;cJSON_Delete(j);return n;}
static bool gas_flag(const char *key){cJSON *j=gas_status_json(fake_ms);bool b=cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(j,key));cJSON_Delete(j);return b;}
static void gas_sample(const char *value){fake_ms+=1000;fake_epoch++;assert(gas_receive("/gas/total",value,false,fake_ms));}
int main(void){
 char reason[160],header[400];service_config_t a,b;
 assert(rest_heat_init()==ESP_OK);
 rest_heat_config_t rc;rest_heat_get(&rc);assert(rc.enabled&&rc.stop_c==30&&rc.max_s==1800);
 assert(rest_heat_update("{\"stop_c\":19}",reason,sizeof(reason))==ESP_ERR_INVALID_ARG);
 commit_failure=true;assert(rest_heat_update("{\"stop_c\":35}",reason,sizeof(reason))!=ESP_OK);commit_failure=false;
 rest_heat_get(&rc);assert(rc.stop_c==30);
 assert(rest_heat_update("{\"stop_c\":35,\"max_s\":600}",reason,sizeof(reason))==ESP_OK&&rest_heat_init()==ESP_OK);
 rest_heat_get(&rc);assert(rc.stop_c==35&&rc.max_s==600);

 char joined[256];assert(mqtt_topic_join(joined,sizeof(joined),"haus/heizung","config/set")&&!strcmp(joined,"haus/heizung/config/set"));
 assert(mqtt_topic_under("haus/heizung","haus/heizung/status")&&!mqtt_topic_under("haus/heizung","haus/heizung2/status"));
 assert(!mqtt_base_valid("/haus")&&!mqtt_base_valid("haus//heizung")&&!mqtt_base_valid("haus/#")&&!mqtt_base_valid("heizung/pv/extra"));

 double parsed;assert(gas_parse_total("123456.789",&parsed)&&fabs(parsed-123456.789)<1e-8);
 const char *bad[]={"nan","inf","-1","1,2","12 m3","{}","1e999","1 2","0x12",""};
 for(unsigned i=0;i<sizeof(bad)/sizeof(*bad);i++)assert(!gas_parse_total(bad[i],&parsed));
 assert(gas_init()==ESP_OK);
 assert(gas_config_update("{\"enabled\":true,\"topic\":\"/gas/total\"}",reason,sizeof(reason))==ESP_OK);
 assert(gas_config_update("{\"topic\":\"heizung/anlage/gas\"}",reason,sizeof(reason))==ESP_ERR_INVALID_ARG);
 assert(gas_config_update("{\"stale_s\":20,\"stale_s\":30}",reason,sizeof(reason))==ESP_ERR_INVALID_ARG);
 gas_set_connected(true);assert(gas_receive("/gas/total","100",true,fake_ms));
 cJSON *gj=gas_status_json(fake_ms);assert(cJSON_IsNull(cJSON_GetObjectItemCaseSensitive(gj,"total_m3")));cJSON_Delete(gj);
 gas_sample("100.125");gas_sample("101.375");assert(fabs(gas_number("today_m3")-1.25)<1e-9&&gas_flag("fresh"));
 assert(gas_config_update("{\"baseline_m3\":99.375}",reason,sizeof(reason))==ESP_OK);
 assert(gas_number("today_m3")==2);
 assert(gas_config_update("{\"baseline_m3\":102}",reason,sizeof(reason))==ESP_ERR_INVALID_ARG);
 commit_failure=true;assert(gas_config_update("{\"baseline_m3\":98}",reason,sizeof(reason))!=ESP_OK);commit_failure=false;
 assert(gas_number("today_m3")==2);
 assert(gas_init()==ESP_OK&&gas_number("today_m3")==2&&!gas_flag("fresh"));
 gas_set_connected(true);gas_sample("102.375");assert(gas_number("today_m3")==3);
 fake_ms+=3600000;assert(!gas_flag("fresh"));
 fake_epoch+=86400;gas_sample("103");assert(gas_number("today_m3")==0);
 gas_sample("1");assert(gas_number("today_m3")==0&&gas_number("resets")==1);
 clock_valid=false;gas_sample("2");gj=gas_status_json(fake_ms);assert(cJSON_IsNull(cJSON_GetObjectItemCaseSensitive(gj,"today_m3")));cJSON_Delete(gj);
 clock_valid=true;gas_sample("3");assert(gas_number("today_m3")==2);
 assert(gas_config_update("{\"topic\":\"/gas/new\"}",reason,sizeof(reason))==ESP_OK);
 assert(gas_init()==ESP_OK);gj=gas_status_json(fake_ms);assert(cJSON_IsNull(cJSON_GetObjectItemCaseSensitive(gj,"total_m3")));cJSON_Delete(gj);

 service_config_t legacy={0};legacy.version=1;strcpy(legacy.uri,"mqtt://legacy.local:1883");strcpy(legacy.username,"legacy");strcpy(legacy.password,"test_mqtt_secret");strcpy(legacy.ntp_primary,"192.168.0.1");strcpy(legacy.ntp_secondary,"0.at.pool.ntp.org");strcpy(legacy.ntp_tertiary,"1.at.pool.ntp.org");
 assert(nvs_set_blob(1,"connection",&legacy,offsetof(service_config_t,base_topic))==ESP_OK&&nvs_commit(1)==ESP_OK);
 assert(service_config_init()==ESP_OK);service_config_get(&a);
 assert(a.version==2&&!strcmp(a.base_topic,"heizung/anlage")&&sizes[9]&&sizes[1]);
 assert(!strcmp(a.ntp_primary,"192.168.0.1")&&!strcmp(a.ntp_tertiary,"1.at.pool.ntp.org"));
 char *json=service_config_json();assert(strstr(json,"password_set")&&!strstr(json,"test_mqtt_secret")&&!strstr(json,"\"password\":"));free(json);
 assert(service_config_update("{\"uri\":\"mqtts://new.local:8883\",\"username\":\"changed\"}",reason,sizeof(reason))==ESP_OK);
 service_config_get(&a);assert(!strcmp(a.uri,"mqtts://new.local:8883")&&!strcmp(a.password,"test_mqtt_secret"));
 assert(service_config_update("{\"base_topic\":\"haus/heizung\"}",reason,sizeof(reason))==ESP_OK);
 service_config_get(&a);assert(!strcmp(a.base_topic,"haus/heizung"));
 assert(service_config_update("{\"base_topic\":\"haus/+/anlage\"}",reason,sizeof(reason))==ESP_ERR_INVALID_ARG);
 assert(service_config_update("{\"base_topic\":\"haus/anlage/\"}",reason,sizeof(reason))==ESP_ERR_INVALID_ARG);
 assert(service_config_update("{\"base_topic\":\"heizung\"}",reason,sizeof(reason))==ESP_ERR_INVALID_ARG);
 assert(service_config_update("{\"uri\":\"http://wrong\"}",reason,sizeof(reason))==ESP_ERR_INVALID_ARG);
 assert(service_config_update("{\"uri\":\"mqtt://host:abc\"}",reason,sizeof(reason))==ESP_ERR_INVALID_ARG);
 assert(service_config_update("{\"uri\":\"mqtt://host:65536\"}",reason,sizeof(reason))==ESP_ERR_INVALID_ARG);
 assert(service_config_update("{\"ntp_primary\":\"\"}",reason,sizeof(reason))==ESP_ERR_INVALID_ARG);
 assert(service_config_update("{\"ntp_primary\":\"pool ntp\"}",reason,sizeof(reason))==ESP_ERR_INVALID_ARG);
 assert(service_config_update("{\"unknown\":1}",reason,sizeof(reason))==ESP_ERR_INVALID_ARG);
 assert(service_config_update("{\"username\":\"a\",\"username\":\"b\"}",reason,sizeof(reason))==ESP_ERR_INVALID_ARG);
 commit_failure=true;assert(service_config_update("{\"password\":\"should_not_persist\"}",reason,sizeof(reason))!=ESP_OK);
 service_config_get(&b);assert(!memcmp(&a,&b,sizeof(a)));commit_failure=false;
 assert(service_config_update("{\"password\":\"\",\"ntp_secondary\":\"\"}",reason,sizeof(reason))==ESP_OK);
 assert(service_config_init()==ESP_OK);service_config_get(&b);assert(!b.password[0]&&!b.ntp_secondary[0]);
 assert(web_auth_init()==ESP_OK&&web_auth_configured());
 auth_header(header,"admin","test_web_password");assert(web_auth_verify(header));
 auth_header(header,"admin","wrong");assert(!web_auth_verify(header));assert(!web_auth_verify("Basic !!!!"));assert(!web_auth_verify("Bearer x"));
 assert(web_auth_update("{\"username\":\"newadmin\",\"password\":\"abc\"}",reason,sizeof(reason))==ESP_ERR_INVALID_ARG);
 assert(web_auth_update("{\"username\":\"bad:name\",\"password\":\"long_enough_password\"}",reason,sizeof(reason))==ESP_ERR_INVALID_ARG);
 commit_failure=true;assert(web_auth_update("{\"username\":\"newadmin\",\"password\":\"long_enough_password\"}",reason,sizeof(reason))!=ESP_OK);
 auth_header(header,"admin","test_web_password");assert(web_auth_verify(header));commit_failure=false;
 assert(web_auth_update("{\"username\":\"newadmin\",\"password\":\"long_enough_password\"}",reason,sizeof(reason))==ESP_OK);
 assert(!web_auth_verify(header));auth_header(header,"newadmin","long_enough_password");assert(web_auth_verify(header));
 assert(web_auth_init()==ESP_OK&&web_auth_verify(header));
 assert(web_auth_update("{\"username\":\"admin\",\"password\":\"kurz\"}",reason,sizeof(reason))==ESP_OK);
 auth_header(header,"admin","kurz");assert(web_auth_verify(header));
 assert(web_auth_init()==ESP_OK&&web_auth_verify(header));
 assert(!strstr((const char*)stored[2],"long_enough_password"));
 stove_config_t sc;stove_state_t ss;
 assert(stove_init()==ESP_OK);stove_get(&sc,&ss,0);assert(!sc.enabled&&!ss.valid);
 assert(stove_config_update("{\"enabled\":true,\"topic\":\"haus/kachelofen/temperatur\"}",reason,sizeof(reason))==ESP_OK);
 stove_set_connected(true);assert(stove_receive("haus/kachelofen/temperatur","45.5",false,1000));
 stove_get(&sc,&ss,1000);assert(ss.valid&&ss.active);
 assert(!stove_receive("wrong/topic","90",false,1000));
 stove_set_connected(false);assert(stove_receive("haus/kachelofen/temperatur","90",false,1001));
 stove_get(NULL,&ss,1001);assert(!ss.valid&&!ss.active);
 stove_set_connected(true);stove_receive("haus/kachelofen/temperatur","90",true,1002);
 stove_get(NULL,&ss,1002);assert(!ss.valid);
 assert(stove_config_update("{\"topic\":\"haus/heizung/kessel\"}",reason,sizeof(reason))==ESP_ERR_INVALID_ARG);
 assert(gas_config_update("{\"topic\":\"haus/heizung/gas\"}",reason,sizeof(reason))==ESP_ERR_INVALID_ARG);
 assert(stove_config_update("{\"on_c\":30,\"off_c\":32}",reason,sizeof(reason))==ESP_ERR_INVALID_ARG);
 assert(stove_config_update("{\"on_c\":46,\"on_c\":47}",reason,sizeof(reason))==ESP_ERR_INVALID_ARG);
 assert(stove_config_update("{\"topic\":\"heizung/anlage/kessel\"}",reason,sizeof(reason))==ESP_ERR_INVALID_ARG);
 commit_failure=true;assert(stove_config_update("{\"on_c\":50}",reason,sizeof(reason))!=ESP_OK);
 stove_get(&sc,NULL,2000);assert(sc.on_c==45);commit_failure=false;
 assert(stove_config_update("{\"on_c\":50,\"off_c\":35,\"stale_s\":60}",reason,sizeof(reason))==ESP_OK);
 assert(stove_init()==ESP_OK);stove_get(&sc,&ss,3000);assert(sc.on_c==50&&sc.off_c==35&&sc.stale_s==60&&!ss.valid);
 heating_config_t old,actual;config_defaults(&old);old.version=1;old.mode=MODE_OFF;old.curve_zero=46;
 assert(nvs_set_blob(4,"config_v1",&old,offsetof(heating_config_t,holiday_room_target))==ESP_OK&&nvs_commit(4)==ESP_OK);
 assert(config_init()==ESP_OK);config_get(&actual);assert(actual.version==2&&actual.mode==MODE_FROST&&actual.curve_zero==46&&actual.holiday_room_target==15);
 assert(sizes[5]==sizeof(heating_config_t));
 assert(config_update_json("{\"mode\":\"holiday\",\"holiday_room_target\":14}",reason,sizeof(reason))==ESP_OK);
 config_get(&actual);assert(actual.mode==MODE_HOLIDAY&&actual.holiday_room_target==14);
 assert(config_update_json("{\"holiday_room_target\":26}",reason,sizeof(reason))==ESP_ERR_INVALID_ARG);
 commit_failure=true;assert(config_update_json("{\"mode\":\"off\"}",reason,sizeof(reason))!=ESP_OK);
 config_get(&actual);assert(actual.mode==MODE_HOLIDAY);commit_failure=false;
 assert(config_init()==ESP_OK);config_get(&actual);assert(actual.mode==MODE_HOLIDAY&&actual.holiday_room_target==14);
 assert(config_mode_update_json("{\"mode\":\"heating\"}",reason,sizeof(reason))==ESP_OK);
 assert(config_mode_update_json("{\"mode\":\"manual\"}",reason,sizeof(reason))==ESP_ERR_INVALID_ARG);
 assert(config_mode_update_json("{\"mode\":\"off\",\"room_target\":25}",reason,sizeof(reason))==ESP_ERR_INVALID_ARG);
 assert(config_mode_update_json("{\"mode\":\"off\",\"mode\":\"auto\"}",reason,sizeof(reason))==ESP_ERR_INVALID_ARG);
 config_get(&actual);assert(actual.mode==MODE_HEATING_ONLY);

 /* Retained and malformed samples never enter the history; five-minute buckets are bounded. */
 assert(stove_init()==ESP_OK);stove_set_connected(true);
 stove_receive("haus/kachelofen/temperatur","50",true,1000);
 char *hist=stove_history_json(1000);cJSON *hj=cJSON_Parse(hist);free(hist);assert(cJSON_GetArraySize(cJSON_GetObjectItemCaseSensitive(hj,"points"))==0);cJSON_Delete(hj);
 for(unsigned i=0;i<300;i++)stove_receive("haus/kachelofen/temperatur","50.125",false,(int64_t)i*300000+1000);
 hist=stove_history_json(299*300000LL+1000);hj=cJSON_Parse(hist);free(hist);assert(cJSON_GetArraySize(cJSON_GetObjectItemCaseSensitive(hj,"points"))==288);cJSON_Delete(hj);
 stove_receive("haus/kachelofen/temperatur","bad",false,299*300000LL+2000);
 /* Full backup preflight does not mutate settings, excludes credentials and rejects extra fields. */
 char *backup=backup_export();assert(backup&&!strstr(backup,"password")&&!strstr(backup,"kurz"));
 cJSON *bj=cJSON_Parse(backup),*bh=cJSON_GetObjectItemCaseSensitive(bj,"heating");
 cJSON_SetNumberValue(cJSON_GetObjectItemCaseSensitive(bh,"curve_zero"),48);
 char *restore=cJSON_PrintUnformatted(bj);assert(backup_stage(restore,reason,sizeof(reason))==ESP_OK);
 config_get(&actual);assert(actual.curve_zero==46);assert(sizes[8]);
 commit_failure=true;assert(backup_apply_pending()!=ESP_OK);assert(sizes[8]);commit_failure=false;
 assert(backup_apply_pending()==ESP_OK&&!sizes[8]);config_get(&actual);assert(actual.curve_zero==48);
 service_config_get(&a);assert(!a.password[0]);auth_header(header,"admin","kurz");assert(web_auth_verify(header));
 assert(backup_apply_pending()==ESP_OK);
 /* Interruption after one section: keep journal, replay to completion. */
 assert(backup_stage(restore,reason,sizeof(reason))==ESP_OK);fail_after=1;
 assert(backup_apply_pending()!=ESP_OK&&sizes[8]);fail_after=-1;
 assert(backup_apply_pending()==ESP_OK&&!sizes[8]);
 cJSON *v1=cJSON_Parse(backup);cJSON_SetNumberValue(cJSON_GetObjectItemCaseSensitive(v1,"version"),1);cJSON_DeleteItemFromObjectCaseSensitive(v1,"rest_heat");cJSON_DeleteItemFromObjectCaseSensitive(cJSON_GetObjectItemCaseSensitive(v1,"services"),"base_topic");
 char *old_backup=cJSON_PrintUnformatted(v1);assert(backup_stage(old_backup,reason,sizeof(reason))==ESP_OK&&backup_apply_pending()==ESP_OK);free(old_backup);cJSON_Delete(v1);
 service_config_get(&a);assert(!strcmp(a.base_topic,"heizung/anlage"));rest_heat_get(&rc);assert(rc.stop_c==30&&rc.max_s==1800);
 cJSON_AddStringToObject(cJSON_GetObjectItemCaseSensitive(bj,"services"),"password","forbidden");free(restore);restore=cJSON_PrintUnformatted(bj);
 assert(backup_stage(restore,reason,sizeof(reason))==ESP_ERR_INVALID_ARG&&!sizes[8]);cJSON_DeleteItemFromObjectCaseSensitive(cJSON_GetObjectItemCaseSensitive(bj,"services"),"password");
 cJSON_SetNumberValue(cJSON_GetObjectItemCaseSensitive(bh,"curve_zero"),999);free(restore);restore=cJSON_PrintUnformatted(bj);
 assert(backup_stage(restore,reason,sizeof(reason))==ESP_ERR_INVALID_ARG);config_get(&actual);assert(actual.curve_zero==46);
 cJSON_Delete(bj);free(restore);free(backup);
 /* Fehlerhafte V2-Daten duerfen nicht still auf die alte V1 zurueckfallen. */
 sizes[5]=1;assert(config_init()==ESP_ERR_INVALID_STATE);
 puts("PASS: Backup/Restore-Journal/Kachelofen-Historie, Gasverbrauch/Tageswechsel/Reset/Korrektur/Persistenz, MQTT/NTP-Validierung, Passwortredaktion, persistente Updates, Commit-Fehler, Kachelofen, V1/V2-Konfigurationsmigration, Web-Anmeldung, Passwortwechsel und Neustartpersistenz.");
}
