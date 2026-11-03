#include "web.h"
#include "touch_access.h"
#include "web_auth.h"
#include "sensors.h"
#include "outputs.h"
#include "config.h"
#include "ethernet.h"
#include "mqtt.h"
#include "service_config.h"
#include "ntp.h"
#include "stove.h"
#include "gas.h"
#include "backup.h"
#include "ota.h"
#include "rest_heat.h"
#include "esp_ota_ops.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_heap_caps.h"
#include "cJSON.h"
#include "mbedtls/platform_util.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
static web_status_reader_t read_status;
static char csrf[65];
static int64_t last_write=-2000;
static esp_timer_handle_t restart_timer;
extern const unsigned char favicon_start[] asm("_binary_favicon_svg_start");
extern const unsigned char favicon_end[] asm("_binary_favicon_svg_end");
extern const unsigned char html_start[] asm("_binary_index_html_start");
extern const unsigned char html_end[] asm("_binary_index_html_end");
static void headers(httpd_req_t *r) {
    httpd_resp_set_hdr(r,"Cache-Control","no-store");
    httpd_resp_set_hdr(r,"X-Content-Type-Options","nosniff");
    httpd_resp_set_hdr(r,"X-Frame-Options","DENY");
    httpd_resp_set_hdr(r,"Referrer-Policy","no-referrer");
    httpd_resp_set_hdr(r,"Content-Security-Policy","default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'; connect-src 'self'; frame-ancestors 'none'; form-action 'self'; base-uri 'none'");
}
static esp_err_t json_send(httpd_req_t *r,cJSON *j) {
    if(!j)return httpd_resp_send_err(r,HTTPD_500_INTERNAL_SERVER_ERROR,"Speicher nicht verfuegbar");
    char *s=cJSON_PrintUnformatted(j);cJSON_Delete(j);
    if(!s)return httpd_resp_send_err(r,HTTPD_500_INTERNAL_SERVER_ERROR,"Speicher nicht verfuegbar");
    headers(r);httpd_resp_set_type(r,"application/json; charset=utf-8");
    esp_err_t e=httpd_resp_send(r,s,HTTPD_RESP_USE_STRLEN);free(s);return e;
}
static esp_err_t result(httpd_req_t *r,const char *status,const char *message) {
    httpd_resp_set_status(r,status);cJSON *j=cJSON_CreateObject();if(j)cJSON_AddStringToObject(j,"message",message);return json_send(r,j);
}
static bool auth(httpd_req_t *r) {
    if(touch_access_authorized(r))return true;
    char header[256]={0};
    bool okay=httpd_req_get_hdr_value_str(r,"Authorization",header,sizeof(header))==ESP_OK&&web_auth_verify(header);
    mbedtls_platform_zeroize(header,sizeof(header));
    if(!okay)result(r,"401 Unauthorized",web_auth_configured()?"Bitte anmelden":"Webzugang zuerst in menuconfig einrichten (Passwort mindestens 4 Zeichen)");
    return okay;
}
static esp_err_t favicon_get(httpd_req_t *r) {
    headers(r);httpd_resp_set_type(r,"image/svg+xml");return httpd_resp_send(r,(const char*)favicon_start,favicon_end-favicon_start-1);
}
static esp_err_t index_get(httpd_req_t *r) {
    headers(r);httpd_resp_set_type(r,"text/html; charset=utf-8");return httpd_resp_send(r,(const char*)html_start,html_end-html_start-1);
}
static esp_err_t status_get(httpd_req_t *r) {
    heating_status_t s;read_status(&s);sensor_snapshot_t v;sensors_get(&v);heating_config_t c;config_get(&c);
    output_request_t o;float position;bool timeout;outputs_get(&o,&position,&timeout);
    int64_t now=esp_timer_get_time()/1000;int minute=0;bool synced=ntp_local_minute(&minute);
    cJSON *j=cJSON_CreateObject();if(!j)return json_send(r,NULL);
    cJSON_AddStringToObject(j,"csrf",csrf);cJSON_AddBoolToObject(j,"login_configured",web_auth_configured());
    cJSON_AddNumberToObject(j,"faults",s.faults);cJSON_AddBoolToObject(j,"output_timeout",timeout);
    cJSON_AddBoolToObject(j,"actuators_enabled",outputs_enabled());cJSON_AddBoolToObject(j,"frost",s.frost);
    cJSON_AddBoolToObject(j,"dhw_demand",s.dhw_demand);cJSON_AddNumberToObject(j,"mixer_position",position);
    cJSON_AddNumberToObject(j,"uptime_s",now/1000);cJSON_AddNumberToObject(j,"heap_free",esp_get_free_heap_size());
    cJSON_AddNumberToObject(j,"heap_min",esp_get_minimum_free_heap_size());cJSON_AddStringToObject(j,"idf",esp_get_idf_version());
    cJSON_AddStringToObject(j,"version",esp_app_get_description()->version);
    const esp_app_desc_t *app=esp_app_get_description();char build[48];snprintf(build,sizeof(build),"%s %s",app->date,app->time);cJSON_AddStringToObject(j,"firmware_build",build);
    const esp_partition_t *slot=esp_ota_get_running_partition();cJSON_AddStringToObject(j,"firmware_slot",slot?slot->label:"unbekannt");esp_ota_img_states_t state;cJSON_AddBoolToObject(j,"ota_pending",slot&&esp_ota_get_state_partition(slot,&state)==ESP_OK&&state==ESP_OTA_IMG_PENDING_VERIFY);cJSON_AddBoolToObject(j,"ethernet",ethernet_has_ip());
    cJSON_AddBoolToObject(j,"mqtt",mqtt_connected());cJSON_AddBoolToObject(j,"time_synced",synced);
    char clock[40]="Nicht synchronisiert";
    if(synced){time_t t=time(NULL);struct tm local;localtime_r(&t,&local);strftime(clock,sizeof(clock),"%d.%m.%Y %H:%M:%S",&local);}
    cJSON_AddStringToObject(j,"clock",clock);
    service_config_t services;service_config_get(&services);
    cJSON *display=cJSON_AddObjectToObject(j,"display_settings");
    cJSON_AddStringToObject(display,"base_topic",services.base_topic);
    cJSON_AddStringToObject(display,"ntp_primary",services.ntp_primary);
    cJSON_AddStringToObject(display,"ntp_secondary",services.ntp_secondary);
    cJSON_AddStringToObject(display,"ntp_tertiary",services.ntp_tertiary);
    long sync_age=ntp_sync_age_s();
    cJSON_AddItemToObject(j,"ntp_sync_age_s",sync_age<0?cJSON_CreateNull():cJSON_CreateNumber(sync_age));
    char *stove_json=stove_config_json(),*gas_json=gas_config_json();
    if(stove_json){cJSON_AddItemToObject(display,"stove",cJSON_Parse(stove_json));free(stove_json);}
    if(gas_json){cJSON_AddItemToObject(display,"gas",cJSON_Parse(gas_json));free(gas_json);}

    esp_netif_t *net=esp_netif_get_handle_from_ifkey("ETH_DEF");esp_netif_ip_info_t ip;
    char address[20]="-";if(net&&esp_netif_get_ip_info(net,&ip)==ESP_OK&&ip.ip.addr)snprintf(address,sizeof(address),IPSTR,IP2STR(&ip.ip));
    cJSON_AddStringToObject(j,"ip",address);
    cJSON_AddStringToObject(j,"mode",config_mode_name(c.mode));
    cJSON_AddNumberToObject(j,"room_target",s.room_target);cJSON_AddNumberToObject(j,"flow_target",s.flow_target);
    cJSON_AddNumberToObject(j,"boiler_target",c.boiler_target);cJSON_AddNumberToObject(j,"kessel_target",s.kessel_target);
    stove_config_t sc;stove_state_t ss;stove_get(&sc,&ss,now);
    cJSON *st=cJSON_AddObjectToObject(j,"stove");cJSON_AddBoolToObject(st,"enabled",sc.enabled);
    cJSON_AddBoolToObject(st,"valid",ss.valid);cJSON_AddBoolToObject(st,"active",ss.valid&&ss.active);
    cJSON_AddItemToObject(st,"temperature",ss.valid?cJSON_CreateNumber(ss.celsius):cJSON_CreateNull());
    cJSON_AddNumberToObject(st,"errors",ss.errors);
    cJSON_AddItemToObject(st,"age_s",ss.valid?cJSON_CreateNumber((now-ss.updated_ms)/1000):cJSON_CreateNull());
    cJSON_AddBoolToObject(j,"room_correction_suppressed",s.room_correction_suppressed);
    rest_heat_config_t rest;rest_heat_get(&rest);cJSON *rh=cJSON_AddObjectToObject(j,"rest_heat");
    cJSON_AddBoolToObject(rh,"enabled",rest.enabled);cJSON_AddNumberToObject(rh,"stop_c",rest.stop_c);cJSON_AddNumberToObject(rh,"max_s",rest.max_s);
    cJSON_AddBoolToObject(rh,"active",s.rest_heat_active);cJSON_AddNumberToObject(rh,"remaining_s",s.rest_heat_remaining_s);

    cJSON_AddItemToObject(j,"gas",gas_status_json(now));
    sensor_diagnostic_t diag[SENSOR_COUNT];sensors_get_diagnostics(diag);
    cJSON *a=cJSON_AddArrayToObject(j,"sensors");
    for(int i=0;i<SENSOR_COUNT;i++){
        cJSON *entry=cJSON_CreateObject();bool good=v.value[i].valid&&isfinite(v.value[i].celsius)&&v.value[i].sampled_ms<=now&&now-v.value[i].sampled_ms<(int64_t)c.sensor_stale_s*1000;
        cJSON_AddBoolToObject(entry,"valid",good);cJSON_AddItemToObject(entry,"temperature",good?cJSON_CreateNumber(v.value[i].celsius):cJSON_CreateNull());
        cJSON_AddNumberToObject(entry,"gpio",diag[i].gpio);cJSON_AddStringToObject(entry,"diagnosis",sensor_diagnostic_text(diag[i].code));
        cJSON_AddBoolToObject(entry,"presence",diag[i].presence);cJSON_AddNumberToObject(entry,"attempts",diag[i].attempts);cJSON_AddNumberToObject(entry,"successes",diag[i].successes);
        char rom[17]="",scratch[19]="";
        if(diag[i].have_rom)for(int n=0;n<8;n++)snprintf(rom+2*n,3,"%02X",diag[i].rom[n]);
        if(diag[i].have_scratch)for(int n=0;n<9;n++)snprintf(scratch+2*n,3,"%02X",diag[i].scratch[n]);
        cJSON_AddStringToObject(entry,"rom",rom);cJSON_AddStringToObject(entry,"scratchpad",scratch);
        cJSON_AddNumberToObject(entry,"failures",v.value[i].failures);cJSON_AddItemToArray(a,entry);
    }
    cJSON *out=cJSON_AddObjectToObject(j,"outputs");cJSON_AddBoolToObject(out,"burner",o.burner);
    cJSON_AddBoolToObject(out,"heating_pump",o.heating_pump);cJSON_AddBoolToObject(out,"boiler_pump",o.boiler_pump);
    cJSON_AddStringToObject(out,"mixer",o.mixer==MIX_OPEN?"AUF":o.mixer==MIX_CLOSE?"ZU":"STOP");
    return json_send(r,j);
}
static esp_err_t config_get_handler(httpd_req_t *r) {
    /* Reglerparameter sind lesbar; Zugangsdaten bleiben hinter der Anmeldung. */
    if(strcmp(r->uri,"/api/config")&&!auth(r))return ESP_OK;
    char *s=!strcmp(r->uri,"/api/rest-heat")?rest_heat_json():!strcmp(r->uri,"/api/gas")?gas_config_json():!strcmp(r->uri,"/api/stove")?stove_config_json():!strcmp(r->uri,"/api/services")?service_config_json():config_json();
    if(!s)return result(r,"500 Internal Server Error","Speicher nicht verfuegbar");
    headers(r);httpd_resp_set_type(r,"application/json");esp_err_t e=httpd_resp_send(r,s,HTTPD_RESP_USE_STRLEN);free(s);return e;
}
static esp_err_t history_get(httpd_req_t *r){
 unsigned span=3600;char query[32],hours[4];
 if(httpd_req_get_url_query_str(r,query,sizeof(query))==ESP_OK&&httpd_query_key_value(query,"hours",hours,sizeof(hours))==ESP_OK){
  if(!strcmp(hours,"6"))span=21600;else if(!strcmp(hours,"24"))span=86400;
 }
 int64_t now=esp_timer_get_time()/1000;char *text=stove_history_json(now);
 cJSON *j=text?cJSON_Parse(text):NULL;free(text);if(!j)return result(r,"500 Internal Server Error","Speicher nicht verfuegbar");
 cJSON *history=sensors_history_json(now,span);if(!history){cJSON_Delete(j);return result(r,"500 Internal Server Error","Speicher nicht verfuegbar");}
 cJSON_AddItemToObject(j,"sensors",history);return json_send(r,j);
}
static esp_err_t backup_get(httpd_req_t *r){
 if(!auth(r))return ESP_OK;
 char *text=backup_export();if(!text)return result(r,"500 Internal Server Error","Speicher nicht verfuegbar");
 headers(r);httpd_resp_set_type(r,"application/json");httpd_resp_set_hdr(r,"Content-Disposition","attachment; filename=heizung-einstellungen.json");
 esp_err_t e=httpd_resp_send(r,text,HTTPD_RESP_USE_STRLEN);free(text);return e;
}
static void restart(void *unused) {(void)unused;esp_restart();}
static esp_err_t firmware_upload(httpd_req_t *r){
 if(!auth(r))return ESP_FAIL;
 char token[65]={0},type[64]={0};
 if(httpd_req_get_hdr_value_str(r,"X-CSRF-Token",token,sizeof(token))!=ESP_OK||strcmp(token,csrf)){result(r,"403 Forbidden","Seite neu laden");return ESP_FAIL;}
 if(httpd_req_get_hdr_value_str(r,"Content-Type",type,sizeof(type))!=ESP_OK||strcmp(type,"application/octet-stream")){result(r,"415 Unsupported Media Type","Firmware-Datei erforderlich");return ESP_FAIL;}
 if(ota_restart_required()){result(r,"409 Conflict","Neustart abwarten");return ESP_FAIL;}
 char reason[192]="Update konnte nicht gestartet werden";esp_err_t e=ota_upload(r,reason,sizeof(reason));
 if(ota_restart_required()){
  esp_err_t timer=esp_timer_start_once(restart_timer,3000000);
  if(timer!=ESP_OK)snprintf(reason,sizeof(reason),"Bitte manuell neu starten; Uebertragung abgeschlossen oder abgebrochen");
 }
 httpd_resp_set_hdr(r,"Connection","close");
 esp_err_t sent=result(r,e==ESP_OK?"200 OK":e==ESP_ERR_INVALID_ARG?"400 Bad Request":e==ESP_ERR_INVALID_STATE?"409 Conflict":"500 Internal Server Error",reason);
 return e==ESP_OK?sent:ESP_FAIL;
}
static esp_err_t update(httpd_req_t *r) {
    if(ota_restart_required())return result(r,"409 Conflict","Firmware-Neustart abwarten");
    bool mode_only=!strcmp(r->uri,"/api/mode"),restore=!strcmp(r->uri,"/api/restore");
    if(!mode_only&&!auth(r))return ESP_OK;
    char token[65]={0},type[64]={0};
    if(httpd_req_get_hdr_value_str(r,"X-CSRF-Token",token,sizeof(token))!=ESP_OK||strcmp(token,csrf))return result(r,"403 Forbidden","Seite neu laden");
    if(httpd_req_get_hdr_value_str(r,"Content-Type",type,sizeof(type))!=ESP_OK||strncmp(type,"application/json",16))return result(r,"415 Unsupported Media Type","JSON erforderlich");
    if(!r->content_len||r->content_len>(restore?8191:2048))return result(r,"413 Payload Too Large","Datei oder Anfrage zu gross");
    int64_t now=esp_timer_get_time()/1000;
    if(now-last_write<2000)return result(r,"429 Too Many Requests","Bitte zwei Sekunden warten");
    char *body=calloc(1,r->content_len+1);if(!body)return result(r,"500 Internal Server Error","Speicher nicht verfuegbar");
    size_t received=0;
    while(received<r->content_len){
        int n=httpd_req_recv(r,body+received,r->content_len-received);
        if(n<=0||esp_timer_get_time()/1000-now>4000){free(body);return result(r,"408 Request Timeout","Uebertragung abgebrochen");}
        received+=n;
    }
    if(memchr(body,0,r->content_len)){free(body);return result(r,"400 Bad Request","Ungueltige Eingabe");}
    char reason[160];esp_err_t e;
    bool gas=!strcmp(r->uri,"/api/gas"),stove=!strcmp(r->uri,"/api/stove"),services=!strcmp(r->uri,"/api/services"),account=!strcmp(r->uri,"/api/account");
    if(!strcmp(r->uri,"/api/rest-heat"))e=rest_heat_update(body,reason,sizeof(reason));
    else if(restore)e=backup_stage(body,reason,sizeof(reason));
    else if(mode_only)e=config_mode_update_json(body,reason,sizeof(reason));
    else if(gas)e=gas_config_update(body,reason,sizeof(reason));
    else if(stove)e=stove_config_update(body,reason,sizeof(reason));
    else if(services)e=service_config_update(body,reason,sizeof(reason));
    else if(account)e=web_auth_update(body,reason,sizeof(reason));
    else e=config_update_json(body,reason,sizeof(reason));
    mbedtls_platform_zeroize(body,r->content_len);free(body);last_write=now;
    if(e!=ESP_OK)return result(r,e==ESP_ERR_INVALID_ARG?"400 Bad Request":"500 Internal Server Error",reason);
    if(restore||services||stove||gas){e=esp_timer_start_once(restart_timer,3000000);if(e!=ESP_OK)return result(r,"500 Internal Server Error","Gespeichert; bitte manuell neu starten");}
    return result(r,"200 OK",account?"Webzugang gespeichert. Bitte neu anmelden.":(restore||services||stove||gas)?"Gespeichert. Neustart in drei Sekunden.":"Einstellungen gespeichert");
}
esp_err_t web_init(web_status_reader_t reader) {
    read_status=reader;esp_err_t e=web_auth_init();if(e!=ESP_OK)return e;
    e=touch_access_init();if(e!=ESP_OK)return e;
    unsigned char random[32];esp_fill_random(random,sizeof(random));for(int i=0;i<32;i++)snprintf(csrf+2*i,3,"%02x",random[i]);
    esp_timer_create_args_t timer={.callback=restart,.name="web_restart"};e=esp_timer_create(&timer,&restart_timer);if(e!=ESP_OK)return e;
    httpd_config_t cfg=HTTPD_DEFAULT_CONFIG();cfg.stack_size=12288;cfg.max_uri_handlers=21;
    cfg.max_open_sockets=4;cfg.lru_purge_enable=true;cfg.recv_wait_timeout=3;cfg.send_wait_timeout=3;
    httpd_handle_t server=NULL;e=httpd_start(&server,&cfg);if(e!=ESP_OK)return e;
    const httpd_uri_t routes[]={
        {.uri="/api/rest-heat",.method=HTTP_GET,.handler=config_get_handler},
        {.uri="/api/rest-heat",.method=HTTP_POST,.handler=update},
        {.uri="/api/firmware",.method=HTTP_POST,.handler=firmware_upload},
        {.uri="/api/stove/history",.method=HTTP_GET,.handler=history_get},
        {.uri="/api/backup",.method=HTTP_GET,.handler=backup_get},
        {.uri="/api/restore",.method=HTTP_POST,.handler=update},
        {.uri="/favicon.svg",.method=HTTP_GET,.handler=favicon_get},
        {.uri="/api/mode",.method=HTTP_POST,.handler=update},
        {.uri="/",.method=HTTP_GET,.handler=index_get}, {.uri="/api/status",.method=HTTP_GET,.handler=status_get},
        {.uri="/api/config",.method=HTTP_GET,.handler=config_get_handler}, {.uri="/api/config",.method=HTTP_POST,.handler=update},
        {.uri="/api/services",.method=HTTP_GET,.handler=config_get_handler}, {.uri="/api/services",.method=HTTP_POST,.handler=update},
        {.uri="/api/account",.method=HTTP_POST,.handler=update},
        {.uri="/api/stove",.method=HTTP_GET,.handler=config_get_handler},
        {.uri="/api/stove",.method=HTTP_POST,.handler=update},
        {.uri="/api/gas",.method=HTTP_GET,.handler=config_get_handler},
        {.uri="/api/gas",.method=HTTP_POST,.handler=update}};
    for(unsigned i=0;i<sizeof(routes)/sizeof(routes[0]);i++){e=httpd_register_uri_handler(server,&routes[i]);if(e!=ESP_OK){httpd_stop(server);return e;}}
    ESP_LOGI("web","HTTP-Weboberflaeche auf Port 80; Bearbeitung %s",web_auth_configured()?"mit Login":"erst nach Web-Ersteinrichtung");return ESP_OK;
}
