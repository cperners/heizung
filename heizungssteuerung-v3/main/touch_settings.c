#include "touch_settings.h"
#include "touch_access.h"
#include "esp_http_client.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "mbedtls/platform_util.h"
#include "cJSON.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <errno.h>
#include <stdarg.h>
#define MAX_FIELDS 16
#define TEXT_SIZE 192
#define API_LIMIT 16384
/* All mutations use the existing HTTP handlers through the device's loopback.
   Only a boot-random token plus an unlocked physical PIN session is accepted;
   the server additionally checks that the socket peer is 127.0.0.1. */
typedef enum {NUMBER,TEXT,SECRET,BOOL,CLOCK} field_type_t;
typedef struct {const char *key,*label;field_type_t type;} spec_t;
typedef struct {spec_t spec;char text[TEXT_SIZE];bool checked;lv_obj_t *button,*value;} field_t;
typedef enum {REG_ROOM,REG_CURVE,REG_BOILER,REG_TIME,REG_KESSEL,REG_MIXER,REG_REST,REG_MQTT,REG_NTP,REG_STOVE,REG_GAS,REG_BASIS,REG_ACCOUNT,REG_PIN,REG_FILES,REG_SYSTEM,REG_COUNT} section_t;
static const char *sections[]={"Raumtemperatur","Heizkurve & Vorlauf","Boiler","Zeitprogramm","Kessel & Brenner","Mischer & Frostschutz","Restwaerme","MQTT","Zeitserver","Kachelofen","Gaszaehler","Gas-Tagesbasis","Webzugang","Display-Zahlencode","Sicherung & Firmware","System & Fuehler"};
static const char *routes[]={"/api/config","/api/config","/api/config","/api/config","/api/config","/api/config","/api/rest-heat","/api/services","/api/services","/api/stove","/api/gas","/api/gas","/api/account","","","/api/status"};
#define F(k,l,t) {k,l,t}
static const spec_t room[]={F("room_target","Raumsoll (C)",NUMBER),F("holiday_room_target","Raumsoll Urlaub (C)",NUMBER),F("unsynced_room","Raumsoll ohne Zeit (C)",NUMBER)};
static const spec_t curve[]={F("curve_zero","Vorlauf bei 0 C aussen",NUMBER),F("curve_slope","Kurvensteilheit",NUMBER),F("room_gain","Raumkorrektur (K/K)",NUMBER),F("summer_temp","Sommerabschaltung (C)",NUMBER),F("flow_min","Vorlauf Minimum (C)",NUMBER),F("flow_max","Vorlauf Maximum (C)",NUMBER)};
static const spec_t boiler[]={F("boiler_target","Boiler-Soll (C)",NUMBER),F("boiler_hysteresis","Ladehysterese (K)",NUMBER),F("dhw_lift","Kesselanhebung (K)",NUMBER),F("dhw_priority","Warmwasser-Vorrang",BOOL)};
static const spec_t kessel[]={F("kessel_max","Kessel Maximum (C)",NUMBER),F("kessel_margin","Kesselreserve (K)",NUMBER),F("burner_hysteresis","Brennerhysterese (K)",NUMBER),F("burner_min_on_s","Brenner Mindestlaufzeit (s)",NUMBER),F("burner_min_off_s","Brenner Mindeststillstand (s)",NUMBER),F("pump_overrun_s","Pumpennachlauf (s)",NUMBER)};
static const spec_t mixer[]={F("mixer_travel_s","Vollstaendige Laufzeit (s)",NUMBER),F("mixer_dead_ms","Richtungswechselpause (ms)",NUMBER),F("mixer_band","Regelband (K)",NUMBER),F("frost_outside","Frostgrenze aussen (C)",NUMBER),F("frost_water","Frostgrenze Wasser (C)",NUMBER),F("frost_flow","Frost-Vorlaufsoll (C)",NUMBER),F("sensor_stale_s","Max. Sensoralter (s)",NUMBER),F("buffer_enabled","Pufferfuehler auswerten",BOOL)};
static const spec_t rest[]={F("enabled","Restwaerme nutzen",BOOL),F("stop_c","Kessel-Abschalttemperatur (C)",NUMBER),F("max_s","Maximale Laufzeit (s)",NUMBER)};
static const spec_t mqtt[]={F("uri","Broker-Adresse mit Port",TEXT),F("base_topic","MQTT-Basistopic",TEXT),F("username","MQTT-Benutzer",TEXT),F("password","Neues MQTT-Passwort (leer = unveraendert)",SECRET),F("clear_password","Gespeichertes MQTT-Passwort loeschen",BOOL)};
static const spec_t ntp[]={F("ntp_primary","Zeitserver 1",TEXT),F("ntp_secondary","Zeitserver 2",TEXT),F("ntp_tertiary","Zeitserver 3",TEXT)};
static const spec_t stove[]={F("enabled","Kachelofen-Umschaltung aktiv",BOOL),F("topic","Empfangstopic",TEXT),F("on_c","Aktiv ab (C)",NUMBER),F("off_c","Inaktiv bis (C)",NUMBER),F("stale_s","Max. Empfangsalter (s)",NUMBER)};
static const spec_t gas[]={F("enabled","Gaszaehler aktiv",BOOL),F("topic","Empfangstopic",TEXT),F("stale_s","Aktualitaetshinweis nach (s)",NUMBER)};
static const spec_t basis[]={F("baseline_m3","Gesamtstand bei Tagesbeginn (m3)",NUMBER)};
static const spec_t account[]={F("username","Neuer Web-Benutzer",TEXT),F("password","Neues Web-Kennwort",SECRET),F("repeat","Kennwort wiederholen",SECRET)};
static const spec_t pin[]={F("pin","Neuer Displaycode (4-8 Ziffern)",SECRET),F("repeat","Displaycode wiederholen",SECRET)};
static field_t fields[MAX_FIELDS];static unsigned field_count;
static section_t current;
static lv_obj_t *page,*body,*message,*selector,*login_button,*save_button,*modal,*editor,*keyboard;
static int editing=-1;static bool busy,loaded,was_unlocked;
static QueueHandle_t jobs,results;
typedef enum {JOB_LOGIN,JOB_LOAD,JOB_SAVE,JOB_PIN} job_kind_t;
typedef struct {job_kind_t kind;unsigned epoch;section_t section;char *payload;} job_t;
typedef struct {job_kind_t kind;unsigned epoch;section_t section;bool ok;char *text;} result_t;
static void clear_fields(void){for(unsigned i=0;i<field_count;i++)mbedtls_platform_zeroize(fields[i].text,sizeof(fields[i].text));field_count=0;}
static void say(const char *text){lv_label_set_text(message,text?text:"");}
static void close_editor(void){if(editor)lv_textarea_set_text(editor,"");if(modal)lv_obj_delete(modal);modal=editor=keyboard=NULL;editing=-1;}
static void request_job(job_kind_t kind,const char *payload){
 if(!jobs||!results){say("Einstellungsbedienung nicht verfuegbar.");return;}
 if(busy){say("Bitte laufenden Vorgang abwarten.");return;}
 job_t *j=calloc(1,sizeof(*j));if(!j){say("Nicht genug Speicher.");return;}
 j->kind=kind;j->epoch=touch_access_epoch();j->section=current;j->payload=strdup(payload?payload:"");
 if(!j->payload||xQueueSend(jobs,&j,0)!=pdPASS){free(j->payload);free(j);say("Vorgang konnte nicht gestartet werden.");return;}
 busy=true;say("Bitte warten ...");
}
static char *api_request(const char *route,const char *payload,int *status){
 char token[65]={0},csrf[65]={0},url[96];*status=0;
 if(!touch_access_token(token,sizeof(token)))return strdup("{\"message\":\"Bitte am Display entsperren.\"}");
 if(payload){int code;char *state=api_request("/api/status",NULL,&code);cJSON *j=state?cJSON_Parse(state):NULL;free(state);cJSON *v=cJSON_GetObjectItemCaseSensitive(j,"csrf");if(cJSON_IsString(v))snprintf(csrf,sizeof(csrf),"%s",v->valuestring);cJSON_Delete(j);if(code!=200||!csrf[0])return strdup("{\"message\":\"Statusabfrage fehlgeschlagen.\"}");}
 snprintf(url,sizeof(url),"http://127.0.0.1%s",route);
 esp_http_client_config_t config={.url=url,.timeout_ms=15000,.disable_auto_redirect=true,.buffer_size=1024};
 esp_http_client_handle_t h=esp_http_client_init(&config);if(!h)return NULL;
 esp_http_client_set_header(h,"X-Touch-Session",token);mbedtls_platform_zeroize(token,sizeof(token));
 size_t length=payload?strlen(payload):0;esp_http_client_set_method(h,payload?HTTP_METHOD_POST:HTTP_METHOD_GET);
 if(payload){esp_http_client_set_header(h,"X-CSRF-Token",csrf);esp_http_client_set_header(h,"Content-Type","application/json");}
 esp_err_t e=esp_http_client_open(h,(int)length);size_t sent=0;
 while(e==ESP_OK&&sent<length){int n=esp_http_client_write(h,payload+sent,(int)(length-sent));if(n<=0)e=ESP_FAIL;else sent+=(size_t)n;}
 char *response=NULL;
 if(e==ESP_OK&&esp_http_client_fetch_headers(h)>=0){*status=esp_http_client_get_status_code(h);response=calloc(1,API_LIMIT+1);if(response){int got=esp_http_client_read_response(h,response,API_LIMIT);if(got<0||got>=API_LIMIT){free(response);response=NULL;}}}
 esp_http_client_close(h);esp_http_client_cleanup(h);
 if(!response)response=strdup("{\"message\":\"Lokale Verbindung unterbrochen. Neustart pruefen.\"}");
 return response;
}
static void worker(void *unused){
 (void)unused;
 for(;;){job_t *j; xQueueReceive(jobs,&j,portMAX_DELAY);result_t result={0};result_t *r=&result;r->kind=j->kind;r->section=j->section;r->epoch=j->epoch;
  if(j->kind==JOB_LOGIN){r->ok=touch_access_unlock(j->payload,j->epoch);r->text=strdup(r->ok?"Display entsperrt.":"Code falsch, Sitzung abgelaufen oder zu schnell. Zwei Sekunden warten.");}
  else if(!touch_access_unlocked()||j->epoch!=touch_access_epoch())r->text=strdup("Bitte erneut mit Displaycode entsperren.");
  else if(j->kind==JOB_PIN){r->ok=touch_access_change(j->payload)==ESP_OK;r->text=strdup(r->ok?"Displaycode gespeichert. Bitte neu entsperren.":"Code muss aus 4 bis 8 Ziffern bestehen.");r->epoch=touch_access_epoch();}
  else if(j->kind==JOB_LOAD||j->kind==JOB_SAVE){int status;r->text=api_request(routes[j->section],j->kind==JOB_SAVE?j->payload:NULL,&status);r->ok=status==200;}
  if(!r->text)r->text=strdup("Vorgang fehlgeschlagen.");
  mbedtls_platform_zeroize(j->payload,strlen(j->payload));free(j->payload);free(j);
  xQueueSend(results,r,portMAX_DELAY);
 }
}
static bool restarts(void){return current==REG_MQTT||current==REG_NTP||current==REG_STOVE||current==REG_GAS||current==REG_BASIS;}
static void draw_section(cJSON *json);
static void submit_editor(lv_event_t *e){
 (void)e;if(!editor)return;const char *value=lv_textarea_get_text(editor);
 if(editing==-2){request_job(JOB_LOGIN,value);close_editor();return;}
 if(editing>=0&&(unsigned)editing<field_count&&touch_access_unlocked()){
  field_t *f=&fields[editing];snprintf(f->text,sizeof(f->text),"%s",value);lv_label_set_text(f->value,f->spec.type==SECRET?(f->text[0]?"********":"(leer)"):f->text);
 }close_editor();
}
static void cancel_editor(lv_event_t *e){(void)e;close_editor();}
static lv_obj_t *button(lv_obj_t *parent,const char *text,int x,int y,int w,lv_event_cb_t cb,void *data){lv_obj_t *b=lv_button_create(parent);lv_obj_set_pos(b,x,y);lv_obj_set_size(b,w,48);lv_obj_t *l=lv_label_create(b);lv_label_set_text(l,text);lv_obj_center(l);lv_obj_add_event_cb(b,cb,LV_EVENT_CLICKED,data);return b;}
static void open_editor(int index){
 if(modal)return;
 editing=index;
 bool login=index==-2;field_t *f=login?NULL:&fields[index];
 modal=lv_obj_create(lv_layer_top());lv_obj_set_pos(modal,0,0);lv_obj_set_size(modal,1024,600);lv_obj_remove_flag(modal,LV_OBJ_FLAG_SCROLLABLE);lv_obj_set_style_bg_color(modal,lv_color_hex(0xf4f8f9),0);
 lv_obj_t *title=lv_label_create(modal);lv_label_set_text(title,login?"Displaycode eingeben":f->spec.label);lv_obj_set_pos(title,10,5);
 editor=lv_textarea_create(modal);lv_obj_set_pos(editor,10,40);lv_obj_set_size(editor,970,65);lv_textarea_set_one_line(editor,true);lv_textarea_set_max_length(editor,login?8:f->spec.type==SECRET?127:191);lv_textarea_set_password_mode(editor,login||f->spec.type==SECRET);
 if(!login)lv_textarea_set_text(editor,f->text);
 keyboard=lv_keyboard_create(modal);lv_obj_set_pos(keyboard,0,200);lv_obj_set_size(keyboard,990,365);lv_keyboard_set_textarea(keyboard,editor);
 bool numeric=login||(!login&&(f->spec.type==NUMBER||current==REG_PIN));
 if(numeric)lv_keyboard_set_mode(keyboard,LV_KEYBOARD_MODE_NUMBER);
 if(login||current==REG_PIN)lv_textarea_set_accepted_chars(editor,"0123456789");
 else if(f->spec.type==NUMBER)lv_textarea_set_accepted_chars(editor,"0123456789.-");
 else if(f->spec.type==CLOCK)lv_textarea_set_accepted_chars(editor,"0123456789:");
 button(modal,"Uebernehmen",10,120,230,submit_editor,NULL);button(modal,"Abbrechen",270,120,230,cancel_editor,NULL);
 lv_obj_add_event_cb(keyboard,submit_editor,LV_EVENT_READY,NULL);lv_obj_add_event_cb(keyboard,cancel_editor,LV_EVENT_CANCEL,NULL);
}
static void edit_field(lv_event_t *e){if(busy){say("Bitte laufenden Vorgang abwarten.");return;}if(!touch_access_unlocked()){say("Bitte zuerst Display entsperren.");return;}unsigned i=(unsigned)(uintptr_t)lv_event_get_user_data(e);if(i>=field_count)return;
 if(fields[i].spec.type==BOOL){fields[i].checked=!fields[i].checked;lv_label_set_text(fields[i].value,fields[i].checked?"Ein":"Aus");}else open_editor((int)i);
}
static void login_click(lv_event_t *e){(void)e;if(touch_access_unlocked()){touch_settings_lock();return;}open_editor(-2);}
static void field(spec_t spec,cJSON *value){
 if(field_count>=MAX_FIELDS)return;
 unsigned i=field_count++;field_t *f=&fields[i];f->spec=spec;
 if(spec.type==BOOL)f->checked=cJSON_IsTrue(value);
 else if(cJSON_IsString(value))snprintf(f->text,sizeof(f->text),"%s",value->valuestring);
 else if(cJSON_IsNumber(value)){snprintf(f->text,sizeof(f->text),"%.3f",value->valuedouble);char *end=f->text+strlen(f->text)-1;while(end>f->text&&*end=='0')*end--=0;if(*end=='.')*end=0;}
 lv_obj_t *row=lv_obj_create(body);lv_obj_set_pos(row,0,(int)i*76);lv_obj_set_size(row,940,70);lv_obj_remove_flag(row,LV_OBJ_FLAG_SCROLLABLE);
 lv_obj_t *name=lv_label_create(row);lv_label_set_text(name,spec.label);lv_obj_set_pos(name,0,8);lv_obj_set_width(name,430);
 f->button=button(row,"",445,-5,455,edit_field,(void*)(uintptr_t)i);f->value=lv_obj_get_child(f->button,0);lv_label_set_long_mode(f->value,LV_LABEL_LONG_DOT);lv_obj_set_width(f->value,415);
 lv_label_set_text(f->value,spec.type==BOOL?(f->checked?"Ein":"Aus"):spec.type==SECRET?"(leer)":f->text);
}
static const char *json_text(cJSON *j,const char *key){cJSON *v=cJSON_GetObjectItemCaseSensitive(j,key);return cJSON_IsString(v)?v->valuestring:"--";}
static double json_number(cJSON *j,const char *key){cJSON *v=cJSON_GetObjectItemCaseSensitive(j,key);return cJSON_IsNumber(v)?v->valuedouble:0;}
static bool json_bool(cJSON *j,const char *key){return cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(j,key));}
static void append_text(char *out,size_t cap,const char *format,...){size_t n=strlen(out);if(n>=cap)return;va_list args;va_start(args,format);vsnprintf(out+n,cap-n,format,args);va_end(args);}
static void system_view(cJSON *j){
 char *out=calloc(1,7500);if(!out)return;
 append_text(out,7500,"Ethernet-IP: %s\nEthernet: %s   MQTT: %s\nZeit: %s\nNTP: %s   Letzte Synchronisation vor %.0f s\nLaufzeit: %.0f Minuten\nFreier Speicher: %.0f KiB   Minimum: %.0f KiB\nFirmware: %s   Speicherplatz: %s\nTeststart: %s   Regler-Timeout: %s\n\n",json_text(j,"ip"),json_bool(j,"ethernet")?"verbunden":"getrennt",json_bool(j,"mqtt")?"verbunden":"getrennt",json_text(j,"clock"),json_bool(j,"time_synced")?"synchronisiert":"warte",json_number(j,"ntp_sync_age_s"),json_number(j,"uptime_s")/60,json_number(j,"heap_free")/1024,json_number(j,"heap_min")/1024,json_text(j,"firmware_build"),json_text(j,"firmware_slot"),json_bool(j,"ota_pending")?"laeuft":"abgeschlossen",json_bool(j,"output_timeout")?"ja":"nein");
 cJSON *d=cJSON_GetObjectItemCaseSensitive(j,"display_settings"),*st=cJSON_GetObjectItemCaseSensitive(d,"stove"),*g=cJSON_GetObjectItemCaseSensitive(d,"gas");
 append_text(out,7500,"MQTT-Basistopic: %s\nStatus: %s/status\nDiagnose: %s/diagnostics\nAlle Anlagenthemen: %s/#\nKachelofen-Topic: %s\nGas-Topic: %s\nZeitserver 1: %s\nZeitserver 2: %s\nZeitserver 3: %s\n\n",json_text(d,"base_topic"),json_text(d,"base_topic"),json_text(d,"base_topic"),json_text(d,"base_topic"),json_text(st,"topic"),json_text(g,"topic"),json_text(d,"ntp_primary"),json_text(d,"ntp_secondary"),json_text(d,"ntp_tertiary"));
 static const char *names[]={"Aussen","Raum","Kessel","Vorlauf","Boiler","Puffer oben","Puffer Mitte","Puffer unten"};cJSON *sensor;unsigned i=0;
 cJSON_ArrayForEach(sensor,cJSON_GetObjectItemCaseSensitive(j,"sensors")){
  if(i>=8)break;
  append_text(out,7500,"%s - GPIO %.0f: %s\n",names[i++],json_number(sensor,"gpio"),json_text(sensor,"diagnosis"));
  if(json_bool(sensor,"valid"))append_text(out,7500,"Temperatur: %.2f C\n",json_number(sensor,"temperature"));
  append_text(out,7500,"Versuche %.0f / erfolgreich %.0f / Fehler %.0f\nROM: %s\nRohdaten: %s\n\n",json_number(sensor,"attempts"),json_number(sensor,"successes"),json_number(sensor,"failures"),json_text(sensor,"rom"),json_text(sensor,"scratchpad"));
 }
 lv_obj_t *label=lv_label_create(body);lv_obj_set_width(label,910);lv_label_set_text(label,out);free(out);
}
static void draw_section(cJSON *json){
 clear_fields();lv_obj_clean(body);loaded=true;
 lv_label_set_text(lv_obj_get_child(save_button,0),restarts()?"Speichern & Neustart":"Speichern");
 const spec_t *spec=NULL;unsigned count=0;
 #define USE(a) do{spec=a;count=sizeof(a)/sizeof(a[0]);}while(0)
 switch(current){case REG_ROOM:USE(room);break;case REG_CURVE:USE(curve);break;case REG_BOILER:USE(boiler);break;case REG_KESSEL:USE(kessel);break;case REG_MIXER:USE(mixer);break;case REG_REST:USE(rest);break;case REG_MQTT:USE(mqtt);break;case REG_NTP:USE(ntp);break;case REG_STOVE:USE(stove);break;case REG_GAS:USE(gas);break;case REG_BASIS:USE(basis);break;case REG_ACCOUNT:USE(account);break;case REG_PIN:USE(pin);break;case REG_FILES:break;default:break;}
 for(unsigned i=0;i<count;i++)field(spec[i],cJSON_GetObjectItemCaseSensitive(json,spec[i].key));
 if(current==REG_TIME){cJSON *periods=cJSON_GetObjectItemCaseSensitive(json,"periods");static const char *start[]={"Periode 1 Start (HH:MM)","Periode 2 Start (HH:MM)","Periode 3 Start (HH:MM)","Periode 4 Start (HH:MM)"};static const char *target[]={"Periode 1 Raumsoll (C)","Periode 2 Raumsoll (C)","Periode 3 Raumsoll (C)","Periode 4 Raumsoll (C)"};
  for(int i=0;i<4;i++){cJSON *p=cJSON_GetArrayItem(periods,i),*minute=cJSON_GetObjectItemCaseSensitive(p,"start_minute");char clock[16];int m=cJSON_IsNumber(minute)&&minute->valueint>=0&&minute->valueint<1440?minute->valueint:0;snprintf(clock,sizeof(clock),"%02d:%02d",m/60,m%60);cJSON *v=cJSON_CreateString(clock);field((spec_t){"start",start[i],CLOCK},v);cJSON_Delete(v);field((spec_t){"target",target[i],NUMBER},cJSON_GetObjectItemCaseSensitive(p,"room_target"));}}
 if(current==REG_FILES){lv_obj_t *hint=lv_label_create(body);lv_obj_set_width(hint,910);lv_label_set_text(hint,"Sicherungsdateien herunterladen und einspielen:\nBitte die vorhandene Weboberflaeche verwenden.\n\nFirmwaredateien ueber Webseite oder USB installieren.\nEs wird keine SD-Karte benoetigt oder angesprochen.\n\nAlle Einstellwerte sind in den anderen Displaybereichen bearbeitbar.");}
 if(current==REG_SYSTEM)system_view(json);
 if(current==REG_FILES||current==REG_SYSTEM)lv_obj_add_flag(save_button,LV_OBJ_FLAG_HIDDEN);else lv_obj_remove_flag(save_button,LV_OBJ_FLAG_HIDDEN);
 lv_obj_scroll_to_y(body,0,LV_ANIM_OFF);
}
static void load_section(void){
 clear_fields();lv_obj_clean(body);loaded=false;
 if(!touch_access_unlocked()){say("Mit Displaycode entsperren (anfangs 1234).");return;}
 if(current==REG_ACCOUNT||current==REG_PIN||current==REG_FILES||current==REG_BASIS){draw_section(NULL);say(current==REG_PIN?"Vier bis acht Ziffern. Neuer Code gilt nach Speichern.":current==REG_FILES?"Dateiuebertragung bleibt auf Webseite und USB.":"Werte eingeben und speichern.");}
 else request_job(JOB_LOAD,NULL);
}
static void select_section(lv_event_t *e){(void)e;if(busy){lv_dropdown_set_selected(selector,current);return;}current=(section_t)lv_dropdown_get_selected(selector);load_section();}
static bool number(const char *text,double *out){char *end;errno=0;if(!text[0])return false;double n=strtod(text,&end);if(errno||*end||!isfinite(n))return false;*out=n;return true;}
static void confirm_save(lv_event_t *e){
 (void)e;if(!touch_access_unlocked()||!loaded){say("Bitte entsperren und Bereich laden.");return;}
 if(current==REG_PIN){if(strcmp(fields[0].text,fields[1].text)){say("Codes stimmen nicht ueberein.");return;}request_job(JOB_PIN,fields[0].text);return;}
 cJSON *j=cJSON_CreateObject();bool ok=j!=NULL;
 if(current==REG_TIME){cJSON *a=cJSON_AddArrayToObject(j,"periods");for(unsigned i=0;i<4&&ok;i++){int h,m;char tail;double n;if(sscanf(fields[i*2].text,"%d:%d%c",&h,&m,&tail)!=2||h<0||h>23||m<0||m>59||!number(fields[i*2+1].text,&n)){ok=false;break;}cJSON *p=cJSON_CreateObject();cJSON_AddNumberToObject(p,"start_minute",h*60+m);cJSON_AddNumberToObject(p,"room_target",n);cJSON_AddItemToArray(a,p);}}
 else for(unsigned i=0;i<field_count&&ok;i++){field_t *f=&fields[i];if(!strcmp(f->spec.key,"repeat")||!strcmp(f->spec.key,"clear_password"))continue;
  if(f->spec.type==NUMBER){double n;if(!number(f->text,&n)){ok=false;break;}cJSON_AddNumberToObject(j,f->spec.key,n);}
  else if(f->spec.type==BOOL)cJSON_AddBoolToObject(j,f->spec.key,f->checked);
  else if(current==REG_MQTT&&!strcmp(f->spec.key,"password")){if(fields[4].checked)cJSON_AddStringToObject(j,"password","");else if(f->text[0])cJSON_AddStringToObject(j,"password",f->text);}
  else cJSON_AddStringToObject(j,f->spec.key,f->text);
 }
 if(current==REG_ACCOUNT&&strcmp(fields[1].text,fields[2].text))ok=false;
 if(!ok){cJSON_Delete(j);say("Eingabe ungueltig oder Wiederholung stimmt nicht ueberein.");return;}
 char *text=cJSON_PrintUnformatted(j);cJSON_Delete(j);if(text){request_job(JOB_SAVE,text);mbedtls_platform_zeroize(text,strlen(text));free(text);}else say("Nicht genug Speicher.");
}
void touch_settings_lock(void){touch_access_lock();close_editor();clear_fields();if(body)lv_obj_clean(body);loaded=false;if(message)say("Display gesperrt. Zahlencode zum Bearbeiten eingeben.");}
void touch_settings_poll(void){
 if(!results)return;
 bool u=touch_access_unlocked();bool changed=u!=was_unlocked;if(was_unlocked&&!u){close_editor();clear_fields();lv_obj_clean(body);loaded=false;}was_unlocked=u;
 if(changed)lv_label_set_text(lv_obj_get_child(login_button,0),u?"Sperren":"Code eingeben");
 if(!u||busy||!loaded)lv_obj_add_state(save_button,LV_STATE_DISABLED);else lv_obj_remove_state(save_button,LV_STATE_DISABLED);
 if(busy)lv_obj_add_state(selector,LV_STATE_DISABLED);else lv_obj_remove_state(selector,LV_STATE_DISABLED);
 result_t value;result_t *r=&value;if(xQueueReceive(results,r,0)!=pdPASS)return;busy=false;
 if(r->epoch==touch_access_epoch()){
  cJSON *j=r->text?cJSON_Parse(r->text):NULL;cJSON *m=cJSON_GetObjectItemCaseSensitive(j,"message");
  if(r->kind==JOB_LOGIN&&r->ok){load_section();}
  else if(r->kind==JOB_LOAD&&r->ok&&r->section==current){draw_section(j);say(restarts()?"Speichern uebernimmt die Daten und startet in 3 Sekunden neu.":current==REG_REST?"30 C sind der gewaehlte Startwert, keine bestaetigte Herstellergrenze.":"Werte geladen. Speichern uebernimmt gepruefte Einstellungen.");}
  else say(cJSON_IsString(m)?m->valuestring:r->text?r->text:"Zu wenig freier Speicher.");
  if(r->kind==JOB_SAVE&&r->ok){for(unsigned i=0;i<field_count;i++)if(fields[i].spec.type==SECRET){mbedtls_platform_zeroize(fields[i].text,sizeof(fields[i].text));lv_label_set_text(fields[i].value,"(leer)");}}
  cJSON_Delete(j);
 }
 if(r->text){mbedtls_platform_zeroize(r->text,strlen(r->text));free(r->text);}
}
void touch_settings_create(lv_obj_t *parent){
 page=parent;lv_obj_set_style_text_font(page,&lv_font_montserrat_20,0);
 selector=lv_dropdown_create(page);lv_obj_set_pos(selector,0,0);lv_obj_set_size(selector,430,50);lv_dropdown_clear_options(selector);for(int i=0;i<REG_COUNT;i++)lv_dropdown_add_option(selector,sections[i],LV_DROPDOWN_POS_LAST);lv_obj_add_event_cb(selector,select_section,LV_EVENT_VALUE_CHANGED,NULL);
 login_button=button(page,"Code eingeben",450,0,230,login_click,NULL);save_button=button(page,"Speichern",700,0,260,confirm_save,NULL);
 body=lv_obj_create(page);lv_obj_set_pos(body,0,65);lv_obj_set_size(body,970,330);lv_obj_set_scroll_dir(body,LV_DIR_VER);lv_obj_set_style_pad_all(body,8,0);
 message=lv_label_create(page);lv_obj_set_pos(message,0,405);lv_obj_set_width(message,960);lv_label_set_long_mode(message,LV_LABEL_LONG_WRAP);say("Displaycode eingeben (anfangs 1234). Web-Anmeldung bleibt getrennt.");
 jobs=xQueueCreate(2,sizeof(job_t*));results=xQueueCreate(2,sizeof(result_t));
 if(!jobs||!results||xTaskCreate(worker,"touch_settings",12288,NULL,1,NULL)!=pdPASS){say("Einstellungsbedienung konnte nicht starten.");lv_obj_add_state(login_button,LV_STATE_DISABLED);lv_obj_add_state(save_button,LV_STATE_DISABLED);}
}
