#include "web_auth.h"
#include "sdkconfig.h"
#include "cJSON.h"
#include "nvs.h"
#include "esp_random.h"
#include "mbedtls/base64.h"
#include "mbedtls/pkcs5.h"
#include "mbedtls/platform_util.h"
#include <string.h>
#include <stdio.h>
#include <ctype.h>
/* Nur die Webtask greift hierauf zu. Keine gemeinsamen Sperren mit dem Regler. */
typedef struct {unsigned version;char user[33];unsigned char salt[16],hash[32];} account_t;
static account_t account;
static bool configured;
static bool user_valid(const char *user) {
    size_t len=strlen(user);if(!len || len>32)return false;
    for(const unsigned char *p=(const unsigned char*)user;*p;p++) if(!isalnum(*p)&&*p!='_'&&*p!='-'&&*p!='.')return false;
    return true;
}
static bool hash(const char *password,const unsigned char *salt,unsigned char *out) {
    return mbedtls_pkcs5_pbkdf2_hmac_ext(MBEDTLS_MD_SHA256,(const unsigned char*)password,strlen(password),salt,16,10000,32,out)==0;
}
esp_err_t web_auth_init(void) {
    nvs_handle_t h;esp_err_t e=nvs_open("web_auth",NVS_READONLY,&h);
    if(e==ESP_OK) {
        size_t n=sizeof(account);e=nvs_get_blob(h,"account",&account,&n);nvs_close(h);
        if(e==ESP_OK) {
            if(n!=sizeof(account)||account.version!=1||!memchr(account.user,0,sizeof(account.user))||!user_valid(account.user))return ESP_ERR_INVALID_STATE;
            configured=true;return ESP_OK;
        }
    }
    if(e!=ESP_ERR_NVS_NOT_FOUND)return e;
    const char *password=CONFIG_HEATING_WEB_PASSWORD;
    if(!password[0])return ESP_OK; /* Anzeige ohne Aenderungen. */
    if(strlen(password)<4||strlen(password)>96||!user_valid(CONFIG_HEATING_WEB_USERNAME))return ESP_ERR_INVALID_ARG;
    account.version=1;snprintf(account.user,sizeof(account.user),"%s",CONFIG_HEATING_WEB_USERNAME);
    esp_fill_random(account.salt,sizeof(account.salt));if(!hash(password,account.salt,account.hash))return ESP_FAIL;
    configured=true;return ESP_OK;
}
bool web_auth_configured(void) {return configured;}
bool web_auth_verify(const char *header) {
    if(!configured || strncmp(header,"Basic ",6))return false;
    unsigned char decoded[132]={0},actual[32];size_t n=0;
    if(mbedtls_base64_decode(decoded,sizeof(decoded)-1,&n,(const unsigned char*)header+6,strlen(header+6)) || memchr(decoded,0,n))return false;
    decoded[n]=0;char *password=strchr((char*)decoded,':');
    if(!password){mbedtls_platform_zeroize(decoded,sizeof(decoded));return false;}
    *password++=0;
    bool okay=!strcmp((char*)decoded,account.user)&&strlen(password)<=96&&hash(password,account.salt,actual);
    unsigned char difference=0;if(okay)for(unsigned i=0;i<sizeof(actual);i++)difference|=actual[i]^account.hash[i];
    mbedtls_platform_zeroize(actual,sizeof(actual));mbedtls_platform_zeroize(decoded,sizeof(decoded));return okay&&!difference;
}
esp_err_t web_auth_update(const char *text,char *reason,unsigned size) {
    cJSON *j=cJSON_ParseWithOpts(text,NULL,true);
    cJSON *u=cJSON_GetObjectItemCaseSensitive(j,"username"),*p=cJSON_GetObjectItemCaseSensitive(j,"password");
    bool okay=cJSON_IsObject(j)&&cJSON_GetArraySize(j)==2&&cJSON_IsString(u)&&cJSON_IsString(p);
    if(okay)okay=user_valid(u->valuestring)&&strlen(p->valuestring)>=4&&strlen(p->valuestring)<=96;
    if(!okay){cJSON_Delete(j);snprintf(reason,size,"Benutzer: 1-32 Zeichen (a-z,0-9,._-); Passwort: 4-96 Zeichen");return ESP_ERR_INVALID_ARG;}
    account_t next={.version=1};snprintf(next.user,sizeof(next.user),"%s",u->valuestring);
    esp_fill_random(next.salt,sizeof(next.salt));esp_err_t e=hash(p->valuestring,next.salt,next.hash)?ESP_OK:ESP_FAIL;
    nvs_handle_t h;
    if(e==ESP_OK){e=nvs_open("web_auth",NVS_READWRITE,&h);if(e==ESP_OK){e=nvs_set_blob(h,"account",&next,sizeof(next));if(e==ESP_OK)e=nvs_commit(h);nvs_close(h);}}
    if(e==ESP_OK){account=next;configured=true;}
    mbedtls_platform_zeroize(p->valuestring,strlen(p->valuestring));cJSON_Delete(j);
    snprintf(reason,size,"%s",e==ESP_OK?"Webzugang gespeichert. Bitte neu anmelden.":"Webzugang konnte nicht gespeichert werden");return e;
}
