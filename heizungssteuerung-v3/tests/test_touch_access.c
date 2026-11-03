#include "touch_access.h"
#include "nvs.h"
#include "lwip/sockets.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <openssl/evp.h>
#include <openssl/rand.h>
#include <openssl/crypto.h>
static unsigned char stored[128],pending[128];static size_t stored_n,pending_n;static bool fail_commit;static int64_t now_ms=1000;
esp_err_t nvs_open(const char *name,int mode,nvs_handle_t *h){assert(!strcmp(name,"touch_access"));*h=1;return mode==NVS_READONLY&&!stored_n?ESP_ERR_NVS_NOT_FOUND:ESP_OK;}
esp_err_t nvs_get_blob(nvs_handle_t h,const char *key,void *p,size_t *n){(void)h;assert(!strcmp(key,"pin"));if(!stored_n)return ESP_ERR_NVS_NOT_FOUND;if(*n<stored_n)return ESP_FAIL;memcpy(p,stored,stored_n);*n=stored_n;return ESP_OK;}
esp_err_t nvs_set_blob(nvs_handle_t h,const char *key,const void *p,size_t n){(void)h;assert(!strcmp(key,"pin")&&n<=sizeof(pending));memcpy(pending,p,n);pending_n=n;return ESP_OK;}
esp_err_t nvs_commit(nvs_handle_t h){(void)h;if(fail_commit)return ESP_FAIL;memcpy(stored,pending,pending_n);stored_n=pending_n;return ESP_OK;}
void nvs_close(nvs_handle_t h){(void)h;pending_n=0;}
void esp_fill_random(void *p,size_t n){assert(RAND_bytes(p,(int)n)==1);}
int64_t esp_timer_get_time(void){return now_ms*1000;}
void mbedtls_platform_zeroize(void *p,size_t n){OPENSSL_cleanse(p,n);}
int mbedtls_pkcs5_pbkdf2_hmac_ext(int md,const unsigned char *p,size_t n,const unsigned char *salt,size_t sn,unsigned rounds,unsigned len,unsigned char *out){(void)md;return PKCS5_PBKDF2_HMAC((const char*)p,(int)n,salt,(int)sn,(int)rounds,EVP_sha256(),(int)len,out)?0:-1;}
int httpd_req_to_sockfd(httpd_req_t *r){return r->fd;}
esp_err_t httpd_req_get_hdr_value_str(httpd_req_t *r,const char *key,char *out,size_t n){assert(!strcmp(key,"X-Touch-Session"));if(!r->token||strlen(r->token)>=n)return ESP_FAIL;strcpy(out,r->token);return ESP_OK;}
int test_getpeername(int fd,struct sockaddr *peer,socklen_t *n){
 if(fd>=3){struct sockaddr_in6 v={.sin6_family=AF_INET6};unsigned char *a=(unsigned char*)&v.sin6_addr;
  if(fd==5)a[15]=1;else{a[10]=255;a[11]=255;a[12]=fd==3?127:192;a[13]=fd==3?0:168;a[14]=0;a[15]=fd==3?1:50;}
  assert(*n>=sizeof(v));memcpy(peer,&v,sizeof(v));*n=sizeof(v);return 0;
 }
 struct sockaddr_in v={.sin_family=AF_INET,.sin_addr.s_addr=htonl(fd==1?INADDR_LOOPBACK:0xc0a80032)};assert(*n>=sizeof(v));memcpy(peer,&v,sizeof(v));*n=sizeof(v);return 0;
}
int main(void){
 assert(touch_access_init()==ESP_OK);assert(!touch_access_unlocked());char token[65];assert(!touch_access_token(token,sizeof(token)));
 unsigned epoch=touch_access_epoch();assert(!touch_access_unlock("0000",epoch));assert(!touch_access_unlock("1234",epoch));now_ms+=2000;
 assert(touch_access_unlock("1234",epoch));assert(touch_access_token(token,sizeof(token)));
 httpd_req_t local={1,token},remote={2,token},missing={1,NULL},bad={1,"bad"};
 assert(touch_access_authorized(&local));httpd_req_t mapped={3,token},mapped_remote={4,token},ipv6={5,token};assert(touch_access_authorized(&mapped));assert(!touch_access_authorized(&mapped_remote));assert(touch_access_authorized(&ipv6));assert(!touch_access_authorized(&remote));assert(!touch_access_authorized(&missing));assert(!touch_access_authorized(&bad));
 touch_access_lock();assert(!touch_access_authorized(&local));now_ms+=2000;assert(!touch_access_unlock("1234",epoch));
 epoch=touch_access_epoch();assert(touch_access_unlock("1234",epoch));assert(touch_access_change("12a4")==ESP_ERR_INVALID_ARG);
 fail_commit=true;assert(touch_access_change("5678")==ESP_FAIL);fail_commit=false;touch_access_lock();now_ms+=2000;assert(touch_access_unlock("1234",touch_access_epoch()));
 assert(touch_access_change("5678")==ESP_OK);assert(!touch_access_unlocked());now_ms+=2000;assert(!touch_access_unlock("1234",touch_access_epoch()));now_ms+=2000;assert(touch_access_unlock("5678",touch_access_epoch()));
 assert(touch_access_init()==ESP_OK);assert(!touch_access_unlocked());assert(touch_access_unlock("5678",touch_access_epoch()));
 char rotated[65];assert(touch_access_token(rotated,sizeof(rotated)));assert(strcmp(token,rotated));assert(!touch_access_authorized(&local));
 assert(touch_access_change("12345678")==ESP_OK);now_ms+=2000;assert(touch_access_unlock("12345678",touch_access_epoch()));
 assert(touch_access_change("123456789")==ESP_ERR_INVALID_ARG);
 puts("PASS: Display-PIN, Zeitbegrenzung, Sitzungssperre, veraltete Auftraege, NVS-Fehler/Persistenz und Ablehnung externer Requests trotz gueltigem Token.");
}
