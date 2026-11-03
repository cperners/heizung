/* Exact production stage machine; GPIO read samples scripted, no claim about
   physical timing. Hardware measurements remain necessary. */
#include "../main/sensors.c"
#include <assert.h>
#include <stdio.h>
static int samples[512];static unsigned used,count;
int gpio_get_level(int p){assert(p==2&&used<count);return samples[used++];}
esp_err_t gpio_set_level(int p,int level){(void)p;(void)level;return ESP_OK;}
esp_err_t gpio_config(const gpio_config_t *g){(void)g;return ESP_OK;}
void esp_rom_delay_us(unsigned us){(void)us;}
int64_t esp_timer_get_time(void){return 0;}
void config_get(heating_config_t *c){memset(c,0,sizeof(*c));}
void vTaskDelay(unsigned ticks){(void)ticks;}
int xTaskCreate(void(*fn)(void*),const char *s,unsigned n,void *a,unsigned p,void *h){(void)fn;(void)s;(void)n;(void)a;(void)p;(void)h;return pdPASS;}
static void sample(int n){samples[count++]=n;}
static void pulse(void){sample(1);sample(0);sample(0);sample(1);}
static void bytes(const uint8_t *p,unsigned n){for(unsigned i=0;i<n;i++)for(int k=0;k<8;k++)sample((p[i]>>k)&1);}
static void clear(sensor_diagnostic_t *d){memset(d,0,sizeof(*d));count=used=0;}
int main(void){
 sensor_diagnostic_t d;float temperature;
 clear(&d);sample(0);assert(!reset(2,&d)&&d.code==SENSOR_DIAG_IDLE_LOW);
 clear(&d);sample(1);sample(1);assert(!reset(2,&d)&&d.code==SENSOR_DIAG_DRIVE_HIGH);
 clear(&d);sample(1);sample(0);sample(1);sample(1);assert(!reset(2,&d)&&d.code==SENSOR_DIAG_NO_PRESENCE&&!d.presence);
 clear(&d);sample(1);sample(0);sample(0);sample(0);assert(!reset(2,&d)&&d.code==SENSOR_DIAG_RELEASE_LOW);
 uint8_t rom[8]={0x28,1,2,3,4,5,6,0};rom[7]=ds18b20_crc(rom,7);
 clear(&d);pulse();bytes(rom,8);pulse();sample(1);pulse();assert(begin(2,&d)&&d.presence&&d.have_rom&&used==count);
 clear(&d);rom[7]^=1;pulse();bytes(rom,8);assert(!begin(2,&d)&&d.code==SENSOR_DIAG_ROM_CRC);rom[7]^=1;
 clear(&d);rom[0]=0x10;rom[7]=ds18b20_crc(rom,7);pulse();bytes(rom,8);assert(!begin(2,&d)&&d.code==SENSOR_DIAG_FAMILY);rom[0]=0x28;rom[7]=ds18b20_crc(rom,7);
 clear(&d);pulse();bytes(rom,8);pulse();sample(0);assert(!begin(2,&d)&&d.code==SENSOR_DIAG_PARASITE);
 clear(&d);sample(0);assert(!read_result(2,true,&d,&temperature)&&d.code==SENSOR_DIAG_CONVERSION);
 uint8_t scratch[9]={0x98,1,0,0,0x7f,0xff,0x0c,0x10,0};scratch[8]=ds18b20_crc(scratch,8);
 clear(&d);sample(1);pulse();bytes(scratch,9);assert(read_result(2,true,&d,&temperature)&&d.code==SENSOR_DIAG_OK&&temperature==25.5f&&d.have_scratch);
 clear(&d);scratch[8]^=1;sample(1);pulse();bytes(scratch,9);assert(!read_result(2,true,&d,&temperature)&&d.code==SENSOR_DIAG_SCRATCH_CRC);
 clear(&d);scratch[4]=0;scratch[8]=ds18b20_crc(scratch,8);sample(1);pulse();bytes(scratch,9);assert(!read_result(2,true,&d,&temperature)&&d.code==SENSOR_DIAG_SCRATCH_INVALID);
 clear(&d);d.code=SENSOR_DIAG_NO_PRESENCE;assert(!read_result(2,false,&d,&temperature)&&used==0&&d.code==SENSOR_DIAG_NO_PRESENCE);
 /* Per-minute replacement, invalid samples, age filtering and bounded RAM. */
 history_count=history_next=0;
 for(int i=0;i<5;i++){values.value[i].valid=true;values.value[i].celsius=24+i*.1f;}
 record_history(1000);values.value[0].celsius=25;record_history(2000);assert(history_count==1);
 values.value[2].valid=false;record_history(61000);
 cJSON *h=sensors_history_json(61000,3600),*points=cJSON_GetObjectItemCaseSensitive(h,"points");
 assert(cJSON_GetArraySize(points)==2);
 cJSON *last=cJSON_GetArrayItem(points,1),*temps=cJSON_GetObjectItemCaseSensitive(last,"temperatures");
 assert(cJSON_IsNull(cJSON_GetArrayItem(temps,2))&&cJSON_GetArrayItem(temps,0)->valuedouble==25);
 cJSON_Delete(h);h=sensors_history_json(90000000,3600);assert(cJSON_GetArraySize(cJSON_GetObjectItemCaseSensitive(h,"points"))==0);cJSON_Delete(h);
 for(unsigned i=2;i<1450;i++)record_history((int64_t)i*60000+1000);
 assert(history_count==1440);h=sensors_history_json(1449*60000LL+1000,86400);
 assert(cJSON_GetArraySize(cJSON_GetObjectItemCaseSensitive(h,"points"))<=289);
 assert(cJSON_GetObjectItemCaseSensitive(h,"interval_s")->valueint==300);cJSON_Delete(h);
 puts("PASS: produktive Fuehlerdiagnose, Ruhepegel, GPIO-Ansteuerung, Praesenz, ROM, Versorgung, Wandlung und Rohdaten/CRC und begrenzter gemeinsamer Fuehlerverlauf.");
}
