#include "ds18b20.h"
uint8_t ds18b20_crc(const uint8_t *d,size_t n) {
 uint8_t crc=0;while(n--) {uint8_t b=*d++;for(int i=0;i<8;i++) {uint8_t mix=(crc^b)&1;crc>>=1;if(mix)crc^=0x8c;b>>=1;}}return crc;
}
bool ds18b20_decode(const uint8_t b[9],float *t) {
 /* Bytes 5-7 are internal/reserved, not temperature validity markers.
    Verify all received bytes by CRC, then configuration and temperature. */
 if(ds18b20_crc(b,8)!=b[8]||(b[4]&0x9f)!=0x1f)return false;
 int16_t raw=(int16_t)((uint16_t)b[0]|((uint16_t)b[1]<<8));
 switch((b[4]>>5)&3) {case 0:raw&=~7;break;case 1:raw&=~3;break;case 2:raw&=~1;break;default:break;}
 *t=raw/16.0f;return *t>=-55&&*t<=125;
}
