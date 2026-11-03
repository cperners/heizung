#include "ota_rules.h"
#include <string.h>
static unsigned u16(const uint8_t *p){return p[0]|((unsigned)p[1]<<8);}
bool ota_prefix_valid(const uint8_t *b,size_t n,unsigned rev,const char *project){
 if(n<OTA_PREFIX_SIZE||b[0]!=0xe9||!b[1]||b[1]>16||u16(b+12)!=0x12||b[23]!=1)return false;
 unsigned min=u16(b+15),max=u16(b+17);
 if(rev<min||(max&&max!=65535&&rev>max))return false;
 if(b[32]!=0x32||b[33]!=0x54||b[34]!=0xcd||b[35]!=0xab)return false;
 return memchr(b+80,0,32)&&strlen(project)<32&&!strcmp((const char*)b+80,project);
}
