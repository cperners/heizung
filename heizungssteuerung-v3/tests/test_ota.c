#include "ota_rules.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
static void word(unsigned char *p,unsigned n){p[0]=n&255;p[1]=n>>8;}
int main(void){
 unsigned char b[288]={0};b[0]=0xe9;b[1]=7;word(b+12,0x12);word(b+15,301);word(b+17,399);b[23]=1;b[32]=0x32;b[33]=0x54;b[34]=0xcd;b[35]=0xab;strcpy((char*)b+80,"heizungssteuerung");
 assert(ota_prefix_valid(b,sizeof(b),302,"heizungssteuerung"));
 assert(!ota_prefix_valid(b,287,302,"heizungssteuerung"));
 assert(!ota_prefix_valid(b,sizeof(b),100,"heizungssteuerung"));
 assert(!ota_prefix_valid(b,sizeof(b),400,"heizungssteuerung"));
 assert(!ota_prefix_valid(b,sizeof(b),302,"anderesprojekt"));
 b[0]=0;assert(!ota_prefix_valid(b,sizeof(b),302,"heizungssteuerung"));b[0]=0xe9;
 b[12]=9;assert(!ota_prefix_valid(b,sizeof(b),302,"heizungssteuerung"));b[12]=0x12;
 b[23]=0;assert(!ota_prefix_valid(b,sizeof(b),302,"heizungssteuerung"));b[23]=1;
 b[32]=0;assert(!ota_prefix_valid(b,sizeof(b),302,"heizungssteuerung"));b[32]=0x32;
 memset(b+80,'x',32);assert(!ota_prefix_valid(b,sizeof(b),302,"heizungssteuerung"));
 puts("PASS: OTA-Dateikopf, Chip, Revision, Projektname, SHA-Kennzeichen und abgeschnittene Datei.");
}
