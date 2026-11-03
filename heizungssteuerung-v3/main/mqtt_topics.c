#include "mqtt_topics.h"
#include <string.h>
#include <stdio.h>
bool mqtt_topic_under(const char *base,const char *topic){size_t n=strlen(base);return n&&!strncmp(base,topic,n)&&(topic[n]=='/'||!topic[n]);}
bool mqtt_base_valid(const char *s){
 size_t n=strlen(s);if(!n||n>=MQTT_BASE_SIZE||s[0]=='/'||s[0]=='$'||s[n-1]=='/'||strstr(s,"//"))return false;
 for(const unsigned char *p=(const unsigned char*)s;*p;p++)if(*p<=32||*p>=127||*p=='+'||*p=='#')return false;
 return !mqtt_topic_under("heizung/pv",s)&&!mqtt_topic_under(s,"heizung/pv");
}
bool mqtt_topic_join(char *out,size_t size,const char *base,const char *suffix){int n=snprintf(out,size,"%s/%s",base,suffix);return n>=0&&(size_t)n<size;}
