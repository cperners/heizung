"""Exercise the production login, expiry and WLAN transaction with fake HTTP/NVS/WiFi."""
from pathlib import Path
import subprocess, tempfile
root=Path(__file__).resolve().parents[2]
head=r'''
#include <cassert>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <string>
#include <map>
#include <functional>
#include <vector>
#define portMUX_TYPE int
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) ((void)0)
#define portEXIT_CRITICAL(x) ((void)0)
#define HTTP_GET 1
#define HTTP_POST 2
#define WL_CONNECTED 3
#define HEIZUNG_WEB_USER "test"
#define HEIZUNG_WEB_PASSWORD "12345678"
uint32_t tick=0;
uint32_t millis(){return tick;}
struct String:std::string {
 using std::string::string;
 String(const std::string& s):std::string(s){}
 size_t length()const{return size();}
 bool endsWith(const char* suffix) const {size_t n=strlen(suffix);return size()>=n && compare(size()-n,n,suffix)==0;}
};
struct JsonValue {template<class T> void operator=(const T&) {}};
struct JsonDocument {JsonValue operator[](const char*) {return {};}};
void serializeJson(const JsonDocument&,String& out){out="{}";}
struct Field {String content; String value(){return content;}};
struct Response { int code; String body; void addHeader(const char*,const char*){} };
struct AsyncWebServerRequest {
 std::map<std::string,Field> headers,params; int code=0; String body;
 bool hasHeader(const char* k){return headers.count(k);}
 Field* getHeader(const char* k){return &headers.at(k);}
 bool hasParam(const char* k,bool=false){return params.count(k);}
 Field* getParam(const char* k,bool=false){return &params.at(k);}
 void send(int c,const char*,String b){code=c;body=b;}
 Response* beginResponse(int c,const char*,String b){return new Response{c,b};}
 void send(Response* r){code=r->code;body=r->body;delete r;}
};
struct Server { std::map<std::string,std::function<void(AsyncWebServerRequest*)>> routes;
 void on(const char* path,int method,std::function<void(AsyncWebServerRequest*)> f){routes[std::string(path)+std::to_string(method)]=f;}
} server;
void esp_fill_random(uint8_t* p,size_t n){static uint8_t seed=1; for(size_t i=0;i<n;++i)p[i]=seed++;}
char activeWifiSsid[33]="old",activeWifiPassword[65]="old-password";
const char* ssid=activeWifiSsid;const char* password=activeWifiPassword;
struct Driver {int state=0; uint32_t ip=0; int begins=0;std::string name,pass;
 int status(){return state;} bool isConnected(){return state==WL_CONNECTED;} uint32_t localIP(){return ip;}
 void disconnect(bool,bool){state=0;ip=0;}
 void begin(const char* n,const char* p){++begins;name=n;pass=p;}
} WiFi;
struct Preferences {static std::vector<uint8_t> bytes;static bool fail;static int writes;
 static std::map<std::string,std::vector<uint8_t>> data;std::string ns;
 bool begin(const char* name,bool){ns=name;return true;}
 size_t getBytesLength(const char* key){return data[ns+key].size();}
 size_t getBytes(const char* key,void* out,size_t n){auto& v=data[ns+key];if(v.size()!=n)return 0;memcpy(out,v.data(),n);return n;}
 size_t putBytes(const char* key,const void* in,size_t n){++writes;if(fail)return 0;auto p=static_cast<const uint8_t*>(in);data[ns+key].assign(p,p+n);return n;}
};
std::vector<uint8_t> Preferences::bytes;bool Preferences::fail=false;int Preferences::writes=0;
std::map<std::string,std::vector<uint8_t>> Preferences::data;
struct Ota {std::string password;void setAuth(const char*,const char* p){password=p;}} ElegantOTA;
int mqttSettingsMux=0;bool mqttSettingsChanging=false,mqttTransportIdle=true;
uint32_t mqttTopicGeneration=0;
char mqttBaseTopic[81]="/old/",mqttControlTopic[128]="/old/set",KACHELOFEN_MQTT_TOPIC[128]="/old/kachelofenTemp",gasMqttTopic[128]="/old/gaszaehler";
#define GAS_MQTT_TOPIC gasMqttTopic
struct Mqtt {bool live=false,clean=false;int connections=0,clears=0;const char* host=nullptr;unsigned port=0;std::vector<std::string> subscriptions;
 const char* user=nullptr;const char* pass=nullptr;
 void setCredentials(const char* u,const char* p){user=u;pass=p;}
 bool connected(){return live;}void setCleanSession(bool c){clean=c;}void setServer(const char* h,uint16_t p){host=h;port=p;}
 void connect(){++connections;}void disconnect(bool){live=false;mqttTransportIdle=true;}
 void clearQueue(){++clears;}void subscribe(const char* s,int){subscriptions.push_back(s);}
} asyncMqttClient;
struct DefaultHost {String toString(){return "old-broker";}};
#define MQTT_HOST DefaultHost()
#define MQTT_PORT 1883
#define MQTT_USERNAME "initial-user"
#define MQTT_PASSWORD "initial-pass"
bool connect2mqtt=false,mqttRecon=false,mqtt2update=false,pendingGas=false;uint32_t kachelofenLastUpdate=999;int gasMux=0;
void* mqttMessageQueue=nullptr;int resets=0;void xQueueReset(void*){++resets;}
bool mqttSettingsAreChanging();void subscribeActiveMqttTopics();
struct AsyncMqttClientMessageProperties {bool retain=false;};
struct PendingMqttMessage {char topic[128];char payload[64];size_t length;AsyncMqttClientMessageProperties properties;unsigned long receivedAt;};
#define pdTRUE 1
PendingMqttMessage lastMessage={};int queueSends=0;
int xQueueSend(void*,const PendingMqttMessage* msg,int){lastMessage=*msg;++queueSends;return pdTRUE;}
struct Logger{void println(const char*){}} Serial;
int retries=0;void connectToWifi(){++retries;}
'''
tests=r'''
String login(const char* pass="12345678") {
 tick+=2100;AsyncWebServerRequest q;q.headers["X-Heizung-Login"]={"1"};q.params["benutzer"]={"test"};q.params["passwort"]={pass};
 server.routes["/anmeldung2"](&q);
 if(q.code!=200)return "";
 return q.body.substr(10,32);
}
AsyncWebServerRequest authenticated(const String& token) {AsyncWebServerRequest q;q.headers["X-Heizung-Session"]={token};return q;}
void queueWifi(const String& token) {
 auto q=authenticated(token);q.params["ssid"]={"new"};q.params["passwort"]={"new-password"};
 server.routes["/wlan2"](&q);assert(q.code==202);
 serviceWifiSettings();tick+=1500;serviceWifiSettings();assert(WiFi.name=="new");
}
int main(){
 registerWebAccess();registerWifiSettings();loadWifiSettings();loadWebPassword();loadMqttSettings();registerMqttSettings();
 AsyncWebServerRequest no;assert(!requireWebSession(&no)&&no.code==401);
 auto n=no;n.params["ssid"]={"new"};n.params["passwort"]={"12345678"};server.routes["/wlan2"](&n);assert(n.code==401&&!wifiSettingsPending);
 assert(login("bad").empty());assert(login("123456789").empty());assert(login("").empty());
 String token=login();assert(token.size()==32);auto q=authenticated(token);assert(requireWebSession(&q));
 tick+=WEB_SESSION_MS-1;assert(webSession(&q,false));++tick;assert(!webSession(&q,false));
 token=login();q=authenticated(token);assert(webSession(&q,false,true));assert(!webSession(&q));
 // Overflow-safe session expiry and independent browser sessions.
 tick=0xfffff000; token=login();q=authenticated(token);tick+=WEB_SESSION_MS-1;assert(webSession(&q,false));++tick;assert(!webSession(&q,false));
 token=login();q=authenticated(token);
 auto malformed=q;malformed.headers["X-Heizung-Session"]={"short"};assert(!webSession(&malformed));
 auto bad=q;bad.params["ssid"]={"new"};bad.params["passwort"]={"short"};server.routes["/wlan2"](&bad);assert(bad.code==400);
 // Trial success: old credentials remain current until stable and committed.
 queueWifi(token);assert(std::string(ssid)=="old"&&Preferences::writes==0);
 auto busy=q;busy.params["ssid"]={"another"};busy.params["passwort"]={"12345678"};server.routes["/wlan2"](&busy);assert(busy.code==409);
 WiFi.state=WL_CONNECTED;WiFi.ip=1;serviceWifiSettings();tick+=4999;serviceWifiSettings();assert(Preferences::writes==0);
 ++tick;serviceWifiSettings();assert(!wifiTrialInProgress()&&wifiSettingsState==2&&std::string(ssid)=="new"&&Preferences::writes==1);
 // A reboot during a later trial loads only the previously committed network.
 queueWifi(token);std::strcpy(activeWifiSsid,"scratch");loadWifiSettings();assert(std::string(ssid)=="new");
 // Failed connection restores old credentials and never writes NVS.
 tick+=30000;serviceWifiSettings();assert(wifiSettingsState==3&&WiFi.name=="new"&&Preferences::writes==1);
 // NVS failure also rolls back; saved blob is untouched.
 queueWifi(token);Preferences::fail=true;WiFi.state=WL_CONNECTED;WiFi.ip=1;serviceWifiSettings();tick+=5000;serviceWifiSettings();
 assert(wifiSettingsState==4&&!wifiTrialInProgress()&&std::string(ssid)=="new");
 // Lost connection resets stable timer.
 Preferences::fail=false;queueWifi(token);WiFi.state=WL_CONNECTED;WiFi.ip=1;serviceWifiSettings();tick+=4000;WiFi.state=0;serviceWifiSettings();
 tick+=1000;WiFi.state=WL_CONNECTED;serviceWifiSettings();tick+=4999;serviceWifiSettings();assert(wifiTrialInProgress());

 ++tick;serviceWifiSettings();assert(!wifiTrialInProgress());
 // Password changes: old password required; no success on flash failure.
 auto changePassword=[&](const char* old,const char* next,const char* repeat){auto r=authenticated(token);r.params["alt"]={old};r.params["neu"]={next};r.params["wiederholen"]={repeat};server.routes["/anmeldepasswort2"](&r);return r.code;};
 assert(changePassword("12345678","abcd","different")==400);
 std::string tooLongPassword(65,'x');assert(changePassword("12345678",tooLongPassword.c_str(),tooLongPassword.c_str())==400);
 assert(changePassword("wrong","abcdefgh","abcdefgh")==202);serviceWebPasswordChange();assert(webPasswordState==3&&std::string(webAdminPassword)=="12345678");
 Preferences::fail=true;assert(changePassword("12345678","abcdefgh","abcdefgh")==202);serviceWebPasswordChange();assert(webPasswordState==3&&std::string(webAdminPassword)=="12345678");
 Preferences::fail=false;assert(changePassword("12345678","abcdefgh","abcdefgh")==202);serviceWebPasswordChange();assert(webPasswordState==2&&std::string(webAdminPassword)=="abcdefgh"&&ElegantOTA.password.empty());assert(!webSession(&q));
 assert(login("12345678").empty());token=login("abcdefgh");assert(!token.empty());std::strcpy(webAdminPassword,"12345678");loadWebPassword();assert(std::string(webAdminPassword)=="abcdefgh");
 // MQTT host/base/port validation and unauthenticated access.
 AsyncWebServerRequest mq;mq.params["server"]={"new-broker"};mq.params["port"]={"1884"};mq.params["basis"]={"/new"};server.routes["/mqtt-einstellungen2"](&mq);assert(mq.code==401);
 auto mqttRequest=[&](const char* host,const char* port,const char* base){auto r=authenticated(token);r.params["server"]={host};r.params["port"]={port};r.params["basis"]={base};server.routes["/mqtt-einstellungen2"](&r);return r.code;};
 assert(mqttRequest("new-broker","0","/new/")==400);assert(mqttRequest("new-broker","65536","/new/")==400);assert(mqttRequest("http://wrong","1883","/new/")==400);assert(mqttRequest("new-broker","1883","/new/#")==400);
 assert(mqttRequest("new-broker","1884","/new")==202);assert(mqttSettingsAreChanging());assert(mqttRequest("other","1883","/other/")==409);
 serviceMqttSettings();serviceMqttSettings();assert(std::string(mqttControlTopic)=="/new/set"&&std::string(KACHELOFEN_MQTT_TOPIC)=="/new/kachelofenTemp"&&asyncMqttClient.port==1884&&asyncMqttClient.clean);
 asyncMqttClient.live=true;serviceMqttSettings();assert(mqttSettingsState==2&&!mqttSettingsAreChanging());assert(asyncMqttClient.subscriptions.size()==3&&asyncMqttClient.subscriptions[0]=="/new/set"&&kachelofenLastUpdate==0);
 // Saved settings survive restart; old broker/configuration are not restored by an OTA build.
 std::strcpy(mqttBaseTopic,"/scratch/");loadMqttSettings();assert(std::string(mqttBaseTopic)=="/new/");
 // Unreachable target: timeout reconnects to the old broker and restores all topics.
 assert(mqttRequest("bad-broker","1883","/bad/")==202);serviceMqttSettings();serviceMqttSettings();tick+=30000;serviceMqttSettings();serviceMqttSettings();assert(mqttSettingsState==3&&std::string(activeMqttSettings.host)=="new-broker"&&std::string(mqttControlTopic)=="/new/set");
 // Successful connection but storage failure must also roll back.
 assert(mqttRequest("other-broker","1883","/other/")==202);serviceMqttSettings();serviceMqttSettings();asyncMqttClient.live=true;Preferences::fail=true;serviceMqttSettings();serviceMqttSettings();assert(mqttSettingsState==3&&std::string(mqttBaseTopic)=="/new/");
 Preferences::fail=false;loadMqttSettings();assert(std::string(activeMqttSettings.host)=="new-broker");

 // Credentials are transactional, optional, and survive firmware migration/restarts.
 auto credentialRequest=[&](const char* user,const char* pass,bool keep){auto r=authenticated(token);r.params["server"]={"new-broker"};r.params["port"]={"1884"};r.params["basis"]={"/new/"};r.params["benutzer"]={user};r.params["passwort"]={pass};if(keep)r.params["passwortbehalten"]={"1"};server.routes["/mqtt-einstellungen2"](&r);return r.code;};
 assert(std::string(activeMqttSettings.username)=="initial-user"&&std::string(activeMqttSettings.password)=="initial-pass");
 assert(credentialRequest("","password-only",false)==400);
 std::string longUser(64,'x'),longPass(128,'x');assert(credentialRequest(longUser.c_str(),"p",false)==400);assert(credentialRequest("u",longPass.c_str(),false)==400);
 assert(credentialRequest("broker-user","broker-secret",false)==202);serviceMqttSettings();serviceMqttSettings();assert(std::string(asyncMqttClient.user)=="broker-user"&&std::string(asyncMqttClient.pass)=="broker-secret");
 asyncMqttClient.live=true;serviceMqttSettings();assert(mqttSettingsState==2);loadMqttSettings();assert(std::string(activeMqttSettings.password)=="broker-secret");
 // Checked keep-password preserves the secret when browser submits an empty field.
 assert(credentialRequest("broker-user","",true)==202);serviceMqttSettings();serviceMqttSettings();asyncMqttClient.live=true;serviceMqttSettings();assert(std::string(activeMqttSettings.password)=="broker-secret");
 // Bad credentials (no successful CONNACK) restore previous user and password too.
 assert(credentialRequest("wrong-user","wrong-password",false)==202);serviceMqttSettings();serviceMqttSettings();tick+=30000;serviceMqttSettings();serviceMqttSettings();assert(mqttSettingsState==3&&std::string(activeMqttSettings.username)=="broker-user"&&std::string(activeMqttSettings.password)=="broker-secret");
 // Failed NVS commit also restores all credentials; saved configuration is intact.
 assert(credentialRequest("other-user","other-secret",false)==202);serviceMqttSettings();serviceMqttSettings();asyncMqttClient.live=true;Preferences::fail=true;serviceMqttSettings();serviceMqttSettings();Preferences::fail=false;
 assert(std::string(activeMqttSettings.username)=="broker-user"&&std::string(activeMqttSettings.password)=="broker-secret");loadMqttSettings();assert(std::string(activeMqttSettings.password)=="broker-secret");
 // Explicitly empty fields disable auth, even if the browser keep checkbox remained checked.
 assert(credentialRequest("","",true)==202);serviceMqttSettings();serviceMqttSettings();assert(asyncMqttClient.user==nullptr&&asyncMqttClient.pass==nullptr);
 asyncMqttClient.live=true;serviceMqttSettings();loadMqttSettings();configureMqttClient();assert(asyncMqttClient.user==nullptr&&asyncMqttClient.pass==nullptr);
 // Username-only with an empty password is possible when keep is not selected.
 assert(credentialRequest("name-only","",false)==202);serviceMqttSettings();serviceMqttSettings();asyncMqttClient.live=true;serviceMqttSettings();assert(std::string(activeMqttSettings.username)=="name-only"&&!activeMqttSettings.password[0]);
 // Old format migration preserves host, port and base, adding firmware credential defaults.
 auto savedBlob=Preferences::data["heizung-mqttsettings"];LegacyMqttSettings legacy={};legacy.version=1;legacy.port=1885;std::strcpy(legacy.host,"legacy-broker");std::strcpy(legacy.base,"/legacy/");
 auto bytes=reinterpret_cast<const uint8_t*>(&legacy);Preferences::data["heizung-mqttsettings"].assign(bytes,bytes+sizeof(legacy));loadMqttSettings();
 assert(std::string(activeMqttSettings.host)=="legacy-broker"&&activeMqttSettings.port==1885&&std::string(mqttBaseTopic)=="/legacy/"&&std::string(activeMqttSettings.password)=="initial-pass");
 Preferences::data["heizung-mqttsettings"]=savedBlob;loadMqttSettings();
 // The actual MQTT receive callback accepts new topics only; no stale command replay.
 mqttMessageQueue=reinterpret_cast<void*>(1);AsyncMqttClientMessageProperties prop;
 char payload[]="setheizung",oldTopic[]="/old/set",newTopic[]="/new/set";
 onMqttMessage(oldTopic,payload,prop,10,0,10);assert(queueSends==0);
 onMqttMessage(newTopic,payload,prop,10,0,10);assert(queueSends==1&&std::string(lastMessage.topic)=="/new/set");
 mqttSettingsChanging=true;onMqttMessage(newTopic,payload,prop,10,0,10);assert(queueSends==1);mqttSettingsChanging=false;
 // Existing eight-character NVS records still work, including after an upgrade.
 char legacyPassword[9]="abcdefgh";auto legacyBytes=reinterpret_cast<const uint8_t*>(legacyPassword);
 Preferences::data["heizung-adminpassword"].assign(legacyBytes,legacyBytes+sizeof(legacyPassword));
 loadWebPassword();token=login("abcdefgh");assert(!token.empty());
 // New limit: 64 bytes succeeds, 65 is rejected without modifying the current password.
 std::string fullPassword(64,'L');assert(changePassword("abcdefgh",fullPassword.c_str(),fullPassword.c_str())==202);serviceWebPasswordChange();
 assert(std::string(webAdminPassword)==fullPassword);assert(login("abcdefgh").empty());token=login(fullPassword.c_str());assert(!token.empty());
 loadWebPassword();assert(std::string(webAdminPassword)==fullPassword);
 assert(changePassword(fullPassword.c_str(),tooLongPassword.c_str(),tooLongPassword.c_str())==400);
 assert(login(tooLongPassword.c_str()).empty());
 Preferences::fail=true;assert(changePassword(fullPassword.c_str(),"replacement-long","replacement-long")==202);serviceWebPasswordChange();Preferences::fail=false;
 assert(webPasswordState==3&&std::string(webAdminPassword)==fullPassword);

}

'''
with tempfile.TemporaryDirectory() as tmp:
 source=Path(tmp)/'test.cpp';binary=Path(tmp)/'test';mqtt=(root/'src/firmware/mqtt.inc').read_text()
 subscribe=mqtt[mqtt.index('void subscribeActiveMqttTopics()'):mqtt.index('void onMqttConnect(bool')]
 receive=mqtt[mqtt.index('void onMqttMessage('):mqtt.index('void processMqttMessages()')]
 source.write_text(head+(root/'src/firmware/web_access.inc').read_text()+(root/'src/firmware/wifi_settings.inc').read_text()+(root/'src/firmware/mqtt_settings.inc').read_text()+subscribe+receive+tests)
 subprocess.run(['clang++','-std=c++11','-fsanitize=address,undefined','-g',str(source),'-o',str(binary)],check=True)
 subprocess.run([str(binary)],check=True)
# Route coverage: the exceptions are explicit; old threshold GET must be gone.
setup=(root/'src/firmware/setup.inc').read_text()
for route in ['raumtemperaturen','vorlaufmaximum','boilertemperatur','kachelofen']:
 start=setup.index('server.on("/'+route+'", HTTP_POST')
 assert 'requireWebSession(request)' in setup[start:start+180]
for route in ['regelung','betriebsart']:
 start=setup.index('server.on("/'+route+'", HTTP_POST')
 assert 'requireWebSession' not in setup[start:start+180]
assert 'server.on("/kachelofen", HTTP_GET' not in setup
assert 'ElegantOTA.begin(&server);' in setup
assert 'ElegantOTA.setAuth' not in (root/'src/firmware/web_access.inc').read_text()
assert 'initWebSocket();' not in setup
settings_source=(root/'src/firmware/mqtt_settings.inc').read_text()
get_route=settings_source[settings_source.index('server.on("/mqtt-einstellungen",HTTP_GET'): ]
assert 'doc["passwort"]' not in get_route and 'doc["password"]' not in get_route
print('Web/WLAN/MQTT: Anmeldung, Passwortwechsel, Neustart, alle Themen, Rueckfall und Speicherfehler geprueft.')
