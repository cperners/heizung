// Alte Datensaetze bleiben lesbar, neue enthalten auch optionale Broker-Zugangsdaten.
struct LegacyMqttSettings { uint32_t version; char host[64]; uint16_t port; char base[81]; };
struct StoredMqttSettings { uint32_t version; char host[64]; uint16_t port; char base[81]; char username[64]; char password[128]; };
static_assert(sizeof(MQTT_USERNAME)<=64 && sizeof(MQTT_PASSWORD)<=128, "MQTT-Zugangsdaten zu lang");
static StoredMqttSettings activeMqttSettings={}, pendingMqttSettings={}, oldMqttSettings={};
static bool mqttSettingsPending=false;
static uint8_t mqttSettingsState=0, mqttChangePhase=0;
static uint32_t mqttChangeStarted=0, mqttTrialLastAttempt=0;
static Preferences mqttSettingsStorage;
static bool mqttSettingsStorageReady=false;

bool mqttSettingsAreChanging() {
  bool value; portENTER_CRITICAL(&mqttSettingsMux); value=mqttSettingsChanging; portEXIT_CRITICAL(&mqttSettingsMux); return value;
}

static bool validMqttSettings(const StoredMqttSettings& settings) {
  const size_t hostLength=strnlen(settings.host,sizeof(settings.host));
  const size_t baseLength=strnlen(settings.base,sizeof(settings.base));
  if (settings.version!=2 || !hostLength || hostLength>=sizeof(settings.host) || !settings.port ||
      !memchr(settings.username,0,sizeof(settings.username)) || !memchr(settings.password,0,sizeof(settings.password)) ||
      (!settings.username[0] && settings.password[0]) ||
      !baseLength || baseLength>=sizeof(settings.base) || settings.base[baseLength-1]!='/') return false;
  for (size_t i=0;i<hostLength;++i) {
    const char c=settings.host[i];
    if (!((c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='.'||c=='-')) return false;
  }
  for (size_t i=0;i<baseLength;++i) {
    const unsigned char c=settings.base[i];
    if (c<=32 || c>=127 || c=='+' || c=='#') return false;
  }
  return true;
}

static void useMqttSettings(const StoredMqttSettings& settings) {
  portENTER_CRITICAL(&mqttSettingsMux);
  activeMqttSettings=settings;
  ++mqttTopicGeneration;
  memcpy(mqttBaseTopic,settings.base,sizeof(mqttBaseTopic));
  snprintf(mqttControlTopic,sizeof(mqttControlTopic),"%sset",settings.base);
  snprintf(KACHELOFEN_MQTT_TOPIC,sizeof(KACHELOFEN_MQTT_TOPIC),"%skachelofenTemp",settings.base);
  snprintf(gasMqttTopic,sizeof(gasMqttTopic),"%sgaszaehler",settings.base);
  portEXIT_CRITICAL(&mqttSettingsMux);
}

static void loadMqttSettings() {
  StoredMqttSettings settings={};settings.version=2;settings.port=MQTT_PORT;
  memcpy(settings.username,MQTT_USERNAME,sizeof(MQTT_USERNAME));
  memcpy(settings.password,MQTT_PASSWORD,sizeof(MQTT_PASSWORD));
  const String defaultHost=MQTT_HOST.toString();
  snprintf(settings.host,sizeof(settings.host),"%s",defaultHost.c_str());
  snprintf(settings.base,sizeof(settings.base),"%s",mqttBaseTopic);
  mqttSettingsStorageReady=mqttSettingsStorage.begin("heizung-mqtt",false);
  StoredMqttSettings saved={};
  if (mqttSettingsStorageReady) {
    const size_t storedLength=mqttSettingsStorage.getBytesLength("settings");
    if (storedLength==sizeof(saved) && mqttSettingsStorage.getBytes("settings",&saved,sizeof(saved))==sizeof(saved) && validMqttSettings(saved)) settings=saved;
    else if (storedLength==sizeof(LegacyMqttSettings)) {
      LegacyMqttSettings legacy={};
      if (mqttSettingsStorage.getBytes("settings",&legacy,sizeof(legacy))==sizeof(legacy) && legacy.version==1) {
        StoredMqttSettings migrated=settings;
        memcpy(migrated.host,legacy.host,sizeof(legacy.host));migrated.port=legacy.port;
        memcpy(migrated.base,legacy.base,sizeof(legacy.base));
        if (validMqttSettings(migrated)) settings=migrated;
      }
    }
  }
  useMqttSettings(settings);
}

static void configureMqttClient() {
  asyncMqttClient.setCredentials(activeMqttSettings.username[0] ? activeMqttSettings.username : nullptr,
                                 activeMqttSettings.username[0] ? activeMqttSettings.password : nullptr);
  asyncMqttClient.setCleanSession(true);
  asyncMqttClient.setServer(activeMqttSettings.host,activeMqttSettings.port);
}

static void finishMqttChange(uint8_t state) {
  portENTER_CRITICAL(&mqttSettingsMux);
  mqttSettingsChanging=false; mqttSettingsState=state;
  portEXIT_CRITICAL(&mqttSettingsMux);
  mqttChangePhase=0;
}

static void beginMqttTrialConnection() {
  asyncMqttClient.clearQueue();
  if (mqttMessageQueue) xQueueReset(mqttMessageQueue);
  configureMqttClient(); mqttTrialLastAttempt=millis();
  portENTER_CRITICAL(&mqttSettingsMux); mqttTransportIdle=false; portEXIT_CRITICAL(&mqttSettingsMux);
  asyncMqttClient.connect();
}

static void serviceMqttSettings() {
  const uint32_t now=millis();
  if (!mqttChangePhase) {
    bool pending;
    portENTER_CRITICAL(&mqttSettingsMux);
    pending=mqttSettingsPending; mqttSettingsPending=false;
    portEXIT_CRITICAL(&mqttSettingsMux);
    if (!pending) return;
    oldMqttSettings=activeMqttSettings;
    connect2mqtt=false; mqttRecon=false;
    mqttChangePhase=1; mqttChangeStarted=now;
    asyncMqttClient.disconnect(true);
    return;
  }
  bool idle;
  portENTER_CRITICAL(&mqttSettingsMux); idle=mqttTransportIdle; portEXIT_CRITICAL(&mqttSettingsMux);
  if (mqttChangePhase==1) {
    if (!idle) {
      if (static_cast<uint32_t>(now-mqttChangeStarted)>=10000UL) { finishMqttChange(3); connect2mqtt=true; }
      return;
    }
    useMqttSettings(pendingMqttSettings);
    beginMqttTrialConnection(); mqttChangePhase=2; mqttChangeStarted=now; return;
  }
  if (mqttChangePhase==2) {
    if (asyncMqttClient.connected()) {
      if (mqttSettingsStorageReady && mqttSettingsStorage.putBytes("settings",&activeMqttSettings,sizeof(activeMqttSettings))==sizeof(activeMqttSettings)) {
        // Keine alten Daten aus dem bisherigen Thema als neue Messung verwenden.
        kachelofenLastUpdate=0;
        portENTER_CRITICAL(&gasMux); pendingGas=false; portEXIT_CRITICAL(&gasMux);
        finishMqttChange(2); mqttRecon=false; connect2mqtt=false;
        subscribeActiveMqttTopics(); mqtt2update=true; return;
      }
      mqttChangePhase=3; mqttChangeStarted=now; asyncMqttClient.disconnect(true); return;
    }
    // Erneuter Versuch nur nach vollstaendigem Abbruch, ohne Blockieren des Loops.
    if (idle && WiFi.isConnected() && static_cast<uint32_t>(now-mqttTrialLastAttempt)>=2000UL && static_cast<uint32_t>(now-mqttChangeStarted)<28000UL) beginMqttTrialConnection();
    if (static_cast<uint32_t>(now-mqttChangeStarted)>=30000UL) {
      mqttChangePhase=3; mqttChangeStarted=now; asyncMqttClient.disconnect(true);
    }
    return;
  }
  if (mqttChangePhase==3 && idle) {
    useMqttSettings(oldMqttSettings); asyncMqttClient.clearQueue();
    if (mqttMessageQueue) xQueueReset(mqttMessageQueue);
    configureMqttClient(); finishMqttChange(3); connect2mqtt=true;
  }
}

static void registerMqttSettings() {
  server.on("/mqtt-einstellungen",HTTP_POST,[](AsyncWebServerRequest* request) {
    if (!requireWebSession(request)) return;
    if (!request->hasParam("server",true)||!request->hasParam("port",true)||!request->hasParam("basis",true)) {
      request->send(400,"text/plain","MQTT-Einstellungen fehlen"); return;
    }
    const String host=request->getParam("server",true)->value(), port=request->getParam("port",true)->value();
    String base=request->getParam("basis",true)->value();
    if (!base.endsWith("/")) base+="/";
    StoredMqttSettings candidate={};candidate.version=2;
    // Alte Browser/Formulare duerfen bestehende Zugangsdaten nicht still loeschen.
    portENTER_CRITICAL(&mqttSettingsMux);
    memcpy(candidate.username,activeMqttSettings.username,sizeof(candidate.username));
    memcpy(candidate.password,activeMqttSettings.password,sizeof(candidate.password));
    portEXIT_CRITICAL(&mqttSettingsMux);
    const bool hasCredentials=request->hasParam("benutzer",true)||request->hasParam("passwort",true);
    if (hasCredentials) {
      if (!request->hasParam("benutzer",true)||!request->hasParam("passwort",true)) {request->send(400,"text/plain","MQTT-Benutzer und Passwortfeld erforderlich");return;}
      const String user=request->getParam("benutzer",true)->value(), pass=request->getParam("passwort",true)->value();
      if (user.length()>=sizeof(candidate.username)||pass.length()>=sizeof(candidate.password)||
          strlen(user.c_str())!=user.length()||strlen(pass.c_str())!=pass.length()||(!user.length()&&pass.length())) {
        request->send(400,"text/plain","MQTT-Zugangsdaten ungueltig");return;
      }
      const bool keep=request->hasParam("passwortbehalten",true)&&request->getParam("passwortbehalten",true)->value()=="1";
      memset(candidate.username,0,sizeof(candidate.username));memcpy(candidate.username,user.c_str(),user.length());
      if (!user.length() || pass.length() || !keep) {
        memset(candidate.password,0,sizeof(candidate.password));memcpy(candidate.password,pass.c_str(),pass.length());
      }
    }
    if (!host.length()||host.length()>=sizeof(candidate.host)||base.length()>=sizeof(candidate.base)||
        strlen(host.c_str())!=host.length()||strlen(base.c_str())!=base.length()||!port.length()||port.length()>5) {
      request->send(400,"text/plain","Ungueltige Laenge"); return;
    }
    unsigned long parsed=0;
    for (size_t i=0;i<port.length();++i) {
      if (port[i]<'0'||port[i]>'9') {request->send(400,"text/plain","Ungueltiger Port");return;}
      parsed=parsed*10+port[i]-'0';
    }
    if (parsed<1||parsed>65535) {request->send(400,"text/plain","Port 1 bis 65535 erforderlich");return;}
    candidate.port=parsed;
    memcpy(candidate.host,host.c_str(),host.length());memcpy(candidate.base,base.c_str(),base.length());
    if (!validMqttSettings(candidate)) { request->send(400,"text/plain","Server oder Basispfad ungueltig; keine Platzhalter oder Leerzeichen");return; }
    if (wifiSettingsAreBusy()) {request->send(409,"text/plain","WLAN-Wechsel laeuft");return;}
    bool accepted=false;
    portENTER_CRITICAL(&mqttSettingsMux);
    if (!mqttSettingsChanging) { pendingMqttSettings=candidate;mqttSettingsChanging=true;mqttSettingsPending=true;mqttSettingsState=1;accepted=true; }
    portEXIT_CRITICAL(&mqttSettingsMux);
    request->send(accepted?202:409,"application/json",accepted?"{\"state\":1}":"{\"busy\":true}");
  });
  server.on("/mqtt-einstellungen",HTTP_GET,[](AsyncWebServerRequest* request) {
    StoredMqttSettings settings;uint8_t state;
    portENTER_CRITICAL(&mqttSettingsMux); settings=activeMqttSettings;state=mqttSettingsState;portEXIT_CRITICAL(&mqttSettingsMux);
    JsonDocument doc;doc["server"]=settings.host;doc["port"]=settings.port;doc["basis"]=settings.base;doc["state"]=state;doc["benutzer"]=settings.username;doc["passwortGesetzt"]=settings.password[0]!=0;
    String body;serializeJson(doc,body);
    auto* response=request->beginResponse(200,"application/json",body);response->addHeader("Cache-Control","no-store");request->send(response);
  });
}
