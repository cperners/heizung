// Nur die Hauptschleife aendert WLAN/Flash. Ein Test wird erst nach stabiler
// Verbindung gespeichert; bei Stromausfall waehrend des Tests bleiben die alten Daten.
struct StoredWifiSettings { uint32_t version; char ssid[33]; char password[65]; };
static portMUX_TYPE wifiSettingsMux = portMUX_INITIALIZER_UNLOCKED;
static StoredWifiSettings pendingWifiSettings = {};
static bool wifiSettingsPending = false;
static bool wifiSettingsBusy = false;
static uint8_t wifiSettingsState = 0; // 0 unbenutzt, 1 Test, 2 gespeichert, 3 zurueck, 4 Speicherfehler
static StoredWifiSettings trialWifiSettings = {};
static uint8_t wifiTrialPhase = 0;
static uint32_t wifiTrialStarted = 0;
static uint32_t wifiStableSince = 0;
static bool wifiWasStable = false;
static Preferences wifiStorage;
static bool wifiStorageReady = false;

static bool wifiTrialInProgress() { return wifiTrialPhase != 0; }
static bool wifiSettingsAreBusy() {
  bool busy;portENTER_CRITICAL(&wifiSettingsMux);busy=wifiSettingsBusy;portEXIT_CRITICAL(&wifiSettingsMux);return busy;
}

static void loadWifiSettings() {
  wifiStorageReady = wifiStorage.begin("heizung-wifi", false);
  StoredWifiSettings saved = {};
  if (wifiStorageReady && wifiStorage.getBytesLength("network") == sizeof(saved) &&
      wifiStorage.getBytes("network", &saved, sizeof(saved)) == sizeof(saved) && saved.version == 1 &&
      saved.ssid[0] && memchr(saved.ssid, 0, sizeof(saved.ssid)) &&
      memchr(saved.password, 0, sizeof(saved.password))) {
    memcpy(activeWifiSsid, saved.ssid, sizeof(activeWifiSsid));
    memcpy(activeWifiPassword, saved.password, sizeof(activeWifiPassword));
  }
}

static void finishWifiTrial(uint8_t state) {
  wifiTrialPhase = 0;
  memset(&trialWifiSettings, 0, sizeof(trialWifiSettings));
  portENTER_CRITICAL(&wifiSettingsMux);
  wifiSettingsState=state; wifiSettingsBusy=false;
  portEXIT_CRITICAL(&wifiSettingsMux);
}

static void serviceWifiSettings() {
  const uint32_t now=millis();
  if (!wifiTrialPhase) {
    bool pending;
    portENTER_CRITICAL(&wifiSettingsMux);
    pending=wifiSettingsPending;
    if (pending) {
      trialWifiSettings=pendingWifiSettings;
      memset(&pendingWifiSettings,0,sizeof(pendingWifiSettings));
      wifiSettingsPending=false;
    }
    portEXIT_CRITICAL(&wifiSettingsMux);
    if (!pending) return;
    wifiTrialStarted=now; wifiTrialPhase=1;
    return;
  }
  // Zeit fuer die Annahmeantwort an den Browser, ohne die Regelung anzuhalten.
  if (wifiTrialPhase==1) {
    if (static_cast<uint32_t>(now-wifiTrialStarted)<1500UL) return;
    WiFi.disconnect(false, false);
    WiFi.begin(trialWifiSettings.ssid, trialWifiSettings.password);
    wifiTrialPhase=2; wifiTrialStarted=now; wifiWasStable=false;
    return;
  }
  if (WiFi.status()==WL_CONNECTED && static_cast<uint32_t>(WiFi.localIP()) != 0) {
    if (!wifiWasStable) { wifiStableSince=now; wifiWasStable=true; }
    if (static_cast<uint32_t>(now-wifiStableSince)>=5000UL) {
      if (wifiStorageReady && wifiStorage.putBytes("network", &trialWifiSettings, sizeof(trialWifiSettings)) == sizeof(trialWifiSettings)) {
        memcpy(activeWifiSsid,trialWifiSettings.ssid,sizeof(activeWifiSsid));
        memcpy(activeWifiPassword,trialWifiSettings.password,sizeof(activeWifiPassword));
        finishWifiTrial(2); connectToWifi();
        return;
      }
      WiFi.disconnect(false,false); WiFi.begin(ssid,password);
      finishWifiTrial(4); connectToWifi(); return;
    }
  } else wifiWasStable=false;
  if (static_cast<uint32_t>(now-wifiTrialStarted)>=30000UL) {
    WiFi.disconnect(false,false); WiFi.begin(ssid,password);
    finishWifiTrial(3); connectToWifi();
  }
}

static void registerWifiSettings() {
  server.on("/wlan", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (!requireWebSession(request)) return;
    if (mqttSettingsAreChanging()) { request->send(409,"text/plain","MQTT-Wechsel laeuft"); return; }
    if (!request->hasParam("ssid",true) || !request->hasParam("passwort",true)) {
      request->send(400,"text/plain","WLAN-Daten fehlen"); return;
    }
    const String name=request->getParam("ssid",true)->value();
    const String pass=request->getParam("passwort",true)->value();
    // WPA/WPA2-Passphrase 8..63 Bytes; kein stilles Abschneiden.
    if (!name.length() || name.length()>32 || pass.length()<8 || pass.length()>63 ||
        strlen(name.c_str())!=name.length() || strlen(pass.c_str())!=pass.length()) {
      request->send(400,"text/plain","WLAN-Name: 1 bis 32 Bytes; WLAN-Passwort: 8 bis 63 Zeichen"); return;
    }
    bool accepted=false;
    portENTER_CRITICAL(&wifiSettingsMux);
    if (!wifiSettingsBusy) {
      pendingWifiSettings={}; pendingWifiSettings.version=1;
      memcpy(pendingWifiSettings.ssid,name.c_str(),name.length());
      memcpy(pendingWifiSettings.password,pass.c_str(),pass.length());
      wifiSettingsPending=true; wifiSettingsBusy=true; wifiSettingsState=1; accepted=true;
    }
    portEXIT_CRITICAL(&wifiSettingsMux);
    request->send(accepted?202:409,"application/json",accepted?"{\"state\":1}":"{\"busy\":true}");
  });
  server.on("/wlan-status", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (!requireWebSession(request)) return;
    uint8_t state;
    portENTER_CRITICAL(&wifiSettingsMux); state=wifiSettingsState; portEXIT_CRITICAL(&wifiSettingsMux);
    char body[24]; snprintf(body,sizeof(body),"{\"state\":%u}",state);
    auto* response=request->beginResponse(200,"application/json",body);
    response->addHeader("Cache-Control","no-store"); request->send(response);
  });
}
