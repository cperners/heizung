// Sitzungen bleiben nur im RAM; Neustart meldet alle Browser ab.
static portMUX_TYPE webAccessMux = portMUX_INITIALIZER_UNLOCKED;
struct WebLoginSession { char token[33] = {}; uint32_t touched = 0; };
static WebLoginSession webLoginSessions[4];
static char webAdminPassword[65] = HEIZUNG_WEB_PASSWORD;
static Preferences webPasswordStorage;
static bool webPasswordStorageReady=false;
struct WebPasswordChange { bool pending; char oldPassword[65]; char newPassword[65]; };
static WebPasswordChange webPasswordChange={};
static uint8_t webPasswordState=0;
static bool loginAttempted = false;
static uint32_t lastLoginAttempt = 0;
static constexpr uint32_t WEB_SESSION_MS = 15UL * 60UL * 1000UL;

static bool sameWebToken(const char* a, const char* b) {
  unsigned difference = 0;
  for (size_t i = 0; i < 32; ++i) difference |= static_cast<unsigned char>(a[i] ^ b[i]);
  return difference == 0;
}

static bool webSession(AsyncWebServerRequest* request, bool touch = true, bool remove = false) {
  if (!request->hasHeader("X-Heizung-Session")) return false;
  const String value = request->getHeader("X-Heizung-Session")->value();
  if (value.length() != 32) return false;
  bool accepted = false;
  const uint32_t now = millis();
  portENTER_CRITICAL(&webAccessMux);
  for (auto& session : webLoginSessions) {
    if (session.token[0] && static_cast<uint32_t>(now - session.touched) >= WEB_SESSION_MS)
      memset(&session, 0, sizeof(session));
    if (session.token[0] && sameWebToken(value.c_str(), session.token)) {
      accepted = true;
      if (remove) memset(&session, 0, sizeof(session));
      else if (touch) session.touched = now;
      break;
    }
  }
  portEXIT_CRITICAL(&webAccessMux);
  return accepted;
}

static bool requireWebSession(AsyncWebServerRequest* request) {
  if (webSession(request)) return true;
  request->send(401, "text/plain; charset=utf-8", "Bitte zuerst anmelden.");
  return false;
}

static void loadWebPassword() {
  webPasswordStorageReady=webPasswordStorage.begin("heizung-admin",false);
  char stored[65]={};
  if (!webPasswordStorageReady) return;
  const size_t storedLength=webPasswordStorage.getBytesLength("password");
  // Bisherige 9-Byte-Datensaetze (acht Zeichen plus Null) bleiben gueltig.
  if ((storedLength==9 || storedLength==sizeof(stored)) &&
      webPasswordStorage.getBytes("password",stored,storedLength)==storedLength &&
      stored[0] && memchr(stored,0,storedLength)) {
    memcpy(webAdminPassword,stored,sizeof(stored));
  }
}

static void serviceWebPasswordChange() {
  WebPasswordChange change={};
  portENTER_CRITICAL(&webAccessMux);
  if (webPasswordChange.pending) { change=webPasswordChange; memset(&webPasswordChange,0,sizeof(webPasswordChange)); }
  portEXIT_CRITICAL(&webAccessMux);
  if (!change.pending) return;
  bool matches;portENTER_CRITICAL(&webAccessMux);matches=strcmp(change.oldPassword,webAdminPassword)==0;portEXIT_CRITICAL(&webAccessMux);
  const bool saved=matches && webPasswordStorageReady && webPasswordStorage.putBytes("password",change.newPassword,sizeof(change.newPassword))==sizeof(change.newPassword);
  portENTER_CRITICAL(&webAccessMux);
  if (saved) {
    memcpy(webAdminPassword,change.newPassword,sizeof(webAdminPassword));
    memset(webLoginSessions,0,sizeof(webLoginSessions));
  }
  webPasswordState=saved?2:3;
  portEXIT_CRITICAL(&webAccessMux);
  // Ein Passwortwechsel betrifft nur Web-Einstellungen, nicht die offene Update-Seite.
  memset(&change,0,sizeof(change));
}

static void registerWebAccess() {
  server.on("/anmeldepasswort",HTTP_POST,[](AsyncWebServerRequest* request) {
    if (!requireWebSession(request)) return;
    if (!request->hasParam("alt",true)||!request->hasParam("neu",true)||!request->hasParam("wiederholen",true)) {request->send(400,"text/plain","Passwortangaben fehlen");return;}
    const String oldPass=request->getParam("alt",true)->value(), newPass=request->getParam("neu",true)->value(), repeat=request->getParam("wiederholen",true)->value();
    if (!oldPass.length()||oldPass.length()>64||!newPass.length()||newPass.length()>64||newPass!=repeat||
        strlen(oldPass.c_str())!=oldPass.length()||strlen(newPass.c_str())!=newPass.length()) {request->send(400,"text/plain","Passwort: 1 bis 64 Bytes, beide neuen Eingaben muessen gleich sein");return;}
    bool accepted=false;
    portENTER_CRITICAL(&webAccessMux);
    if (webPasswordState!=1) {
      webPasswordChange={};webPasswordChange.pending=true;
      memcpy(webPasswordChange.oldPassword,oldPass.c_str(),oldPass.length());
      memcpy(webPasswordChange.newPassword,newPass.c_str(),newPass.length());
      webPasswordState=1;accepted=true;
    }
    portEXIT_CRITICAL(&webAccessMux);
    request->send(accepted?202:409,"application/json",accepted?"{\"state\":1}":"{\"busy\":true}");
  });
  server.on("/anmeldepasswort-status",HTTP_GET,[](AsyncWebServerRequest* request) {
    uint8_t state;portENTER_CRITICAL(&webAccessMux);state=webPasswordState;portEXIT_CRITICAL(&webAccessMux);
    char body[24];snprintf(body,sizeof(body),"{\"state\":%u}",state);
    // Keine Zugangsdaten: Erfolg bleibt nach Aufhebung aller Sitzungen lesbar.
    auto* response=request->beginResponse(200,"application/json",body);response->addHeader("Cache-Control","no-store");request->send(response);
  });
  server.on("/anmeldung", HTTP_POST, [](AsyncWebServerRequest* request) {
    // Der eigene Header verhindert Anmeldungen durch fremde HTML-Formulare.
    if (!request->hasHeader("X-Heizung-Login") ||
        !request->hasParam("benutzer", true) || !request->hasParam("passwort", true)) {
      request->send(400, "text/plain", "Anmeldedaten fehlen"); return;
    }
    const uint32_t now = millis();
    bool rateLimited;
    portENTER_CRITICAL(&webAccessMux);
    rateLimited = loginAttempted && static_cast<uint32_t>(now-lastLoginAttempt) < 2000UL;
    if (!rateLimited) { loginAttempted = true; lastLoginAttempt = now; }
    portEXIT_CRITICAL(&webAccessMux);
    if (rateLimited) { request->send(429, "text/plain", "Bitte kurz warten"); return; }
    const String user = request->getParam("benutzer", true)->value();
    const String pass = request->getParam("passwort", true)->value();
    char supplied[65] = {}, expected[65] = {};
    if (pass.length() <= 64) memcpy(supplied, pass.c_str(), pass.length());
    portENTER_CRITICAL(&webAccessMux);
    memcpy(expected, webAdminPassword, sizeof(expected));
    portEXIT_CRITICAL(&webAccessMux);
    unsigned difference = 0;
    for (size_t i=0; i<sizeof(expected); ++i) difference |= supplied[i] ^ expected[i];
    if (!user.length() || user != HEIZUNG_WEB_USER || !pass.length() || pass.length() > 64 || difference) {
      request->send(401, "text/plain", "Anmeldung fehlgeschlagen"); return;
    }
    uint8_t random[16]; esp_fill_random(random, sizeof(random));
    char token[33];
    for (size_t i=0; i<sizeof(random); ++i) snprintf(token+i*2, 3, "%02x", random[i]);
    portENTER_CRITICAL(&webAccessMux);
    size_t slot=0;
    for (size_t i=0; i<4; ++i) {
      if (!webLoginSessions[i].token[0] || static_cast<uint32_t>(now-webLoginSessions[i].touched) >= WEB_SESSION_MS) { slot=i; break; }
      if (static_cast<uint32_t>(now-webLoginSessions[i].touched) > static_cast<uint32_t>(now-webLoginSessions[slot].touched)) slot=i;
    }
    memcpy(webLoginSessions[slot].token, token, sizeof(token));
    webLoginSessions[slot].touched=now;
    portEXIT_CRITICAL(&webAccessMux);
    auto* response = request->beginResponse(200, "application/json", String("{\"token\":\"")+token+"\"}");
    response->addHeader("Cache-Control", "no-store"); request->send(response);
  });
  server.on("/anmeldung-status", HTTP_GET, [](AsyncWebServerRequest* request) {
    auto* response=request->beginResponse(200, "application/json", webSession(request, false) ? "{\"angemeldet\":true}" : "{\"angemeldet\":false}");
    response->addHeader("Cache-Control", "no-store"); request->send(response);
  });
  server.on("/abmelden", HTTP_POST, [](AsyncWebServerRequest* request) {
    webSession(request, false, true); request->send(200, "application/json", "{}");
  });
}
