void notifyClients() {
  ws.textAll(String(HeizungsRelais));
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  if (arg == nullptr || data == nullptr) {
    return;
  }

  const AwsFrameInfo *info = static_cast<AwsFrameInfo*>(arg);
  if (!info->final || info->index != 0 ||
      info->len != len || info->opcode != WS_TEXT) {
    return;
  }

  // Empfangspuffer unveraendert lassen; exakt sechs Zeichen pruefen.
  if (len == 6 && memcmp(data, "toggle", 6) == 0) {
    HeizungsRelais = !HeizungsRelais;
    notifyClients();
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      //Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      //Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

String processor(const String& var){
  if (var != "WEBTIME") Serial.println(var);
  if(var == "MQTTUPDATE"){
    return asyncMqttClient.getClientId();
  }
  if(var == "TimeString"){
    return TimeString;
  }
  if(var == "HEIZUNGSPUMPE"){
    if (HeizungsRelais){
      return "ON";
    }
    else{
      return "OFF";
    }
  }else if(var == "BOILERPUMPE"){
    if (BoilerRelais){
      return "ON";
    }
    else{
      return "OFF";
    }
  }else if(var == "TROOM"){
    return readTemperature(tRoom);
  }else if(var == "GASDAY"){
    return gasDisplay(true);
  }else if(var == "GASTOTAL"){
    return gasDisplay(false);
  }else if(var == "WEBTIME"){
    const time_t utc = time(nullptr);
    if (utc < static_cast<time_t>(1577836800UL)) return "NTP nicht synchron";
    time_t local = utc + GMT_TIME_ZONE * utcOffsetInSeconds;
    struct tm calendar = {};
    if (!gmtime_r(&local, &calendar)) return "Uhrzeit nicht verfuegbar";
    if (Sommerzeit_EinAus && summertime_EU(calendar.tm_year + 1900,
        calendar.tm_mon + 1, calendar.tm_mday, calendar.tm_hour, GMT_TIME_ZONE)) {
      local += 3600;
      if (!gmtime_r(&local, &calendar)) return "Uhrzeit nicht verfuegbar";
    }
    char text[48];
    snprintf(text, sizeof(text), "%s, %02d.%02d.%04d %02d:%02d:%02d",
             daysOfTheWeek[calendar.tm_wday], calendar.tm_mday, calendar.tm_mon + 1,
             calendar.tm_year + 1900, calendar.tm_hour, calendar.tm_min, calendar.tm_sec);
    return String(text);
  }else if(var == "BRENNERSPERRE"){
    return sensorIsUsable(0) ? "Kesselsensor OK - keine Sensorsperre" :
                              "BRENNER GESPERRT - Kesselsensor ungueltig";
  }else if(var == "TKESSEL"){
    return readTemperature(tKessel);
  }else if(var == "TVORLAUF"){
    return readTemperature(tVorlauf);
  }else if(var == "TBOILER"){
    return readTemperature(tBoiler);
  }else if(var == "TAUSSEN"){
    return readTemperature(tAussen);
  }else if(var == "TKACHELOFEN"){
    return readTemperature(tKachelofen);
  }else if(var == "KACHELOFENSTATUS"){
    return kachelofenAktiv ? "AKTIV" : "INAKTIV";
  }else if(var == "BETRIEBSART"){
    return WinterBetrieb ? "AUTOMATIKBETRIEB" : BoilerBetrieb ? "NUR BOILER" : NurHeizung ? "NUR HEIZUNG" : "AUS";
  }else if(var == "BETRIEBAUTO"){
    return WinterBetrieb ? "selected" : "";
  }else if(var == "BETRIEBBOILER"){
    return !WinterBetrieb && BoilerBetrieb ? "selected" : "";
  }else if(var == "BETRIEBHEIZUNG"){
    return !WinterBetrieb && !BoilerBetrieb && NurHeizung ? "selected" : "";
  }else if(var == "BETRIEBAUS"){
    return !WinterBetrieb && !BoilerBetrieb && !NurHeizung ? "selected" : "";
  }else if(var == "RAUMTAG"){
    return String(tRoomTag, 1);
  }else if(var == "RAUMNACHT"){
    return String(tRoomNacht, 1);
  }else if(var == "MODEAUTO"){
    return regelungsModus == REGELUNG_AUTO ? "selected" : "";
  }else if(var == "MODERAUM"){
    return regelungsModus == REGELUNG_RAUM ? "selected" : "";
  }else if(var == "MODEAUSSEN"){
    return regelungsModus == REGELUNG_AUSSEN ? "selected" : "";
  }else if(var == "REGELUNGSART"){
    if(regelungsModus == REGELUNG_AUTO){
      return AussentemperaturRegelung ? "AUTO / AUSSENTEMPERATUR" : "AUTO / RAUMTEMPERATUR";
    }
    return regelungsModus == REGELUNG_RAUM ? "MANUELL / RAUMTEMPERATUR" : "MANUELL / AUSSENTEMPERATUR";
  }else if(var == "KACHELOFENEIN"){
    return String(kachelofenEinTemp, 1);
  }else if(var == "KACHELOFENAUS"){
    return String(kachelofenAusTemp, 1);
  }else if(var == "WIFIRSSI"){
    return readValue(WiFi.RSSI());
  }else if(var == "WIFISSID"){
    return ssid; 
  }
  return String();
}
//#################################################################################################################################

String readTemperature(const float& var){
  const float* readings[5] = {&tKessel, &tVorlauf, &tAussen, &tRoom, &tBoiler};
  for (byte i = 0; i < 5; ++i) {
    if (&var == readings[i] && !sensorIsUsable(i)) return "Sensorfehler";
  }
    if (!isfinite(var)) {
    return "--";
  }
  else {
    return String(var);
  }
}

//#################################################################################################################################

String readValue(const int& var){
    if (isnan(var)) {    
    return "--";
  }
  else {
    return String(var);
  }
}

