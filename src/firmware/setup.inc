void setup() {
  beginCpuLoad();
  mqttMessageQueue = xQueueCreate(MQTT_QUEUE_LENGTH, sizeof(PendingMqttMessage));

  Serial.begin(BAUD_RATE);
  esp_task_wdt_init(WDT_TIMEOUT, true); //disable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch

  pinMode(BrennerPin, OUTPUT);
  pinMode(HeizungPin, OUTPUT);
  pinMode(BoilerPin, OUTPUT);
  pinMode(MischerAuf_Pin, OUTPUT);
  pinMode(MischerZu_Pin, OUTPUT);

  if (!EEPROM.begin(EE_SIZE)) {
    Serial.println("Einstellungsspeicher konnte nicht gestartet werden");
    while (true) { delay(1000); }
  }
  initializeStoredSettings();
  gasStorageReady = gasStorage.begin("gas-meter", false);
  if (gasStorageReady) {
    gasDayBase = gasStorage.getDouble("base", NAN);
    gasDayKey = gasStorage.getInt("day", 0);
  }


//float tVorlauf,tAussen,tKessel,tKesselDest,tKesselDiff,tBoiler,tBoilerDest,tBoilerDiff,tRoom,tRoomTag,tRoomDiff;
  EEPROM.get( EEADDRESS_BOILER, tBoilerDest );
  Serial.println( tBoilerDest, 1 );
  Serial.print("...\n");
  EEPROM.get( EEADDRESS_RAUM, tRoomTag );
  Serial.println( tRoomTag, 1 );
  Serial.print("...\n");
  EEPROM.get( EEADDRESS_RAUMNACHT, tRoomNacht );
  Serial.println( tRoomNacht, 1 );
  Serial.print("...\n");
  EEPROM.get( EEADDRESS_KESSEL, tKesselDest );
  Serial.println( tKesselDest, 1 );
  Serial.print("...\n");
  EEPROM.get( EEADDRESS_DIFFRAUM, tRoomDiff );
  Serial.println( tRoomDiff, 1 );
  Serial.print("...\n");
  EEPROM.get( EEADDRESS_DIFFKESSEL, tKesselDiff );
  Serial.println( tKesselDiff, 1 );
  Serial.print("...\n");
  EEPROM.get( EEADDRESS_DIFFBOILER, tBoilerDiff );
  Serial.println( tBoilerDiff, 1 );
  Serial.print("...\n");
  EEPROM.get( EEADDRESS_TVMAX, tvmax );
  Serial.println( tvmax, 1 );
  Serial.print("...\n");
  EEPROM.get( EEADDRESS_TAUMIN, taumin );
  taumin*=-1.0;
  Serial.println( taumin, 1 );
  Serial.print("...\n");
  EEPROM.get( EEADDRESS_NN, n );
  Serial.println( n, 1 );
  Serial.print("...\n");
  EEPROM.get( EEADDRESS_AUSSENTEMPREGELUNG, AussentemperaturRegelung);
  Serial.println( AussentemperaturRegelung, 1 );
  Serial.print("...\n");

  EEPROM.get( EEADDRESS_KACHELOFEN_EIN, kachelofenEinTemp );
  EEPROM.get( EEADDRESS_KACHELOFEN_AUS, kachelofenAusTemp );
  // Neue EEPROM-Felder koennen beim ersten Start uninitialisiert sein.
  if(!isfinite(kachelofenEinTemp) || !isfinite(kachelofenAusTemp) ||
     kachelofenEinTemp < 10.0 || kachelofenEinTemp > 150.0 ||
     kachelofenAusTemp < 0.0 || kachelofenAusTemp > 140.0 ||
     kachelofenAusTemp >= kachelofenEinTemp){
    kachelofenEinTemp = 50.0;
    kachelofenAusTemp = 40.0;
    EEPROM.put( EEADDRESS_KACHELOFEN_EIN, kachelofenEinTemp );
    EEPROM.put( EEADDRESS_KACHELOFEN_AUS, kachelofenAusTemp );
    EEPROM.commit();
  }
  Serial.print("Kachelofen EIN: ");
  Serial.println(kachelofenEinTemp, 1);
  Serial.print("Kachelofen AUS: ");
  Serial.println(kachelofenAusTemp, 1);
  EEPROM.get( EEADDRESS_TAG, TagBegin );
  TagBeginHr = (int)(TagBegin);
  TagBeginMi = static_cast<int>(roundf((TagBegin - TagBeginHr)*100.0f));
  Serial.println( TagBegin, 2 );
  Serial.print("Tag - \n");
  Serial.println( TagBeginHr );
  Serial.print(":");
  Serial.println( TagBeginMi );
  Serial.print("...\n");
  EEPROM.get( EEADDRESS_NACHT, NachtBegin );
  NachtBeginHr = (int)(NachtBegin);
  NachtBeginMi = static_cast<int>(roundf((NachtBegin - NachtBeginHr)*100.0f));
  Serial.println( TagBegin, 2 );
  Serial.print("Nacht - \n");
  Serial.println( NachtBeginHr );
  Serial.print(":");
  Serial.println( NachtBeginMi );
  Serial.print("...\n");
  EEPROM.get( EEADDRESS_BOILER_SOMMERBETRIEB, BoilerBetrieb );
  Serial.println(BoilerBetrieb);
  Serial.print("-BoilerBetrieb\n");
  EEPROM.get( EEADDRESS_NUR_HEIZUNG, NurHeizung );
  Serial.println(NurHeizung);
  Serial.print("-NurHeizung\n");
  EEPROM.get( EEADDRESS_WINTER, WinterBetrieb );
  Serial.println(WinterBetrieb);
  Serial.print("-WinterBetrieb\n");
  EEPROM.get( EEADDRESS_BR_LAUFZEIT, BrennerLaufzeit );
  Serial.println(BrennerLaufzeit);
  Serial.print("-BrennerLaufzeit\n");
  EEPROM.get( EEADDRESS_SOMMERZEIT_EINAUS, Sommerzeit_EinAus );
  Serial.println(Sommerzeit_EinAus);
  Serial.print("-Sommerzeit_EinAus");
  Wire.begin();
  auto hardwarePresent = [](uint8_t address) {
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
  };
  lcdPresent = hardwarePresent(lcd_addr);
  keypadPresent = hardwarePresent(keypad_addr);
  ioExtenderPresent = hardwarePresent(ioextender0_addr);
  Serial.printf("Zusatzhardware: Display %s, Tastatur %s, Erweiterung %s\n",
                lcdPresent ? "vorhanden" : "fehlt", keypadPresent ? "vorhanden" : "fehlt",
                ioExtenderPresent ? "vorhanden" : "fehlt");
  //I2C ESP32 -> SDA (default is GPIO 21), SCL (default is GPIO 22)
// i2c ioextender
  ioextender0_indicate = 0b11111111;
  I2C_IO_Init(ioextender0_addr,ioextender0_indicate);
//
  lcd.begin();
  lcd.setBacklight(HIGH);
  //MenueBooting();
  if (keypadPresent) I2C_Keypad.begin();
  if (lcdPresent || keypadPresent || ioExtenderPresent) delay(1000);
   //For each tture sensor: Do a pinMode and a digitalWrite
   for (bLoopCounter = 0;
     bLoopCounter <= kTtureSensorMaxIndex;
     bLoopCounter++)
   {
      pinMode(tture[bLoopCounter], INPUT);
      digitalWrite(tture[bLoopCounter], LOW);//Disable internal pull-up.
   }
   //pinMode(pLED,OUTPUT);//Just so it can "pulse" to show Arduino
   // is working
   delay(300);//Wait for newly restarted system to stabilize
   Serial.println(sPrgmName);
   Serial.println(sVers);
   Serial.println();
   Serial.println("See http://sheepdogguides.com/arduino/ar3ne1tt2.htm");
   Serial.println("Temperature measurement, Multiple Dallas DSxxxx sensors");
   Serial.println("The 'S' value at the start of each line identifies the sensor reading was from.");
   Serial.println("*************************************************");
   Serial.println("Die MQTT Variablen\n");
   String bez = MQTT_TEXT;
   Serial.println(bez);
   Serial.println("Per Serial den Arbeitsmodus umschalten:");
   Serial.println("a ->AutomatikBetrieb");
   Serial.println("b ->BoilerBetrieb");
   Serial.println("h ->NurHeizungsBetrieb");
   Serial.println("r ->Aussen-/Raum-Regelung");
   Serial.println("s ->AUS");
   Serial.println("*************************************************");
   Serial.println("\n\n\n");
   // Configures static IP address
  // Production address is assigned by the router via DHCP.
   Serial.print("Connecting to :");
   Serial.println(ssid);
   WiFi.onEvent(WiFiEvent);
   WiFi.mode(WIFI_STA);
   WiFi.begin(ssid, password);
   configTime(0, 0,
           NTP_SERVER1,
           NTP_SERVER2,
           NTP_SERVER3);
   Serial.println("");
   Serial.println("WiFi connected");
   Serial.println("IP address: ");
   Serial.println(WiFi.localIP());
   lcd.clear();

   if (WiFi.status()) {
     wifi_rssi = WiFi.RSSI();
     ntpupdate();
     Schaltuhr();
     lcd.setCursor(1, 0);
     lcd.print(WiFi.localIP());
     Serial.println(WiFi.localIP());
     //onMqttConnect();
     esp_task_wdt_reset(); //watchdog Zeit rücksetzen
     Serial.print("connecting to MQTT broker...\n");
   }
   
   Serial.println("Reading from EEPROM\n");
   Serial.println(EEPROM.read(EEADDRESS_KESSEL));
   asyncMqttClient.onConnect(onMqttConnect);
   asyncMqttClient.onDisconnect(onMqttDisconnect); 
   asyncMqttClient.onSubscribe(onMqttSubscribe);
   asyncMqttClient.setCredentials(mqttUser, mqttPassword);
   asyncMqttClient.setClientId(mqttClientID);
   asyncMqttClient.onMessage(onMqttMessage);
   asyncMqttClient.setServer(MQTT_HOST,MQTT_PORT);
   

   timer0_time.start();
   timer1_2lcd.start();
   timer2_ntp.start();
   timer3_mqttupdate.start();
  serial_clear_screen();
  initWebSocket();
  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
  String page = FPSTR(index_html);
  const char* placeholders[] = {
    "TimeString", "MQTTUPDATE", "HEIZUNGSPUMPE", "BOILERPUMPE",
    "TROOM", "TAUSSEN", "TKESSEL", "TVORLAUF", "TBOILER",
    "TKACHELOFEN", "KACHELOFENSTATUS", "REGELUNGSART",
    "KACHELOFENEIN", "KACHELOFENAUS", "WIFISSID", "WIFIRSSI", "BRENNERSPERRE", "WEBTIME", "MODEAUTO", "MODERAUM", "MODEAUSSEN", "GASDAY", "GASTOTAL", "BETRIEBSART", "BETRIEBAUTO", "BETRIEBBOILER", "BETRIEBHEIZUNG", "BETRIEBAUS", "RAUMTAG", "RAUMNACHT"
  };
  for (const char* name : placeholders) {
    page.replace(String("%") + name + "%", processor(String(name)));
  }
  request->send(200, "text/html; charset=utf-8", page);
});
  server.on("/webstatus", HTTP_GET, [](AsyncWebServerRequest *request){
    char buffer[WEB_STATUS_CAPACITY];
    portENTER_CRITICAL(&webStatusMux);
    const size_t length = webStatusLength;
    memcpy(buffer, webStatusCache, length+1);
    portEXIT_CRITICAL(&webStatusMux);
    if (length == 0) { request->send(503, "text/plain", "Status wird vorbereitet"); return; }
    AsyncWebServerResponse* response = request->beginResponse(200, "application/json", buffer);
    response->addHeader("Cache-Control", "no-store"); request->send(response);
  });
  server.on("/gas", HTTP_GET, [](AsyncWebServerRequest *request){
    const bool daily = request->hasParam("daily");
    request->send(200, "text/plain; charset=utf-8", gasDisplay(daily));
  });
  server.on("/raumtemperaturen", HTTP_POST, [](AsyncWebServerRequest *request){
    if (!request->hasParam("tag", true) || !request->hasParam("nacht", true)) {
      request->send(400, "text/plain; charset=utf-8", "Tag und Nacht erforderlich"); return;
    }
    const String dayText = request->getParam("tag", true)->value();
    const String nightText = request->getParam("nacht", true)->value();
    char *dayEnd = nullptr, *nightEnd = nullptr;
    const float day = strtof(dayText.c_str(), &dayEnd), night = strtof(nightText.c_str(), &nightEnd);
    if (dayText.length() > 16 || nightText.length() > 16 || dayEnd == dayText.c_str() || *dayEnd != '\0' ||
        nightEnd == nightText.c_str() || *nightEnd != '\0' || !isfinite(day) || !isfinite(night) ||
        day < 5 || day > 40 || night < 5 || night > 40) {
      request->send(400, "text/plain; charset=utf-8", "Ungueltige Wunschtemperaturen (5 bis 40 Grad)"); return;
    }
    uint32_t id = 0;
    portENTER_CRITICAL(&roomSetpointMux);
    if (!requestedRoomSetpoints.pending) {
      uint32_t candidate = nextRoomSaveId + 1;
      if (candidate == 0) candidate = 1;
      RoomSaveResult& result = roomSaveResults[candidate % 8];
      if (result.state != 1) {
        nextRoomSaveId = id = candidate;
        result = {id, 1};
        requestedRoomSetpoints = {true, day, night, id};
      }
    }
    portEXIT_CRITICAL(&roomSetpointMux);
    if (id == 0) { request->send(409, "text/plain", "Speichern laeuft bereits"); return; }
    char response[40]; snprintf(response, sizeof(response), "{\"id\":%lu}", static_cast<unsigned long>(id));
    request->send(202, "application/json", response);
  });
  server.on("/raumtemperaturen-status", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!request->hasParam("id")) { request->send(400, "text/plain", "ID fehlt"); return; }
    const String text = request->getParam("id")->value();
    if (text.length() == 0 || text.length() > 10) { request->send(400, "text/plain", "ID ungueltig"); return; }
    for (unsigned int i = 0; i < text.length(); ++i) {
      if (text[i] < '0' || text[i] > '9') { request->send(400, "text/plain", "ID ungueltig"); return; }
    }
    const uint64_t parsed = strtoull(text.c_str(), nullptr, 10);
    if (parsed == 0 || parsed > UINT32_MAX) { request->send(400, "text/plain", "ID ungueltig"); return; }
    const uint32_t id = static_cast<uint32_t>(parsed);
    portENTER_CRITICAL(&roomSetpointMux);
    const RoomSaveResult result = roomSaveResults[id % 8];
    portEXIT_CRITICAL(&roomSetpointMux);
    if (result.id != id) { request->send(404, "text/plain", "Speicherauftrag nicht mehr verfuegbar"); return; }
    char body[24]; snprintf(body, sizeof(body), "{\"state\":%u}", result.state);
    AsyncWebServerResponse* response = request->beginResponse(200, "application/json", body);
    response->addHeader("Cache-Control", "no-store"); request->send(response);
  });
  server.on("/ausgangsstatus", HTTP_GET, [](AsyncWebServerRequest *request){
    char result[32];
    snprintf(result, sizeof(result), "[%d,%d,%d,%d,%d]",
      digitalRead(BrennerPin) == HIGH, digitalRead(BoilerPin) == HIGH,
      digitalRead(HeizungPin) == HIGH, digitalRead(MischerAuf_Pin) == HIGH,
      digitalRead(MischerZu_Pin) == HIGH);
    AsyncWebServerResponse* response = request->beginResponse(200, "application/json", result);
    response->addHeader("Cache-Control", "no-store");
    request->send(response);
  });
  server.on("/ntp-abweichung", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain; charset=utf-8", ntpProbeDisplay());
  });
  server.on("/betriebsart", HTTP_POST, [](AsyncWebServerRequest *request){
    if (!request->hasParam("modus", true)) {
      request->send(400, "text/plain", "Betriebsart fehlt"); return;
    }
    const String value = request->getParam("modus", true)->value();
    const int mode = value == "auto" ? 0 : value == "boiler" ? 1 : value == "heizung" ? 2 : value == "aus" ? 3 : -1;
    if (mode < 0) { request->send(400, "text/plain", "Ungueltige Betriebsart"); return; }
    portENTER_CRITICAL(&operatingModeMux);
    requestedOperatingMode = mode;
    portEXIT_CRITICAL(&operatingModeMux);
    request->redirect("/");
  });
  server.on("/betriebsart", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain; charset=utf-8", processor("BETRIEBSART"));
  });
  server.on("/regelung", HTTP_POST, [](AsyncWebServerRequest *request){
    if (!request->hasParam("modus", true)) {
      request->send(400, "text/plain", "Regelungsart fehlt"); return;
    }
    const String mode = request->getParam("modus", true)->value();
    const int selectedMode = mode == "auto" ? REGELUNG_AUTO : mode == "raum" ? REGELUNG_RAUM : mode == "aussen" ? REGELUNG_AUSSEN : -1;
    if (selectedMode < 0) { request->send(400, "text/plain", "Ungueltige Regelungsart"); return; }
    portENTER_CRITICAL(&webSettingsMux);
    requestedWebMode = selectedMode;
    portEXIT_CRITICAL(&webSettingsMux);
    request->redirect("/");
  });
  server.on("/regelungsart", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain; charset=utf-8", processor("REGELUNGSART"));
  });
  server.on("/uhrzeit", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain; charset=utf-8", processor("WEBTIME"));
  });
  server.on("/boilerdiagnose", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain; charset=utf-8", boilerDiagnosticDetails());
  });
  server.on("/brennersperre", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain; charset=utf-8", processor("BRENNERSPERRE"));
  });
  server.on("/tkachelofen", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", readTemperature(tKachelofen));
  });
  server.on("/kachelofen", HTTP_GET, [](AsyncWebServerRequest *request){
    if(!request->hasParam("ein") || !request->hasParam("aus")){
      request->send(400, "text/plain", "Parameter ein und aus erforderlich");
      return;
    }
    String einText = request->getParam("ein")->value();
    String ausText = request->getParam("aus")->value();
    char* einEnd = nullptr;
    char* ausEnd = nullptr;
    float newEin = strtof(einText.c_str(), &einEnd);
    float newAus = strtof(ausText.c_str(), &ausEnd);
    if(einEnd == einText.c_str() || *einEnd != '\0' ||
       ausEnd == ausText.c_str() || *ausEnd != '\0' ||
       !isfinite(newEin) || !isfinite(newAus) ||
       newEin < 10.0 || newEin > 150.0 ||
       newAus < 0.0 || newAus > 140.0 ||
       newAus >= newEin){
      request->send(400, "text/plain", "Ungueltige Kachelofen-Schaltschwellen");
      return;
    }
    portENTER_CRITICAL(&webSettingsMux);
    requestedKachelofenThresholds = {true, newEin, newAus};
    portEXIT_CRITICAL(&webSettingsMux);
    request->redirect("/");
  });
  // Start ElegantOTA
  ElegantOTA.begin(&server);
  server.begin();
  esp_task_wdt_reset(); //watchdog Zeit rücksetzen
  serial_clear_screen();
}//end of setup()


/* *******************************************************************************************************
                                         MAIN LOOP
******************************************************************************************************* */
