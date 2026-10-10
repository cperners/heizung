void loop(){
  static bool ntpProbeStarted = false;
  if (!ntpProbeStarted) {
    ntpProbeStarted = xTaskCreate(ntpProbeTask, "ntp-probe", 4096, nullptr, 1, nullptr) == pdPASS;
  }
  ElegantOTA.loop();
  processMqttMessages();
  processGasReading();
  portENTER_CRITICAL(&operatingModeMux);
  const int operatingMode = requestedOperatingMode;
  requestedOperatingMode = -1;
  portEXIT_CRITICAL(&operatingModeMux);
  if (operatingMode >= 0 && operatingMode <= 3) {
    WinterBetrieb = operatingMode == 0;
    BoilerBetrieb = operatingMode == 1;
    NurHeizung = operatingMode == 2;
    Betriebsart = operatingMode == 0 ? 'A' : operatingMode == 1 ? 'B' : operatingMode == 2 ? 'H' : '0';
    EEPROM.put(EEADDRESS_WINTER, WinterBetrieb);
    EEPROM.put(EEADDRESS_BOILER_SOMMERBETRIEB, BoilerBetrieb);
    EEPROM.put(EEADDRESS_NUR_HEIZUNG, NurHeizung);
    EEPROM.commit();
  }
  portENTER_CRITICAL(&roomSetpointMux);
  const RoomSetpointRequest roomRequest = requestedRoomSetpoints;
  requestedRoomSetpoints.pending = false;
  portEXIT_CRITICAL(&roomSetpointMux);
  if (roomRequest.pending) {
    const float previousDay = tRoomTag, previousNight = tRoomNacht;
    const float previousTagBegin = TagBegin, previousNachtBegin = NachtBegin;
    const float newTagBegin = roomRequest.dayMinutes/60 + (roomRequest.dayMinutes%60)/100.0f;
    const float newNachtBegin = roomRequest.nightMinutes/60 + (roomRequest.nightMinutes%60)/100.0f;
    if (roomRequest.hasSchedule) {
      EEPROM.put(EEADDRESS_TAG, newTagBegin);
      EEPROM.put(EEADDRESS_NACHT, newNachtBegin);
    }
    EEPROM.put(EEADDRESS_RAUM, roomRequest.day);
    EEPROM.put(EEADDRESS_RAUMNACHT, roomRequest.night);
    const bool saved = EEPROM.commit();
    if (saved) {
      tRoomTag = roomRequest.day; tRoomNacht = roomRequest.night;
      if (roomRequest.hasSchedule) {
        TagBegin = newTagBegin; NachtBegin = newNachtBegin;
        TagBeginHr = roomRequest.dayMinutes/60; TagBeginMi = roomRequest.dayMinutes%60;
        NachtBeginHr = roomRequest.nightMinutes/60; NachtBeginMi = roomRequest.nightMinutes%60;
        Schaltuhr();
      }
    } else {
      if (roomRequest.hasSchedule) {
        EEPROM.put(EEADDRESS_TAG, previousTagBegin);
        EEPROM.put(EEADDRESS_NACHT, previousNachtBegin);
      }
      EEPROM.put(EEADDRESS_RAUM, previousDay);
      EEPROM.put(EEADDRESS_RAUMNACHT, previousNight);
      Serial.println("Wunschtemperaturen: Speichern fehlgeschlagen");
    }
    portENTER_CRITICAL(&roomSetpointMux);
    RoomSaveResult& result = roomSaveResults[roomRequest.id % 8];
    if (result.id == roomRequest.id) result.state = saved ? 2 : 3;
    portEXIT_CRITICAL(&roomSetpointMux);
  }
  portENTER_CRITICAL(&webSettingsMux);
  const int webMode = requestedWebMode;
  requestedWebMode = -1;
  const KachelofenThresholdRequest thresholdRequest = requestedKachelofenThresholds;
  requestedKachelofenThresholds.pending = false;
  portEXIT_CRITICAL(&webSettingsMux);
  if (thresholdRequest.pending) {
    kachelofenEinTemp = thresholdRequest.ein;
    kachelofenAusTemp = thresholdRequest.aus;
    EEPROM.put(EEADDRESS_KACHELOFEN_EIN, kachelofenEinTemp);
    EEPROM.put(EEADDRESS_KACHELOFEN_AUS, kachelofenAusTemp);
    if (!EEPROM.commit()) Serial.println("Kachelofen-Schaltschwellen: Speichern fehlgeschlagen");
    updateKachelofenStatus();
  }
  if (webMode >= 0) {
    regelungsModus = static_cast<RegelungsModus>(webMode);
    updateKachelofenStatus();
  }
  enforceKesselSensorLock();
  enforceVorlaufSensorLock();
  publishBrennerSensorLock();
  publishSensorStates();
  static unsigned long lastWsCleanup = 0;
    const unsigned long wsNow = millis();
    if (wsNow - lastWsCleanup >= 1000UL) {
      lastWsCleanup = wsNow;
      ws.cleanupClients();
    }
  unsigned long currentTime;
  updateKachelofenStatus();
  SetOutPin(); // Enforce pump fault deadlines on every loop pass.
  MischerInit();
  if(WinterBetrieb){
    Automatik();
    Betriebsart='A';
  }else if(BoilerBetrieb){
    Boilerbetrieb();
    Betriebsart='B';
  }else if (NurHeizung) {
    Betriebsart='H';
    Heizungsbetrieb();
  }else {
    //HeizungAus
    Betriebsart='0';
    kein_Betrieb();
  }
  if(connect2wifi){
    connect2wifi=false;
    WiFi.disconnect();
    WiFi.begin(ssid,password);
  }
  if(connect2mqtt){
    connect2mqtt=false;
    Serial.println("MQTT-Verbindungsversuch");
    asyncMqttClient.connect();
  }
  if(set2myuhr){
    set2myuhr=false;
    myuhr();
  }
  if(mqtt2update){
    mqtt2update=false;
      mqttupdate();
  }
  esp_task_wdt_reset(); //watchdog Zeit rücksetzen
  uptime();//um zu wissen, wie lange der Arduino durchläuft
  KeyPad();
  Serial_Read();
  currentTime = millis();
  esp_task_wdt_reset(); //watchdog Zeit rücksetzen
  if(readSensor == false){
    //ds_alt    readTturePt1(tture[bWhichSensor]);//N.B.: Values passed back in globals
    if (bWhichSensor==BOILER_NUMBER){readTturePt1(tture[bWhichSensor]);}
    else{
      sensorDS1820_indicateChip(bWhichSensor);
      sensorDS1820_reset(bWhichSensor);
   } //ds_neu
    // Conversion time starts when the command has actually been sent.
    previousTime = millis();
    currentTime = previousTime;
    readSensor = true;
  }
  if (readSensor && (unsigned long)(currentTime - previousTime) > ANSWER_TIME){ //delay 1900
    if (lcd_display_light){
      lcd.setBacklight(HIGH);
      lcd_display_light--;
    }else{
       lcd.setBacklight(LOW);
    }
    previousTime = currentTime;
//ds_alt      readTturePt2(tture[bWhichSensor],bWhichSensor);//N.B.: Values passed back in globals
//ds_alt      printTture();//N.B.: Takes values from globals.
if (bWhichSensor==BOILER_NUMBER){
readTturePt2(tture[bWhichSensor],bWhichSensor);
}
else {sensorDS1820_read(bWhichSensor);} //ds_neu
      delay(5);//war 50
      bWhichSensor++;
      readSensor = false;
      if (wait_for_connect>0){
        wait_for_connect--;
        if (!MenuPage) {
          Serial.printf("Zeit: %s\n", TimeString);
        }
      }
      if (jumptoDefault){
        if (jumptoDefault==1){
          memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
          jumptoDefault=0;
          MenuPage=0;
          if (jump==1)jump=0;
          MenueDefault();
        }else{
          jumptoDefault--;
        }
      }
      switch (Betriebsart) {
        case 'A':
          Automatik();
          break;
        case 'B':
          Boilerbetrieb();
          break;
        case 'H':
          Heizungsbetrieb();
          break;
        case '0':
          // Anlage ausschalten
          kein_Betrieb();
          //Serial.print("Anlage ist ausgeschaltet\n");
          break;
      }
    }
   if(bWhichSensor == (kTtureSensorMaxIndex+1)){
      bWhichSensor = 0;
      if(!MenuPage)Serial.print("\n");//Start new line
   }
   if (currentTime - previousTime_wifi > WIFI_RECON_TIMER && wifiRecon){
      previousTime_wifi = currentTime;
      wifiRecon=false;
      mqttRecon=true;
      connectToWifi();
   }
   if (currentTime - previousTime_mqtt > MQTT_RECON_TIMER && mqttRecon){
      previousTime_mqtt = currentTime;
      mqttRecon=false;
      connectToMqtt();
   }
   if (currentTime - previousTime1 > NTP_UPDATE){
     previousTime1 = currentTime;
     if(!MenuPage)print_Uptime();
   }
   if (myhours==PUMPENL_HR && myminutes >= PUMPENL_MIN_B && myminutes <=PUMPENL_MIN_E){
     Pumpenloesen=1;
   }else{
     Pumpenloesen=0;
   }
   timer0_time.update();
   timer1_2lcd.update();
   timer2_ntp.update();
   timer3_mqttupdate.update();
  esp_task_wdt_reset(); //watchdog Zeit wieder rücksetzen
  updateCpuLoad();
  refreshWebStatus();
}//end of loop()






/* *******************************************************************************************************
                                         KeyPad
******************************************************************************************************* */
