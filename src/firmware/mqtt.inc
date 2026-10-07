void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  mqtt_message++;
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void applyMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total, unsigned long receivedAt){
  if (!topic || !payload || index != 0 || len != total || len == 0 || len >= 64 ||
      memchr(payload, '\0', len) != nullptr) {
    Serial.println("MQTT: ungueltige oder aufgeteilte Nachricht verworfen");
    return;
  }
/*    /SmartHome/Keller/Heizung/setRaumTemp
in FHEM:   set MQTT_SERVER publish /SmartHome/Keller/Heizung/setRaumTemp up
           set MQTT_SERVER publish /SmartHome/Keller/Heizung/setRaumTemp down
*/
  if (strcmp(topic, GAS_MQTT_TOPIC) == 0) {
    char text[64];
    memcpy(text, payload, len);
    text[len] = '\0';
    char* end = nullptr;
    const double value = strtod(text, &end);
    if (end == text || *end != '\0' || !isfinite(value) || value < 0 || value > 1e12) {
      Serial.println("Gaszaehler: ungueltigen Stand verworfen");
      return;
    }
    portENTER_CRITICAL(&gasMux);
    pendingGasTotal = value;
    pendingGasRetained = properties.retain;
    pendingGas = true;
    portEXIT_CRITICAL(&gasMux);
    return;
  }
  // Kachelofen-Temperatur: eigenes Topic, Payload ist nur die Temperatur (z.B. 65.4).
  if(strcmp(topic, KACHELOFEN_MQTT_TOPIC) == 0){
    if(index == 0 && len == total && len > 0 && len < 16){
      char tempPayload[16];
      memcpy(tempPayload, payload, len);
      tempPayload[len] = '\0';
      char* endPtr = nullptr;
      float newTemp = strtof(tempPayload, &endPtr);
      if(endPtr != tempPayload && *endPtr == '\0' && isfinite(newTemp) && newTemp >= -40.0 && newTemp <= 200.0){
        tKachelofen = newTemp;
        kachelofenLastUpdate = receivedAt;
        updateKachelofenStatus();
        Serial.print("Kachelofen MQTT: ");
        Serial.print(tKachelofen, 1);
        Serial.print(" C, aktiv=");
        Serial.println(kachelofenAktiv ? "JA" : "NEIN");
      }else{
        Serial.println("Kachelofen MQTT: ungueltiger Temperaturwert verworfen");
      }
    }
    return;
  }

  mqtt_message++;
  const String controlTopic = String(MQTT_TEXT) + "set";
  if (strcmp(topic, controlTopic.c_str()) != 0) return;
  char new_payload[64];
  memcpy(new_payload, payload, len);
  new_payload[len] = '\0';

    Serial.print("\nReceived message [");
    Serial.print(topic);
    Serial.print("] ");
  //  payload[len] = '\0';

    if(strcmp(new_payload,"up")==0){
        tRoomTag += 0.2; //RaumTemperatur um 0.2 *C erhöhen
        tRoomNacht += 0.2;
        Serial.print("MQTT Message UP ->");
        Serial.print(tRoomTag);
        strcpy(mqtt_payload,"h_up");
        return;
    }
    if(strcmp(new_payload,"down")==0){
        tRoomTag -= 0.2; //RaumTemperatur um 0.2 *C senken
        tRoomNacht -= 0.1;
        Serial.print("MQTT Message DOWN ->");
        Serial.print(tRoomTag);
        strcpy(mqtt_payload,"h_down");
        return;
    }
    if(strcmp(new_payload,"save")==0){
        tRoomTag -= 2.0; //RaumTemperatur um 2.0 *C senken
        tRoomNacht -= 2.0;
        Serial.print("MQTT Message SAVE ->");
        Serial.print(tRoomTag);
        strcpy(mqtt_payload,"h_save");
        return;
    }
    if(strcmp(new_payload,"normal")==0){
        EEPROM.get( EEADDRESS_RAUM, tRoomTag );//RaumTemperatur vom eeprom holen
        EEPROM.get( EEADDRESS_RAUMNACHT, tRoomNacht );//RaumTemperatur vom eeprom holen
        Serial.print("MQTT Message NORMAL ->");
        Serial.print(tRoomTag);
        strcpy(mqtt_payload,"h_norm");
        return;
    }
    if(strcmp(new_payload,"party")==0){
        tRoomTag += 2.0; //RaumTemperatur um 0.2 *C senken
        tRoomNacht += 2.0;
        Serial.print("MQTT Message PARTY ->");
        Serial.print(tRoomTag);
        strcpy(mqtt_payload,"h_party");
        return;
    }
    if(strcmp(new_payload,"setoff")==0){
        BoilerBetrieb=0;
        NurHeizung=0;
        WinterBetrieb=0;
        EEPROM.put( EEADDRESS_BOILER_SOMMERBETRIEB, BoilerBetrieb );
        EEPROM.put( EEADDRESS_NUR_HEIZUNG, NurHeizung );
        EEPROM.put( EEADDRESS_WINTER, WinterBetrieb );
        strcpy(mqtt_payload,"h_aus");
        return;
    }
    if(strcmp(new_payload,"seton")==0){
        BoilerBetrieb=0;
        NurHeizung=0;
        WinterBetrieb=1;
        EEPROM.put( EEADDRESS_BOILER_SOMMERBETRIEB, BoilerBetrieb );
        EEPROM.put( EEADDRESS_NUR_HEIZUNG, NurHeizung );
        EEPROM.put( EEADDRESS_WINTER, WinterBetrieb );
        strcpy(mqtt_payload,"h_ein");
        return;
    }
    if(strcmp(new_payload,"setboiler")==0){
        Betriebsart='B';
        WinterBetrieb=0;
        BoilerBetrieb=1;
        NurHeizung=0;
        EEPROM.put( EEADDRESS_BOILER_SOMMERBETRIEB, BoilerBetrieb );
        EEPROM.put( EEADDRESS_NUR_HEIZUNG, NurHeizung );
        EEPROM.put( EEADDRESS_WINTER, WinterBetrieb );
        strcpy(mqtt_payload,"h_boiler");
        return;
    }
    if(strcmp(new_payload,"setwinter")==0){
        Betriebsart='A';
        WinterBetrieb=1;
        BoilerBetrieb=0;
        NurHeizung=0;
        EEPROM.put( EEADDRESS_BOILER_SOMMERBETRIEB, BoilerBetrieb );
        EEPROM.put( EEADDRESS_NUR_HEIZUNG, NurHeizung );
        EEPROM.put( EEADDRESS_WINTER, WinterBetrieb );
        strcpy(mqtt_payload,"h_winter");
        return;
    }
    if(strcmp(new_payload,"setheizung")==0){
        Betriebsart='H';
        WinterBetrieb=0;
        BoilerBetrieb=0;
        NurHeizung=1;
        EEPROM.put( EEADDRESS_BOILER_SOMMERBETRIEB, BoilerBetrieb );
        EEPROM.put( EEADDRESS_NUR_HEIZUNG, NurHeizung );
        EEPROM.put( EEADDRESS_WINTER, WinterBetrieb );
        strcpy(mqtt_payload,"h_heizung");
        return;
    }
    if(strcmp(new_payload,"ntpupdate")==0){
        ntpupdate();
        strcpy(mqtt_payload,"h_ntpup");
        return;
    }
    else if(strcmp(new_payload,"tin")==0){
        regelungsModus = REGELUNG_RAUM;
        updateKachelofenStatus();
        strcpy(mqtt_payload,"h_tin");
        return;
    }
    if(strcmp(new_payload,"tau")==0){
        regelungsModus = REGELUNG_AUSSEN;
        updateKachelofenStatus();
        strcpy(mqtt_payload,"h_tau");
        return;
    }
    if(strcmp(new_payload,"tauto")==0){
        regelungsModus = REGELUNG_AUTO;
        updateKachelofenStatus();
        strcpy(mqtt_payload,"h_tauto");
        return;
    }
    if(strstr(new_payload,"kachelofenEin:") == new_payload){
        const char* numeric = new_payload + 14;
        char* endPtr = nullptr;
        const float tempTemp = strtof(numeric, &endPtr);
        if (endPtr == numeric || *endPtr != '\0' || !isfinite(tempTemp) ||
            tempTemp < 10.0f || tempTemp > 150.0f) {
          Serial.println("MQTT: ungueltiger Zahlenwert verworfen");
          return;
        }
        if(isfinite(tempTemp) && tempTemp >= 10.0 && tempTemp <= 150.0 && tempTemp > kachelofenAusTemp){
          kachelofenEinTemp = tempTemp;
          EEPROM.put( EEADDRESS_KACHELOFEN_EIN, kachelofenEinTemp );
          EEPROM.commit();
          updateKachelofenStatus();
          strcpy(mqtt_payload,"h_kof_ein");
          Serial.print("Kachelofen EIN neu: ");
          Serial.println(kachelofenEinTemp, 1);
        }else{
          Serial.println("MQTT kachelofenEin: ungueltiger Wert");
        }
        return;
    }
    if(strstr(new_payload,"kachelofenAus:") == new_payload){
        const char* numeric = new_payload + 14;
        char* endPtr = nullptr;
        const float tempTemp = strtof(numeric, &endPtr);
        if (endPtr == numeric || *endPtr != '\0' || !isfinite(tempTemp) ||
            tempTemp < 0.0f || tempTemp > 140.0f) {
          Serial.println("MQTT: ungueltiger Zahlenwert verworfen");
          return;
        }
        if(isfinite(tempTemp) && tempTemp >= 0.0 && tempTemp <= 140.0 && tempTemp < kachelofenEinTemp){
          kachelofenAusTemp = tempTemp;
          EEPROM.put( EEADDRESS_KACHELOFEN_AUS, kachelofenAusTemp );
          EEPROM.commit();
          updateKachelofenStatus();
          strcpy(mqtt_payload,"h_kof_aus");
          Serial.print("Kachelofen AUS neu: ");
          Serial.println(kachelofenAusTemp, 1);
        }else{
          Serial.println("MQTT kachelofenAus: ungueltiger Wert");
        }
        return;
    }
    if(strncmp(new_payload,"newRoomTemp:", 12) == 0){ //Raumtemperatur per mqtt setzen
        const char* numeric = new_payload + 12;
        char* endPtr = nullptr;
        const float tempTemp = strtof(numeric, &endPtr);
        if (endPtr == numeric || *endPtr != '\0' || !isfinite(tempTemp) ||
            tempTemp < 5.0f || tempTemp > 40.0f) {
          Serial.println("MQTT: ungueltiger Zahlenwert verworfen");
          return;
        }
        tRoomTag = tempTemp;
        tRoomNacht = tempTemp;
        strcpy(mqtt_payload,"h_newRT");
        return;
    }
    if(strncmp(new_payload,"newBoilerTemp:", 14) == 0){ //Raumtemperatur per mqtt setzen
        const char* numeric = new_payload + 14;
        char* endPtr = nullptr;
        const float tempTemp = strtof(numeric, &endPtr);
        if (endPtr == numeric || *endPtr != '\0' || !isfinite(tempTemp) ||
            tempTemp < 0.0f || tempTemp > 100.0f) {
          Serial.println("MQTT: ungueltiger Zahlenwert verworfen");
          return;
        }
        tBoilerDest = tempTemp;
        EEPROM.put( EEADDRESS_BOILER, tBoilerDest );
        strcpy(mqtt_payload,"h_newBT");
        return;
    }
    if(strncmp(new_payload,"tvmax:", 6) == 0){ //max. Vorlauftemperatur  per mqtt setzen
        const char* numeric = new_payload + 6;
        char* endPtr = nullptr;
        const float tempTemp = strtof(numeric, &endPtr);
        if (endPtr == numeric || *endPtr != '\0' || !isfinite(tempTemp) ||
            tempTemp < 0.0f || tempTemp > 100.0f) {
          Serial.println("MQTT: ungueltiger Zahlenwert verworfen");
          return;
        }
        tvmax = tempTemp;
        strcpy(mqtt_payload,"h_tvmax");
        return;
    }
    if(strncmp(new_payload,"taumin:", 7) == 0){ //minimal Aussentemp per mqtt setzen
        const char* numeric = new_payload + 7;
        char* endPtr = nullptr;
        const float tempTemp = strtof(numeric, &endPtr);
        if (endPtr == numeric || *endPtr != '\0' || !isfinite(tempTemp) ||
            tempTemp < -50.0f || tempTemp > 0.0f) {
          Serial.println("MQTT: ungueltiger Zahlenwert verworfen");
          return;
        }
        taumin = tempTemp;
        strcpy(mqtt_payload,"h_tvmin");
        return;
    }
    if(strncmp(new_payload,"tn:", 3) == 0){ //Steigung per mqtt setzen
        const char* numeric = new_payload + 3;
        char* endPtr = nullptr;
        const float tempTemp = strtof(numeric, &endPtr);
        if (endPtr == numeric || *endPtr != '\0' || !isfinite(tempTemp) ||
            tempTemp < 0.1f || tempTemp > 10.0f) {
          Serial.println("MQTT: ungueltiger Zahlenwert verworfen");
          return;
        }
        n = tempTemp;
        strcpy(mqtt_payload,"h_tn");
        return;
    }
}


void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties,
                   size_t len, size_t index, size_t total) {
  if (!topic || !payload || index != 0 || len != total || len == 0 || len >= 64 ||
      memchr(payload, '\0', len) != nullptr) {
    Serial.println("MQTT: ungueltige oder aufgeteilte Nachricht verworfen");
    return;
  }
  if (strcmp(topic, MQTT_TEXT "set") != 0 &&
      strcmp(topic, KACHELOFEN_MQTT_TOPIC) != 0 && strcmp(topic, GAS_MQTT_TOPIC) != 0) return;
  const size_t topicLength = strnlen(topic, 128);
  if (topicLength >= 128) return;
  PendingMqttMessage message{};
  memcpy(message.topic, topic, topicLength + 1);
  memcpy(message.payload, payload, len);
  message.payload[len] = '\0';
  message.length = len;
  message.properties = properties;
  message.receivedAt = millis();
  if (!mqttMessageQueue || xQueueSend(mqttMessageQueue, &message, 0) != pdTRUE) {
    Serial.println("MQTT: Empfangswarteschlange voll oder nicht verfuegbar, Nachricht verworfen");
  }
}

void processMqttMessages() {
  if (!mqttMessageQueue) return;
  PendingMqttMessage message;
  // Bound each pass, so a stream of commands cannot starve the controller loop.
  for (unsigned i = 0; i < MQTT_QUEUE_LENGTH; ++i) {
    if (xQueueReceive(mqttMessageQueue, &message, 0) != pdTRUE) break;
    applyMqttMessage(message.topic, message.payload, message.properties,
                     message.length, 0, message.length, message.receivedAt);
  }
}

/* *******************************************************************************************************
                                         mqtt clearstring
******************************************************************************************************* */
/*void clearstring() {
  //Serial.flush(); // clears the buffer, you dont need this
  for (int r=0; r<7; r++){
  my_str[r] = '\0'; // deletes each block
  }
}
*/

/* *******************************************************************************************************
                                         mqtt reconnect
******************************************************************************************************* */
void onMqttConnect(bool sessionPresent) {
      const String subsc = String(MQTT_TEXT) + "set";
      Serial.println("connect to MQTT\n");
      Serial.print("Session present: ");
      Serial.println(sessionPresent);
      asyncMqttClient.subscribe(subsc.c_str(),1);
      asyncMqttClient.subscribe(KACHELOFEN_MQTT_TOPIC,1);
      asyncMqttClient.subscribe(GAS_MQTT_TOPIC,1);
      //uint16_t packetIdSub = asyncMqttClient.subscribe("/SmartHome/Keller/Heizung/setRaumTemp", 2);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.printf("MQTT-Abbruchgrund: %u\n",
              static_cast<unsigned>(reason));
  if (!WiFi.isConnected()){
    WiFi.disconnect();
    WiFi.begin(ssid,password);
  }
        mqttRecon=true;
        previousTime_mqtt=millis();
}
/* *******************************************************************************************************
                                         print LCD Values
******************************************************************************************************* */
