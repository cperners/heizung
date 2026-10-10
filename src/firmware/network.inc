void WiFiEvent(WiFiEvent_t event) {
    Serial.printf("[WiFi-event] event: %d\n", event);
    switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        connectToMqtt();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("WiFi lost connection");
        wifiRecon=true;
        mqttRecon=false;
        previousTime_wifi=millis();
        break;
    case SYSTEM_EVENT_WIFI_READY: 
        break;
    case SYSTEM_EVENT_SCAN_DONE:
        break;
    case SYSTEM_EVENT_STA_START:
        break;
    case SYSTEM_EVENT_STA_STOP:
        break;
    case SYSTEM_EVENT_STA_CONNECTED:
        break;
    case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
        break;
    case SYSTEM_EVENT_STA_LOST_IP:
        wifiRecon=true;
        mqttRecon=false;
        previousTime_wifi=millis();
        break;
    case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:
        break;
    case SYSTEM_EVENT_STA_WPS_ER_FAILED:
        break;
    case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
        break;
    case SYSTEM_EVENT_STA_WPS_ER_PIN:
        break;
    case SYSTEM_EVENT_AP_START:
        break;
    case SYSTEM_EVENT_AP_STOP:
        break;
    case SYSTEM_EVENT_AP_STACONNECTED:
        break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
        break;
    case SYSTEM_EVENT_AP_STAIPASSIGNED:
        break;
    case SYSTEM_EVENT_AP_PROBEREQRECVED:
        break;
    case SYSTEM_EVENT_GOT_IP6:
        break;
    case SYSTEM_EVENT_ETH_START:
        break;
    case SYSTEM_EVENT_ETH_STOP:
        break;
    case SYSTEM_EVENT_ETH_CONNECTED:
        break;
    case SYSTEM_EVENT_ETH_DISCONNECTED:
        break;
    case SYSTEM_EVENT_ETH_GOT_IP:
        break;
    default: 
        break; 
    }
}
void connectToMqtt() {
 connect2mqtt=true; 
}
/* *******************************************************************************************************
                                         NTP update
******************************************************************************************************* */
void set2mqttupdate(){
  mqtt2update=true;
}
void mqttupdate() {
  if (!asyncMqttClient.connected()) {
    Serial.println("MQTT JSON: keine Verbindung");
    return;
  }
  String bez;
  char msg[50];
  char line0[48];
  char buffer[2048];
  JsonDocument doc;
  doc["S0Kessel"].set(tKessel);
  doc["S1Vorlauf"].set(tVorlauf);
  doc["S2Aussen"].set(tAussen);
  doc["S3Kueche"].set(tRoom);
  doc["S4Boiler"].set(tBoiler);
  doc["T0Room"].set(tmyRoomdest);
  doc["T1Boiler"].set(tBoilerDest);
  doc["T2Vorlauf"].set(vorlaufTemperatur);
  snprintf(line0, sizeof(line0), "%ld,%02d:%02d:%02d", uDay,uHour,uMinute,uSecond);
  doc["U0Uptime"].set(line0);
  snprintf(line0, sizeof(line0), "%ld:%02d:%02d", brhours,brminutes,brsecunds);
  doc["B0Brenner"].set(line0);
  snprintf(line0, sizeof(line0), "%02d:%02d", TagBeginHr,TagBeginMi);
  doc["Day"].set(line0);
  snprintf(line0, sizeof(line0), "%02d:%02d", NachtBeginHr,NachtBeginMi);
  doc["Night"].set(line0);
  snprintf(line0, sizeof(line0), "%s", BrennerRelais ? "1" : "0");
  doc["BrennerRelais"].set(line0);
  snprintf(line0, sizeof(line0), "%s", HeizungsRelais ? "1" : "0");
  doc["HeizungsRelais"].set(line0);
  snprintf(line0, sizeof(line0), "%s", BoilerRelais ? "1" : "0");
  doc["BoilerRelais"].set(line0);
  snprintf(line0, sizeof(line0), "%s", MischerAufRelais ? "1" : "0");
  doc["MischerAufRelais"].set(line0);
  snprintf(line0, sizeof(line0), "%s", MischerZuRelais ? "1" : "0");
  doc["MischerZuRelais"].set(line0);
  snprintf(line0, sizeof(line0), "%02d.%02d.%4d %02d:%02d", myday,mymonth,myyear,myhours,myminutes);
  doc["Uhrzeit"].set(line0);
  doc["WlanRetry"].set(wifi_retry);
  doc["WlanRSSI"].set(WiFi.RSSI());
  int mqttRegelungsArt = -1;
  switch (regelungsModus) {
    case REGELUNG_RAUM: mqttRegelungsArt = 0; break;
    case REGELUNG_AUSSEN: mqttRegelungsArt = 1; break;
    case REGELUNG_AUTO: mqttRegelungsArt = 2; break;
    default: break;
  }
  doc["atRegelung"].set(mqttRegelungsArt);
  doc["PumpenNachlauf"].set(PumpenNachlauf);
  doc["tvmax"].set(tvmax);
  doc["taumin"].set(taumin);
  doc["n"].set(n);
  if(strcmp(mqtt_payload,"")==0){
    doc["answer"].set("---");
  }
    else{
    snprintf(line0, sizeof(line0), "%s", mqtt_payload);
    strcpy(mqtt_payload,"");
    doc["answer"].set(line0);
  }
  snprintf(line0, sizeof(line0), "%c" ,Betriebsart);
  doc["betriebsart"].set(line0);
  if (doc.overflowed() || measureJson(doc) >= sizeof(buffer)) {
    Serial.println("MQTT: Statusnachricht zu gross");
    return;
  }
  const size_t payloadSize = serializeJson(doc, buffer, sizeof(buffer));
//  serializeJsonPretty(doc, buffer);
  bez = bez + MQTT_TEXT + "JDATA";
  bez.toCharArray(msg,50);
  const uint16_t jsonPacketId = asyncMqttClient.publish(msg, 1, true, buffer, payloadSize);
  Serial.printf("MQTT JSON: %u Bytes, Versand %s\n", static_cast<unsigned>(payloadSize),
                jsonPacketId ? "eingereiht" : "abgelehnt");
}
/*
void mqttupdate() {
  String bez;
  char msg[50];
  char res[8];
  char line0[20];
    bez = bez + MQTT_TEXT + "S0" + "Kessel";
    snprintf(res, sizeof(res), "%6.2f", static_cast<double>(tKessel));
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1,true, res);
    bez="";
    bez = bez + MQTT_TEXT + "S1" + "Vorlauf";
    snprintf(res, sizeof(res), "%6.2f", static_cast<double>(tVorlauf));
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, res);
    bez="";
    bez = bez + MQTT_TEXT + "S2" + "Aussen";
    snprintf(res, sizeof(res), "%6.2f", static_cast<double>(tAussen));
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, res);
    bez="";
    bez = bez + MQTT_TEXT + "S3" + "Kueche";
    snprintf(res, sizeof(res), "%6.2f", static_cast<double>(tRoom));
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, res);
    bez="";
    bez = bez + MQTT_TEXT + "S4" + "Boiler";
    snprintf(res, sizeof(res), "%6.2f", static_cast<double>(tBoiler));
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, res);
    bez="";
    bez = bez + MQTT_TEXT + "T0" + "Room";
    snprintf(res, sizeof(res), "%6.2f", static_cast<double>(tmyRoomdest));
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, res);
    bez="";
    bez = bez + MQTT_TEXT + "U0" + "Uptime";
    snprintf(line0, sizeof(line0), "%ld,%02d:%02d:%02d", uDay,uHour,uMinute,uSecond);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "B0" + "Brenner";
    snprintf(line0, sizeof(line0), "%ld:%02d:%02d", brhours,brminutes,brsecunds);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "Day";
    snprintf(line0, sizeof(line0), "%02d:%02d", TagBeginHr,TagBeginMi);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "Night";
    snprintf(line0, sizeof(line0), "%02d:%02d", NachtBeginHr,NachtBeginMi);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "BrennerRelais";
    snprintf(line0, sizeof(line0), "%s", BrennerRelais ? "1" : "0");
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "HeizungsRelais";
    snprintf(line0, sizeof(line0), "%s", HeizungsRelais ? "1" : "0");
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "BoilerRelais";
    snprintf(line0, sizeof(line0), "%s", BoilerRelais ? "1" : "0");
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "Uhrzeit";
    snprintf(line0, sizeof(line0), "%02d.%02d.%4d %02d:%02d", myday,mymonth,myyear,myhours,myminutes);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "WlanRetryConnect";
    snprintf(line0, sizeof(line0), "%d", wifi_retry);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "WlanRSSI";
    snprintf(line0, sizeof(line0), "%d", WiFi.RSSI());
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "vorlaufTemperatur";
    snprintf(line0, sizeof(line0), "%.2f", vorlaufTemperatur);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "atRegelung";
    snprintf(line0, sizeof(line0), "%d", AussentemperaturRegelung);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
//    bez="";
//    bez = bez + MQTT_TEXT + "WlanAvgTimeMs";
//    snprintf(line0, sizeof(line0), "%d", avg_time_ms);
//    bez.toCharArray(msg,50);
//    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "PumpenNachlauf";
    snprintf(line0, sizeof(line0), "%d", PumpenNachlauf);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
//    bez="";
//    bez = bez + MQTT_TEXT + "MischerAuf";
//    snprintf(line0, sizeof(line0), "%d", MischerAufRelais);
//    bez.toCharArray(msg,50);
//    asyncMqttClient.publish(msg,1, true, line0);
//    bez="";
//    bez = bez + MQTT_TEXT + "MischerZu";
//    snprintf(line0, sizeof(line0), "%d", MischerZuRelais);
//    bez.toCharArray(msg,50);
//    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "tvmax";
    snprintf(line0, sizeof(line0), "%.2f", tvmax);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "taumin";
    snprintf(line0, sizeof(line0), "%.2f", taumin);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    bez="";
    bez = bez + MQTT_TEXT + "n";
    snprintf(line0, sizeof(line0), "%.2f", n);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
//    bez="";
//    bez = bez + MQTT_TEXT + "dataToI2Cextender";
//    snprintf(line0, sizeof(line0), "%d", dataToI2C);
//    bez.toCharArray(msg,50);
//    asyncMqttClient.publish(msg,1, true, line0);
    if(strcmp(mqtt_payload,"")==0){
    }
    else{
      bez="";
      bez = bez + MQTT_TEXT + "answer"; //Antwort nach Subscribe Message
      snprintf(line0, sizeof(line0), "%s", mqtt_payload);
      strcpy(mqtt_payload,"");
      bez.toCharArray(msg,50);
      asyncMqttClient.publish(msg,1, true, line0);
    }
    bez="";
    bez = bez + MQTT_TEXT + "Betriebsart";
    snprintf(line0, sizeof(line0), "%c" ,Betriebsart);
    bez.toCharArray(msg,50);
    asyncMqttClient.publish(msg,1, true, line0);
    esp_task_wdt_reset(); //watchdog Zeit rücksetzen
}
*/
/* *******************************************************************************************************
                                         WLAN-MQTT Connecting
******************************************************************************************************* */
void connectToWifi() {
  //WiFi.begin(SECRET_SSID, SECRET_PASS);
  connect2wifi=true;
}

void connect() {
  if(wait_for_connect==0){
    if(WiFi.status() != WL_CONNECTED) {
      WiFi.begin(ssid, password);
     if(WiFi.status() == WL_CONNECTED) {
      if(!MenuPage){
        Serial.print("Wifi connection successful - IP-Address: ");
        Serial.println(WiFi.localIP());
      }
     }
     else {
      wait_for_connect=100;
    }
   }
  }//if waitForConnectResult
}
/* *******************************************************************************************************
                                         ds18b20
******************************************************************************************************* */
