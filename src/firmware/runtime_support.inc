void serviceWifiReconnect() {
  static bool attempted = false;
  static unsigned long lastAttempt = 0;
  const unsigned long now = millis();
  bool due = false;
  portENTER_CRITICAL(&wifiReconnectMux);
  if (wifiRecon && (unsigned long)(now - previousTime_wifi) >= WIFI_RECON_TIMER &&
      (!attempted || (unsigned long)(now - lastAttempt) >= 15000UL)) {
    wifiRecon = false;
    due = true;
  }
  portEXIT_CRITICAL(&wifiReconnectMux);
  if (!due) return;
  // A late request must never disconnect a connection that already recovered.
  if (WiFi.status() == WL_CONNECTED && static_cast<uint32_t>(WiFi.localIP()) != 0) return;
  portENTER_CRITICAL(&wifiReconnectMux);
  ++wifiReconnectCount;
  portEXIT_CRITICAL(&wifiReconnectMux);
  attempted = true;
  lastAttempt = now;
  // Leave an in-progress connection time to complete before the next attempt.
  // Arm the next attempt even if the driver produces no new disconnect event.
  connectToWifi();
  WiFi.reconnect();
}

bool saveBoilerSetpoint(float value, bool hasDifference = false, float difference = 0) {
  if (!isfinite(value) || value < 0 || value > 100) return false;
  if (hasDifference && (!isfinite(difference) || difference < 0 || difference > 100)) return false;
  if (value == tBoilerDest && (!hasDifference || difference == tBoilerDiff)) return true;
  const float previous = tBoilerDest, previousDifference = tBoilerDiff;
  if (hasDifference) EEPROM.put(EEADDRESS_DIFFBOILER, difference);
  EEPROM.put(EEADDRESS_BOILER, value);
  if (!EEPROM.commit()) {
    EEPROM.put(EEADDRESS_BOILER, previous);
    if (hasDifference) EEPROM.put(EEADDRESS_DIFFBOILER, previousDifference);
    return false;
  }
  tBoilerDest = value;
  if (hasDifference) tBoilerDiff = difference;
  return true;
}

bool saveFlowLimit(float value, bool hasCurve = false, float curve = 0,
                   bool hasRecovery = false, float recovery = 0) {
  if (!isfinite(value) || value < 0 || value > 100) return false;
  if (hasCurve && (!isfinite(curve) || curve < 0.1f || curve > 10)) return false;
  if (hasRecovery && (!isfinite(recovery) || recovery < 30 || recovery > 40)) return false;
  if (value == tvmax && (!hasCurve || curve == n) && (!hasRecovery || recovery == restwaermeZielTemp)) return true;
  const float previous = tvmax, previousCurve = n, previousRecovery = restwaermeZielTemp;
  if (hasRecovery) EEPROM.put(EEADDRESS_RESTWAERME_ZIEL, recovery);
  if (hasCurve) EEPROM.put(EEADDRESS_NN, curve);
  EEPROM.put(EEADDRESS_TVMAX, value);
  if (!EEPROM.commit()) {
    EEPROM.put(EEADDRESS_TVMAX, previous);
    if (hasCurve) EEPROM.put(EEADDRESS_NN, previousCurve);
    if (hasRecovery) EEPROM.put(EEADDRESS_RESTWAERME_ZIEL, previousRecovery);
    return false;
  }
  tvmax = value;
  if (hasCurve) n = curve;
  if (hasRecovery) restwaermeZielTemp = recovery;
  return true;
}

int currentGasDay() {
  const time_t utc = time(nullptr);
  if (utc < static_cast<time_t>(1577836800UL)) return 0;
  time_t local = utc + GMT_TIME_ZONE * utcOffsetInSeconds;
  struct tm calendar = {};
  if (!gmtime_r(&local, &calendar)) return 0;
  if (Sommerzeit_EinAus && summertime_EU(calendar.tm_year + 1900,
      calendar.tm_mon + 1, calendar.tm_mday, calendar.tm_hour, GMT_TIME_ZONE)) {
    local += 3600;
    if (!gmtime_r(&local, &calendar)) return 0;
  }
  return (calendar.tm_year + 1900) * 10000 + (calendar.tm_mon + 1) * 100 + calendar.tm_mday;
}

void processGasReading() {
  double value;
  bool retained;
  portENTER_CRITICAL(&gasMux);
  const bool available = pendingGas;
  value = pendingGasTotal;
  retained = pendingGasRetained;
  pendingGas = false;
  portEXIT_CRITICAL(&gasMux);
  const int day = currentGasDay();
  if (day != gasDayKey) gasDayReady = false;
  if (!available) return;
  if ((isfinite(gasTotal) && value < gasTotal) ||
      (day == gasDayKey && isfinite(gasDayBase) && value < gasDayBase)) {
    Serial.println("Gaszaehler: ruecklaeufigen Stand verworfen");
    return;
  }
  gasTotal = value;
  if (day == 0) return;
  if (day != gasDayKey || !isfinite(gasDayBase)) {
    // Ein alter Retain-Wert ist keine neue Tagesmessung.
    if (retained) return;
    gasDayKey = day;
    gasDayBase = value;
    if (gasStorageReady) {
      gasStorage.putDouble("base", gasDayBase);
      gasStorage.putInt("day", gasDayKey);
    }
  }
  gasDayReady = true;
}

String gasDisplay(bool daily) {
  if (!isfinite(gasTotal)) return "Noch keine Meldung";
  if (daily && (!gasDayReady || gasDayKey != currentGasDay())) return "Warte auf Tagesmessung";
  char text[32];
  snprintf(text, sizeof(text), "%.3f", daily ? gasTotal - gasDayBase : gasTotal);
  return String(text);
}

enum BoilerDiagnostic { BOILER_WAITING, BOILER_OK, BOILER_NO_RESPONSE, BOILER_ZERO_DATA, BOILER_HIGH_DATA,
                        BOILER_BAD_CRC, BOILER_BAD_VALUE };
volatile BoilerDiagnostic boilerDiagnostic = BOILER_WAITING;
byte boilerRawBytes[9] = {};
bool boilerRawAvailable = false;
portMUX_TYPE boilerDiagnosticMux = portMUX_INITIALIZER_UNLOCKED;


const char* boilerDiagnosticText() {
  switch (boilerDiagnostic) {
    case BOILER_NO_RESPONSE: return "Keine Sensorantwort bei Anwesenheitspruefung";
    case BOILER_ZERO_DATA: return "Datenblock durchgehend Null";
    case BOILER_HIGH_DATA: return "Datenblock durchgehend 255";
    case BOILER_BAD_CRC: return "Pruefsumme fehlerhaft";
    case BOILER_BAD_VALUE: return "Temperaturwert ungueltig";
    case BOILER_OK:
      return (unsigned long)(millis() - sensorLastSuccess[4]) > SENSOR_MAX_AGE_MS ?
        "Letzter gueltiger Messwert zu alt" : "Messung gueltig";
    default: return "Warte auf erste Boiler-Messung";
  }
}

String boilerDiagnosticDetails() {
  byte bytes[9];
  bool available;
  portENTER_CRITICAL(&boilerDiagnosticMux);
  memcpy(bytes, boilerRawBytes, sizeof(bytes));
  available = boilerRawAvailable;
  portEXIT_CRITICAL(&boilerDiagnosticMux);
  String text = boilerDiagnosticText();
  if (available) {
    char raw[40];
    snprintf(raw, sizeof(raw), "%02X %02X %02X %02X %02X %02X %02X %02X %02X",
             bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7], bytes[8]);
    text += " | Letzter gelesener Datenblock (Hex): ";
    text += raw;
  }
  return text;
}

bool sensorIsUsable(byte index) {
  return index < 5 && sensorReadingValid[index] &&
         millis() - sensorLastSuccess[index] <= SENSOR_MAX_AGE_MS;
}

// Status bei Aenderung und nach Wiederverbindung erneut senden.
void publishSensorStates() {
  if (mqttSettingsAreChanging()) return;
  static bool published[5] = {};
  static uint32_t publishedGeneration=0;
  if (publishedGeneration!=mqttTopicGeneration) { memset(published,0,sizeof(published)); publishedGeneration=mqttTopicGeneration; }
  static bool lastValid[5] = {};
  static float lastTemperature[5] = {};
  static unsigned long lastAttempt = 0;
  if (!asyncMqttClient.connected()) {
    for (byte i = 0; i < 5; ++i) published[i] = false;
    return;
  }
  if (millis() - lastAttempt < 1000UL) return;
  lastAttempt = millis();
  static String lastBoilerReason;
  const String boilerReason = boilerDiagnosticText();
  if (!published[4] || boilerReason != lastBoilerReason) {
    const String diagnosticTopic = String(MQTT_TEXT) + "sensordiagnose/boiler";
    if (asyncMqttClient.publish(diagnosticTopic.c_str(), 1, true, boilerReason.c_str()) != 0)
      lastBoilerReason = boilerReason;
  }
  const char* suffixes[5] = {"kessel", "vorlauf", "aussen", "raum", "boiler"};
  const float* values[5] = {&tKessel, &tVorlauf, &tAussen, &tRoom, &tBoiler};
  for (byte i = 0; i < 5; ++i) {
    const float temperature = *values[i];
    const bool valid = sensorIsUsable(i) && isfinite(temperature);
    if (published[i] && valid == lastValid[i] &&
        (!valid || temperature == lastTemperature[i])) continue;
    const String topic = String(MQTT_TEXT) + "sensorstatus/" + suffixes[i];
    char message[32];
    if (valid) snprintf(message, sizeof(message), "%.1f", temperature);
    else snprintf(message, sizeof(message), "SENSORFEHLER");
    if (asyncMqttClient.publish(topic.c_str(), 1, true, message) != 0) {
      published[i] = true;
      lastValid[i] = valid;
      lastTemperature[i] = temperature;
    }
  }
}

void publishBrennerSensorLock() {
  if (mqttSettingsAreChanging()) return;
  static bool published = false;
  static uint32_t publishedGeneration=0;
  if (publishedGeneration!=mqttTopicGeneration) { published=false; publishedGeneration=mqttTopicGeneration; }
  static bool lastLocked = false;
  static unsigned long lastAttempt = 0;
  if (!asyncMqttClient.connected()) { published = false; return; }
  const bool locked = !sensorIsUsable(0);
  if (published && locked == lastLocked) return;
  if (millis() - lastAttempt < 1000UL) return;
  lastAttempt = millis();
  const String topic = String(MQTT_TEXT) + "brennersperre";
  const char* message = locked ? "BRENNER GESPERRT - Kesselsensor ungueltig" :
                                 "Kesselsensor OK - keine Sensorsperre";
  if (asyncMqttClient.publish(topic.c_str(), 1, true, message) != 0) {
    lastLocked = locked;
    published = true;
  }
}

void enforceKesselSensorLock() {
  static bool lastLocked = false;
  const bool locked = !sensorIsUsable(0);
  if (locked) {
    BrennerRelais = false;
    digitalWrite(BrennerPin, LOW);
  }
  if (locked != lastLocked) {
    Serial.println(locked ? "Brenner gesperrt: Kesselsensor ungueltig" :
                            "Kesselsensor gueltig: Brennersperre aufgehoben");
    lastLocked = locked;
  }
}

bool validSensorScratchpad(const byte* data) {
  bool allZero = true;
  bool allHigh = true;
  for (byte i = 0; i < 9; ++i) {
    allZero &= (data[i] == 0);
    allHigh &= (data[i] == 0xff);
  }
  return !allZero && !allHigh && OneWire::crc8(data, 8) == data[8];
}

void markSensorFailure(byte index) {
  if (index >= 5) return;
  if (sensorReadingValid[index] || sensorLastSuccess[index] == 0)
    Serial.printf("Sensorfehler: %s\n", sensorNames[index]);
  sensorReadingValid[index] = false;
}

bool markSensorSuccess(byte index, float temperature) {
  if (index >= 5) return false;
  if (!isfinite(temperature) || temperature < -55.0f || temperature > 125.0f) {
    if (index == 4) boilerDiagnostic = BOILER_BAD_VALUE;
    markSensorFailure(index);
    return false;
  }
  if (!sensorReadingValid[index]) Serial.printf("Sensor wieder gueltig: %s\n", sensorNames[index]);
  if (index == 4) boilerDiagnostic = BOILER_OK;
  sensorReadingValid[index] = true;
  sensorLastSuccess[index] = millis();
  return true;
}

// Bestehende Einstellungen erhalten; uninitialisierte Werte einzeln reparieren.
template <typename T>
bool repairSetting(int address, T fallback, double minimum, double maximum) {
  T value;
  EEPROM.get(address, value);
  if (isfinite(static_cast<double>(value)) && value >= minimum && value <= maximum) return false;
  EEPROM.put(address, fallback);
  Serial.printf("Einstellung an Speicherstelle %d auf Standardwert gesetzt\n", address);
  return true;
}

bool repairSchedule(int address, float fallback) {
  float value;
  EEPROM.get(address, value);
  if (isfinite(value) && value >= 0.0f && value < 24.0f) {
    const int hours = static_cast<int>(value);
    const int minutes = static_cast<int>(roundf((value - hours) * 100.0f));
    if (minutes >= 0 && minutes <= 59) return false;
  }
  EEPROM.put(address, fallback);
  Serial.printf("Schaltzeit an Speicherstelle %d auf Standardwert gesetzt\n", address);
  return true;
}

void initializeStoredSettings() {
  bool changed = false;
  changed |= repairSetting(EEADDRESS_URLAUB_TEMP, URLAUB_DEFAULT_TEMP, 5, 40);
  changed |= repairSetting(EEADDRESS_RESTWAERME_ZIEL, float(MISCHER_AUS_KESSEL_MAX_TEMP), 30, 40);
  changed |= repairSetting(EEADDRESS_URLAUB_AKTIV, byte(0), 0, 1);
  changed |= repairSetting(EEADDRESS_BOILER, eeBoiler, 0, 100);
  changed |= repairSetting(EEADDRESS_RAUM, eeRaum, 5, 40);
  changed |= repairSetting(EEADDRESS_RAUMNACHT, eeRaumNacht, 5, 40);
  changed |= repairSetting(EEADDRESS_KESSEL, eeKessel, 0, 100);
  changed |= repairSetting(EEADDRESS_DIFFRAUM, eeDiffRaum, 0, 20);
  changed |= repairSetting(EEADDRESS_DIFFKESSEL, eeDiffKessel, 0, 100);
  changed |= repairSetting(EEADDRESS_DIFFBOILER, eeDiffBoiler, 0, 100);
  changed |= repairSetting(EEADDRESS_TVMAX, eetvmax, 0, 100);
  changed |= repairSetting(EEADDRESS_TAUMIN, eetaumin, 0, 50);
  changed |= repairSetting(EEADDRESS_NN, een, 0.1, 10);
  changed |= repairSetting(EEADDRESS_AUSSENTEMPREGELUNG, eeAuTempRegel, 0, 1);
  changed |= repairSetting(EEADDRESS_WINTER, eeWinter, 0, 1);
  changed |= repairSetting(EEADDRESS_BOILER_SOMMERBETRIEB, eeBoilerBetrieb, 0, 1);
  changed |= repairSetting(EEADDRESS_NUR_HEIZUNG, eeNurHeizung, 0, 1);
  changed |= repairSetting(EEADDRESS_SOMMERZEIT_EINAUS, eeSommerzeit_EinAus, 0, 1);
  changed |= repairSetting(EEADDRESS_BR_LAUFZEIT, eeBrennerLaufzeit, 0, 2147483647);
  changed |= repairSchedule(EEADDRESS_TAG, eeTag);
  changed |= repairSchedule(EEADDRESS_NACHT, eeNacht);
  if (changed && !EEPROM.commit()) Serial.println("Einstellungen konnten nicht gespeichert werden");
}


struct PendingMqttMessage {
  char topic[128];
  char payload[64];
  AsyncMqttClientMessageProperties properties;
  size_t length;
  unsigned long receivedAt;
};
constexpr unsigned MQTT_QUEUE_LENGTH = 8;
QueueHandle_t mqttMessageQueue = nullptr;


bool operatingModeStorageVerified = false;

String vorlaufCalculationDisplay(bool difference) {
  const bool curveSelected = offHeatRecoveryActive ||
      (!UrlaubBetrieb && (WinterBetrieb || NurHeizung));
  if (!curveSelected) return "Nicht aktiv";
  if (!sensorIsUsable(0) || !sensorIsUsable(1) || !sensorIsUsable(2) ||
      (!offHeatRecoveryActive && AussentemperaturRegelung == 0 && !sensorIsUsable(3)))
    return "Nicht verfuegbar - Sensorfehler";
  if (!heizkurveGueltig || !isfinite(vorlaufTemperatur) || !isfinite(tvmax) ||
      vorlaufTemperatur > tvmax || !isfinite(tVorlauf)) return "Keine gueltige Berechnung";
  char text[32];
  if (difference) snprintf(text, sizeof(text), "%+.2f \xC2\xB0" "C", static_cast<double>(tVorlauf-vorlaufTemperatur));
  else snprintf(text, sizeof(text), "%.2f \xC2\xB0" "C", static_cast<double>(vorlaufTemperatur));
  return String(text);
}

String boilerTemperatureDifferenceDisplay() {
  if (!sensorIsUsable(0) || !sensorIsUsable(4) || !isfinite(tKessel) || !isfinite(tBoiler)) {
    return "Nicht verfuegbar - Sensorfehler";
  }
  char text[32];
  snprintf(text, sizeof(text), "%+.2f \xC2\xB0" "C", static_cast<double>(tKessel-tBoiler));
  return String(text);
}

String boilerSettingDisplay() {
  if (!isfinite(tBoilerDest) || !isfinite(tBoilerDiff)) return "Boilereinstellungen ungueltig";
  char text[144];
  snprintf(text, sizeof(text), "Soll: %.1f \xC2\xB0" "C; Einschaltdifferenz: %.1f \xC2\xB0" "C; neue Ladung unter %.1f \xC2\xB0" "C",
           static_cast<double>(tBoilerDest), static_cast<double>(tBoilerDiff),
           static_cast<double>(tBoilerDest-tBoilerDiff));
  return String(text);
}

void leaveVacationMode() {
  operatingModeStorageVerified = false;
  UrlaubBetrieb = 0;
  EEPROM.put(EEADDRESS_URLAUB_AKTIV, UrlaubBetrieb);
}

bool applyOperatingMode(int operatingMode) {
  if (operatingMode < 0 || operatingMode > 4) return false;
  {
    const byte winter = operatingMode == 0;
    const byte boiler = operatingMode == 1;
    const byte heating = operatingMode == 2 || operatingMode == 4;
    const byte vacation = operatingMode == 4;
    const char modeCode = vacation ? 'U' : winter ? 'A' : boiler ? 'B' : heating ? 'H' : '0';
    if (operatingModeStorageVerified && WinterBetrieb == winter && BoilerBetrieb == boiler && NurHeizung == heating &&
        UrlaubBetrieb == vacation) {
      Betriebsart = modeCode;
      return true; // An already active mode needs no new flash write.
    }
    EEPROM.put(EEADDRESS_WINTER, winter);
    EEPROM.put(EEADDRESS_BOILER_SOMMERBETRIEB, boiler);
    EEPROM.put(EEADDRESS_NUR_HEIZUNG, heating);
    EEPROM.put(EEADDRESS_URLAUB_AKTIV, vacation);
    if (EEPROM.commit()) {
      WinterBetrieb = winter; BoilerBetrieb = boiler; NurHeizung = heating;
      UrlaubBetrieb = vacation;
      Betriebsart = modeCode;
      operatingModeStorageVerified = true;
      return true;
    } else {
      EEPROM.put(EEADDRESS_WINTER, WinterBetrieb);
      EEPROM.put(EEADDRESS_BOILER_SOMMERBETRIEB, BoilerBetrieb);
      EEPROM.put(EEADDRESS_NUR_HEIZUNG, NurHeizung);
      EEPROM.put(EEADDRESS_URLAUB_AKTIV, UrlaubBetrieb);
      Serial.println("Betriebsart: Speichern fehlgeschlagen, bisheriger Modus bleibt aktiv");
    }
  }
  return false;
}
