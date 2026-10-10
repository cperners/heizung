constexpr size_t WEB_STATUS_CAPACITY = 2048;
char webStatusCache[WEB_STATUS_CAPACITY] = {};
size_t webStatusLength = 0;
portMUX_TYPE webStatusMux = portMUX_INITIALIZER_UNLOCKED;
// Update only for a new measurement, not for every browser poll.
struct TemperatureTrend {
  bool hasPrevious = false;
  unsigned long stamp = 0;
  float previous = 0;
  int direction = 0;
};
TemperatureTrend temperatureTrends[6];
void addTemperatureTrend(JsonArray trends, byte index, float value,
                         unsigned long stamp, bool valid) {
  TemperatureTrend& trend = temperatureTrends[index];
  if (!valid || !isfinite(value)) {
    trend.hasPrevious = false;
    trend.direction = 0;
    trends.add(nullptr);
    return;
  }
  if (!trend.hasPrevious || stamp != trend.stamp) {
    const float difference = value - trend.previous;
    trend.direction = !trend.hasPrevious ? 0 :
                      difference >= 0.09999f ? 1 : difference <= -0.09999f ? -1 : 0;
    trend.previous = value;
    trend.stamp = stamp;
    trend.hasPrevious = true;
  }
  trends.add(trend.direction);
}
void refreshWebStatus() {
  static unsigned long lastRefresh = 0;
  static uint32_t generation = 0;
  const unsigned long now = millis();
  if (generation != 0 && now-lastRefresh < 1000UL) return;
  lastRefresh = now;
  JsonDocument doc;
  doc["generation"] = ++generation;
  JsonObject values = doc["values"].to<JsonObject>();
  values["troom"] = readTemperature(tRoom);
  values["taussen"] = readTemperature(tAussen);
  values["tkessel"] = readTemperature(tKessel);
  values["tvorlauf"] = readTemperature(tVorlauf);
  values["tboiler"] = readTemperature(tBoiler);
  values["tkachelofen"] = readTemperature(tKachelofen);
  JsonArray trends = doc["trends"].to<JsonArray>();
  addTemperatureTrend(trends, 0, tRoom, sensorLastSuccess[3], sensorIsUsable(3));
  addTemperatureTrend(trends, 1, tAussen, sensorLastSuccess[2], sensorIsUsable(2));
  addTemperatureTrend(trends, 2, tKessel, sensorLastSuccess[0], sensorIsUsable(0));
  addTemperatureTrend(trends, 3, tVorlauf, sensorLastSuccess[1], sensorIsUsable(1));
  addTemperatureTrend(trends, 4, tBoiler, sensorLastSuccess[4], sensorIsUsable(4));
  addTemperatureTrend(trends, 5, tKachelofen, kachelofenLastUpdate,
                      kachelofenLastUpdate != 0 && now-kachelofenLastUpdate <= kachelofenTimeout);
  values["header-clock"] = processor("WEBTIME");
  const uint64_t uptimeSeconds = static_cast<uint64_t>(esp_timer_get_time()) / 1000000ULL;
  char uptimeText[64];
  snprintf(uptimeText, sizeof(uptimeText), "%llu Tage %02u:%02u:%02u",
           static_cast<unsigned long long>(uptimeSeconds / 86400ULL),
           static_cast<unsigned>((uptimeSeconds / 3600ULL) % 24ULL),
           static_cast<unsigned>((uptimeSeconds / 60ULL) % 60ULL),
           static_cast<unsigned>(uptimeSeconds % 60ULL));
  values["uptime"] = uptimeText;
  values["cpu-load"] = cpuLoadDisplay();
  values["brennersperre"] = sensorIsUsable(0) ? "Kesselsensor OK - keine Sensorsperre" : "BRENNER GESPERRT - Kesselsensor ungueltig";
  values["regelungsart"] = regelungsModus == REGELUNG_AUTO ? (AussentemperaturRegelung ? "AUTO / AUSSENTEMPERATUR" : "AUTO / RAUMTEMPERATUR") : (regelungsModus == REGELUNG_RAUM ? "MANUELL / RAUMTEMPERATUR" : "MANUELL / AUSSENTEMPERATUR");
  values["betriebsart"] = WinterBetrieb ? "AUTOMATIKBETRIEB" : BoilerBetrieb ? "NUR BOILER" : NurHeizung ? "NUR HEIZUNG" : "AUS";
  values["gas-day"] = gasDisplay(true);
  values["gas-total"] = gasDisplay(false);
  values["ntp-abweichung"] = ntpProbeDisplay();
  values["kachelofen-status"] = kachelofenAktiv ? "AKTIV" : "INAKTIV";
  JsonArray outputs = doc["outputs"].to<JsonArray>();
  outputs.add(digitalRead(BrennerPin) == HIGH ? 1 : 0);
  outputs.add(digitalRead(BoilerPin) == HIGH ? 1 : 0);
  outputs.add(digitalRead(HeizungPin) == HIGH ? 1 : 0);
  outputs.add(digitalRead(MischerAuf_Pin) == HIGH ? 1 : 0);
  outputs.add(digitalRead(MischerZu_Pin) == HIGH ? 1 : 0);
  if (doc.overflowed() || measureJson(doc) >= WEB_STATUS_CAPACITY) return;
  char buffer[WEB_STATUS_CAPACITY];
  const size_t length = serializeJson(doc, buffer, sizeof(buffer));
  portENTER_CRITICAL(&webStatusMux);
  memcpy(webStatusCache, buffer, length+1);
  webStatusLength = length;
  portEXIT_CRITICAL(&webStatusMux);
}

