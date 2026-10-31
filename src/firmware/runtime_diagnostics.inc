enum LoopPhase : uint32_t { PHASE_BOOT=1, PHASE_MODE_SAVE, PHASE_MIXER, PHASE_OUTPUTS,
  PHASE_NETWORK, PHASE_LCD, PHASE_SENSOR, PHASE_WEB, PHASE_IDLE };
RTC_NOINIT_ATTR uint32_t retainedLoopPhase;
RTC_NOINIT_ATTR uint32_t retainedLoopPhaseCheck;
uint32_t previousLoopPhase = 0;
esp_reset_reason_t bootResetReason = ESP_RST_UNKNOWN;
esp_err_t watchdogInitResult = ESP_FAIL;
esp_err_t watchdogAddResult = ESP_FAIL;
esp_err_t watchdogStatusResult = ESP_FAIL;
void markLoopPhase(LoopPhase phase) {
  retainedLoopPhase = static_cast<uint32_t>(phase);
  retainedLoopPhaseCheck = retainedLoopPhase ^ 0x5e791b3cUL;
}
const char* loopPhaseText(uint32_t phase) {
  switch (phase) {
    case PHASE_BOOT: return "Start";
    case PHASE_MODE_SAVE: return "Betriebsart speichern";
    case PHASE_MIXER: return "Mischer";
    case PHASE_OUTPUTS: return "Regelung und Ausgaenge";
    case PHASE_NETWORK: return "Netzwerk";
    case PHASE_LCD: return "LCD und Tasten";
    case PHASE_SENSOR: return "Sensoren";
    case PHASE_WEB: return "Webstatus";
    case PHASE_IDLE: return "Schleifenende";
    default: return "Nicht verfuegbar";
  }
}
const char* resetReasonText() {
  switch (bootResetReason) {
    case ESP_RST_POWERON: return "Einschalten";
    case ESP_RST_SW: return "Software-Neustart";
    case ESP_RST_PANIC: return "Programmabsturz";
    case ESP_RST_INT_WDT: return "Interrupt-Watchdog";
    case ESP_RST_TASK_WDT: return "Task-Watchdog";
    case ESP_RST_WDT: return "Watchdog";
    case ESP_RST_BROWNOUT: return "Unterspannung";
    case ESP_RST_EXT: return "Reset-Pin";
    default: return "Anderer Neustartgrund";
  }
}
void beginRecoveryDiagnostics() {
  bootResetReason = esp_reset_reason();
  if (bootResetReason != ESP_RST_POWERON && bootResetReason != ESP_RST_BROWNOUT &&
      retainedLoopPhaseCheck == (retainedLoopPhase ^ 0x5e791b3cUL) &&
      retainedLoopPhase >= PHASE_BOOT && retainedLoopPhase <= PHASE_IDLE)
    previousLoopPhase = retainedLoopPhase;
  markLoopPhase(PHASE_BOOT);
}
String recoveryDiagnosticDisplay() {
  uint32_t disconnects, reconnects;
  uint8_t reason;
  portENTER_CRITICAL(&wifiReconnectMux);
  disconnects = wifiDisconnectCount; reconnects = wifiReconnectCount;
  reason = wifiLastDisconnectReason;
  portEXIT_CRITICAL(&wifiReconnectMux);
  char text[280];
  snprintf(text, sizeof(text), "Start: %s; vorherige Phase: %s; WLAN-Abbrueche: %lu; Wiederanlaeufe: %lu; letzter WLAN-Grund: %u; Watchdog: %s (%u s)",
    resetReasonText(), loopPhaseText(previousLoopPhase), static_cast<unsigned long>(disconnects),
    static_cast<unsigned long>(reconnects), reason, watchdogStatusResult == ESP_OK ? "registriert" : "Registrierung fehlgeschlagen", unsigned(WDT_TIMEOUT));
  return String(text);
}
void addRecoveryDiagnostics(JsonDocument& doc) {
  uint32_t disconnects, reconnects; uint8_t reason;
  portENTER_CRITICAL(&wifiReconnectMux);
  disconnects=wifiDisconnectCount; reconnects=wifiReconnectCount; reason=wifiLastDisconnectReason;
  portEXIT_CRITICAL(&wifiReconnectMux);
  JsonObject recovery = doc["recovery"].to<JsonObject>();
  recovery["resetReason"] = resetReasonText();
  recovery["previousLoopPhase"] = loopPhaseText(previousLoopPhase);
  recovery["wifiDisconnects"] = disconnects;
  recovery["wifiReconnectAttempts"] = reconnects;
  recovery["wifiLastDisconnectReason"] = reason;
  recovery["watchdogInitResult"] = int(watchdogInitResult);
  recovery["watchdogAddResult"] = int(watchdogAddResult);
  recovery["watchdogStatusResult"] = int(watchdogStatusResult);
  recovery["watchdogTimeoutSeconds"] = WDT_TIMEOUT;
}

// Sample in the main loop only; web callbacks read the existing status snapshot.
struct RuntimeDiagnostics {
  uint32_t freeHeap = 0;
  uint32_t minimumHeap = 0;
  uint32_t largestBlock = 0;
  uint64_t recentLoopMaxUs = 0;
  uint64_t maximumLoopUs = 0;
  uint32_t slowLoopCount = 0;
};
RuntimeDiagnostics runtimeDiagnostics;

void updateRuntimeDiagnostics() {
  static int64_t previousStart = 0;
  static int64_t lastSample = 0;
  static uint64_t windowMaxUs = 0;
  const int64_t now = esp_timer_get_time();
  if (previousStart != 0) {
    const uint64_t interval = static_cast<uint64_t>(now - previousStart);
    if (interval > windowMaxUs) windowMaxUs = interval;
    if (interval > runtimeDiagnostics.maximumLoopUs) runtimeDiagnostics.maximumLoopUs = interval;
    if (interval > 100000ULL && runtimeDiagnostics.slowLoopCount != UINT32_MAX)
      ++runtimeDiagnostics.slowLoopCount;
  }
  previousStart = now;
  if (lastSample != 0 && now - lastSample < 1000000LL) return;
  lastSample = now;
  runtimeDiagnostics.recentLoopMaxUs = windowMaxUs;
  windowMaxUs = 0;
  // All three figures use the same 8-bit-addressable heap capability.
  runtimeDiagnostics.freeHeap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  runtimeDiagnostics.minimumHeap = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
  runtimeDiagnostics.largestBlock = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
}

void addRuntimeDiagnostics(JsonDocument& doc) {
  JsonObject health = doc["diagnostics"].to<JsonObject>();
  health["heapFreeBytes"] = runtimeDiagnostics.freeHeap;
  health["heapMinBytes"] = runtimeDiagnostics.minimumHeap;
  health["heapLargestBlockBytes"] = runtimeDiagnostics.largestBlock;
  health["loopRecentMaxMs"] = runtimeDiagnostics.recentLoopMaxUs / 1000.0;
  health["loopMaxMs"] = runtimeDiagnostics.maximumLoopUs / 1000.0;
  health["loopOver100ms"] = runtimeDiagnostics.slowLoopCount;
}
