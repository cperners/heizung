// Observation only: callbacks and polling never abort Update or alter outputs.
enum class OtaStage : uint8_t { Idle, Requested, Ready, Uploading, Complete,
                               Failed, AlreadyRunning, NoProgress };
struct OtaDiagnosticState {
  OtaStage stage = OtaStage::Idle;
  uint32_t attempts = 0;
  uint32_t started = 0;
  uint32_t lastActivity = 0;
  size_t bytes = 0;
  size_t httpLength = 0; // Multipart length, not the firmware file size.
  uint8_t errorCode = 0;
  char error[96] = {};
};
portMUX_TYPE otaDiagnosticMux = portMUX_INITIALIZER_UNLOCKED;
OtaDiagnosticState otaDiagnosticState;

OtaDiagnosticState otaDiagnosticSnapshot() {
  portENTER_CRITICAL(&otaDiagnosticMux);
  const OtaDiagnosticState copy = otaDiagnosticState;
  portEXIT_CRITICAL(&otaDiagnosticMux);
  return copy;
}

const char* otaStageText(OtaStage stage) {
  switch (stage) {
    case OtaStage::Requested: return "Start angefordert";
    case OtaStage::Ready: return "Bereit, warte auf Upload";
    case OtaStage::Uploading: return "Firmware wird uebertragen";
    case OtaStage::Complete: return "Update erfolgreich gemeldet";
    case OtaStage::Failed: return "Start oder Upload fehlgeschlagen";
    case OtaStage::AlreadyRunning: return "Start abgelehnt: Update bereits offen";
    case OtaStage::NoProgress: return "Seit 30 Sekunden kein OTA-Fortschritt";
    default: return "Kein Update seit Neustart";
  }
}

void initializeOtaDiagnostics() {
  ElegantOTA.onStart([]() {
    const bool alreadyRunning = Update.isRunning();
    const uint32_t now = millis();
    portENTER_CRITICAL(&otaDiagnosticMux);
    const uint32_t attempts = otaDiagnosticState.attempts;
    otaDiagnosticState = OtaDiagnosticState{};
    otaDiagnosticState.attempts = attempts == UINT32_MAX ? attempts : attempts+1;
    otaDiagnosticState.started = otaDiagnosticState.lastActivity = now;
    otaDiagnosticState.stage = alreadyRunning ? OtaStage::AlreadyRunning : OtaStage::Requested;
    portEXIT_CRITICAL(&otaDiagnosticMux);
  });
  ElegantOTA.onProgress([](size_t current, size_t total) {
    const uint32_t now = millis();
    portENTER_CRITICAL(&otaDiagnosticMux);
    otaDiagnosticState.stage = OtaStage::Uploading;
    otaDiagnosticState.bytes = current;
    otaDiagnosticState.httpLength = total;
    otaDiagnosticState.lastActivity = now;
    portEXIT_CRITICAL(&otaDiagnosticMux);
  });
  ElegantOTA.onEnd([](bool success) {
    char error[96] = {};
    const uint8_t errorCode = Update.getError();
    if (!success) snprintf(error, sizeof(error), "%s", Update.errorString());
    const uint32_t now = millis();
    portENTER_CRITICAL(&otaDiagnosticMux);
    otaDiagnosticState.stage = success ? OtaStage::Complete : OtaStage::Failed;
    otaDiagnosticState.errorCode = errorCode;
    memcpy(otaDiagnosticState.error, error, sizeof(error));
    otaDiagnosticState.lastActivity = now;
    portEXIT_CRITICAL(&otaDiagnosticMux);
  });
}

void updateOtaDiagnostics() {
  const OtaDiagnosticState snapshot = otaDiagnosticSnapshot();
  OtaStage next = snapshot.stage;
  uint8_t errorCode = 0;
  char error[96] = {};
  if (snapshot.stage == OtaStage::Requested || snapshot.stage == OtaStage::Ready ||
      snapshot.stage == OtaStage::Uploading || snapshot.stage == OtaStage::NoProgress) {
    errorCode = Update.getError();
    if (errorCode != 0) {
      next = OtaStage::Failed;
      snprintf(error, sizeof(error), "%s", Update.errorString());
    } else if (snapshot.stage == OtaStage::Requested && Update.isRunning()) {
      next = OtaStage::Ready;
    } else if ((uint32_t)(millis()-snapshot.lastActivity) >= 30000UL) {
      next = OtaStage::NoProgress;
    }
  }
  if (next == snapshot.stage) return;
  portENTER_CRITICAL(&otaDiagnosticMux);
  // A fresh callback must not be overwritten by an older polling snapshot.
  if (otaDiagnosticState.attempts == snapshot.attempts &&
      otaDiagnosticState.stage == snapshot.stage &&
      otaDiagnosticState.lastActivity == snapshot.lastActivity) {
    otaDiagnosticState.stage = next;
    otaDiagnosticState.errorCode = errorCode;
    memcpy(otaDiagnosticState.error, error, sizeof(error));
  }
  portEXIT_CRITICAL(&otaDiagnosticMux);
}

void addOtaDiagnostics(JsonDocument& doc) {
  const OtaDiagnosticState snapshot = otaDiagnosticSnapshot();
  JsonObject ota = doc["ota"].to<JsonObject>();
  ota["stage"] = otaStageText(snapshot.stage);
  ota["attempts"] = snapshot.attempts;
  ota["bytesWritten"] = snapshot.bytes;
  ota["httpContentLength"] = snapshot.httpLength;
  ota["errorCode"] = snapshot.errorCode;
  // Force ownership: the snapshot array lives only until this function returns.
  ota["error"] = String(snapshot.error);
  ota["activityAgeMs"] = snapshot.attempts ? (uint32_t)(millis()-snapshot.lastActivity) : 0;
}

String otaDiagnosticDisplay() {
  const OtaDiagnosticState snapshot = otaDiagnosticSnapshot();
  char text[256];
  snprintf(text, sizeof(text), "%s; %u Bytes%s%s", otaStageText(snapshot.stage),
           static_cast<unsigned>(snapshot.bytes), snapshot.error[0] ? "; " : "", snapshot.error);
  return String(text);
}
