void I2C_IO_Init(uint8_t address, uint8_t data){
  if (!ioExtenderPresent) return;
  Wire.beginTransmission(address);
  Wire.write(0xF);
  //Wire.write(data); //alle Ausgänge ausschalten
  Wire.endTransmission();
}
//#################################################################################################################################
void I2C_IO_BitWrite(uint8_t address, uint8_t data){
  if (!ioExtenderPresent) return;
  Wire.beginTransmission(address);
  Wire.write(data); //alle Ausgänge ausschalten
  Wire.endTransmission();
  dataToI2C = data;
//  dataToI2C=I2C_IO_ReadInputs(address);
}
uint8_t I2C_IO_ReadInputs(uint8_t address){
  if (!ioExtenderPresent) return 0xff;
  Wire.beginTransmission(address);
  Wire.requestFrom(address, uint8_t(2));
  uint8_t Data_In = Wire.read();
  Wire.endTransmission();
  return Data_In;
}
//#################################################################################################################################
bool vorlaufFaultOpening = false;
unsigned long vorlaufFaultStarted = 0;
constexpr unsigned long VORLAUF_FAULT_OPEN_MS = 20000UL;
bool mischerOffMode = false;
unsigned long mischerOffStarted = 0;
bool mischerOffFinished = false;
bool mischerOffClosing = false;
static_assert(MISCHER_AUS_KESSEL_MAX_TEMP >= 30.0f && MISCHER_AUS_KESSEL_MAX_TEMP <= 40.0f,
              "Mischer-AUS-Kesseltemperatur ausserhalb des Sensorbereichs");
static_assert(MISCHER_AUS_ZUFAHRT_MINUTEN <= 0xFFFFFFFFUL / 60000UL,
              "Mischer-AUS-Zeit ist zu gross");
constexpr unsigned long MISCHER_OFF_CLOSE_MS = MISCHER_AUS_ZUFAHRT_MINUTEN * 60000UL;

void updateMischerOffMode() {
  const bool off = !UrlaubBetrieb && !WinterBetrieb && !BoilerBetrieb && !NurHeizung;
  if (!off) {
    if (mischerOffMode) {
      digitalWrite(MischerAuf_Pin, LOW);
      digitalWrite(MischerZu_Pin, LOW);
      MischerAufRelais = MischerZuRelais = false;
      mischerPulseActive = false;
      mischerOffMode = false;
      mischerOffClosing = false;
    }
    offHeatRecoveryActive = false;
    return;
  }
  // Never allow retained heat demands to re-enable the burner in AUS.
  BrennerRelais = false;
  digitalWrite(BrennerPin, LOW);
  if (!mischerOffMode) {
    mischerOffMode = true;
    mischerOffFinished = false;
    mischerOffClosing = false;
    offHeatRecoveryActive = false;
    vorlaufFaultOpening = false;
    mischer_wait = mischer_drive = 0;
    mischer_init_laeuft = mischer_init_auf = mischer_init_zu = false;
    previousTime_MischerInit = 0;
    digitalWrite(MischerAuf_Pin, LOW);
    digitalWrite(MischerZu_Pin, LOW);
    MischerAufRelais = MischerZuRelais = false;
      mischerPulseActive = false;
  }
  const bool cool = sensorIsUsable(0) && isfinite(tKessel) &&
                    tKessel <= restwaermeZielTemp;
  if (!cool) {
    if (!offHeatRecoveryActive) {
      // Restart regulated heat removal if the boiler warms again during AUS.
      digitalWrite(MischerAuf_Pin, LOW);
      digitalWrite(MischerZu_Pin, LOW);
      MischerAufRelais = MischerZuRelais = false;
      mischerPulseActive = false;
      mischer_wait = mischer_drive = 0;
      Serial.println("AUS: geregelte Restwaermeabfuhr, Brenner gesperrt");
    }
    offHeatRecoveryActive = true;
    mischerOffClosing = false;
    mischerOffFinished = false;
    return; // kein_Betrieb regulates the heating circuit, never the burner.
  }
  if (offHeatRecoveryActive) {
    mischerPulseActive = false;
    offHeatRecoveryActive = false;
    vorlaufFaultOpening = false;
    mischer_wait = mischer_drive = 0;
    Serial.println("AUS: Restwaermeziel erreicht, Mischer schliessen");
  }
  const unsigned long now = millis();
  if (mischerOffClosing && (unsigned long)(now - mischerOffStarted) >= MISCHER_OFF_CLOSE_MS) {
    mischerOffFinished = true;
    mischerOffClosing = false;
  }
  if (!mischerOffFinished && !mischerOffClosing) {
    mischerOffStarted = now;
    mischerOffClosing = MISCHER_OFF_CLOSE_MS != 0;
    mischerOffFinished = MISCHER_OFF_CLOSE_MS == 0;
  }
  digitalWrite(MischerAuf_Pin, LOW);
  MischerAufRelais = false;
  digitalWrite(MischerZu_Pin, mischerOffClosing ? HIGH : LOW);
  MischerZuRelais = mischerOffClosing;
}

bool enforceVorlaufSensorLock() {
  if (mischerOffMode && !offHeatRecoveryActive) return true;
  const bool locked = !sensorIsUsable(1);
  static bool faultActive = false;
  if (!locked) {
    if (faultActive) {
      vorlaufFaultOpening = false;
      MischerStop();
      faultActive = false;
      Serial.println("Mischer freigegeben: Vorlaufsensor gueltig");
    }
    return false;
  }
  mischerPulseActive = false;
  if (!faultActive) {
    faultActive = true;
    vorlaufFaultStarted = millis();
    vorlaufFaultOpening = true;
    mischer_wait = 0;
    mischer_drive = 0;
    // The fault movement invalidates the previously referenced position.
    mischer_init_laeuft = false;
    mischer_init_auf = false;
    mischer_init_zu = false;
    previousTime_MischerInit = 0;
    Serial.println("Vorlaufsensor ungueltig: Mischer einmalig 20 Sekunden AUF");
  }
  if (vorlaufFaultOpening &&
      (unsigned long)(millis() - vorlaufFaultStarted) < VORLAUF_FAULT_OPEN_MS) {
    // Deliberate fault action, bypassing the normal movement guards.
    digitalWrite(MischerZu_Pin, LOW);
    MischerZuRelais = false;
    digitalWrite(MischerAuf_Pin, HIGH);
    MischerAufRelais = true;
  } else {
    if (vorlaufFaultOpening) {
      vorlaufFaultOpening = false;
      Serial.println("Mischer gesperrt: 20-Sekunden-Auffahrt beendet");
    }
    MischerStop();
  }
  return true;
}

bool mischerBoilerMode = false;
bool mischerBoilerFinished = false;
unsigned long mischerBoilerStarted = 0;
unsigned long mischerBoilerCheckStarted = 0;
bool mischerBoilerCheckDone = false;
bool mischerBoilerReferenceValid = false;
float mischerBoilerReference = 0;
static_assert(MISCHER_BOILER_NACHPRUEFUNG_MINUTEN <= 0xFFFFFFFFUL / 60000UL,
              "Mischer-Boiler-Nachpruefzeit ist zu gross");
constexpr unsigned long MISCHER_BOILER_CHECK_MS = MISCHER_BOILER_NACHPRUEFUNG_MINUTEN * 60000UL;
static_assert(MISCHER_BOILER_ZUFAHRT_MINUTEN <= 0xFFFFFFFFUL / 60000UL,
              "Mischer-Boiler-Zeit ist zu gross");
constexpr unsigned long MISCHER_BOILER_CLOSE_MS = MISCHER_BOILER_ZUFAHRT_MINUTEN * 60000UL;

void updateMischerBoilerMode() {
  const bool boilerOnly = !UrlaubBetrieb && !WinterBetrieb && BoilerBetrieb;
  if (!boilerOnly) {
    if (mischerBoilerMode) {
      // AUS already owns its outputs after updateMischerOffMode.
      if (!mischerOffMode) MischerStop();
      mischerBoilerMode = false;
    }
    return;
  }
  if (!mischerBoilerMode) {
    mischerBoilerMode = true;
    mischerBoilerStarted = millis();
    mischerBoilerFinished = false;
    mischerBoilerCheckDone = false;
    mischerBoilerReferenceValid = false;
    mischer_wait = mischer_drive = 0;
    mischer_init_laeuft = mischer_init_auf = mischer_init_zu = false;
    previousTime_MischerInit = 0;
    Serial.println("NUR BOILER: einmalige Mischer-Schliessfahrt");
  }
  // Existing sensor-fault opening/lock takes priority over this movement.
  if (enforceVorlaufSensorLock()) {
    mischerBoilerReferenceValid = false;
    return;
  }
  const unsigned long now = millis();
  const bool valid = sensorIsUsable(1) && isfinite(tVorlauf);
  if (!mischerBoilerFinished && (unsigned long)(now - mischerBoilerStarted) >= MISCHER_BOILER_CLOSE_MS) {
    mischerBoilerFinished = true;
    if (!mischerBoilerCheckDone) {
      // Compare against the temperature at the end of the first closing run.
      mischerBoilerCheckStarted = now;
      mischerBoilerReferenceValid = valid;
      mischerBoilerReference = tVorlauf;
    }
  }
  if (mischerBoilerFinished && !mischerBoilerCheckDone &&
      (unsigned long)(now - mischerBoilerCheckStarted) >= MISCHER_BOILER_CHECK_MS) {
    mischerBoilerCheckDone = true;
    if (mischerBoilerReferenceValid && valid && tVorlauf >= mischerBoilerReference) {
      mischerBoilerFinished = false;
      mischerBoilerStarted = now;
      Serial.println("NUR BOILER: Vorlauf nicht gesunken, einmalige Mischer-Nachfahrt");
    }
  }
  if (!mischerBoilerFinished) {
    MischerZu();
  } else {
    MischerStop();
  }
}

static_assert(MISCHER_DRIVE > 0 && MISCHER_DRIVE <= 0xFFFFFFFFUL / 1000UL,
              "Ungueltige normale Mischer-Fahrzeit");
static_assert(MISCHER_RUHEZONE_GRAD >= 0.0f && MISCHER_RUHEZONE_GRAD <= 10.0f,
              "Ungueltige Mischer-Ruhezone");
constexpr uint32_t MISCHER_PULSE_MS = MISCHER_DRIVE * 1000UL;

void startRegulatedMischerPulse(bool opening) {
  if (enforceVorlaufSensorLock()) return;
  if (opening) MischerAuf();
  else MischerZu();
  mischerPulseStarted = static_cast<uint32_t>(millis());
  mischerPulseActive = true;
  mischer_drive = MISCHER_DRIVE;
}

void serviceRegulatedMischerPulse() {
  if (!mischerPulseActive) return;
  const bool needsMixer = offHeatRecoveryActive ||
      (!UrlaubBetrieb && (WinterBetrieb || NurHeizung) && HeizungsRelais);
  if (!needsMixer ||
      static_cast<uint32_t>(static_cast<uint32_t>(millis()) - mischerPulseStarted) >= MISCHER_PULSE_MS) {
    MischerStop();
  }
}

void MischerInit(){
  if (enforceVorlaufSensorLock()) return;
  if (mischerBoilerMode || mischerOffMode) return;
  unsigned long currentTime0;
  if(!mischer_init_auf && !mischer_init_laeuft && !mischer_init_zu){
   mischer_init_laeuft=true;
   MischerAuf();
   previousTime_MischerInit=millis();
  }
  if(mischer_init_laeuft && !mischer_init_auf && !mischer_init_zu){
    currentTime0=millis();
   if(currentTime0 - previousTime_MischerInit >= mischer_init_time_auf){
    MischerStop();
    mischer_init_auf=true;
    previousTime_MischerInit=millis();
    delay(125);
    MischerZu();
   }
  }
  if(mischer_init_laeuft && mischer_init_auf && !mischer_init_zu){
    currentTime0=millis();
    if(currentTime0 - previousTime_MischerInit >= mischer_init_time_zu){
      MischerStop();
      mischer_init_zu=true;
      mischer_init_laeuft=false;
      previousTime_MischerInit=0;
    }
  }
}
//#################################################################################################################################
void MischerAuf(){
  if (enforceVorlaufSensorLock()) return;
  mischerPulseActive = false;
  digitalWrite(MischerZu_Pin, LOW);
  MischerZuRelais=false;
  digitalWrite(MischerAuf_Pin, HIGH);
  MischerAufRelais=true;
//  #if defined(EXMISCHER)
//  bitClear(ioextender0_indicate, exMischerZu_Pin);
//  bitSet(ioextender0_indicate,exMischerAuf_Pin);
//  I2C_IO_BitWrite(ioextender0_addr,ioextender0_indicate);
//  #endif
//  timer6_mischer_nachlauf.update();
}
//#################################################################################################################################
void MischerZu(){
  if (enforceVorlaufSensorLock()) return;
  mischerPulseActive = false;
  digitalWrite(MischerAuf_Pin, LOW);
  MischerAufRelais=false;
  digitalWrite(MischerZu_Pin, HIGH);
  MischerZuRelais=true;
//  #if defined(EXMISCHER)
//  bitClear(ioextender0_indicate, exMischerAuf_Pin);
//  bitSet(ioextender0_indicate,exMischerZu_Pin);
//  I2C_IO_BitWrite(ioextender0_addr,ioextender0_indicate);
//  #endif
//  timer6_mischer_nachlauf.update();
}
//#################################################################################################################################
void MischerStop(){
  mischerPulseActive = false;
  mischer_drive = 0;
  // Normal regulation must not interrupt the explicitly requested fault run.
  // Also check its deadline here, in case another caller reaches us first.
  if (vorlaufFaultOpening &&
      (unsigned long)(millis() - vorlaufFaultStarted) < VORLAUF_FAULT_OPEN_MS) return;
  digitalWrite(MischerAuf_Pin, LOW);
  MischerAufRelais=false;
  digitalWrite(MischerZu_Pin, LOW);
  MischerZuRelais=false;
//  #if defined(EXMISCHER)
//  bitClear(ioextender0_indicate, exMischerZu_Pin);
//  bitClear(ioextender0_indicate, exMischerAuf_Pin);
//  I2C_IO_BitWrite(ioextender0_addr,ioextender0_indicate);
//  #endif
}
//#################################################################################################################################
