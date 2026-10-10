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

bool enforceVorlaufSensorLock() {
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

void MischerInit(){
  if (enforceVorlaufSensorLock()) return;
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
