void Automatik(){
  RoomHeizen=true;
  KesselHeizen=true;
  BoilerHeizen=true;
  if (daynight=='D'){
    tmyRoomdest=tRoomTag; //Tag
  }
  if(daynight=='N'){
    tmyRoomdest=tRoomNacht; //Tag
  }
  RoomAnforderungf();
  BoilerAnforderungf();
  if (chckKessel()){
    chckRoom();
    chckBoiler();
  }
  SetOutPin();
}
/* *******************************************************************************************************
                                         Boilerbetrieb
******************************************************************************************************* */
void Boilerbetrieb(){
  RoomHeizen=false;
  KesselHeizen=true;
  BoilerHeizen=true;
  tmyRoomdest=0.0;
  BoilerAnforderungf();
  if (chckKessel()){
  chckBoiler();
  }
  MischerZu(); //2023-10-15 Mischer Zu - sonst heizt Raum mit.
  SetOutPin();
}
/* *******************************************************************************************************
                                         Heizungsbetrieb
******************************************************************************************************* */
void Heizungsbetrieb() {
  RoomHeizen=true;
  KesselHeizen=true;
  BoilerHeizen=false;
  if (daynight=='D'){
    tmyRoomdest=tRoomTag; //Tag
  }
  if(daynight=='N'){
    tmyRoomdest=tRoomNacht; //Tag
  }
  RoomAnforderungf();
  if (chckKessel()){
    chckRoom();
  }
  SetOutPin();
}
/* *******************************************************************************************************
                                         kein Betrieb
******************************************************************************************************* */
void kein_Betrieb() {
    RoomHeizen=false;
    BoilerHeizen=false;
    KesselHeizen=false;
    tmyRoomdest=0.0;
    BrennerRelais=false;//neu bis SetOutPin();
    if(tKessel<=kessel_min_temp){
        HeizungsRelais=false;//erst bei einer Mindesttemperatur des Kessels Pumpe ein
    }
    chckBoiler();//Boiler abschalten
  SetOutPin();
  //Serial.print("Alles ist ausgeschaltet!\n");
}
/* *******************************************************************************************************
                                         RoomAnforerung
******************************************************************************************************* */
bool updateHeatingCurve() {
  heizkurveGueltig = false;
  vorlaufTemperatur = 0.0f;
  if (AussentemperaturRegelung == 0) return false;

  // Snapshot the parameters and reject undefined mathematical domains.
  const double room = tmyRoomdest;
  const double outside = tAussen;
  const double maximum = tvmax;
  const double minimumOutside = taumin;
  const double exponent = n;
  if (!isfinite(room) || !isfinite(outside) || !isfinite(maximum) ||
      !isfinite(minimumOutside) || !isfinite(exponent) ||
      room < 5.0 || room > 40.0 || outside < -55.0 || outside > 125.0 ||
      maximum < room || maximum > 100.0 || minimumOutside < -50.0 ||
      minimumOutside > 0.0 || exponent < 0.1 || exponent > 10.0) return false;

  const double denominator = room - minimumOutside;
  if (denominator <= 0.0) return false;
  double ratio = (room - outside) / denominator;
  // Above the room target use the lower endpoint; below the design outside
  // temperature use the configured maximum, never a negative power base.
  if (ratio < 0.0) ratio = 0.0;
  if (ratio > 1.0) ratio = 1.0;
  const double target = room + (maximum - room) * pow(ratio, 1.0 / exponent);
  if (!isfinite(target) || target < room || target > maximum) return false;
  vorlaufTemperatur = static_cast<float>(floor(target * 100.0) / 100.0);
  heizkurveGueltig = true;
  return true;
}

void RoomAnforderungf(){
  if (heatingPumpSensorFault()) { RoomAnforderung = false; return; }
  if(AussentemperaturRegelung==0){
    if (tRoom < tmyRoomdest){
      RoomAnforderung=true;
    }else{
      RoomAnforderung=false;
    }
  }else{
    if (updateHeatingCurve() && tVorlauf < vorlaufTemperatur){
      RoomAnforderung=true;
    }else{
      RoomAnforderung=false;
    }
  }
}
/* *******************************************************************************************************
                                         Kachelofen Status
******************************************************************************************************* */
void updateKachelofenStatus(){
  const unsigned long now = millis();
  const bool mqttWertGueltig = (kachelofenLastUpdate != 0) &&
                               ((unsigned long)(now - kachelofenLastUpdate) <= kachelofenTimeout);

  if(!mqttWertGueltig){
    // Ohne frischen Kachelofenwert gilt der Ofen als inaktiv.
    kachelofenAktiv = false;
  }

  // Hysterese nur anwenden, wenn ein frischer MQTT-Wert vorhanden ist.
  if(mqttWertGueltig){
    if(!kachelofenAktiv && tKachelofen >= kachelofenEinTemp){
      kachelofenAktiv = true;
    }else if(kachelofenAktiv && tKachelofen <= kachelofenAusTemp){
      kachelofenAktiv = false;
    }
  }

  // AUTO folgt dem Kachelofen. RAUM/AUSSEN sind manuelle Service-Overrides.
  if(regelungsModus == REGELUNG_AUTO){
    AussentemperaturRegelung = kachelofenAktiv ? 1 : 0;
  }else if(regelungsModus == REGELUNG_RAUM){
    AussentemperaturRegelung = 0;
  }else{
    AussentemperaturRegelung = 1;
  }
}


/* *******************************************************************************************************
                                         BoilerAnforderung
******************************************************************************************************* */
void BoilerAnforderungf(){
  if (boilerPumpSensorFault()) { BoilerAnforderung = false; return; }
  if(tBoiler < tBoilerDest){
    BoilerAnforderung=1;
  }else{
    BoilerAnforderung=0;
  }
}


/* *******************************************************************************************************
                                         Ausgänge schalten
******************************************************************************************************* */
bool heatingPumpSensorFault() {
  return !sensorIsUsable(0) || !sensorIsUsable(1) ||
         !sensorIsUsable(AussentemperaturRegelung == 0 ? 3 : 2);
}

bool boilerPumpSensorFault() {
  return !sensorIsUsable(0) || !sensorIsUsable(4);
}

struct PumpFaultState {
  bool active = false;
  bool wasRunning = false;
  bool expiryReported = false;
  unsigned long started = 0;
};
PumpFaultState heatingPumpFault, boilerPumpFault;
constexpr unsigned long PUMP_FAULT_RUNON_MS = 10UL * 60UL * 1000UL;

bool pumpOutputWithFault(PumpFaultState& state, bool fault, bool requested,
                         uint8_t pin, const char* name) {
  if (!fault) {
    if (state.active) Serial.printf("%s: Sensoren wieder gueltig\n", name);
    state.active = false;
    return requested;
  }
  if (!state.active) {
    state.active = true;
    state.wasRunning = digitalRead(pin) == HIGH;
    state.started = millis();
    state.expiryReported = false;
    Serial.printf("%s: Sensorfehler, %s\n", name,
                  state.wasRunning ? "10 Minuten Nachlauf" : "bleibt aus");
  }
  const bool running = state.wasRunning &&
    (unsigned long)(millis() - state.started) < PUMP_FAULT_RUNON_MS;
  if (state.wasRunning && !running && !state.expiryReported) {
    Serial.printf("%s: Nachlauf beendet, gesperrt\n", name);
    state.expiryReported = true;
  }
  return running;
}

void SetOutPin(){
  enforceKesselSensorLock();
  const bool heatingFault = heatingPumpSensorFault();
  const bool boilerFault = boilerPumpSensorFault();
  const bool heatingOn = pumpOutputWithFault(heatingPumpFault, heatingFault,
      HeizungsRelais || Pumpenloesen, HeizungPin, "Heizungspumpe");
  const bool boilerOn = pumpOutputWithFault(boilerPumpFault, boilerFault,
      BoilerRelais || Pumpenloesen, BoilerPin, "Boilerpumpe");
  if (heatingFault) RoomAnforderung = false;
  if (boilerFault) { BoilerAnforderung = false; BoilerAufheizen = false; }
  HeizungsRelais = heatingOn;
  BoilerRelais = boilerOn;
  if (heatingOn){
    digitalWrite(HeizungPin, HIGH);
   // bitSet(ioextender0_indicate, exHeizung_Pin);
  }else{
    digitalWrite(HeizungPin, LOW);
   // bitClear(ioextender0_indicate, exHeizung_Pin);
  }
  if (boilerOn){
    digitalWrite(BoilerPin, HIGH);
   // bitSet(ioextender0_indicate, exBoiler_Pin);
  }else{
    digitalWrite(BoilerPin, LOW);
   // bitClear(ioextender0_indicate, exBoiler_Pin);
  }
  if (BrennerRelais==true){
    digitalWrite(BrennerPin, HIGH);
   // bitSet(ioextender0_indicate, exBrenner_Pin);
  }else{
    digitalWrite(BrennerPin, LOW);
   // bitClear(ioextender0_indicate, exBrenner_Pin);
  }
  //I2C_IO_BitWrite(ioextender0_addr,ioextender0_indicate);
}


/* *******************************************************************************************************
                                         Kessel Check
******************************************************************************************************* */
int chckKessel() {
  float nKesselDiff=tKesselDiff;
  if(BoilerAufheizen){
    nKesselDiff-=5;
  }else{
    nKesselDiff=tKesselDiff;
  }
  if (KesselHeizen==false){
    BrennerRelais=false;
  }else{
    if (tKessel >= (tKesselDest)) {
      BrennerRelais=false;//AUS
      //Serial.write("Brenner ausschalten\n");
    }
    if((tKessel < (tKesselDest-nKesselDiff)) && (BoilerAnforderung)) { //neu && BoilerAufheizen - um Kessel nicht immer aufzuheizen
      BrennerRelais=true;//EIN
      //Serial.write("Brenner einschalten\n");
    }
    if((tKessel < (tKesselDest-nKesselDiff)) && (RoomAnforderung)) { //neues if - um Kessel nicht immer aufzuheizen
      BrennerRelais=true;//EIN
      //Serial.write("Brenner einschalten\n");
    }
  }
  SetOutPin();
  if (tKessel < (tKesselDest-(tKesselDiff+10))){
    return 0;
  }else {
    return 1;
  }
}
/* *******************************************************************************************************
                                         Room Check
******************************************************************************************************* */
void chckRoom() {
  if (heatingPumpSensorFault()) {
    RoomAnforderung = false;
    if (!mischer_init_laeuft) MischerStop();
    SetOutPin();
    return;
  }

  updateHeatingCurve();
  if (AussentemperaturRegelung != 0 && !heizkurveGueltig) {
    RoomAnforderung = false;
    if (!mischer_init_laeuft) MischerStop();
    SetOutPin();
    return;
  }

  if(RoomHeizen==false){
    HeizungsRelais=false;
  }else{
    if(AussentemperaturRegelung == 0){
      if (tRoom >= tmyRoomdest)
      {
        HeizungsRelais=false;//AUS
        //Serial.write("Heizungspumpe ausschalten\n");
      }
      if(tRoom < (tmyRoomdest-tRoomDiff)) {
        HeizungsRelais=true;//EIN
        //Serial.write("Heizungspumpe einschalten\n");
      }
    }else { //AussentemperaturRegelung == 1
      //Serial.println("AussentemperaturRegelung == 1, HeizungsRelais ein");
      if(tKessel>=kessel_min_temp){
        HeizungsRelais=true;//erst bei einer Mindesttemperatur des Kessels Pumpe ein
      }
    if(!mischer_init_laeuft && mischer_init_auf && mischer_init_zu){
      if (tVorlauf < vorlaufTemperatur){
        if(mischer_wait==0){
          mischer_wait=MISCHER_WAIT;
          mischer_drive=MISCHER_DRIVE;
          MischerAuf(); 
        } //MischerAuf
      }else{
        if(mischer_wait==0){
          mischer_wait=MISCHER_WAIT;
          mischer_drive=MISCHER_DRIVE;
          MischerZu(); 
        } //MischerZu
      }
      if((MischerAufRelais || MischerZuRelais) && !mischer_drive){
        MischerStop();
      }
     }
    }
  }
  SetOutPin();
}
/* *******************************************************************************************************
                                         Boiler Check
******************************************************************************************************* */
void chckBoiler() {
  if (boilerPumpSensorFault()) {
    BoilerAnforderung = false;
    BoilerAufheizen = false;
    SetOutPin();
    return;
  }
  if(BoilerHeizen==false){
    BoilerRelais=false;
    BoilerAufheizen=false;
  }else{
  if (tBoiler >= tBoilerDest)
  {
    BoilerRelais=false;//AUS
    BoilerAufheizen=false;//Aufheizen stoppen
    Serial.write("Boiler aufgeheizt\n");
  }
//  if((tBoiler < (tBoilerDest-tBoilerDiff)) && tBoiler < tKessel) {
  if((tBoiler < (tBoilerDest-tBoilerDiff))) {
    //BoilerRelais=true;//EIN
    BoilerAufheizen=true;//Aufheizen starten
    Serial.write("Boiler aufheizen aktiv\n");
  }
  if(BoilerAufheizen){
    if(tBoiler < tKessel){
      BoilerRelais=true;
    }else{
      BoilerRelais=false;
      if(KesselHeizen){//zuerst noch schauen ob KesselHeizen überhaupt aktiv ist
        BrennerRelais=true;//Kessel muss erst Temperatur haben
      }
    }
  }
 }
  SetOutPin();
}

/* *******************************************************************************************************
                                         Umschaltung Regelung Aussen - Innen
******************************************************************************************************* */
void Regelungs_Switch_AI(){
if(AussentemperaturRegelung){AussentemperaturRegelung=0;}
else{AussentemperaturRegelung=1;}
}
/* *******************************************************************************************************
                                         PumpenNachlauf
******************************************************************************************************* */
void hpumpe_nachlauf(){
  PumpenNachlauf = 0;
}
/* *******************************************************************************************************
                                         Serial Read
******************************************************************************************************* */
void Serial_Read() {
/*
   Serial.println("Per Serial den Arbeitsmodus umschalten:");
   Serial.println("a ->AutomatikBetrieb");
   Serial.println("b ->BoilerBetrieb");
   Serial.println("h ->NurHeizungsBetrieb");
   Serial.println("r ->Aussen-/Raum-Regelung");
   Serial.println("s ->AUS");
*/
  if (Serial.available() > 0){
    int input = Serial.read();
    switch (input) {
      case 114:
        Regelungs_Switch_AI();
        break;
      case 97:
        Serial.print("Go to Automatik-Betrieb\n");
        Betriebsart='A';
        break;
      case 98:
        Serial.print("Go to Boiler-Betrieb\n");
        Betriebsart='B';
        break;
      case 104:
        Serial.print("Go to NurHeizungs-Betrieb\n");
        Betriebsart='H';
        break;
      case 115:
        Serial.print("Go to Disabled\n");
        Betriebsart='0';
        break;
      case 48:
        ESP.restart(); // bei 0 restart
        break;
      default:
        break;
    }
  }
}


/* *******************************************************************************************************
                                         mqtt callback
******************************************************************************************************* */
