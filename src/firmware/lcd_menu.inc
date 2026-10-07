void KeyPad(){
  if (!keypadPresent) return;
  // Gedrückte Taste abfragen
    char i2cKey = I2C_Keypad.getKey();
    if (i2cKey) {
      lcd_display_light=30;
      lcd.setBacklight(HIGH);
      if (i2cKey == 'A') { // Menue verlassen -> zurueck zur Standard-Menue-Seite 0
        memset(keyBuffer, 0, sizeof keyBuffer);//Alle Eingaben werden verworfen
        MenuPage=0;
        data_ready=false;
        data_count=0;
        MenueMain();
      }else if (i2cKey == 'B' && MenuPage==0) { // Menue verlassen -> zurueck zur Standard-Menue-Seite 0
        memset(keyBuffer, 0, sizeof keyBuffer);//Alle Eingaben werden verworfen
        MenuPage=0;
        data_ready=false;
        data_count=0;
        Regelungs_Switch_AI();
      }else if (i2cKey == 'D' || i2cKey == 'C' || i2cKey == '*') { //C oder D zum Scrollen im Menue
        memset(keyBuffer, 0, sizeof keyBuffer);//Alle Eingaben werden verworfen
        data_ready=false;
        data_count=0;
        if (i2cKey == 'D'){ if(MenuPage < MENUPAGE_MAX) MenuPage++;}
        if (i2cKey == 'C'){ if(MenuPage) MenuPage--;}
        if (i2cKey == '*'){
          ntpupdate();
          Schaltuhr();
          MenuPage=12;
        }
        switch (MenuPage)
        {
            case 0:
              MenueMain();
              break;
            case 1:
              MenuRoomTemp();
              break;
            case 2:
              MenuBoilerTemp();
              break;
            case 3:
              MenuKesseltemp();
              break;
            case 4:
              MenuDifRaumtemp();
              break;
            case 5:
              MenuDifKesseltemp();
              break;
            case 6:
              MenuRaumTempNacht();
              break;
            case 7:
              MenuBetriebWinter();
              break;
            case 8:
              MenuBetriebBoiler();
              break;
            case 9:
              MenuBetriebNurHeizung();
              break;
            case 10:
              MenuHeizbeginnTag();
              break;
            case 11:
              MenuHeizEndeNacht();
              break;
            case 12:
              Time2LCD();
              break;
            case 13:
              MenuSommerzeitEinAus();
              break;
            case 14:
              MenuDifBoilerTemp();
              break;
            case 15:
              Menu_tvmax();
              break;
            case 16:
              Menu_taumin();
              break;
            case 17:
              Menu_n();
                break;
            case 18:
              MenuAussentempRegelung();
                break;
        }
      }
      // Check, ob ASCII Wert des Char einer Ziffer zwischen 0 und 9 entspricht
      else if ((int(i2cKey) >= 48) && (int(i2cKey) <= 57) && MenuPage > 0){ //Nummerntasten wurden gedrückt
        addToKeyBuffer(i2cKey);
        lcd.setCursor(0, 3);
        lcd.print(keyBuffer);//schreibe den eingegebenen Text ans Display
        if(data_ready==true){lcd.print(" OK");}
      }else if ((i2cKey == '#') && (data_ready == true) && MenuPage > 0) { //# ist die Enter-Taste
        checkEingabe();
        data_ready=false;
        data_count=0;
        memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
      }
    }
}
/* *******************************************************************************************************
                                         KeyPad-Buffer
******************************************************************************************************* */
void addToKeyBuffer(char inkey) {
  switch (MenuPage) {
      case MENUPAGE_TEMPERATUR:
      case MENUPAGE_TEMPERATUR1:
      if (data_count < KEYSLENGTH_T && data_ready==false){
        keyBuffer[data_count] = inkey;
        data_count++;
        if(data_count == KEYSLENGTH_T) {
          data_ready=true;
        }
        if(data_count==2){
          keyBuffer[data_count]='.';
          data_count++;
        }
      }
      break;
    case MENUPAGE_NUM:
      if (data_count < KEYSLENGTH_N && data_ready==false){
        keyBuffer[data_count] = inkey;
        data_count++;
        data_ready=true;
      }
      break;
    case MENUPAGE_TIME:
      if (data_count < KEYSLENGTH_Z && data_ready==false){
        if (data_count==3 && (int(inkey) > 53)){inkey='5';} //neu Minuten max 50
        keyBuffer[data_count] = inkey;
        data_count++;
        if(data_count == KEYSLENGTH_Z) {
          data_ready=true;
        }
        if(data_count==2){
          keyBuffer[data_count]='.';
          data_count++;
        }
      }
      break;
      case MENUPAGE_SZ:
        if (data_count < KEYSLENGTH_N && data_ready==false){
          keyBuffer[data_count] = inkey;
          data_count++;
          data_ready=true;
        }
        break;
    case MENUPAGE_NUM_AT:
      if (data_count < KEYSLENGTH_N && data_ready==false){
        keyBuffer[data_count] = inkey;
        data_count++;
        data_ready=true;
      }
      break;
  }
}
/* *******************************************************************************************************
                                         KeyPad-checkEingabe
******************************************************************************************************* */
void checkEingabe() {
  if ((MenuPage >= 1 && MenuPage <= 6) || (MenuPage >= 14 && MenuPage <= 17)) {//case MENUPAGE_TEMPERATUR:
    lcd.print('#');
    //  if(sizeof(keyBuffer)==(KEYSLENGTH_T+1)){
        float f_char;
        f_char = atof(keyBuffer);
        Serial.println(f_char);
        Serial.print("*C");
        switch (MenuPage) {
          case 1:
            tRoomTag=f_char;
            EEPROM.put(EEADDRESS_RAUM, f_char);
            lcd.print(" ->");
            lcd.print(f_char);
            break;
          case 2:
            tBoilerDest=f_char;
            EEPROM.put(EEADDRESS_BOILER, f_char);
            lcd.print(" ->");
            lcd.print(f_char);
            break;
          case 3:
            tKesselDest=f_char;
            EEPROM.put(EEADDRESS_KESSEL, f_char);
            lcd.print(" ->");
            lcd.print(f_char);
            break;
          case 4:
            tRoomDiff=f_char;
            EEPROM.put(EEADDRESS_DIFFRAUM, f_char);
            lcd.print(" ->");
            lcd.print(f_char);
            break;
          case 5:
            tKesselDiff=f_char;
            EEPROM.put(EEADDRESS_DIFFKESSEL, f_char);
            lcd.print(" ->");
            lcd.print(f_char);
            break;
          case 6:
            tRoomNacht=f_char;
            EEPROM.put(EEADDRESS_RAUMNACHT, f_char);
            lcd.print(" ->");
            lcd.print(f_char);
            break;
          case 14:
            tBoilerDiff=f_char;
            EEPROM.put(EEADDRESS_DIFFBOILER, f_char);
            lcd.print(" ->");
            lcd.print(f_char);
            break;
          case 15:
            tvmax=f_char;
            EEPROM.put(EEADDRESS_TVMAX, f_char);
            lcd.print(" ->");
            lcd.print(f_char);
            break;
          case 16:
            taumin=f_char*-1.0;
            EEPROM.put(EEADDRESS_TAUMIN, f_char);
            lcd.print(" -> -");
            lcd.print(f_char);
            break;
          case 17:
            n=f_char;
            EEPROM.put(EEADDRESS_NN, f_char);
            lcd.print(" ->");
            lcd.print(f_char);
            break;
        }
        EEPROM.commit();
      //}
    }
    if ((MenuPage >= 7 && MenuPage <= 9) || MenuPage == 13 || MenuPage == 18) // case MENUPAGE_NUM:
    {
    //if(sizeof(keyBuffer)==(KEYSLENGTH_N+1)){
      int i_char;
      i_char = atoi(keyBuffer);
      if(i_char>1){i_char=1;}
      switch (MenuPage) {
        case 7:
          WinterBetrieb=i_char;
          EEPROM.put(EEADDRESS_WINTER, i_char);
          lcd.print(" ->");
          lcd.print(i_char);
          break;
        case 8:
          BoilerBetrieb=i_char;
          EEPROM.put(EEADDRESS_BOILER_SOMMERBETRIEB, i_char);
          lcd.print(" ->");
          lcd.print(i_char);
          break;
        case 9:
          NurHeizung=i_char;
          EEPROM.put(EEADDRESS_NUR_HEIZUNG, i_char);
          lcd.print(" ->");
          lcd.print(i_char);
          break;
        case 13:
          Sommerzeit_EinAus = (i_char != 0);
          EEPROM.put(EEADDRESS_SOMMERZEIT_EINAUS, Sommerzeit_EinAus);
          ntpupdate();
          Schaltuhr();
          lcd.print(" ->");
          lcd.print(Sommerzeit_EinAus);
          break;
        case 18:
          EEPROM.put(EEADDRESS_AUSSENTEMPREGELUNG, i_char);
          AussentemperaturRegelung=i_char;
          lcd.print(" ->");
          lcd.print(i_char);
          break;
        }
        EEPROM.commit();
      //}
    }
      if (MenuPage >= 10 && MenuPage <= 11){ //case MENUPAGE_TIME:
    //  if(sizeof(keyBuffer)==KEYSLENGTH_Z+1){
        float f_char;
        f_char = atof(keyBuffer);
        if(f_char<24){
        Serial.print(f_char);
        switch (MenuPage) {
          case 10:
            TagBegin=f_char;
            TagBeginHr = (int)(TagBegin);
            TagBeginMi = static_cast<int>(roundf((TagBegin - TagBeginHr)*100.0f));
            EEPROM.put(EEADDRESS_TAG, f_char);
            lcd.print(" ->");
            lcd.print(f_char);
            break;
          case 11:
            NachtBegin=f_char;
            NachtBeginHr = (int)(NachtBegin);
            NachtBeginMi = static_cast<int>(roundf((NachtBegin - NachtBeginHr)*100.0f));
            EEPROM.put(EEADDRESS_NACHT, f_char);
            lcd.print(" ->");
            lcd.print(f_char);
            break;
          }
          EEPROM.commit();
          lcd.print(" OK!");
        }
        else{
          lcd.print(" Ungültig!");
        }
    //  }
    }
    jumptoDefault=3;
}
/* *******************************************************************************************************
                                         MenüDisplay
******************************************************************************************************* */
void MenueMain() {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("0 >MainMenue");
    //lcd.setCursor(0,1);
    jumptoDefault=60;
}
void MenueDefault(){
    lcd.clear();
}
void MenueBooting(){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("System startet");
    Serial.print("System startet\n");
    lcd.setCursor(0,1);
    for (int i = 0; i <= 100; i++){  // you can change the increment value here
      lcd.setCursor(8,1);
      if (i<=100) {
        lcd.print(" ");
        //print a space if the percentage is < 100
        Serial.print(" ");
      }
      if (i<10) {
        lcd.print(" ");  //print a space if the percentage is < 10
        Serial.print(" ");
      }
      lcd.print(i);
      serial_clear_screen();
      Serial.print(i);
      lcd.print("%");
      Serial.print("%");
      delay(25);  //change the delay to change how fast the boot up screen changes
    }
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(sVers); // Start Print text to Line 1
    lcd.setCursor(0, 1);
    lcd.print(sPrgmName); // Start Print Test to Line 2
    jumptoDefault=60;
}
void MenuRoomTemp() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("1 >RoomTemp");
  lcd.setCursor(0,1);
  lcd.print("2  BoilerTemp");
  lcd.setCursor(0,2);
  char float_str[32];
  char line0[21];
  snprintf(float_str, sizeof(float_str), "%4.2f", static_cast<double>(tRoomTag));
  snprintf(line0, sizeof(line0), "TempNow: %-9sC", float_str); // %6s right pads the string
  lcd.print(line0);
  memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
  jumptoDefault=60;
}
void MenuBoilerTemp() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("2 >BoilerTemp");
  lcd.setCursor(0,1);
  lcd.print("3  KesselTemp");
  lcd.setCursor(0,2);
  char float_str[32];
  char line0[21];
  snprintf(float_str, sizeof(float_str), "%4.2f", static_cast<double>(tBoilerDest));
  snprintf(line0, sizeof(line0), "TempNow: %-9sC", float_str);
  lcd.print(line0);
  memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
  jumptoDefault=60;
}
void MenuKesseltemp() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("3 >KesselTemp");
  lcd.setCursor(0,1);
  lcd.print("4  DiffRaumtemp");
  lcd.setCursor(0,2);
  char float_str[32];
  char line0[21];
  snprintf(float_str, sizeof(float_str), "%4.2f", static_cast<double>(tKesselDest));
  snprintf(line0, sizeof(line0), "TempNow: %-9sC", float_str);
  lcd.print(line0);
  memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
  jumptoDefault=60;
}
void MenuDifRaumtemp() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("4 >DiffRaumtemp");
  lcd.setCursor(0,1);
  lcd.print("5  DiffKesselTemp");
  lcd.setCursor(0,2);
  char float_str[32];
  char line0[21];
  snprintf(float_str, sizeof(float_str), "%4.2f", static_cast<double>(tRoomDiff));
  snprintf(line0, sizeof(line0), "TempNow: %-9sC", float_str);
  lcd.print(line0);
  memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
  jumptoDefault=60;
}
void MenuDifKesseltemp() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("5 >DiffKesselTemp");
  lcd.setCursor(0,1);
  lcd.print("6  RaumNachtTemp");
  lcd.setCursor(0,2);
  char float_str[32];
  char line0[21];
  snprintf(float_str, sizeof(float_str), "%4.2f", static_cast<double>(tKesselDiff));
  snprintf(line0, sizeof(line0), "TempNow: %-9sC", float_str);
  lcd.print(line0);
  memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
  jumptoDefault=60;
}
void MenuRaumTempNacht(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("6 >RaumNachtTemp");
  lcd.setCursor(0,1);
  lcd.print("7  WinterBetrieb");
  lcd.setCursor(0,2);
  char float_str[32];
  char line0[21];
  snprintf(float_str, sizeof(float_str), "%4.2f", static_cast<double>(tRoomNacht));
  snprintf(line0, sizeof(line0), "TempNow: %-9sC", float_str);
  lcd.print(line0);
  memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
  jumptoDefault=60;
}
void MenuBetriebWinter(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("7 >WinterBetrieb");
  lcd.setCursor(0,1);
  lcd.print("8  BoilerBetrieb");
  lcd.setCursor(0,2);
  char line0[21];
  snprintf(line0, sizeof(line0), "WinterBetr: %d", WinterBetrieb);
  lcd.print(line0);
  memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
  jumptoDefault=60;
}
void MenuBetriebBoiler(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("8 >BoilerBetrieb");
  lcd.setCursor(0,1);
  lcd.print("9 NurHeizungBetr");
  lcd.setCursor(0,2);
  char line0[21];
  snprintf(line0, sizeof(line0), "BoilerBetrieb: %d", BoilerBetrieb);
  lcd.print(line0);
  memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
  jumptoDefault=60;
}
void MenuBetriebNurHeizung(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("9 >NurHeizungBetr");
  lcd.setCursor(0,1);
  lcd.print("10  Tagbetrieb");
  lcd.setCursor(0,2);
  char line0[21];
  snprintf(line0, sizeof(line0), "NurHeizBetr: %d", NurHeizung);
  lcd.print(line0);
  memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
  jumptoDefault=60;
}
void MenuHeizbeginnTag(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("10 >Tagbetrieb");
  lcd.setCursor(0,1);
  lcd.print("11  Nachtbetrieb");
  lcd.setCursor(0,2);
  char float_str[32];
  char line0[21];
  snprintf(float_str, sizeof(float_str), "%4.2f", static_cast<double>(TagBegin));
  snprintf(line0, sizeof(line0), "ZeitNow: %-5s", float_str);
  lcd.print(line0);
  memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
  jumptoDefault=60;
}
void MenuHeizEndeNacht(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("11 >Nachtbetrieb");
  lcd.setCursor(0,1);
  lcd.print("12 Uptime/NTP_Sync");
  lcd.setCursor(0,2);
  char float_str[32];
  char line0[21];
  snprintf(float_str, sizeof(float_str), "%4.2f", static_cast<double>(NachtBegin));
  snprintf(line0, sizeof(line0), "ZeitNow: %-5s", float_str);
  lcd.print(line0);
  memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
  jumptoDefault=60;
}
void MenuDifBoilerTemp() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("14 >DiffBoilerTemper");
  lcd.setCursor(0,1);
  lcd.print("15  VorlaufMaxTemp");
  lcd.setCursor(0,2);
  char float_str[32];
  char line0[21];
  snprintf(float_str, sizeof(float_str), "%4.2f", static_cast<double>(tBoilerDiff));
  snprintf(line0, sizeof(line0), "TempNow: %-9sC", float_str);
  lcd.print(line0);
  memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
  jumptoDefault=60;
}

void Menu_tvmax() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("15 >VorlaufMaxTemp");
  lcd.setCursor(0,1);
  lcd.print("16  AussenMinTemp");
  lcd.setCursor(0,2);
  char float_str[32];
  char line0[21];
  snprintf(float_str, sizeof(float_str), "%4.2f", static_cast<double>(tvmax));
  snprintf(line0, sizeof(line0), "TempNow: %-9sC", float_str);
  lcd.print(line0);
  memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
  jumptoDefault=60;
}
void Menu_taumin() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("16 >AussenMinTemp");
  lcd.setCursor(0,1);
  lcd.print("17  Kurvenfaktor n");
  lcd.setCursor(0,2);
  char float_str[32];
  char line0[21];
  snprintf(float_str, sizeof(float_str), "%4.2f", static_cast<double>(taumin));
  snprintf(line0, sizeof(line0), "TempNow: %-9sC", float_str);
  lcd.print(line0);
  memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
  jumptoDefault=60;
}
void Menu_n() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("17 >Kurvenfaktor n");
  lcd.setCursor(0,1);
  lcd.print("18 >AussentempRegel");
  lcd.setCursor(0,2);
  char float_str[32];
  char line0[21];
  snprintf(float_str, sizeof(float_str), "%4.2f", static_cast<double>(n));
  snprintf(line0, sizeof(line0), "FaktorNow: %-9s", float_str);
  lcd.print(line0);
  memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
  jumptoDefault=60;
}
void MenuAussentempRegelung(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("18 >AussentempRegel");
  lcd.setCursor(0,1);
  lcd.print("                    ");
  lcd.setCursor(0,2);
  char float_str[32];
  char line0[21];
  snprintf(float_str, sizeof(float_str), "%4.2f", static_cast<double>(n));
  snprintf(line0, sizeof(line0), "Ja(1)/Nein(0): %d", AussentemperaturRegelung);
  lcd.print(line0);
  memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
  jumptoDefault=60;
}

void Time2LCD(){
  if(mischer_wait>0)mischer_wait--;
  if(mischer_drive>0)mischer_drive--;
  char line0[21];
  if (doppelp==' '){
    doppelp=':';
  }else{
    doppelp=' ';
  }

  if (MenuPage==12){
    lcd.clear();
    lcd.setCursor(0,0);
    snprintf(line0, sizeof(line0), "UP: %ld, %02d:%02d%c%02d", uDay,uHour,uMinute,doppelp,uSecond);
    lcd.print(line0);
    lcd.setCursor(0, 1);
    lcd.printf("H%ld A%c %s %d",BrennerLaufzeit,Betriebsart,mqtt_payload,mqtt_message );
    lcd.setCursor(0, 2);
    lcd.printf("UpdTime:%ld, wr:%d",timeUpdated,wifi_retry);
    lcd.setCursor(0, 3);
    lcd.print(line0);
    lcd.setCursor(8,3);
    lcd.printf("CalVT:%.2f",vorlaufTemperatur);
    if (jump == 0){
      jump=1;
      jumptoDefault=60;
    }
 }
  if(MenuPage==0){
    print_Main_LCD_Values();
  }
}
/* *******************************************************************************************************
                                         Sommerzeit EIN / AUS
******************************************************************************************************* */
void MenuSommerzeitEinAus(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("13 >SommerzeitEINAUS");
  lcd.setCursor(0,1);
  lcd.print("14 DiffBoilerTemper ");
  lcd.setCursor(0,2);
  char line0[21];
  snprintf(line0, sizeof(line0), "Sommerzeit: %d", Sommerzeit_EinAus);
  lcd.print(line0);
  memset(keyBuffer, 0, sizeof keyBuffer);//Der Buffer wird geloescht
  jumptoDefault=10;
}
/* *******************************************************************************************************
                                         Schaltuhr
******************************************************************************************************* */
