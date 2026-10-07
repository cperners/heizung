void print_Main_LCD_Values(){
  if (!lcdPresent) return;
  char rows[4][21];
  for (unsigned r=0; r<4; ++r) { memset(rows[r], ' ', 20); rows[r][20]='\0'; }
  auto put = [&](unsigned row, unsigned column, const char* text) {
    for (unsigned i=0; text[i] && column+i<20; ++i) rows[row][column+i]=text[i];
  };
  auto temperature = [](char* out, size_t size, byte sensor, float value) {
    if (!sensorIsUsable(sensor) || !isfinite(value)) snprintf(out,size,"X");
    else snprintf(out,size,"%4.1f",static_cast<double>(value));
  };
  char text[32], first[16], second[16];
  snprintf(text,sizeof(text),"Br%sH%sB%sA%sZ%s",BrennerRelais?"+":"-",HeizungsRelais?"+":"-",BoilerRelais?"+":"-",MischerAufRelais?"+":"-",MischerZuRelais?"+":"-");
  put(0,0,text);
  snprintf(text,sizeof(text),"%cR=%4.1f",daynight,static_cast<double>(tmyRoomdest)); put(0,12,text);
  temperature(first,sizeof(first),0,tKessel); temperature(second,sizeof(second),1,tVorlauf);
  snprintf(text,sizeof(text),"H:%-5s V:%-5s",first,second); put(1,0,text);
  snprintf(text,sizeof(text),"A%d",AussentemperaturRegelung); put(1,16,text);
  snprintf(text,sizeof(text),"P%d",Pumpenloesen); put(1,18,text);
  temperature(first,sizeof(first),2,tAussen); temperature(second,sizeof(second),3,tRoom);
  snprintf(text,sizeof(text),"A:%-5s R:%-5s",first,second); put(2,0,text);
  snprintf(text,sizeof(text),"SZ%d",Sommerzeit_EinAus); put(2,17,text);
  temperature(first,sizeof(first),4,tBoiler);
  snprintf(text,sizeof(text),"B:%-5s",first); put(3,0,text);
  snprintf(text,sizeof(text),"%02d.%02d. %02d%c%02d",myday,mymonth,myhours,doppelp,myminutes); put(3,8,text);
  static char previous[4][20];
  static uint32_t previousGeneration = UINT32_MAX;
  const bool redraw = previousGeneration != lcdClearGeneration;
  for (unsigned row=0; row<4; ++row) {
    unsigned col=0;
    while (col<20) {
      if (!redraw && previous[row][col]==rows[row][col]) { ++col; continue; }
      lcd.setCursor(col,row);
      do { lcd.write(rows[row][col]); previous[row][col]=rows[row][col]; ++col; }
      while (col<20 && (redraw || previous[row][col]!=rows[row][col]));
    }
  }
  previousGeneration=lcdClearGeneration;
}

boolean summertime_EU(int year, byte month, byte day, byte hour, byte tzHours)
// European Daylight Savings Time calculation by "jurs" for German Arduino Forum
// input parameters: "normal time" for year, month, day, hour and tzHours (0=UTC, 1=MEZ)
// return value: returns true during Daylight Saving Time, false otherwise
{
  if (month<3 || month>10) return false; // keine Sommerzeit in Jan, Feb, Nov, Dez
  if (month>3 && month<10) return true; // Sommerzeit in Apr, Mai, Jun, Jul, Aug, Sep
  if ((month==3 && ((hour + 24 * day)>=(1 + tzHours + 24*(31 - (5 * year /4 + 4) % 7)))) || ((month==10) && ((hour + 24 * day)<(1 + tzHours + 24*(31 - (5 * year /4 + 1) % 7))))){
    return true;
  }else{
    return false;
  }
}

//#################################################################################################################################
//                        I2C_IO_Extender
//#################################################################################################################################
