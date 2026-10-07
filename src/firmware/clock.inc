void Schaltuhr() {
  if (!ntpTimeValid) {
    return;
  }

  const long mytime = 100L * myhours + myminutes;
  const long t_begin = 100L * TagBeginHr + TagBeginMi;
  const long t_ende = 100L * NachtBeginHr + NachtBeginMi;

  // Bisherige Grenzen des Zeitprogramms beibehalten.
  daynight = (mytime > t_begin && mytime < t_ende) ? 'D' : 'N';
}
/* *******************************************************************************************************
                                         mytime update - every seconds
******************************************************************************************************* */
void setmyuhr(){
  set2myuhr=true;
}
void myuhr() {
  ntpupdate();
  Schaltuhr();

  // Vorhandene Brenner-Laufzeitzählung beibehalten.
  if (BrennerRelais) {
    brsecunds++;
  }
  if (brsecunds >= 60) {
    brminutes += brsecunds / 60;
    brsecunds %= 60;
  }
  if (brminutes >= 60) {
    brhours += brminutes / 60;
    brminutes %= 60;
  }
}
/* *******************************************************************************************************
                                         NTP update - via ticker every 24 hours
******************************************************************************************************* */
void dayticker(){
  if(dayticker_hr<=23){dayticker_hr++;
  }else{
    dayticker_hr=0;
    Schaltuhr();
  }
  mqttRecon=true;//mqtt neu starten
  previousTime_mqtt=millis();//mqtt neu starten
}
/* *******************************************************************************************************
                                         NTP update
******************************************************************************************************* */
void ntpupdate() {
  const time_t utcTime = time(nullptr);

  if (utcTime < static_cast<time_t>(1577836800UL)) {
    ntpTimeValid = false;
    snprintf(TimeString, sizeof(TimeString), "NTP nicht synchron");
    return;
  }

  if (!ntpTimeValid) {
    timeUpdated++;
  }
  ntpTimeValid = true;

  time_t localTime =
      utcTime + GMT_TIME_ZONE * utcOffsetInSeconds;
  struct tm calendar = {};
  if (gmtime_r(&localTime, &calendar) == nullptr) {
    return;
  }

  // Sommerzeit auf die Normalzeit anwenden, danach Datum neu berechnen.
  if (Sommerzeit_EinAus &&
      summertime_EU(calendar.tm_year + 1900,
                    calendar.tm_mon + 1,
                    calendar.tm_mday,
                    calendar.tm_hour,
                    GMT_TIME_ZONE)) {
    localTime += 3600;
    if (gmtime_r(&localTime, &calendar) == nullptr) {
      return;
    }
  }

  myyear = calendar.tm_year + 1900;
  mymonth = calendar.tm_mon + 1;
  myday = calendar.tm_mday;
  myweekday = calendar.tm_wday;
  myhours = calendar.tm_hour;
  myminutes = calendar.tm_min;
  mysecunds = calendar.tm_sec;

  const char dd = (mysecunds % 2) ? ' ' : ':';
  snprintf(TimeString, sizeof(TimeString),
           "%s, %02u:%02u%c%02u",
           daysOfTheWeek[myweekday],
           static_cast<unsigned>(myhours),
           static_cast<unsigned>(myminutes),
           dd,
           static_cast<unsigned>(mysecunds));

  if (!MenuPage) {
    lcd.setCursor(0, 3);
    lcd.print(TimeString);
  }
}
/* *******************************************************************************************************
                                         WiFiEvent
******************************************************************************************************* */
