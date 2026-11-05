void uptime(){
  //** Making Note of an expected rollover *****//
  if(millis()>=3000000000){
  uHighMillis=1;
  }
  //** Making note of actual rollover **//
  if(millis()<=100000&&uHighMillis==1){
  uRollover++;
  uHighMillis=0;
  }
  long secsUp = millis()/1000;
  uSecond = secsUp%60;
  uMinute = (secsUp/60)%60;
  uHour = (secsUp/(60*60))%24;
  uDay = (uRollover*50)+(secsUp/(60*60*24));  //First portion takes care of a rollover [around 50 days]
}
/* *******************************************************************************************************
                                         Print UPTIME
******************************************************************************************************* */
void print_Uptime(){
  Serial.print(F("Uptime: ")); // The "F" Portion saves your SRam Space
  Serial.print(uDay);
  Serial.print(F("  Days  "));
  Serial.print(uHour);
  Serial.print(F("  Hours  "));
  Serial.print(uMinute);
  Serial.print(F("  Minutes  "));
  Serial.print(uSecond);
  Serial.println(F("  Seconds"));
};
/* *******************************************************************************************************
                                         AutomatikBetrieb
******************************************************************************************************* */
