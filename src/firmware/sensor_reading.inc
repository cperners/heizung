bool OneWireReset(int Pin) {
   // Preserve the long-line reset duration, sampling presence in the same
   // transaction rather than resetting once with the library and again here.
   digitalWrite(Pin, LOW);
   pinMode(Pin, OUTPUT);
   delayMicroseconds(550);
   noInterrupts();
   pinMode(Pin, INPUT);
   delayMicroseconds(70);
   const bool present = digitalRead(Pin) == LOW;
   interrupts();
   delayMicroseconds(430);
   return present;
}

void OneWireOutByte(int Pin, byte d) // output byte d (least sig bit first).
{
   byte n;
   for(n=8; n!=0; n--)
   {
      if ((d & 0x01) == 1)  // test least sig bit
      {
         digitalWrite(Pin, LOW);
         pinMode(Pin, OUTPUT);
         delayMicroseconds(5);
         pinMode(Pin, INPUT);
         delayMicroseconds(60);
      }
      else
      {
         digitalWrite(Pin, LOW);
         pinMode(Pin, OUTPUT);
         delayMicroseconds(60);
         pinMode(Pin, INPUT);
      }
      d=d>>1; // now the next bit is in
              // the least sig bit position.
   }
}//end OneWireOutByte

byte OneWireInByte(int Pin) // read byte, least sig byte first
{
  byte d, b;
  d=0;
/*This critical line added 04 Oct 16
    I hate to think how many derivatives of
      this code exist elsewhere on my web pages
      which have NOT HAD this. You may "get away"
      with not setting d to zero here... but it
      is A Very Bad Idea to trust to "hidden"
      initializations!
    The matter was brought to my attention by
      a kind reader who was THINKING OF YOU!!!
    If YOU spot an error, please write in, bring
      it to my attention, to save the next person
      grief.*/
// habe im internet das gefunden (chris) https://crazy-electronic.de/index.php/arduino/23-temperatur-messen-mit-ds18b20
   for (bLoopCounter=0; bLoopCounter<8; bLoopCounter++)
     {
      noInterrupts();//perni neu
      digitalWrite(Pin, LOW);
      pinMode(Pin, OUTPUT);
      delayMicroseconds(3);//perni war 5
      pinMode(Pin, INPUT);
      delayMicroseconds(10);//perni war 5
      b = digitalRead(Pin);
      interrupts();
      delayMicroseconds(53);//perni war 50
      d = (d >> 1) | (b << 7); // shift d to right and
         //insert b in most sig bit position
     }
   return(d);
}//end OneWireInByte()

void readTturePt1(byte Pin){
   sensorDS1820_indicateChip(BOILER_NUMBER);
   if (!sensorDS1820[BOILER_NUMBER].reset()) {
     boilerDiagnostic = BOILER_NO_RESPONSE;
     markSensorFailure(BOILER_NUMBER);
     return;
   }
   sensorDS1820[BOILER_NUMBER].write(0xCC, POWER_MODE);
   sensorDS1820[BOILER_NUMBER].write(0x44, POWER_MODE);
}


void readTturePt2(byte Pin, const byte tmp_bWhichSensor){
   if (tmp_bWhichSensor != BOILER_NUMBER) return;
   OneWire& wire = sensorDS1820[tmp_bWhichSensor];
   if (!wire.reset()) {
     boilerDiagnostic = BOILER_NO_RESPONSE;
     markSensorFailure(tmp_bWhichSensor);
     return;
   }
   wire.write(0xCC, POWER_MODE);
   wire.write(0xBE, POWER_MODE);
   byte scratchpad[9];
   for (byte i = 0; i < 9; ++i) scratchpad[i] = wire.read();
   portENTER_CRITICAL(&boilerDiagnosticMux);
   memcpy(boilerRawBytes, scratchpad, sizeof(scratchpad));
   boilerRawAvailable = true;
   portEXIT_CRITICAL(&boilerDiagnosticMux);
   if (!validSensorScratchpad(scratchpad)) {
     bool allZero = true, allHigh = true;
     for (byte i = 0; i < 9; ++i) {
       allZero &= scratchpad[i] == 0;
       allHigh &= scratchpad[i] == 0xff;
     }
     boilerDiagnostic = allZero ? BOILER_ZERO_DATA : (allHigh ? BOILER_HIGH_DATA : BOILER_BAD_CRC);
     markSensorFailure(tmp_bWhichSensor);
     return;
   }
   if (sensorFamily[tmp_bWhichSensor] == 0) {
     boilerDiagnostic = BOILER_BAD_VALUE;
     markSensorFailure(tmp_bWhichSensor);
     return;
   }
   int16_t raw = static_cast<int16_t>((scratchpad[1] << 8) | scratchpad[0]);
   if (sensorFamily[tmp_bWhichSensor] == 0x10) {
     raw = static_cast<int16_t>(int32_t(raw) * 8);
     if (scratchpad[7] == 0x10) raw = (raw & 0xFFF0) + 12 - scratchpad[6];
   } else {
     const byte resolution = scratchpad[4] & 0x60;
     if (resolution == 0x00) raw &= ~7;
     else if (resolution == 0x20) raw &= ~3;
     else if (resolution == 0x40) raw &= ~1;
   }
   const float measured = raw / 16.0f;
   if (!markSensorSuccess(tmp_bWhichSensor, measured)) return;
   if (tBoiler != measured + OS4) {
     tBoiler = measured + OS4;
     temp_update = true;
   }
}//end readTturePt2


void printTture(){//Uses values from global variables.
   if (bWhichSensor > kTtureSensorMaxIndex) return;
   String temp;
   if (Whole[bWhichSensor] < 10)
   /* To line up decimal points. This assumes that no tture will be < -99.9 or > +99.0 As these are in degrees C, that seems reasonable.
   And if a very high tture is measured, it will only slightly disturb the contents of the serial monitor.*/
   {
      temp = " ";
   }
   if (SignBit[bWhichSensor]) // If it is negative
     {
       temp = temp + "-";
     }//no ; here
   temp = temp + Whole[bWhichSensor];
   temp = temp + ".";
   if (Fract[bWhichSensor] < 10)
   {
      temp = temp + "0";
   }
   temp = temp + Fract[bWhichSensor];
   char buf[32];
   snprintf(buf, sizeof(buf), "%s", temp.c_str());

     switch (bWhichSensor) {
        case 0:
          //bez = bez + MQTT_TEXT + "S" + bWhichSensor + "Kessel";
          if(tKessel!=atof(buf)+OS0){
            temp_update=true;
            tKessel = atof(buf)+OS0;
          }
          break;
        case 1:
          //bez = bez + MQTT_TEXT + "S" + bWhichSensor + "Vorlauf";
          if(tVorlauf != atof(buf)+OS1){
            temp_update=true;
            tVorlauf = atof(buf)+OS1;
          }
          break;
        case 2:
          //bez = bez + MQTT_TEXT + "S" + bWhichSensor + "Aussen";
          if(tAussen != atof(buf)+OS2){
            tAussen = atof(buf)+OS2;
            temp_update=true;
          }
          break;
        case 3:
          //bez = bez + MQTT_TEXT + "S" + bWhichSensor + "Kueche";
          if(tRoom != atof(buf)+OS3){
            tRoom = atof(buf)+OS3;
            temp_update=true;
          }
          break;
        case 4:
          //bez = bez + MQTT_TEXT + "S" + bWhichSensor + "Boiler";
          if(tBoiler != atof(buf)+OS4){
            tBoiler = atof(buf)+OS4;
            temp_update=true;
          }
          break;
        default:
          break;
     }
}//end of printTture()



/* *******************************************************************************************************
                                         UPTIME
******************************************************************************************************* */
