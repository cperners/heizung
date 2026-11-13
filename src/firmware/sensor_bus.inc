void sensorDS1820_indicateChip(byte pin)
{
  if (pin > kTtureSensorMaxIndex || sensorFamily[pin] != 0) return;
  const unsigned long now = millis();
  if (sensorFamilyAttempted[pin] && now - sensorFamilyLastAttempt[pin] < 30000UL) return;
  sensorFamilyAttempted[pin] = true;
  sensorFamilyLastAttempt[pin] = now;
  // Each existing sensor bus has one sensor (temperature commands use Skip ROM).
  // Read its ROM directly, avoiding the extra bit exchanges of Search ROM.
  byte address[8];
  if (!sensorDS1820[pin].reset()) { markSensorFailure(pin); return; }
  sensorDS1820[pin].write(0x33, POWER_MODE); // Read ROM
  for (byte i = 0; i < sizeof(address); ++i) address[i] = sensorDS1820[pin].read();
  if (OneWire::crc8(address, 7) != address[7]) {
    markSensorFailure(pin);
    return;
  }
  if (address[0] != 0x10 && address[0] != 0x28 && address[0] != 0x22) {
    markSensorFailure(pin);
    return;
  }
  sensorFamily[pin] = address[0];
}
//#################################################################################################################################
void sensorDS1820_reset(byte pin)
{
  if (pin > kTtureSensorMaxIndex) return;
  sensorDS1820[pin].reset();
  sensorDS1820[pin].write(0xCC, POWER_MODE);
  sensorDS1820[pin].write(0x44, POWER_MODE);
}
//#################################################################################################################################
void sensorDS1820_read(byte pin)
{
  if (pin > kTtureSensorMaxIndex) return;
  if (sensorFamily[pin] == 0) { markSensorFailure(pin); return; }
  float temperature;
  //byte bufData[9];
  if (!sensorDS1820[pin].reset()) { markSensorFailure(pin); return; }
  sensorDS1820[pin].write(0xCC, POWER_MODE);
  sensorDS1820[pin].write(0xBE, POWER_MODE);
  //sensorDS1820[pin].read_bytes(bufData, 9);
//  if(OneWire::crc8(bufData,8)==bufData[8]){
    //data is correct
/*****************************************/

byte data[12];
int16_t raw;

for ( int i = 0; i < 9; i++)
    { data[i] = sensorDS1820[pin].read(); }
if(validSensorScratchpad(data)){

  raw = (data[1] << 8) | data[0];
  if (sensorFamily[pin] == 0x10)
    {
    raw = static_cast<int16_t>(int32_t(raw) * 8);
    if (data[7] == 0x10)
      // Vorzeichen expandieren
      { raw = (raw & 0xFFF0) + 12 - data[6]; }
    }
  else
    {
    byte cfg = (data[4] & 0x60);
    // Aufloesung bestimmen, bei niedrigerer Aufloesung sind
    // die niederwertigen Bits undefiniert -> auf 0 setzen
    if (cfg == 0x00) raw = raw & ~7;      //  9 Bit Aufloesung,  93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 Bit Aufloesung, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 Bit Aufloesung, 375.0 ms
    // Default ist 12 Bit Aufloesung, 750 ms Wandlungszeit
    }
  temperature = ((float)raw / 16.0);
  if (!markSensorSuccess(pin, temperature)) return;

/*****************************************/
  //  temperature= ((float)((int)((unsigned int)bufData[0] | (((unsigned int)bufData[1]) << 8)))) * 0.0625 + 0.03125;
     switch (bWhichSensor) {
        case 0:
          //bez = bez + MQTT_TEXT + "S" + bWhichSensor + "Kessel";
          if(tKessel!=temperature+OS0){
            temp_update=true;
            tKessel = temperature+OS0;
          }
          break;
        case 1:
          //bez = bez + MQTT_TEXT + "S" + bWhichSensor + "Vorlauf";
          if(tVorlauf != temperature+OS1){
            temp_update=true;
            tVorlauf = temperature+OS1;
          }
          break;
        case 2:
          //bez = bez + MQTT_TEXT + "S" + bWhichSensor + "Aussen";
          if(tAussen != temperature+OS2){
            tAussen = temperature+OS2;
            temp_update=true;
          }
          break;
        case 3:
          //bez = bez + MQTT_TEXT + "S" + bWhichSensor + "Kueche";
          if(tRoom != temperature+OS3){
            tRoom = temperature+OS3;
            temp_update=true;
          }
          break;
        case 4:
          //bez = bez + MQTT_TEXT + "S" + bWhichSensor + "Boiler";
          if(tBoiler != temperature+OS4){
            tBoiler = temperature+OS4;
            temp_update=true;
          }
          break;
        default:
          break;
     }
  } else {
    markSensorFailure(pin);
  }
}
//#################################################################################################################################
