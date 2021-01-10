float volt()
{
  /*while(1){
    Sensor=analogRead(voltageSensorPin);
    inputStats.input(Sensor);
    if((unsigned long)(millis() - previousMillis) >= period){
      previousMillis=millis();
      current_Volts = intercept + slope * inputStats.sigma();
      current_Volts = current_Volts * (40.3231)*1.57432;
      //Serial.println(current_Volts);
      break;
    }
  }
  return (int)current_Volts;*/
  float analogV = analogRead(voltageSensorPin);
  float voltage = (analogV*5.0/1024.0);
  return voltage;
}
