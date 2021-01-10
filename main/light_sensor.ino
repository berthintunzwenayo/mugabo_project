int lightSensor()
{
  float sensorValue=0;
  int percentage=0;
  
  sensorValue=analogRead(LDR);
  percentage=map(sensorValue,0,1000,0,100);
  delay(250);
  
  return percentage;
}
