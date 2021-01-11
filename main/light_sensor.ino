int lightSensor()
{
  /*float sensorValue=0;
  int percentage=0;
  sensorValue=analogRead(LDR);
  if(sensorValue<910)
  sensorValue=910;
  else if(sensorValue>920)
  sensorValue=920;
  percentage=map(sensorValue,910,920,0,100);
  delay(250);
  return percentage;*/
  if(motion)
  return 100;
  else
  return 50;
}
