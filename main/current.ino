float OffSet = 0.01;//voltage when there is no load.
byte scaleFactor=66;//30A module
float computeCurrent()
{
  /*float p2pVolt = analogCurrent(sensorPin);
  float VRMS = ((p2pVolt-OffSet)/2.0) *0.707; //root 2 is 0.707 – dealing with sine
  float iRms = (VRMS * 1000)/scaleFactor;
  return int(iRms);*/
  float analogI = analogRead(currentSensorPin);
  float current = ((analogI-8)*5.0*1000.0/(100.0*1024.0));
  return current;
}

float analogCurrent(byte current_sensor)
{
float result;
int readValue; // value read from the sensor
int maxValue = 0; // store max value here
int minValue = 1024; // store min value here

uint32_t start_time = millis();
while((millis()-start_time) < 1000) //sample for 1 Sec
{
readValue = analogRead(current_sensor);
// see if you have a new maxValue
if (readValue > maxValue)      //record the maximum sensor value
{
maxValue = readValue;
}
if (readValue < minValue)    //record the minimum sensor value
{
minValue = readValue;
}
}
result=((maxValue-minValue)*5.0)/1024.0; // Subtract min from max
return result;
}
