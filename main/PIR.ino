void sensePerson()
{
  if(digitalRead(pir_pin) == HIGH){
    motion = true;
    motionAt = millis();
    digitalWrite(actuator,HIGH);
  }
} 
