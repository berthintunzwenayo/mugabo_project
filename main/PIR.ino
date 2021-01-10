void sensePerson()
{
  String condition;
  if(digitalRead(pir_pin) == HIGH){
    
  }
  condition = digitalRead(pir_pin) == HIGH ? "Standard" : "Medium";
  if(condition.equals("Standard")){
    digitalWrite(actuator,HIGH);
  }
  else if(condition.equals("Medium")){
    digitalWrite(actuator,LOW);
  }
  delay(1000);
} 
