void scanIMEI()
{ 
  byte counter=1;
  String response;
  while(1){
    GPRS.println("AT+GSN"); 
    while(GPRS.available()){
      char c=GPRS.read();
      response+=c;
    }
    if(response.length()>0 && response.indexOf("OK")>=0 && response.indexOf("AT+GSN")>=0){
      String temp;
      for(int i=0;i<response.length();i++){
        if(response[i]>=48 && response[i]<=57)
        temp+=response[i];
      }
      if(temp.length()==15){
        temp.toCharArray(IMEI,15);
        Serial.print("IMEI: ");Serial.println(IMEI);//Serial.print(" ");
        //Serial.println(temp.length());
        return ;//break;
      }
    }
    else{
      if(counter<=10){
        delay(1000);
        counter++;
        GPRS.println("AT+GSN"); 
      }
      else{ 
        Serial.println("Timeout detected, thus Unable to obtain Serial Number.");
        return ;
      }
    }
  }
}
