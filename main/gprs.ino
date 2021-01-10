
void initGSM()           // init GSM module
{
  connectGSM("AT","OK",1000); //
  connectGSM("ATE1","OK",1000);  //Echo Enabled
  connectGSM("AT+CPIN?","READY",1000);
}/*
void initGPRS()
{
  GPRS.println("AT");delay(2000);
  readGPRS();
  //GPRS.println("AT+CGATT=0");delay(2000);
  //readGPRS();
  GPRS.println("AT+CGATT=1");delay(5000);
  readGPRS();
  GPRS.println("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"");delay(5000);
  readGPRS();
  GPRS.println("AT+SAPBR=3,1,\"APN\",\"RCELL\"");delay(5000);
  readGPRS();
  GPRS.println("AT+SAPBR=1,1");delay(5000);
  readGPRS();
  GPRS.println("AT+SAPBR=2,1");delay(5000);
  readGPRS();
  Serial.println("done initializing");
}*/
void initGPRS()                 // init GPRS in GSM Module
{
  connectGSM("AT+CIPSHUT","OK",1000); //stop connection if any
  connectGSM("AT+CGATT=0","OK",1000);  //DE-Attach for GPRS service
  connectGSM("AT+CGATT=1","OK",1000);  //Attach for GPRS service
  connectGSM("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"","OK",5000);
  connectGSM("AT+SAPBR=3,1,\"APN\",\"RCELL\"","OK",5000);
  connectGSM("AT+SAPBR=1,1","OK",5000);
  connectGSM("AT+SAPBR=2,1","OK",5000);
  Serial.println("done initializing");
}
void connectGSM (String cmd, char *res, int mls)
{
  while(1)
  {
    Serial.println(cmd);
    GPRS.println(cmd);
    delay(mls);
    while(GPRS.available()>0)
    {
      Serial.write(GPRS.read());
      if(GPRS.find(res))
      {
        delay(1000);
        return;
      }
    }
   }
 }
