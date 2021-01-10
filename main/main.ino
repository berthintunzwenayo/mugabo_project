#include <Filters.h>
#include <SoftwareSerial.h>
#include<NoDelay.h>
SoftwareSerial GPRS(7,8);//TX,RX
RunningStatistics inputStats;
#define pir_pin 13 
#define switchPin 11
#define currentSensorPin A2  
#define voltageSensorPin A1  
#define LDR A3
#define actuator 12
#define testFrequency 50

float windowLength=40.0/testFrequency;
float intercept=0.0;
float slope=0.0405;
float current_Volts;
float sumOfCurrent = 0;

int Sensor=0;
int counter=1;

unsigned long period=1000;
unsigned long previousMillis=0;
void scanIMEI();
void sendData();
noDelay elapseTime(60000, sendData);

char IMEI[15];//serial number
void sendData(){
  String msg=String("AT+HTTPPARA=\"URL\",\"http://iotstls.herokuapp.com/add/data?data=861508039275185*" + String(volt()) + "*" + String(computeCurrent())+"*" +String(lightSensor()) + "*1*"+ String(sumOfCurrent*volt())+"\"");
  Serial.print("sending: ");
  Serial.println(msg);
  GPRS.println("AT+HTTPTERM");//TERMINATE HTTP service
  delay(1000);
  readGPRS();
  GPRS.println("AT+HTTPINIT");//initialize HTTP service
  delay(1000);
  readGPRS();
  GPRS.println("AT+HTTPPARA=\"CID\",1");//set parameters of HTTP session
  delay(1000);
  readGPRS();
  GPRS.println(msg);
  //GPRS.println("AT+HTTPPARA=\"URL\",\"http://iotstls.herokuapp.com/add/data?data=86150803927518*250*1*100*1\"");
  delay(1000);
  readGPRS();
  //connectGSM(msg,"OK",1000);//set parameters of HTTP session
  connectGSM("AT+HTTPACTION=0","+HTTPACTION:0,200",10000);//submit HTTP GET request
  counter++;
  sumOfCurrent = 0;
}
 
void setup() {
  Serial.begin(19200);
  GPRS.begin(19200);
  inputStats.setWindowSecs(windowLength);
  //testAT();
  //scanIMEI();
  //initGSM();
  //initGPRS();
  pinMode(switchPin,OUTPUT);
  digitalWrite(switchPin,HIGH);
  pinMode(pir_pin,INPUT);
  pinMode(LDR,INPUT);
  pinMode(actuator,OUTPUT);//to switch between different light intensity.
  //digitalWrite(actuator,HIGH);
}

void loop() {
  //volt();
  //elapseTime.fupdate();
  //sensePerson();
  //sumOfCurrent = sumOfCurrent + computeCurrent(currentSensorPin);
  //testAT();
  //float a = analogRead(currentSensorPin);
  //Serial.println(float((a*5000.0)/1024.0));
  //String msg=String("AT+HTTPPARA=\"URL\",\"http://iotstls.herokuapp.com/add/data?data=861508039275185*" + String(volt()) + "*" + String(computeCurrent())+"*" +String(lightSensor()) + "*1*"+ String(sumOfCurrent*volt())+"\"");
  Serial.println(volt());
  delay(1000);
}
void readGPRS()
{
  while(GPRS.available())
  Serial.write(GPRS.read());
}
