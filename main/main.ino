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
float sumOfVoltage = 0;

int Sensor=0;
int counter=1;
int checkSum=0;

unsigned long period=1000;
unsigned long previousMillis=0;
void scanIMEI();
void sendData();
noDelay elapseTime(60000, sendData);

char IMEI[15];//serial number
void sendData(){
  String msg=String("AT+HTTPPARA=\"URL\",\"http://iotstls.herokuapp.com/add/data?data=861508039275185*" + String(volt()) + "*" + String(computeCurrent())+"*" +String(lightSensor()) + "*1*"+ String(sumOfCurrent*(sumOfVoltage/(float)checkSum))+"\"");
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
  delay(1000);
  readGPRS();
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
  initGSM();
  initGPRS();
  pinMode(switchPin,OUTPUT);
  digitalWrite(switchPin,HIGH);delay(1000);Serial.println(computeCurrent());
  pinMode(pir_pin,INPUT);
  pinMode(LDR,INPUT);
  pinMode(actuator,OUTPUT);//to switch between different light intensity.
  //digitalWrite(actuator,HIGH);delay(1000);Serial.println(computeCurrent());
}

void loop() {
  //elapseTime.fupdate();
  //sensePerson();
  //sumOfCurrent = sumOfCurrent + computeCurrent(currentSensorPin);
  //sumOfVoltage = sumOfVoltage + volt();
  //checkSum++;
  delay(1000);
}
void readGPRS()
{
  while(GPRS.available())
  Serial.write(GPRS.read());
}
