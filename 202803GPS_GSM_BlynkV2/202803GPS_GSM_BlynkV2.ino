/*  GPS code
 *   Last update; 28/03/2020
 * 
 */
#include <DHT.h>
#include <Wire.h>
#include "MQ135.h"
#include "LowPower.h"         //https://github.com/LowPowerLab/LowPower
#include <TinyGPS++.h>        //https://github.com/mikalhart/TinyGPSPlus
#include <SimpleTimer.h>      //https://github.com/jfturcot/SimpleTimer
#include <Adafruit_INA219.h>  //https://github.com/adafruit/Adafruit_INA219
//***********************************************************
#define GSM_Serial Serial1
Stream* stream1 = &GSM_Serial;
//***********************************************************
TinyGPSPlus gps;
#define GPS_Serial Serial2
//***********************************************************
SimpleTimer timer;
Adafruit_INA219 ina219;
//***********************************************************
const char auth[] = "Blynk Auth Token";
//***********************************************************
// APN data
#define GPRS_APN       "mobile.vodafone.it"    // Replace your GPRS APN
#define GPRS_USER      ""           // Replace with your GPRS user
#define GPRS_PASSWORD  ""           // Replace with your GPRS password
//***********************************************************
#define GSM_Power   9 
#define PowerLED    3

#define pinMQ135    A0
#define DHTPIN      5
#define DHTTYPE     DHT22 
//***********************************************************
DHT dht(DHTPIN, DHTTYPE);
MQ135 AirSensor = MQ135(pinMQ135);
//***********************************************************
// Blynk cloud server
const char* host = "139.59.206.133";
//***********************************************************
float Temp, Humi;
                     //min , max
float limit_Temp[] = {15, 28};
float limit_Humi[] = {50, 90}; 

float latitude;
float longitude;

float ppm, cppm;
float busvoltage = 0;
//***********************************************************
bool DHT22Notif = false;
bool DHT22Notif_ac = false;

bool TempSMS = true;
bool HumiSMS = true;

bool BatteryState = true;

bool GPS_Sat = false;
bool GPS_Loc = false;
//***********************************************************
String number[] = {"+xxxxxxxxxxxx", "+xxxxxxxxxxxx", "+xxxxxxxxxxxx", "+xxxxxxxxxxxx"}; 
String Message = "";
String response;
//***********************************************************
unsigned long previousMillis = 0, previousMillisL = 0;
//***********************************************************
bool gprsInit();
bool gprsConnect();
bool gprsDisconnect();
//***********************************************************
void CheckBattery(){
  busvoltage = 0;
  float shuntvoltage = 0;
  float current_mA   = 0;
  float loadvoltage  = 0;
  float power_mW     = 0;

//  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
//  current_mA = ina219.getCurrent_mA();
//  power_mW = ina219.getPower_mW();
//  loadvoltage = busvoltage + (shuntvoltage / 1000);
  
  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
//  Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
//  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
//  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
//  Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
  Serial.println("");

  if(busvoltage < 11.90){
     BatteryState = false;
     Serial.println("Battery voltage is too low!");
     GSMpowerUpDown();
     check_SMS();
  }
  else{
    BatteryState = true;
  }
}
//***********************************************************
void GSMpowerUpDown(){
  Serial.println("Power Up/Down:");
  digitalWrite(GSM_Power,LOW);
  delay(1000);
  digitalWrite(GSM_Power,HIGH);
  delay(2000);
  digitalWrite(GSM_Power,LOW);
  delay(8000);
}
//***********************************************************
void GetLocation(){
  Serial.println("Getting location: ");
  while (GPS_Sat == false){
    while (GPS_Serial.available() > 0){
      if (gps.encode(GPS_Serial.read())){
        displayInfo();
      }
      if(GPS_Loc){break;}
    }
    if(GPS_Loc){break;}
    
    if (millis() > 5000 && gps.charsProcessed() < 10){
      Serial.println("No GPS detected");
      GPS_Sat = false;
    }
    
    if (millis() - previousMillis > 30000){   //wait 0.5min
      previousMillis = millis();
      break;
    }
  }
  GPS_Loc = false;
}
//***********************************************************
void displayInfo() {
  if (gps.location.isValid()) {
    Serial.println("---------------------------");
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6); 
    latitude = (gps.location.lat());
    longitude = (gps.location.lng());
    GPS_Loc = true;
  }
  else {
    Serial.println("Location: Not Available");
    GPS_Loc = false;
  }
  Serial.println("---------------------------");
  delay(1000);
}
//***********************************************************
void SendData(){
  CheckSensor();
  //------------
  GSMpowerUpDown();     //2
  //------------
  if(gprsInit()){       //3
    if(gprsConnect()){  //4
      sendValues();
      MapValues();
      gprsDisconnect();
      check_SMS();
      GSMpowerUpDown();
    }
    else{
      Serial.println("GSM Failed!!");
      check_SMS();
      GSMpowerUpDown();
    }
  }
  else{
    Serial.println("GPRS Failed!!");
      check_SMS();
      GSMpowerUpDown();
  }
}
//***********************************************************
void CheckSensor(){
  Serial.println("Check sensor: ");
  //------------
  Temp = dht.readTemperature();
  Humi = dht.readHumidity();
  //------------------------------------------------
  if (isnan(Temp) || isnan(Humi)) {
    DHT22Notif_ac = true;
    Temp = 0;
    Humi = 0;
  }
  else{
    DHT22Notif_ac = false;
    DHT22Notif = false;
  }
  ppm = AirSensor.getPPM();
  cppm = AirSensor.getCorrectedPPM(Temp, Humi);
}
//***********************************************************
void sendValues(){
  Serial.println("Sending values: ");
  // Send value to the Blynk
  //Temperature
  if (httpRequest("GET", String("/") + auth + "/update/V0?value=" + String(Temp), "", response)) {
    if (response.length() != 0) {
      Serial.print("WARNING: ");
      Serial.println(response);
    }
  }
  //Humidity
  if (httpRequest("GET", String("/") + auth + "/update/V1?value=" + String(Humi), "", response)) {
    if (response.length() != 0) {
      Serial.print("WARNING: ");
      Serial.println(response);
    }
  }
  //Co2 PPM
  if (httpRequest("GET", String("/") + auth + "/update/V2?value=" + String(ppm), "", response)) {
    if (response.length() != 0) {
      Serial.print("WARNING: ");
      Serial.println(response);
    }
  }
  //Co2 CPPM
  if (httpRequest("GET", String("/") + auth + "/update/V3?value=" + String(cppm), "", response)) {
    if (response.length() != 0) {
      Serial.print("WARNING: ");
      Serial.println(response);
    }
  }
  //latitude
  if (httpRequest("GET", String("/") + auth + "/update/V4?value=" + String(latitude, 6), "", response)) {
    if (response.length() != 0) {
      Serial.print("WARNING: ");
      Serial.println(response);
    }
  }
  //longitude
  if (httpRequest("GET", String("/") + auth + "/update/V5?value=" + String(longitude, 6), "", response)) {
    if (response.length() != 0) {
      Serial.print("WARNING: ");
      Serial.println(response);
    }
  }
  //Battery voltage
  if (httpRequest("GET", String("/") + auth + "/update/V6?value=" + String(busvoltage), "", response)) {
    if (response.length() != 0) {
      Serial.print("WARNING: ");
      Serial.println(response);
    }
  }
}
//***********************************************************
void MapValues(){
  //Map values
  String valueTemp = "&value=" + String(latitude, 6) + "&value=" + String(longitude, 6) + "&value=Temp:" + String(Temp) + "_Humi:" + String(Humi) + "_Co2:" + String(cppm);
  if (httpRequest("GET", String("/") + auth + "/update/V10?value=1" + valueTemp, "", response)) {
    if (response.length() != 0) {
      Serial.print("WARNING: ");
      Serial.println(response);
    }
  }
}
//***********************************************************
void check_SMS(){
  Serial.println("Check SMS: ");
  if(!BatteryState){
    Message = "Low battery";
    send_SMS();
    Serial.println("System is Shuting down!");
    delay(2000);
    GSMpowerUpDown();
    LowPower.idle(SLEEP_FOREVER, ADC_OFF, TIMER5_OFF, TIMER4_OFF, TIMER3_OFF, 
        TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART3_OFF, 
        USART2_OFF, USART1_OFF, USART0_OFF, TWI_OFF);
  }
  else{
    if(Temp > limit_Temp[0] && Temp < limit_Temp[1]){
      TempSMS = true;
    }
    if(Humi > limit_Humi[0] && Humi < limit_Humi[1]){
      HumiSMS = true;
    }
    if(Temp < limit_Temp[0]){
      //Send 4 SMS
      if(TempSMS){
        TempSMS = false;
        Message = "Low Temperatrue";
        send_SMS();
      }
    }
    else if(Temp > limit_Temp[1]){
      //Send 4 SMS
      if(TempSMS){
        TempSMS = false;
        Message = "High Temperatrue";
        send_SMS();
      }
    }
    if(Humi < limit_Humi[0]){
      //Send 4 SMS
      if(HumiSMS){
        HumiSMS = false;
        Message = "Low Humidity";
        send_SMS();
      }
    }
    else if(Humi > limit_Humi[1]){
      //Send SMS
      if(HumiSMS){
        HumiSMS = false;
        Message = "High Humidity";
        send_SMS();
      }
    }
  }
}
//***********************************************************
void setup(){
  Serial.begin(9600);
  //***********************************************************
  GSM_Serial.begin(9600);
  GPS_Serial.begin(9600);
  //***********************************************************
  dht.begin();
  //***********************************************************
  ina219.begin();
  //***********************************************************
  pinMode(GSM_Power, OUTPUT);       //GSM power ON switch
  pinMode(PowerLED, OUTPUT);        //LED power ON indicator
  pinMode(pinMQ135, INPUT);
  //***********************************************************
  digitalWrite(PowerLED, HIGH);
  delay(2000);
  digitalWrite(PowerLED, LOW);
  Serial.println("System is Powered UP!");
  //***********************************************************
  CheckBattery();
  GetLocation();
  SendData();
  //***********************************************************
  timer.setInterval(60000L, GetLocation);
  timer.setInterval(300000L, SendData);     //send data every 5mins
}
//***********************************************************
void loop(){
  timer.run();

  if (millis() - previousMillisL > 20000){
    previousMillisL = millis();
    CheckBattery();
  }
  
}
//***********************************************************
typedef const __FlashStringHelper* GsmConstStr;
//***********************************************************
void sendAT(const String& cmd) {
  stream1->print("AT");
  stream1->println(cmd);
}
//***********************************************************
uint8_t waitResponse(uint32_t timeout, GsmConstStr r1,
                     GsmConstStr r2 = NULL, GsmConstStr r3 = NULL) {
  String data;
  data.reserve(64);
  int index = 0;
  for (unsigned long start = millis(); millis() - start < timeout; ) {
    while (stream1->available() > 0) {
      int c = stream1->read();
      if (c < 0) continue;
      data += (char)c;
      if (data.indexOf(r1) >= 0) {
        index = 1;
        goto finish;
      } else if (r2 && data.indexOf(r2) >= 0) {
        index = 2;
        goto finish;
      } else if (r3 && data.indexOf(r3) >= 0) {
        index = 3;
        goto finish;
      }
    }
  }
finish:
  return index;
}
//***********************************************************
uint8_t waitResponse(GsmConstStr r1,
                     GsmConstStr r2 = NULL, GsmConstStr r3 = NULL){
  return waitResponse(1000, r1, r2, r3);
}
//***********************************************************
uint8_t waitOK_ERROR(uint32_t timeout = 2000) {
  return waitResponse(timeout, F("OK\r\n"), F("ERROR\r\n"));
}
//***********************************************************
void send_SMS(){
  stream1->println("AT+CMGF=1");    //Sets the GSM Module in Text Mode
  waitOK_ERROR();
  for(int i=0; i<4; i++){
    Serial.println ("Set SMS Number to: " + number[i]);
    stream1->println("AT+CMGS=\"" + number[i] + "\""); //Mobile phone number to send message
    delay(1000);
    stream1->print(Message);
    delay(1000);
    stream1->write(26);
    waitOK_ERROR();

    delay(2000);
  }
}
//***********************************************************
bool gprsInit() {
  Serial.print("GPRS init: ");
  sendAT(F("E0"));
  waitOK_ERROR();

  sendAT(F("+SAPBR=3,1,\"Contype\",\"GPRS\""));
  waitOK_ERROR();

  sendAT(F("+SAPBR=3,1,\"APN\",\"" GPRS_APN "\""));
  waitOK_ERROR();

#ifdef GPRS_USER
  sendAT(F("+SAPBR=3,1,\"USER\",\"" GPRS_USER "\""));
  waitOK_ERROR();
#endif
#ifdef GPRS_PASSWORD
  sendAT(F("+SAPBR=3,1,\"PWD\",\"" GPRS_PASSWORD "\""));
  waitOK_ERROR();
#endif

  sendAT(F("+CGDCONT=1,\"IP\",\"" GPRS_APN "\""));
  waitOK_ERROR();
  return true;
}
//***********************************************************
// Start the GSM connection
bool gprsConnect() {
  
  Serial.println("Connecting to GSM...");

  sendAT(F("+CGACT=1,1"));
  waitOK_ERROR(60000L);

  // Open a GPRS context
  sendAT(F("+SAPBR=1,1"));
  waitOK_ERROR(85000L);
  // Query the GPRS context
  sendAT(F("+SAPBR=2,1"));
  if (waitOK_ERROR(30000L) != 1)
    return false;

  Serial.println("GSM connected");
  return true;
}
//***********************************************************
bool gprsDisconnect() {
  sendAT(F("+SAPBR=0,1"));
  if (waitOK_ERROR(30000L) != 1)
    return;

  sendAT(F("+CGACT=0"));
  waitOK_ERROR(60000L);

  Serial.println("GSM disconnected");
  return true;
}
//***********************************************************
int httpRequest(const String& method,
                const String& url,
                const String& request,
                String&       response)
{
  Serial.print(F("  Request: "));
  Serial.print(host);
  Serial.println(url);

  sendAT(F("+HTTPTERM"));
  waitOK_ERROR();

  sendAT(F("+HTTPINIT"));
  waitOK_ERROR();

  sendAT(F("+HTTPPARA=\"CID\",1"));
  waitOK_ERROR();

#ifdef USE_HTTPS
  sendAT(F("+HTTPSSL=1"));
  waitOK_ERROR();
  sendAT(String(F("+HTTPPARA=\"URL\",\"https://")) + host + url + "\"");
  waitOK_ERROR();
#else
  sendAT(String(F("+HTTPPARA=\"URL\",\"")) + host + url + "\"");
  waitOK_ERROR();
#endif

  if (request.length()) {
    sendAT(F("+HTTPPARA=\"CONTENT\",\"application/json\""));
    waitOK_ERROR();
    sendAT(String(F("+HTTPDATA=")) + request.length() + "," + 10000);
    waitResponse(F("DOWNLOAD\r\n"));
    stream1->print(request);
    waitOK_ERROR();
  }

  if (method == "GET") {
    sendAT(F("+HTTPACTION=0"));
  } else if (method == "POST") {
    sendAT(F("+HTTPACTION=1"));
  } else if (method == "HEAD") {
    sendAT(F("+HTTPACTION=2"));
  } else if (method == "DELETE") {
    sendAT(F("+HTTPACTION=3"));
  }
  waitOK_ERROR();

  if (waitResponse(30000L, F("+HTTPACTION:")) != 1) {
    Serial.println("HTTPACTION Timeout");
    return false;
  }
  stream1->readStringUntil(',');
  int code = stream1->readStringUntil(',').toInt();
  size_t len = stream1->readStringUntil('\n').toInt();

  if (code != 200) {
    Serial.print("Error code:");
    Serial.println(code);
    sendAT(F("+HTTPTERM"));
    waitOK_ERROR();
    return false;
  }

  response = "";

  if (len > 0) {
    response.reserve(len);

    sendAT(F("+HTTPREAD"));
    if (waitResponse(10000L, F("+HTTPREAD: ")) != 1) {
      Serial.println("HTTPREAD Timeout");
      return false;
    }
    len = stream1->readStringUntil('\n').toInt();

    while (len--) {
      while (!stream1->available()) {
        delay(1);
      }
      response += (char)(stream1->read());
    }
    waitOK_ERROR();
  }

  sendAT(F("+HTTPTERM"));
  waitOK_ERROR();

  return true;
}
//***********************************************************
