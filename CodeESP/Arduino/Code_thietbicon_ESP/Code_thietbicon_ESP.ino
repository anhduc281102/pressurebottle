#include <ArduinoJson.h>
#include <SoftwareSerial.h>
SoftwareSerial mySerial(17, 16);
//D7:RX, D8:TX
void setup()
{
  Serial.begin(1200);
  mySerial.begin(1200);
}

void loop()
{
 
    StaticJsonDocument<200> doc;
    doc["No"] =2;
    // doc["Time"]=21022023;
    doc["Temp"]=32;
    doc["Humi"]=1;
    doc["Pressure"]=1;
    doc["Threshold"]=32;
    serializeJson(doc, mySerial);
    // float Pam_setting = (float)mySerial.read();
    // Serial.println(Pam_setting);
   // Serial.println();
     delay(2000);
}