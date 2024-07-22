#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include <cstring>
#include <stdio.h>
#include <String.h>
SoftwareSerial mySerial(16, 17);
TaskHandle_t Task1; 
TaskHandle_t Task2;
static char buffer[64]; // Buffer to store the incoming string  
SemaphoreHandle_t mutex;
StaticJsonDocument<7> doc;
void setup() 
{
Serial.begin(1200);
mySerial.begin(1200);
mutex = xSemaphoreCreateMutex();
if (mutex != NULL) {
  Serial.println("Mutex created");
}
xTaskCreate(Setting, "Setting", 4096, NULL, 1, &Task1);
xTaskCreate(Sensors, "Sensors", 1024*16, NULL, 1, &Task2);

}

void loop() {}

void Setting(void *pvParameters)
{

  int index = 0;   // Index for the buffer

  for (;;)
  {   
    if (xSemaphoreTake(mutex, portTICK_PERIOD_MS) == pdTRUE)
    {
      //Serial.print(pcTaskGetName(NULL));

      while (mySerial.available() && index < sizeof(buffer) - 1)
      {
        char c = mySerial.read();
        if (c == '\n') {
          buffer[index] = '\0'; 
          Serial.println(buffer); 
          index = 0; 
        } else {
          buffer[index++] = c; 
        }
      }

      xSemaphoreGive(mutex);
      vTaskDelay(100);
    }
  }
}

void Sensors(void *pvParameters)
{

  for (;;)
  {
     if (xSemaphoreTake(mutex, portTICK_PERIOD_MS) == pdTRUE)
    {
      Serial.print(pcTaskGetName(NULL));
      Serial.println();
      Serial.println(buffer);
      String buff=(String)buffer;
      String threshold=buff.substring(13,15);
      int threshold_int=threshold.toInt();
      xSemaphoreGive(mutex);
      Serial.println(threshold_int);
      doc["Pressure"]=1;
      doc["No"] =2;
      doc["Temp"]=32;
      doc["Humi"]=1;
      doc["Threshold"]=threshold_int;  
      serializeJson(doc, mySerial);
      serializeJson(doc, Serial);
      vTaskDelay(1000);
    }
    }
    
  }