#include <Wire.h>
#include <DS3231.h>
#include <ADS1256.h>
#include <U8g2lib.h>
#include <ArduinoJson.h>

// SHT25 I2C address is 0x40(64)
#define Addr 0x40
const int Relay = 27;
#define u8g2_PIN_NONE 100

U8G2_ST7567_ENH_DG128064I_F_SW_I2C u8g2(U8G2_R0, SCL, SDA, U8X8_PIN_NONE);

static char buffer[64] = ""; // Buffer to store the incoming string  
static SemaphoreHandle_t mutex1;
static SemaphoreHandle_t mutex2;

StaticJsonDocument<10> document;
const char* dn;
volatile float humidity ;
volatile float Temp ;
volatile int threshold=0;
volatile static int sample = 10;
volatile float pressure;
volatile static int SAMPLEDELAY =10000;

#define PERIOD_GET_DATA_FROM_SENSORS (TickType_t)(SAMPLEDELAY / portTICK_PERIOD_MS)

String relayon="OPEN";
String relayoff="CLOSE";

ADS1256 A(32, 33, 0, 5, 1.500);  //DRDY, RESET, SYNC(PDWN), CS, VREF(float).   //ESP32 WROOM 32

#define RXD2 16
#define TXD2 17
TaskHandle_t TaskHandle_1;
TaskHandle_t TaskHandle_2;
TaskHandle_t TaskHandle_3;
TaskHandle_t TaskHandle_4;
TaskHandle_t TaskHandle_5;
TaskHandle_t TaskHandle_6;

//--------------------------------------------
void setup()
 {
  // Initialise I2C communication as MASTER
  Wire.begin();
  // Initialise serial communication, set baud rate = 9600
  Serial.begin(1200);
  Serial2.begin(1200, SERIAL_8N1, RXD2, TXD2);

  u8g2.setI2CAddress(0x7E);  //(0x3F * 2);
  u8g2.begin(); 
  Serial.println("ADS1256");

  A.InitializeADC();       //See the documentation for every details
  A.setPGA(PGA_1);         //0b00000000 - DEC: 0
  A.setMUX(DIFF_6_7);      //0b01100111 - DEC: 103
  A.setDRATE(DRATE_5SPS);  //0b00010011 - DEC: 19

  Serial.print("PGA: ");
  Serial.println(A.readRegister(IO_REG));
  delay(100);

  Serial.print("MUX: ");
  Serial.println(A.readRegister(MUX_REG));
  delay(100);

  Serial.print("DRATE: ");
  Serial.println(A.readRegister(DRATE_REG));
 
  mutex1 = xSemaphoreCreateMutex();  //mutex
  if (mutex1 != NULL)
  {
    Serial.println("Mutex1 created");
  }
  mutex2 = xSemaphoreCreateMutex();  //mutex
  if (mutex2 != NULL)
  {
    Serial.println("Mutex2 created");
  }
  xTaskCreatePinnedToCore(TaskSendData, "TaskSendData", 1024 * 4, NULL,2,&TaskHandle_1,tskNO_AFFINITY);
  xTaskCreatePinnedToCore(TaskSetting, "TaskSetting", 1024 * 2, NULL, 1, &TaskHandle_2,tskNO_AFFINITY);
  xTaskCreatePinnedToCore(TaskPressure,  "TaskPressure", 1024 * 2, NULL, 4, &TaskHandle_3,tskNO_AFFINITY);
  xTaskCreatePinnedToCore(TaskMonitor,  "TaskMonitor", 1024 * 4, NULL, 3, &TaskHandle_4,tskNO_AFFINITY);
  xTaskCreatePinnedToCore(TaskSensors, "TaskSensors", 1024 * 4, NULL, 2,&TaskHandle_5,tskNO_AFFINITY);
  xTaskCreatePinnedToCore(TaskSensors1, "TaskSensors1", 1024 * 4, NULL, 2,&TaskHandle_6,tskNO_AFFINITY);
}

void loop() 
{
}

void TaskSetting(void *pvParameters)
{

  int index = 0;   // Index for the buffer

  while(1)
  {   
    if (xSemaphoreTake(mutex2, portTICK_PERIOD_MS) == pdTRUE)
    {
      Serial.print(pcTaskGetName(NULL));

      while (Serial2.available() && index < sizeof(buffer) - 1)
      {
        char c = Serial2.read();
        if (c == '\n') 
        {
          buffer[index] = '\0'; 
          index = 0; 
        } else 
        {
          buffer[index++] = c; 
        }
      }
    vTaskDelay(200/portTICK_PERIOD_MS);
    DeserializationError error = deserializeJson(document, buffer);
    threshold = document["threshold"];
    sample = document["sample_rate"];
    dn = document["dn"];
    SAMPLEDELAY= (sample) * 1000;
    if(SAMPLEDELAY ==0)
    {
      SAMPLEDELAY=10000;
    }
    xSemaphoreGive(mutex2);
    document["sample_rate"]=sample;
    document["threshold"]=threshold;
    document["dn"]=dn;
    serializeJson(document, Serial2);             
    vTaskDelay(300/portTICK_PERIOD_MS);
    }
  }
}

void TaskSensors(void *pvParameters) 
{  
  while(1)
  {
    if (xSemaphoreTake(mutex1, portTICK_PERIOD_MS) == pdTRUE)
    { 
      Serial.print(pcTaskGetName(NULL));
      volatile unsigned int data[2];
      // Start I2C transmission
      vTaskDelay(500/portTICK_PERIOD_MS);
      Wire.beginTransmission(Addr);
      // Send humidity measurement command, NO HOLD master
      Wire.write(0xF5);
      // Stop I2C transmission
      Wire.endTransmission();
      vTaskDelay(500/portTICK_PERIOD_MS);
      // Request 2 bytes of data
      Wire.requestFrom(Addr, 2);
      if ( Wire.available() == 2)
      {
        data[0] = Wire.read();
        data[1] = Wire.read();
      // Convert the data
      float humidity1 = (((data[0] * 256.0 + data[1]) * 125.0) / 65536.0) - 6;  //Huminity
      // Output data to Serial Monitor
      }
      // Add newline character to mark the end of the JSON
      // Start I2C transmission
      Wire.beginTransmission(Addr);
      // Send temperature measurement command, NO HOLD master
      Wire.write(0xF3);
      
      // Stop I2C transmission
      Wire.endTransmission();
      vTaskDelay(500/portTICK_PERIOD_MS);
      // Request 2 bytes of data
      Wire.requestFrom(Addr, 2);
      // Read 2 bytes of data
      // temp msb, temp lsb
      if ( Wire.available() == 2)
      {
      data[0] = Wire.read();
      data[1] = Wire.read();
      // Convert the data
      Temp = (((data[0] * 256.0 + data[1]) * 175.72) / 65536.0) - 46.85;  //Temperature in Celsius
      xSemaphoreGive(mutex1);
      }
    }
    vTaskDelay(1000/portTICK_PERIOD_MS);
  }
}
void TaskSensors1(void *pvParameters) 
{  
  while(1)
  {
    if (xSemaphoreTake(mutex1, portTICK_PERIOD_MS) == pdTRUE)
    { 
      Serial.print(pcTaskGetName(NULL));
      volatile unsigned int data[2];
      Wire.beginTransmission(Addr);
      // Send temperature measurement command, NO HOLD master
      Wire.write(0xF3);
      
      // Stop I2C transmission
      Wire.endTransmission();
      vTaskDelay(500/portTICK_PERIOD_MS);
      // Request 2 bytes of data
      Wire.requestFrom(Addr, 2);
      // Read 2 bytes of data
      // temp msb, temp lsb
      if ( Wire.available() == 2)
      {
      data[0] = Wire.read();
      data[1] = Wire.read();
      // Convert the data
      volatile float Temp1 = (((data[0] * 256.0 + data[1]) * 175.72) / 65536.0) - 46.85;  //Temperature in Celsius  
      }
      // Start I2C transmission
      Wire.beginTransmission(Addr);
      // Send humidity measurement command, NO HOLD master
      Wire.write(0xF5);
      // Stop I2C transmission
      Wire.endTransmission();
      vTaskDelay(500/portTICK_PERIOD_MS);
      Wire.requestFrom(Addr, 2);
        if ( Wire.available() == 2)
        {
         data[0] = Wire.read();
         data[1] = Wire.read();
        humidity = (((data[0] * 256.0 + data[1]) * 125.0) / 65536.0) - 6;  //Huminity
        xSemaphoreGive(mutex1);
        }
    }
    vTaskDelay(2000/portTICK_PERIOD_MS);
  }
}
void TaskPressure(void *pvParameters)
{
    TickType_t  task_lastWakeTime = xTaskGetTickCount();
    while (1) 
  {
    if (xSemaphoreTake(mutex2, portTICK_PERIOD_MS) == pdTRUE)
     {
      Serial.print(pcTaskGetName(NULL));
      Serial.println();
      pressure = A.convertToVoltage(A.cycleSingle());
      xSemaphoreGive(mutex2);
     }
    vTaskDelayUntil(&task_lastWakeTime, PERIOD_GET_DATA_FROM_SENSORS);
  }
}
void TaskSendData(void *pvParameters) 
{
  TickType_t  task_lastWakeTime = xTaskGetTickCount();
  while (1) 
  {
    if (xSemaphoreTake(mutex1, portTICK_PERIOD_MS) == pdTRUE)
     {
      Serial.print(pcTaskGetName(NULL));
      Serial.println();
        if (xSemaphoreTake(mutex2, portTICK_PERIOD_MS) == pdTRUE)
        {      
        document["Pressure"] = pressure;
        xSemaphoreGive(mutex2);    
        }
        document["Temp"] = Temp;
        document["Humi"] = humidity;
        xSemaphoreGive(mutex1);    
        serializeJson(document,Serial2);        
        vTaskDelayUntil(&task_lastWakeTime, PERIOD_GET_DATA_FROM_SENSORS);    
    }
  }
}

void TaskMonitor(void *pvParameters)
{
  pinMode(Relay, OUTPUT);
  while(1)
  {    
    if (xSemaphoreTake(mutex1, portMAX_DELAY) == pdTRUE)
    {   

      if(sample == 0)
      {
        sample = 10;
      }
      if(pressure> threshold)
      {
        digitalWrite(Relay,LOW);
      } else
      {
        digitalWrite(Relay,HIGH);
      }      
      Serial.print(pcTaskGetName(NULL));
      Serial.println(); 
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_ncenB08_tr); 
      u8g2.setCursor(40, 10);
      u8g2.print(dn);             
      u8g2.setCursor(0, 20);
      u8g2.print("Temp:  ");
      u8g2.print(Temp);
      u8g2.println(" C");
      u8g2.setCursor(0, 30);
      u8g2.print("Humi:  ");
      u8g2.print(humidity);
      u8g2.println(" %RH");
      u8g2.setCursor(0, 40);
      u8g2.print("Pressure:  ");
      u8g2.print(pressure);
      u8g2.println(" mbar");                 
      u8g2.setCursor(0, 50);
      u8g2.print("Threshold:  ");
      u8g2.print(threshold);
      if(pressure > threshold)
      {
        u8g2.setCursor(0, 60);
        u8g2.print("Valve:  ");
        u8g2.print(relayon);
      } else
      {
        u8g2.setCursor(0, 60);
        u8g2.print("Valve:  ");
        u8g2.print(relayoff);
      }   
      u8g2.sendBuffer();
      xSemaphoreGive(mutex1);
    }
    vTaskDelay(300/portTICK_PERIOD_MS);
  }
}         