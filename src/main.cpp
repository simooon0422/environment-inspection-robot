#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <SoftwareSerial.h>
#include "DHT.h"

#define trigPin 4
#define echoPin 5
#define DHT11_PIN 6

void TaskHCSR4(void *pvParameters);
void TaskXM15(void *pvParameters);
void TaskDHT11(void *pvParameters);

SemaphoreHandle_t xSemaphore;
SoftwareSerial BTSerial(2, 3); // RX, TX
DHT dht;

int getDistance();

volatile byte hcsr04_sampling = 20; // frequency of detection in Hz
volatile int obstacleDistance;
volatile int temperature;
volatile int humidity;
volatile int minPeriod = dht.getMinimumSamplingPeriod();

String command = "";

void setup()
{
  Serial.begin(9600);
  BTSerial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  dht.setup(DHT11_PIN);

  if (xSemaphore == NULL)
  {
    xSemaphore = xSemaphoreCreateMutex();
    if ((xSemaphore) != NULL)
      xSemaphoreGive((xSemaphore));
  }

  xTaskCreate(
      TaskHCSR4, "GetObstacleDistance", 128, NULL, 2, NULL);

  xTaskCreate(
      TaskXM15, "GetBluetoothCommand", 128, NULL, 2, NULL);

  xTaskCreate(
      TaskDHT11, "GetTemperatureHumidity", 128, NULL, 2, NULL);
}

void loop()
{
}

void TaskHCSR4(void *pvParameters)
{
  (void)pvParameters;

  for (;;)
  {
    if (xSemaphoreTake(xSemaphore, (TickType_t)5) == pdTRUE)
    {
      obstacleDistance = getDistance();
      // Serial.print(obstacleDistance); //debug only
      // Serial.println(" cm"); //debug only
      xSemaphoreGive(xSemaphore);
      vTaskDelay(((1 / hcsr04_sampling) * 1000) / portTICK_PERIOD_MS);
    }
  }
}

void TaskXM15(void *pvParameters)
{
  (void)pvParameters;

  for (;;)
  {
    if (xSemaphoreTake(xSemaphore, (TickType_t)5) == pdTRUE)
    {
      if (BTSerial.available())
      {
        command = BTSerial.readStringUntil('\n');
        // Serial.println(command); // debug only
      }
      xSemaphoreGive(xSemaphore);
    }
  }
}

void TaskDHT11(void *pvParameters)
{
  (void)pvParameters;

  for (;;)
  {
    if (xSemaphoreTake(xSemaphore, (TickType_t)5) == pdTRUE)
    {
      humidity = dht.getHumidity();
      temperature = dht.getTemperature();

      if (dht.getStatusString() == "OK") // debug only
      {                                  //
        Serial.print(humidity);          //
        Serial.print("%RH | ");          //
        Serial.print(temperature);       //
        Serial.println("*C");            //
      }                                  //
      xSemaphoreGive(xSemaphore);
      vTaskDelay(minPeriod / portTICK_PERIOD_MS);
    }
  }
}

int getDistance()
{
  long time, distance;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  time = pulseIn(echoPin, HIGH);
  distance = time / 58;

  return distance;
}
