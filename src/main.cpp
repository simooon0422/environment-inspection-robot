#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <SoftwareSerial.h>

#define trigPin 4
#define echoPin 5

void TaskGetObstacleDistance(void *pvParameters);
void TaskGetBluetoothCommand(void *pvParameters);

int getDistance();

volatile byte hcsr04_sampling = 20;
volatile int obstacleDistance;

String command = "";

SoftwareSerial BTSerial(2, 3); // RX, TX

void setup()
{
  Serial.begin(9600);
  BTSerial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  xTaskCreate(
      TaskGetObstacleDistance, "GetObstacleDistance", 128, NULL, 2, NULL);

  xTaskCreate(
      TaskGetBluetoothCommand, "GetBluetoothCommand", 128, NULL, 2, NULL);
}

void loop()
{
}

void TaskGetObstacleDistance(void *pvParameters)
{
  (void)pvParameters;

  for (;;)
  {
    obstacleDistance = getDistance();
    // Serial.print(obstacleDistance); //debug only
    // Serial.println(" cm"); //debug only
    vTaskDelay(((1 / hcsr04_sampling) * 1000) / portTICK_PERIOD_MS);
  }
}

void TaskGetBluetoothCommand(void *pvParameters)
{
  (void)pvParameters;

  for (;;)
  {
    if (BTSerial.available())
    {
      command = BTSerial.readStringUntil('\n');
      Serial.println(command); //debug only
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
