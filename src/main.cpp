#include <Arduino.h>
#include <Arduino_FreeRTOS.h>

#define trigPin 4
#define echoPin 5

void TaskGetObstacleDistance(void *pvParameters);

int getDistance();

volatile byte hcsr04_sampling = 20;
volatile int obstacleDistance;

void setup()
{
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  xTaskCreate(
      TaskGetObstacleDistance, "GetObstacleDistance", 128, NULL, 2, NULL);
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
    Serial.print(obstacleDistance); //debug only
    Serial.println(" cm"); //debug only
    vTaskDelay(((1 / hcsr04_sampling) * 1000) / portTICK_PERIOD_MS);
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
