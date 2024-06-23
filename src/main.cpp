#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <SoftwareSerial.h>
#include "DHT.h"

// Distance sensor
#define trigPin A4
#define echoPin A3

// Humidity sensor
#define DHT11_PIN A5

// Right motor
#define M1PWM 9
#define M1DIR 7

// Left Motor
#define M2PWM 10
#define M2DIR 8

// Motors speed
#define speed 150
#define turnSpeed 100

// Tasks for robot's modules
void TaskHCSR4(void *pvParameters);
void TaskXM15(void *pvParameters);
void TaskDHT11(void *pvParameters);
void TaskMotors(void *pvParameters);

// Objects for semaphore, bluetooth module and dht11 sensor
SemaphoreHandle_t xSemaphore;
SoftwareSerial BTSerial(2, 3); // RX, TX
DHT dht;

// Functions to control directions of motors
void goForward();
void goBackward();
void turnRight();
void turnLeft();
void stopMotors();

// Function getting distance from HC-SR04 sensor
int getDistance();

// Variables for sensors
volatile byte hcsr04_sampling = 5; // frequency of detection in Hz
volatile int obstacleDistance = 0;
volatile int temperature = 0;
volatile int humidity = 0;
volatile int minPeriod = dht.getMinimumSamplingPeriod();

String command = "";
String previousCommand = "";

void setup()
{
  // Initialize communication and configure pins
  Serial.begin(9600);
  BTSerial.begin(9600);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(M1PWM, OUTPUT);
  pinMode(M1DIR, OUTPUT);
  pinMode(M2PWM, OUTPUT);
  pinMode(M2DIR, OUTPUT);

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

  xTaskCreate(
      TaskMotors, "ControlMotors", 128, NULL, 2, NULL);
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
      // Serial.print(obstacleDistance); // debug only
      // Serial.println(" cm");          // debug only
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
        command.trim();
        Serial.println(command); // debug only

        if (command == "data")
        {
          BTSerial.print("Distance: ");
          BTSerial.println(obstacleDistance);
          BTSerial.print("Humidity: ");
          BTSerial.println(humidity);
          BTSerial.print("Temperature: ");
          BTSerial.println(temperature);
        }
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

      // if (dht.getStatusString() == "OK") // debug only
      // {                                  //
      //   Serial.print(humidity);          //
      //   Serial.print("%RH | ");          //
      //   Serial.print(temperature);       //
      //   Serial.println("*C");            //
      // } //
      xSemaphoreGive(xSemaphore);
      vTaskDelay(minPeriod / portTICK_PERIOD_MS);
    }
  }
}

void TaskMotors(void *pvParameters)
{
  (void)pvParameters;

  for (;;)
  {
    if (xSemaphoreTake(xSemaphore, (TickType_t)5) == pdTRUE)
    {
      if (command != previousCommand)
      {
        // Serial.println(command); // debug only
        if (command == "forward")
        {
          stopMotors();
          goForward();
        }
        else if (command == "backward")
        {
          stopMotors();
          goBackward();
        }
        else if (command == "right")
        {
          stopMotors();
          turnRight();
        }
        else if (command == "left")
        {
          stopMotors();
          turnLeft();
        }
        else if (command == "stop")
        {
          stopMotors();
        }
      }
      previousCommand = command;
      xSemaphoreGive(xSemaphore);
    }
  }
}

void goForward()
{
  digitalWrite(M1DIR, HIGH);
  digitalWrite(M2DIR, HIGH);
  analogWrite(M1PWM, speed);
  analogWrite(M2PWM, speed);
}

void goBackward()
{
  digitalWrite(M1DIR, LOW);
  digitalWrite(M2DIR, LOW);
  analogWrite(M1PWM, speed);
  analogWrite(M2PWM, speed);
}

void turnRight()
{
  digitalWrite(M1DIR, HIGH);
  digitalWrite(M2DIR, LOW);
  analogWrite(M1PWM, turnSpeed);
  analogWrite(M2PWM, turnSpeed);
}

void turnLeft()
{
  digitalWrite(M1DIR, LOW);
  digitalWrite(M2DIR, HIGH);
  analogWrite(M1PWM, turnSpeed);
  analogWrite(M2PWM, turnSpeed);
}

void stopMotors()
{
  analogWrite(M1PWM, 0);
  analogWrite(M2PWM, 0);
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
