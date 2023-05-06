/*
    ProMini_Sensor BME280 + HC12
    = vlapa = 20230319 - 20230319
    v.001
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <LowPower.h>

const uint8_t numSensor = 22;

const uint8_t plus_BME = 11;
const uint8_t gnd_BME = 10;
const uint8_t plus_HC12 = 2;

Adafruit_BME280 bme;

void setup()
{
  Serial.begin(9600);
  Serial.println();

  pinMode(gnd_BME, OUTPUT);
  pinMode(plus_BME, OUTPUT);
}

void loop()
{
  digitalWrite(gnd_BME, LOW);
  digitalWrite(plus_BME, HIGH);
  digitalWrite(plus_HC12, HIGH);
  delay(100);
  bme.begin(0x76);

  float t = bme.readTemperature();
  uint16_t p = (int)(bme.readPressure() / 100 * 0.750064);
  uint8_t h = (int)(bme.readHumidity());

  Serial.print(numSensor);
  Serial.print('*');
  Serial.print(t);
  Serial.print('#');
  Serial.print(p);
  Serial.print('%');
  Serial.print(h);
  Serial.println(';');

  delay(100);
  digitalWrite(plus_BME, LOW);
  digitalWrite(plus_HC12, LOW);
  delay(5);

  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
}