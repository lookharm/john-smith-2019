#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>
#include "config.h"

#define LED_ON digitalWrite(LED_IR, HIGH)
#define LED_OFF digitalWrite(LED_IR, LOW)
#define SENSOR_LEFT_FRONT analogRead(SENSOR_1)
#define SENSOR_LEFT_SIDE analogRead(SENSOR_2)
#define SENSOR_RIGHT_SIDE analogRead(SENSOR_3)
#define SENSOR_RIGHT_FRONT analogRead(SENSOR_4)

String sName[4] = {"LS", "LF", "RF", "RS"};
int found[4] = {};
int64_t fSum[4] = {};
int16_t fCount[4] = {};
int notFound[4] = {};
int64_t nSum[4] = {};
int16_t nCount = 0;
int tLeftSide = 0;
int tLeftFront = 0;
int tRightFront = 0;
int tRightSide = 0;

void initSensors() {
  pinMode(LED_IR, OUTPUT);
  pinMode(SENSOR_1, INPUT);
  pinMode(SENSOR_2, INPUT);
  pinMode(SENSOR_3, INPUT);
  pinMode(SENSOR_4, INPUT);
}

void calibrateSensors() {
  
}

//relation i(x) = 1/(x^2)
//x = distance
//i = intensity
//trend is 1/(x^2)
void rcRead(int pin, bool led) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delayMicroseconds(2); //charge for 2 us
  pinMode(pin, INPUT);

  noInterrupts();
  int c = 0;
  if (led) digitalWrite(LED_IR, HIGH);
  for (int i = 0; i < 500; i++) {
    if (digitalRead(pin)) {
      Serial.print('*');
      c++;
    }
    delayMicroseconds(2);
  }
  if (led) digitalWrite(LED_IR, LOW);
  //  Serial.print(c);
  Serial.println();
  delay(10);
  interrupts();
}

#endif
