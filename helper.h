#ifndef HELPER_H
#define HELPER_H

#include "config.h"

#define LED_ON digitalWrite(LED_IR, HIGH)
#define LED_OFF digitalWrite(LED_IR, LOW)
#define SW1_PUSHED !digitalRead(SW_1)
#define SW2_PUSHED !digitalRead(SW_2)
#define SENSOR_LEFT_FRONT analogRead(SENSOR_1)
#define SENSOR_LEFT_SIDE analogRead(SENSOR_2)
#define SENSOR_RIGHT_SIDE analogRead(SENSOR_3)
#define SENSOR_RIGHT_FRONT analogRead(SENSOR_4)

#endif
