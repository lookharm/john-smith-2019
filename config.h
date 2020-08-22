#ifndef CONFIG_H
#define CONFIG_H

//Left motor [dri2]
#define MOTOR_L_INA 14
#define MOTOR_L_INB 5
#define MOTOR_L_OUTA 18
#define MOTOR_L_OUTB 17
//Left motor - PWM channel(esp32)[1, 2]
#define MOTOR_L_CA 1
#define MOTOR_L_CB 2
//Right motor [dri1]
#define MOTOR_R_INA 16
#define MOTOR_R_INB 15
#define MOTOR_R_OUTA 2
#define MOTOR_R_OUTB 4
//Right motor - PWM channel(esp32)[3, 4]
#define MOTOR_R_CA 3
#define MOTOR_R_CB 4
//Sensor
#define LED_IR 19
#define SENSOR_1 13//left
#define SENSOR_2 25//left
#define SENSOR_3 27//right
#define SENSOR_4 26//right
//Switch
#define SW_1 34
#define SW_2 35

//Flash Memory
#define EEPROM_SIZE 512
#define FOUND_ADD 0
#define NOTFOUND_ADD 16
#define THRESHOLD_ADD 32
#define SHORTEST_PATH_LEN 48
#define SHORTEST_PATH 49

//field
#define ROW 6//16
#define COL 7//16
//robot
//point (TOP, LEFT) = 0, 0
//point (BOTTOM, RIGHT) = ROW - 1, COL - 1
#define STEP_ONE_BLOCK 210//7.7 V
#define START_PY 5//15
#define START_PX 0//0
//N,E,S,W
#define START_DIRECTION 'N'
//y,x
#define END_POINTS {{3, 2}, {3, 3}, {4, 2}, {4, 3}}

#endif
