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
#define ROW 5//16
#define COL 5//16
//robot
#define STEP_ONE_BLOCK 215//7.7 V
#define START_PY 0//15
#define START_PX 0//0
#define START_DIRECTION 'S'//N,E,S,W
#define END_POINTS {{3, 0}, {3, 1}, {4, 0}, {4, 1}} //{7,8},{7,9},{8,8},{8,9}//y,x

#endif
