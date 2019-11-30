#include <QueueArray.h>
#include <Adafruit_NeoPixel.h>

#include "config.h"
#include "helper.h"
#include "EEPROM.h"
#include "EEPROM_S.h"
#include "Oled.h"
#include "Motor.h"

//mode
#define MAX_MODE 12

/***
   TODO
   -acc sigmoid
   -slide side infared
   -add switch debounce
   -add level shifter
   -add free wheeling diode
   -SD card, ship EEPROM
   -Read battery
*/


/***
   TODO FAST
   -Trackforward Check front wall to stop.
   -Turn Left Forward, Turn Right Forward -> delay()
   -Shortest Path 1, if found target. Shortest Path 2 if found home.
   -set zero for turnLeftForward and turnRightForward if have wall.
   -optimize code
*/

Adafruit_NeoPixel pixels(3, 23, NEO_GRB + NEO_KHZ800);
Oled oled;
Motor motorR;
Motor motorL;

//field
uint8_t endPoints[4][2] = END_POINTS; //{7,8},{7,9},{8,8},{8,9}//y,x
//current position of robot
uint8_t py = START_PY;
uint8_t px = START_PX;
char direction = START_DIRECTION;
//Sensors
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
//mode
uint8_t mode = 0;
String modeName[MAX_MODE] = {
  "Calibrate sensor", "Show state", "Mapping",
  "ShortestPath 1", "ShortestPath 2", "ShortestPath 3",
  "turnLeftFordward", "trackForward", "turnRightFordward",
  "turnLeft", "turnRight"
};

//walls
bool verticalWalls[ROW][COL + 1];
bool horizontalWalls[ROW + 1][COL];
//block
struct Block {
  uint8_t y;
  uint8_t x;
  int16_t value;
  uint8_t count;
  bool flag;
  bool mark;
};
Block blocks[ROW][COL];
//f = forward, l = turn left, r = turn right
//first run after found target(only).
//second run after found target and back home.
String shortestPath = "";

void setup() {
  Serial.begin(115200);

  pixels.begin();
  pixels.clear();

  EEPROM_init(EEPROM_SIZE);
  oled.init();
  motorR.init(MOTOR_R_INA, MOTOR_R_INB, MOTOR_R_OUTA, MOTOR_R_OUTB, MOTOR_R_CA, MOTOR_R_CB);
  motorL.init(MOTOR_L_INA, MOTOR_L_INB, MOTOR_L_OUTA, MOTOR_L_OUTB, MOTOR_L_CA, MOTOR_L_CB);
  motorR.setCount(0);
  motorL.setCount(0);

  pinMode(LED_IR, OUTPUT);
  pinMode(SENSOR_1, INPUT);
  pinMode(SENSOR_2, INPUT);
  pinMode(SENSOR_3, INPUT);
  pinMode(SENSOR_4, INPUT);
  pinMode(SW_1, INPUT);
  pinMode(SW_2, INPUT);

  readSensorThresholdFromFlash();
  initialBlock();
  initialWalls();

  oled.drawString("John Smith");
  delay(1000);
  oled.drawString("Mode " + String(mode) + "\n" + modeName[mode], 1);
}

void loop() {
  //select mode
  if (SW1_PUSHED) {
    //Debouncing, wait 15 us.
    delayMicroseconds(15);
    if (SW1_PUSHED) {
      //Wait until release switch.
      while (SW1_PUSHED);
      mode++;
      mode %= MAX_MODE;
      oled.drawString("Mode " + String(mode) + "\n" + modeName[mode], 1);
    }
  }
  //choose mode[Enter]
  if (SW2_PUSHED) {
    //Debouncing, wait 15 us.
    delayMicroseconds(15);
    if (SW2_PUSHED) {
      //Wait until release switch.
      while (SW1_PUSHED);
      if (mode == 0) {
        oled.drawString(modeName[mode]);
        delay(1000);
        LED_ON;
        int i = 0;
        short sens[4] = {};
        while (true) {
          if (i < 5) {
            sens[0] = SENSOR_LEFT_SIDE;
            sens[1] = SENSOR_LEFT_FRONT;
            sens[2] = SENSOR_RIGHT_FRONT;
            sens[3] = SENSOR_RIGHT_SIDE;
          }
          else {
            oled.drawString("saving...");
            for (int j = 0; j < 4; j++) {
              if (fCount[j] == 0) {
                found[j] = sens[j];
              }
              else {
                found[j] = fSum[j] / fCount[j];
              }
              if (nCount == 0) {
                notFound[j] = sens[j];
              }
              else {
                notFound[j] = nSum[j] / nCount;
              }
            }

            sensorCalThreshold(50, 50);

            oled.drawString("done");
            delay(1000);
            oled.drawString("fLS: " + String(found[0]) + "\n" + "fLF: " + String(found[1]) + "\n" + "fRF: " + String(found[2]) + "\n" + "tRS: " + String(found[3]));
            delay(1500);
            oled.drawString("nLS: " + String(notFound[0]) + "\n" + "nLF: " + String(notFound[1]) + "\n" + "nRF: " + String(notFound[2]) + "\n" + "nRS: " + String(notFound[3]));
            delay(1500);
            oled.drawString("tLS: " + String(tLeftSide) + "\n" + "tLF: " + String(tLeftFront) + "\n" + "tRF: " + String(tRightFront) + "\n" + "tRS: " + String(tRightSide));
            delay(1500);
            oled.drawString("Mode 2");
            break;
          }

          if (SW1_PUSHED) {
            delayMicroseconds(15);
            if (SW1_PUSHED) {
              while (SW1_PUSHED);
              if (i < 4) {
                fCount[i]++;
                fSum[i] += sens[i];
              }
              else if (i == 4) {
                nCount++;
                for (int j = 0; j < 4; j++) {
                  nSum[j] += sens[j];
                }
              }
            }
          }
          if (SW2_PUSHED) {
            delayMicroseconds(15);
            if (SW2_PUSHED) {
              while (SW2_PUSHED);
              i++;
            }
          }
          if (i < 4) {
            oled.drawString("f" + sName[i] + ": " + String(sens[i]) + "\nc: " + fCount[i]);
          }
          else if (i == 4) {
            oled.drawString("nLS: " + String(sens[0]) + "\n" + "nLF: " + String(sens[1]) + "\n" + "nRF: " + String(sens[2]) + "\n" + "nRS: " + String(sens[3]) + "\n" + "c: " + String(nCount), 1);
          }
        }
        LED_OFF;
        oled.drawString("Mode " + String(mode) + "\n" + modeName[mode], 1);
      }
      else if (mode == 1) {
        oled.drawString(modeName[mode]);
        delay(1000);
        LED_ON;
        while (true) {
          if (SW2_PUSHED) {
            delayMicroseconds(15);
            if (SW2_PUSHED) {
              while (SW2_PUSHED);
              break;
            }
          }
          oled.drawString("State: " + String(getState()));
        }
        LED_OFF;
        oled.drawString("Mode " + String(mode) + "\n" + modeName[mode], 1);
      }
      else if (mode == 2) {
        oled.drawString(modeName[mode]);
        delay(1000);
        LED_ON;
        mapping();
        delay(500);
        oled.drawString("mapping done");
        delay(500);
        oled.drawString(shortestPath, 1);
        LED_OFF;
        oled.drawString(shortestPath);
        //oled.drawString("Mode " + String(mode) + "\n" + modeName[mode], 1);
      }
      else if (mode == 3) {
        oled.drawString(modeName[mode]);
        delay(1000);
        gotoTarget(500, true);
        oled.drawString("A5", 6);
      }
      else if (mode == 4) {
        oled.drawString(modeName[mode]);
        delay(1000);
        gotoTarget(250, true);
        oled.drawString("A5", 6);
      }
      else if (mode == 5) {
        oled.drawString(modeName[mode]);
        delay(1000);
        gotoTarget(0, false);
        oled.drawString("A5", 6);
      }
      else if (mode == 6) {
        oled.drawString(modeName[mode]);
        delay(1000);
        LED_ON;
        turnLeftForward();
        LED_OFF;
        oled.drawString("Mode " + String(mode) + "\n" + modeName[mode], 1);
      }
      else if (mode == 7) {
        oled.drawString(modeName[mode]);
        delay(1000);
        LED_ON;
        trackForward();
        LED_OFF;
        oled.drawString("Mode " + String(mode) + "\n" + modeName[mode], 1);
      }
      else if (mode == 8) {
        oled.drawString(modeName[mode]);
        delay(1000);
        LED_ON;
        turnRightForward();
        LED_OFF;
        oled.drawString("Mode " + String(mode) + "\n" + modeName[mode], 1);
      }
      else if (mode == 9) {
        oled.drawString(modeName[mode]);
        delay(1000);
        LED_ON;
        turnLeft();
        LED_OFF;
        oled.drawString("Mode " + String(mode) + "\n" + modeName[mode], 1);
      }
      else if (mode == 10) {
        oled.drawString(modeName[mode]);
        delay(1000);
        LED_ON;
        turnRight();
        LED_OFF;
        oled.drawString("Mode " + String(mode) + "\n" + modeName[mode], 1);
      }
      /*
      else if (mode == 11) {
        oled.drawString(modeName[mode]);
        delay(1000);
        LED_ON;
        while (true) {
          if (SW2_PUSHED) {
            delayMicroseconds(15);
            if (SW2_PUSHED) {
              while (SW2_PUSHED);
              break;
            }
          }
          if (SENSOR_RIGHT_SIDE < found[3] - 100) {
            oled.drawString(String(SENSOR_RIGHT_SIDE) + "\nA5 100");
          }
          else if (SENSOR_RIGHT_SIDE < found[3] - 200) {
            oled.drawString(String(SENSOR_RIGHT_SIDE) + "\nA5 200");
          }
          else if (SENSOR_RIGHT_SIDE < found[3] - 300) {
            oled.drawString(String(SENSOR_RIGHT_SIDE) + "\nA5 300");
          }
          else if (SENSOR_RIGHT_SIDE < found[3] - 400) {
            oled.drawString(String(SENSOR_RIGHT_SIDE) + "\nA5 400");
          }
          else if (SENSOR_RIGHT_SIDE < found[3] - 500) {
            oled.drawString(String(SENSOR_RIGHT_SIDE) + "\nA5 500");
          }
          else if (SENSOR_RIGHT_SIDE < found[3] - 400) {
            oled.drawString(String(SENSOR_RIGHT_SIDE) + "\nA5 400");
          }
          else if (SENSOR_RIGHT_SIDE > found[3] + 900) {
            oled.drawString(String(SENSOR_RIGHT_SIDE) + "\n5A 900");
          }
          else if (SENSOR_RIGHT_SIDE > found[3] + 800) {
            oled.drawString(String(SENSOR_RIGHT_SIDE) + "\n5A 800");
          }
          else if (SENSOR_RIGHT_SIDE > found[3] + 700) {
            oled.drawString(String(SENSOR_RIGHT_SIDE) + "\n5A 700");
          }
          else if (SENSOR_RIGHT_SIDE > found[3] + 600) {
            oled.drawString(String(SENSOR_RIGHT_SIDE) + "\n5A 600");
          }
          else if (SENSOR_RIGHT_SIDE > found[3] + 500) {
            oled.drawString(String(SENSOR_RIGHT_SIDE) + "\n5A 500");
          }
          else if (SENSOR_RIGHT_SIDE > found[3] + 400) {
            oled.drawString(String(SENSOR_RIGHT_SIDE) + "\n5A 400");
          }
          else if (SENSOR_RIGHT_SIDE > found[3] + 300) {
            oled.drawString(String(SENSOR_RIGHT_SIDE) + "\n5A 300");
          }
          else if (SENSOR_RIGHT_SIDE > found[3] + 200) {
            oled.drawString(String(SENSOR_RIGHT_SIDE) + "\n5A 200");
          }
          else if (SENSOR_RIGHT_SIDE > found[3] + 100) {
            oled.drawString(String(SENSOR_RIGHT_SIDE) + "\n5A 100");
          }
        }
        LED_OFF;
        oled.drawString("Mode " + String(mode) + "\n" + modeName[mode], 1);
      }
      //empty
      else if (mode == 12) {
        oled.drawString(modeName[mode]);
        delay(1000);
        LED_ON;
        while (true) {
          if (SW2_PUSHED) {
            delayMicroseconds(15);
            if (SW2_PUSHED) {
              while (SW2_PUSHED);
              break;
            }
          }
        }
        LED_OFF;
        oled.drawString("Mode " + String(mode) + "\n" + modeName[mode], 1);
      }
      else if (mode == 11) {
        oled.drawString(modeName[mode]);
        delay(1000);
        LED_ON;
        while (true) {
          if (SW2_PUSHED) {
            delayMicroseconds(15);
            if (SW2_PUSHED) {
              while (SW2_PUSHED);
              break;
            }
          }
          oled.drawString("LS: " + String(SENSOR_LEFT_SIDE) + "\n" + "LF: " + String(SENSOR_LEFT_FRONT) + "\n" + "RF: " + String(SENSOR_RIGHT_FRONT) + "\n" + "RS: " + String(SENSOR_RIGHT_SIDE) );
        }
        LED_OFF;
        oled.drawString("Mode " + String(mode) + "\n" + modeName[mode], 1);
      }
      */
    }
  }
}

void readSensorThresholdFromFlash() {
  for (int i = 0; i < 4; i++) {
    notFound[i] = EEPROM_read_int(NOTFOUND_ADD + (i * 4));
    found[i] = EEPROM_read_int(FOUND_ADD + (i * 4));
  }
  tLeftSide = EEPROM_read_int(THRESHOLD_ADD);
  tLeftFront = EEPROM_read_int(THRESHOLD_ADD + (1 * 4));
  tRightFront = EEPROM_read_int(THRESHOLD_ADD + (2 * 4));
  tRightSide = EEPROM_read_int(THRESHOLD_ADD + (3 * 4));
}
//write block value
void writeShortestPathToFlash() {
  uint8_t l = shortestPath.length();
  EEPROM.write(SHORTEST_PATH_LEN, l);
  for (int i = 0; i < l; i++) {
    EEPROM.write(SHORTEST_PATH + i, shortestPath[i]);
  }
  EEPROM.commit();
}
//read block value
void readShortestPathFromFlash() {
  shortestPath = "";
  uint8_t l = EEPROM.read(SHORTEST_PATH_LEN);
  for (int i = 0; i < l; i++) {
    shortestPath += char(EEPROM.read(SHORTEST_PATH + i));
    oled.drawString(shortestPath, 1);
    delay(100);
  }
}

void sensorCalThreshold(float weightFound, float weightNotFound) {
  tLeftSide = (float(weightFound / 100) * found[0]) + (float(weightNotFound / 100) * notFound[0]);
  tLeftFront = (float(weightFound / 100) * found[1]) + (float(weightNotFound / 100) * notFound[1]);
  tRightFront = (float(weightFound / 100) * found[2]) + (float(weightNotFound / 100) * notFound[2]);
  tRightSide = (float(weightFound / 100) * found[3]) + (float(weightNotFound / 100) * notFound[3]);

  for (int i = 0; i < 4; i++) {
    EEPROM_write_int(NOTFOUND_ADD + (4 * i), notFound[i]);
  }

  for (int i = 0; i < 4; i++) {
    EEPROM_write_int(FOUND_ADD + (4 * i), found[i]);
  }

  EEPROM_write_int(THRESHOLD_ADD, tLeftSide);
  EEPROM_write_int(THRESHOLD_ADD + (4 * 1), tLeftFront);
  EEPROM_write_int(THRESHOLD_ADD + (4 * 2), tRightFront);
  EEPROM_write_int(THRESHOLD_ADD + (4 * 3), tRightSide);
}

//Forward 1 block.
void trackForward() {
  trackForward(STEP_ONE_BLOCK + STEP_ONE_BLOCK * (20 / 100), 60, true);
}

void trackForward(int stepCount, int spd, bool c) {
  uint8_t spdLeft = 60;
  uint8_t spdRight = 60;
  motorL.setCount(0);
  motorR.setCount(0);
  motorL.forward(spdLeft);
  motorR.forward(spdRight);

  bool add = false;

  while (motorL.getCount() < stepCount && motorR.getCount() < stepCount) {
    //have right wall and no left wall
    if (SENSOR_RIGHT_SIDE < tRightSide && SENSOR_RIGHT_SIDE < found[3] - (found[3] * 0.4) && SENSOR_LEFT_SIDE > notFound[0] - (notFound[0] * 0.1)) {
      spdLeft = spd - 30;
      spdRight = spd;
      if (!add) {
        stepCount += 20;
        add = true;
      }
      //      oled.drawString("A5", 3);
    }
    else if (SENSOR_RIGHT_SIDE < tRightSide && SENSOR_RIGHT_SIDE > found[3] + (found[3] * 0.4) && SENSOR_LEFT_SIDE > notFound[0] - (notFound[0] * 0.1)) {
      spdLeft = spd;
      spdRight = spd - 30;
      if (!add) {
        stepCount += 20;
        add = true;
      }
      //      oled.drawString("5A", 3);
    }
    else if (SENSOR_LEFT_SIDE < tLeftSide && SENSOR_LEFT_SIDE < found[0] - (found[0] * 0.4) && SENSOR_RIGHT_SIDE > notFound[3] - (notFound[0] * 0.1)) {
      spdLeft = spd;
      spdRight = spd - 20;
      if (!add) {
        stepCount += 20;
        add = true;
      }
      //      oled.drawString("B5", 3);
    }
    else if (SENSOR_LEFT_SIDE < tLeftSide && SENSOR_LEFT_SIDE > found[0] + (found[0] * 0.4) && SENSOR_RIGHT_SIDE > notFound[3] - (notFound[0] * 0.1)) {
      spdLeft = spd - 20;
      spdRight = spd;
      if (!add) {
        stepCount += 20;
        add = true;
      }
      //      oled.drawString("5B", 3);
    }
    else if (SENSOR_LEFT_SIDE < found[0] && SENSOR_LEFT_SIDE < notFound[0] - 100) {
      spdLeft = spd;
      spdRight = spd - 25;
      if (!add) {
        stepCount += 10;
        add = true;
      }
      //      oled.drawString("left", 3);
    }
    else if (SENSOR_RIGHT_SIDE < found[3] && SENSOR_RIGHT_SIDE < notFound[3] - 100) {
      spdLeft = spd - 20;
      spdRight = spd;
      if (!add) {
        stepCount += 10;
        add = true;
      }
      //      oled.drawString("right", 3);
    }
    else {
      spdLeft = spd;
      spdRight = spd;
      //      oled.drawString("for", 3);
    }

    if (motorL.getCount() > motorR.getCount()) {
      motorL.forward(spdLeft - 10);
    }
    else {
      motorL.forward(spdLeft);
    }
    if (motorR.getCount() > motorL.getCount()) {
      motorR.forward(spdRight - 10);
    }
    else {
      motorR.forward(spdRight);
    }
  }
  motorL.stop();
  motorR.stop();
  if (c) {
    changePosition();
  }
}

void turnRightForward() {
  //trackForward(50, 50, false);
  forwardDelay(200);
  turnRight();
  trackForward(130, 50, false);
  changePosition();
}

void turnRightSetZeroForward() {
  //change to forward delay.
  forwardDelay(200);
  turnRight();
  backwardDelay(350);
  trackForward(STEP_ONE_BLOCK + STEP_ONE_BLOCK * (20 / 100), 60, false);
  changePosition();
}

void turnLeftForward() {
  //trackForward(50, 50, false);
  forwardDelay(200);
  turnLeft();
  trackForward(130, 50, false);
  changePosition();
}

void turnLeftSetZeroForward() {
  //change to forward delay.
  forwardDelay(200);
  turnLeft();
  backwardDelay(400);
  trackForward(STEP_ONE_BLOCK + STEP_ONE_BLOCK * (20 / 100), 60, false);
  changePosition();
}

void changePosition() {
  if (direction == 'N') py--;
  else if (direction == 'E') px++;
  else if (direction == 'S') py++;
  else px--;
  oled.drawString("py: " + String(py) + "\npx: " + String(px), 3);
  //  oled.drawString("py: " + String(py) + "\npx: " + String(px) + "\nv: " + blocks[py][px].value + "\nc: " + String(blocks[py][px].count) + "\nv: " + String(blocks[py][px].flag), 1);
}

void forward(int stepCount) {
  uint8_t spd = 60;
  motorL.setCount(0);
  motorR.setCount(0);
  motorL.forward(spd);
  motorR.forward(spd);

  while (motorL.getCount() < stepCount && motorR.getCount() < stepCount) {

    if (motorL.getCount() > motorR.getCount()) {
      motorL.stop();
    }
    else {
      motorL.forward(spd);
    }
    if (motorR.getCount() > motorL.getCount()) {
      motorR.stop();
    }
    else {
      motorR.forward(spd);
    }
  }
  motorL.stop();
  motorR.stop();
}

void forwardDelay(int d) {
  int8_t spd = 60;
  motorL.setCount(0);
  motorR.setCount(0);
  motorL.forward(spd);
  motorR.forward(spd);
  delay(d);
  motorR.stop();
  motorL.stop();
}

void backward(int stepCount) {
  backward(stepCount, 60);
  motorR.stop();
  motorL.stop();
}

void backwardDelay(int d) {
  int8_t spd = 60;
  motorL.setCount(0);
  motorR.setCount(0);
  motorL.backward(spd);
  motorR.backward(spd);
  delay(d);
  motorR.stop();
  motorL.stop();
}

void backward(int stepCount, int spd) {
  motorL.setCount(0);
  motorR.setCount(0);
  motorL.backward(spd);
  motorR.backward(spd);

  while (abs(motorL.getCount()) < stepCount && abs(motorR.getCount()) < stepCount) {
    if (abs(motorL.getCount()) > abs(motorR.getCount())) {
      motorL.stop();
    }
    else {
      motorL.backward(spd);
    }
    if (abs(motorR.getCount()) > abs(motorL.getCount())) {
      motorR.stop();
    }
    else {
      motorR.backward(spd);
    }
  }
  motorL.stop();
  motorR.stop();
}

void turnRight() {
  short m = 80;
  uint8_t spd = 40;
  motorL.setCount(0);
  motorR.setCount(0);
  motorL.forward(spd);
  motorR.backward(spd);
  while (motorL.getCount() < m && abs(motorR.getCount()) < m) {
    if (motorL.getCount() > abs(motorR.getCount())) {
      motorL.stop();
    }
    else {
      motorL.forward(spd);
    }
    if (abs(motorR.getCount()) > motorL.getCount()) {
      motorR.stop();
    }
    else {
      motorR.backward(spd);
    }
  }
  motorL.stop();
  motorR.stop();
  if (direction == 'N')direction = 'E';
  else if (direction == 'E') direction = 'S';
  else if (direction == 'S') direction = 'W';
  else direction = 'N';
}

void turnLeft() {
  short m = 80;
  uint8_t spd = 40;
  motorL.setCount(0);
  motorR.setCount(0);
  motorL.backward(spd);
  motorR.forward(spd);
  while (abs(motorL.getCount()) < m && motorR.getCount() < m) {
    if (abs(motorL.getCount()) > motorR.getCount()) {
      motorL.stop();
    }
    else {
      motorL.backward(spd);
    }
    if (motorR.getCount() > abs(motorL.getCount())) {
      motorR.stop();
    }
    else {
      motorR.forward(spd);
    }
  }
  motorL.stop();
  motorR.stop();
  if (direction == 'N')direction = 'W';
  else if (direction == 'E') direction = 'N';
  else if (direction == 'S') direction = 'E';
  else direction = 'S';
}

void turnAround() {
  forward(45);
  turnRight();
  turnRight();
  backwardDelay(350);
}
//mapping split into 2 phases, first find target and lastly find home.
void mapping() {
  py = START_PY;
  px = START_PX;
  direction = START_DIRECTION;
  bool foundTarget[4] = {false, false, false, false};
  //find target
  while (!foundTarget[0] && !foundTarget[1] && !foundTarget[2] && !foundTarget[3]) {
    decisionFindTarget();
    resetBlockValue();
    floodFill();
    for (int i = 0; i < 4; i++) {
      foundTarget[i] = endPoints[i][0] == py && endPoints[i][1] == px;
    }
    delay(500);
  }
  for (int i = 0; i < 4; i++) blocks[endPoints[i][0]][endPoints[i][1]].flag = true;
  //mem py px and direction, because findShortestPath() have to reset it.
  uint8_t _py = py;
  uint8_t _px = px;
  char _direction = direction;

  findShortestPath();
  oled.drawString("Finding shortest-path phase 1 done.", 1);
  delay(1500);

  //resotre py, px and direction
  py = _py;
  px = _px;
  direction = _direction;

  //go home
  trackForward();
  delay(500);
  turnAround();
  delay(500);
  trackForward();
  delay(500);
  trackForward();
  delay(500);
  while (!(py == START_PY && px == START_PX)) {
    decisionFindHome();
    resetBlockValue();
    floodFill();
    delay(500);
  }
  turnAround();
  findShortestPath();
  oled.drawString("Finding shortest-path phase 2 done.", 1);
  delay(1500);
}

void findShortestPath() {
  floodFill();
  px = START_PX;
  py = START_PY;
  direction = START_DIRECTION;
  bool foundTarget[4] = {false, false, false, false};
  shortestPath = "";
  while (!foundTarget[0] && !foundTarget[1] && !foundTarget[2] && !foundTarget[3]) {
    decisionShortestPath();
    for (int i = 0; i < 4; i++) {
      foundTarget[i] = endPoints[i][0] == py && endPoints[i][1] == px;
    }
  }
  writeShortestPathToFlash();
}

void gotoTarget(int d, bool backward) {
  readShortestPathFromFlash();
  LED_ON;
  for (int i = 0; i < shortestPath.length(); i++) {
    if (shortestPath[i] == 'f') {
      trackForward();
    }
    else if (shortestPath[i] == 'r') {
      if (backward) turnRightSetZeroForward();
      else turnRightForward();
    }
    else if (shortestPath[i] == 'l') {
      if (backward) turnLeftSetZeroForward();
      turnLeftForward();
    }
    delay(d);
  }
  LED_OFF;
}

void initialWalls() {
  for (int i = 0; i < ROW; i++) {
    verticalWalls[i][0] = true;
    verticalWalls[i][COL] = true;
  }
  for (int i = 0; i < COL; i++) {
    horizontalWalls[0][i] = true;
    horizontalWalls[COL][i] = true;
  }
}

void printWalls() {
  for (int i = 0; i < ROW; i++) {
    for (int j = 0; j < COL; j++) {
      if (horizontalWalls[i][j]) Serial.print("*---");
      else Serial.print("    ");
    }
    Serial.println();

    for (int j = 0; j < COL; j++) {
      if (verticalWalls[i][j]) Serial.print("|");
      else Serial.print(" ");

      Serial.print("  ");
      //      if (values[i][j] < 10) std::cout << "  ";
      //      else if (values[i][j] < 100) std::cout << " ";
    }
    Serial.println();
  }
  Serial.println();
}

void showWalls() {
  for (int i = 0; i < ROW; i++) {
    for (int j = 0; j < COL; j++) {
      if (horizontalWalls[i][j]) oled.drawString("*---", 1);
      else oled.drawString("    ");
    }
    oled.drawString("\n");

    for (int j = 0; j < COL; j++) {
      if (verticalWalls[i][j]) oled.drawString("|");
      else oled.drawString(" ");

      oled.drawString("  ");
      //      if (values[i][j] < 10) std::cout << "  ";
      //      else if (values[i][j] < 100) std::cout << " ";
    }
    oled.drawString("\n");
  }
  oled.drawString("\n");
}

void initialBlock() {
  for (int i = 0; i < ROW; i++) {
    for (int j = 0; j < COL; j++) {
      blocks[i][j].y = i;
      blocks[i][j].x = j;
      blocks[i][j].flag = false;
      blocks[i][j].mark = false;
      blocks[i][j].count = 0;
      blocks[i][j].value = -1;
    }
  }
}

void resetBlockValue() {
  for (int i = 0; i < ROW; i++) {
    for (int j = 0; j < COL; j++) {
      blocks[i][j].value = -1;
    }
  }
}

void floodFill() {
  QueueArray <Block> q;
  //push targets, endpoint
  for (int i = 0; i < 4; i++) {
    blocks[endPoints[i][0]][endPoints[i][1]].value = 0;
    q.push(blocks[endPoints[i][0]][endPoints[i][1]]);
  }

  while (!q.isEmpty()) {
    Block b = q.front();
    q.pop();
    //#North
    if (b.y - 1 >= 0 && horizontalWalls[b.y][b.x] == false && blocks[b.y - 1][b.x].value == -1) {
      blocks[b.y - 1][b.x].value = b.value + 1;
      q.push(blocks[b.y - 1][b.x]);
    }
    //East
    if (b.x + 1 < COL && verticalWalls[b.y][b.x + 1] == false && blocks[b.y][b.x + 1].value == -1) {
      blocks[b.y][b.x + 1].value = b.value + 1;
      q.push(blocks[b.y][b.x + 1]);
    }
    //South
    if (b.y + 1 < ROW && horizontalWalls[b.y + 1][b.x] == false && blocks[b.y + 1][b.x].value == -1) {
      blocks[b.y + 1][b.x].value = b.value + 1;
      q.push(blocks[b.y + 1][b.x]);
    }
    //West
    if (b.x - 1 >= 0 && verticalWalls[b.y][b.x] == false && blocks[b.y][b.x - 1].value == -1) {
      blocks[b.y][b.x - 1].value = b.value + 1;
      q.push(blocks[b.y][b.x - 1]);
    }
  }
}
//get state from sensors
uint8_t getState() {
  uint8_t state = 0;
  switch (direction) {
    case 'N':
      if (SENSOR_LEFT_SIDE < tLeftSide) {
        verticalWalls[py][px] = true;
        state += 1;
      }
      if (SENSOR_LEFT_FRONT < tLeftFront && SENSOR_RIGHT_FRONT < tRightFront) {
        horizontalWalls[py][px] = true;
        state += 2;
      }
      if (SENSOR_RIGHT_SIDE < tRightSide) {
        verticalWalls[py][px + 1] == true;
        state += 4;
      }
      break;
    case 'E':
      if (SENSOR_LEFT_SIDE < tLeftSide) {
        horizontalWalls[py][px] = true;
        state += 1;
      }
      if (SENSOR_LEFT_FRONT < tLeftFront && SENSOR_RIGHT_FRONT < tRightFront) {
        verticalWalls[py][px + 1] = true;
        state += 2;
      }
      if (SENSOR_RIGHT_SIDE < tRightSide) {
        horizontalWalls[py + 1][px] == true;
        state += 4;
      }
      break;
    case 'S':
      if (SENSOR_LEFT_SIDE < tLeftSide) {
        verticalWalls[py][px + 1] = true;
        state += 1;
      }
      if (SENSOR_LEFT_FRONT < tLeftFront && SENSOR_RIGHT_FRONT < tRightFront) {
        horizontalWalls[py + 1][px] = true;
        state += 2;
      }
      if (SENSOR_RIGHT_SIDE < tRightSide) {
        verticalWalls[py][px] == true;
        state += 4;
      }
      break;
    case 'W':
      if (SENSOR_LEFT_SIDE < tLeftSide) {
        horizontalWalls[py + 1][px] = true;
        state += 1;
      }
      if (SENSOR_LEFT_FRONT < tLeftFront && SENSOR_RIGHT_FRONT < tRightFront) {
        verticalWalls[py][px] = true;
        state += 2;
      }
      if (SENSOR_RIGHT_SIDE < tRightSide) {
        verticalWalls[py][px] == true;
        state += 4;
      }
      break;
  }

  return state;
}
//get state from remember walls.
uint8_t getStateFromWall() {
  uint8_t state = 0;
  switch (direction) {
    case 'N':
      if (verticalWalls[py][px]) {
        state += 1;
      }
      if (horizontalWalls[py][px]) {
        state += 2;
      }
      if (verticalWalls[py][px + 1]) {
        state += 4;
      }
      break;
    case 'E':
      if (horizontalWalls[py][px]) {
        state += 1;
      }
      if (verticalWalls[py][px + 1]) {
        state += 2;
      }
      if (horizontalWalls[py + 1][px]) {
        state += 4;
      }
      break;
    case 'S':
      if (verticalWalls[py][px + 1]) {
        state += 1;
      }
      if (horizontalWalls[py + 1][px]) {
        state += 2;
      }
      if (verticalWalls[py][px]) {
        state += 4;
      }
      break;
    case 'W':
      if (horizontalWalls[py + 1][px]) {
        state += 1;
      }
      if (verticalWalls[py][px]) {
        state += 2;
      }
      if (verticalWalls[py][px]) {
        state += 4;
      }
      break;
  }

  return state;
}
//choose 3 block from current position of robot, left block, front block and right block.
void chooseBlock(bool sf[], Block * left, Block * front, Block * right) {
  //sf = (left, front, right)
  //b[][0] = y
  //b[][1] = x
  if (direction == 'N') {
    if (sf[0] && px - 1 >= 0) *left = blocks[py][px - 1];
    else sf[0] = false;
    if (sf[1] && py - 1 >= 0) *front = blocks[py - 1][px];
    else sf[1] = false;
    if (sf[2] && px + 1 < COL) *right = blocks[py][px + 1];
    else sf[2] = false;
  }
  if (direction == 'E') {
    if (sf[0] && py - 1 >= 0) *left = blocks[py - 1][px];
    else sf[0] = false;
    if (sf[1] && px + 1 < COL) *front = blocks[py][px + 1];
    else sf[1] = false;
    if (sf[2] && py + 1 < ROW) *right = blocks[py + 1][px];
    else sf[2] = false;
  }
  if (direction == 'S') {
    if (sf[0] && px + 1 < ROW) *left = blocks[py][px + 1];
    else sf[0] = false;
    if (sf[1] && py + 1 < COL) *front = blocks[py + 1][px];
    else sf[1] = false;
    if (sf[2] && px - 1 >= 0) *right = blocks[py][px - 1];
    else sf[2] = false;
  }
  if (direction == 'W') {
    if (sf[0] && py + 1 < ROW) *left = blocks[py + 1][px];
    else sf[0] = false;
    if (sf[1] && px - 1 >= 0) *front = blocks[py][px - 1];
    else sf[1] = false;
    if (sf[2] && py - 1 >= 0) *right = blocks[py - 1][px];
    else sf[2] = false;
  }
}
//find target in mapping phase.
void decisionFindTarget() {
  if (blocks[py][px].flag) {
    blocks[py][px].count++;
  }
  else {
    blocks[py][px].flag = true;
  }

  bool over = false;
  if (blocks[py][px].count > 4) {
    over = true;
  }

  uint8_t state = getState();

  //  0 3-Way
  bool sf[3] = {true, true, true};
  //0:left, 1:front, 2:right
  //0: y, 1: x
  Block left, front, right;

  if (state == 0) {
    oled.drawString("\n\n");
    chooseBlock(sf, &left, &front, &right);
    //front
    if (sf[1] && front.flag == false && front.mark == false)
      trackForward();
    //right
    else if (sf[2] && right.flag == false && right.mark == false)
      turnRightForward();
    //left
    else if (sf[0] && left.flag == false && left.mark == false)
      turnLeftForward();
    else if (over && sf[0] && sf[1] && sf[2] && front.count <= right.count && front.count <= left.count)
      trackForward();
    else if (over && sf[0] && sf[1] && sf[2] && right.count <= front.count && right.count <= left.count)
      turnRightForward();
    else if (over && sf[0] && sf[1] && sf[2] && left.count <= front.count && left.count <= right.count)
      turnLeftForward();
    else if (sf[0] && sf[1] && sf[2] && front.value <= right.value && front.value <= left.value)
      trackForward();
    else if (sf[0] && sf[1] && sf[2] && right.value <= front.value && right.value <= left.value)
      turnRightForward();
    else
      turnLeftForward();
  }
  //1 2-Way
  else if (state == 1) {
    //compare potential between front and right
    sf[0] = false;
    //    oled.drawString("L\n\n");
    chooseBlock(sf, &left, &front, &right);

    if (front.flag == false && front.mark == false)
      trackForward();
    else if (right.flag == false && right.mark == false)
      //      turnRightForward();
      turnRightSetZeroForward();
    else if (over && front.count <= right.count)
      trackForward();
    else if (over && right.count <= front.count)
      //      turnRightForward();
      turnRightSetZeroForward();
    else if (front.value <= right.value)
      trackForward();
    else
      //      turnRightForward();
      turnRightSetZeroForward();
  }
  //2 2-Way
  else if (state == 2) {
    //compare potential between left and right
    sf[1] = false;
    //    oled.drawString("\nF\n");
    chooseBlock(sf, &left, &front, &right);

    if (right.flag == false && right.mark == false)
      turnRightForward();
    else if (left.flag == false && left.mark == false)
      turnLeftForward();
    else if (over && right.count <= left.count)
      turnRightForward();
    else if (over && left.count <= right.count)
      turnLeftForward();
    else if (right.value <= left.value)
      turnRightForward();
    else
      turnLeftForward();
  }
  //4 2-Way
  else if (state == 4) {
    //compare potential between front and left
    sf[2] = false;
    //    oled.drawString("\n\nR");
    chooseBlock(sf, &left, &front, &right);

    if (front.flag == false && front.mark == false)
      trackForward();
    else if (left.flag == false && left.mark == false)
      //      turnLeftForward();
      turnLeftSetZeroForward();
    else if (over && front.count <= left.count)
      trackForward();
    else if (over && left.count <= front.count)
      //      turnLeftForward();
      turnLeftSetZeroForward();
    else if (front.value <= left.value)
      trackForward();
    else
      //      turnLeftForward();
      turnLeftSetZeroForward();
  }
  //3 1-Way
  else if (state == 3) {
    //    oled.drawString("L\nF\n");
    //    turnRightForward();
    turnRightSetZeroForward();
  }
  //5 1-Way
  else if (state == 5) {
    //    oled.drawString("L\n\nR");
    trackForward();
  }
  //6 1-Way
  else if (state == 6) {
    //    oled.drawString("\nF\nR");
    //    turnLeftForward();
    turnLeftSetZeroForward();
  }
  //7 0-Way
  else if (state == 7) {
    //    oled.drawString("L\nF\nR");
    blocks[py][px].mark = true;
    turnAround();
  }
}
//find home in mapping phase.
void decisionFindHome() {
  if (blocks[py][px].flag) {
    blocks[py][px].count++;
  }
  else {
    blocks[py][px].flag = true;
  }

  bool over = false;
  if (blocks[py][px].count > 4) {
    over = true;
  }

  uint8_t state = getState();

  //  0 3-Way
  bool sf[3] = {true, true, true};
  //0:left, 1:front, 2:right
  //0: y, 1: x
  Block left, front, right;

  if (state == 0) {
    chooseBlock(sf, &left, &front, &right);
    //front
    if (sf[1] && front.flag == false && front.mark == false)
      trackForward();
    //right
    else if (sf[2] && right.flag == false && right.mark == false)
      turnRightForward();
    //left
    else if (sf[0] && left.flag == false && left.mark == false)
      turnLeftForward();
    else if (over && sf[0] && sf[1] && sf[2] && front.count <= right.count && front.count <= left.count)
      trackForward();
    else if (over && sf[0] && sf[1] && sf[2] && right.count <= front.count && right.count <= left.count)
      turnRightForward();
    else if (over && sf[0] && sf[1] && sf[2] && left.count <= front.count && left.count <= right.count)
      turnLeftForward();
    else if (sf[0] && sf[1] && sf[2] && front.value >= right.value && front.value >= left.value)
      trackForward();
    else if (sf[0] && sf[1] && sf[2] && right.value >= front.value && right.value >= left.value)
      turnRightForward();
    else
      turnLeftForward();
  }
  //1 2-Way
  else if (state == 1) {
    //compare potential between front and right
    sf[0] = false;
    chooseBlock(sf, &left, &front, &right);

    if (front.flag == false && front.mark == false)
      trackForward();
    else if (right.flag == false && right.mark == false)
      //      turnRightForward();
      turnRightSetZeroForward();
    else if (over && front.count <= right.count)
      trackForward();
    else if (over && right.count <= front.count)
      //      turnRightForward();
      turnRightSetZeroForward();
    else if (front.value >= right.value)
      trackForward();
    else
      //      turnRightForward();
      turnRightSetZeroForward();
  }
  //2 2-Way
  else if (state == 2) {
    //compare potential between left and right
    sf[1] = false;
    chooseBlock(sf, &left, &front, &right);

    if (right.flag == false && right.mark == false)
      turnRightForward();
    else if (left.flag == false && left.mark == false)
      turnLeftForward();
    else if (over && right.count <= left.count)
      turnRightForward();
    else if (over && left.count <= right.count)
      turnLeftForward();
    else if (right.value >= left.value)
      turnRightForward();
    else
      turnLeftForward();
  }
  //4 2-Way
  else if (state == 4) {
    //compare potential between front and left
    sf[2] = false;
    chooseBlock(sf, &left, &front, &right);

    if (front.flag == false && front.mark == false)
      trackForward();
    else if (left.flag == false && left.mark == false)
      //      turnLeftForward();
      turnLeftSetZeroForward();
    else if (over && front.count <= left.count)
      trackForward();
    else if (over && left.count <= front.count)
      //      turnLeftForward();
      turnLeftSetZeroForward();
    else if (front.value >= left.value)
      trackForward();
    else
      //      turnLeftForward();
      turnLeftSetZeroForward();
  }
  //3 1-Way
  else if (state == 3) {
    //    turnRightForward();
    turnRightSetZeroForward();
  }
  //5 1-Way
  else if (state == 5) {
    trackForward();
  }
  //6 1-Way
  else if (state == 6) {
    //    turnLeftForward();
    turnLeftSetZeroForward();
  }
  //7 0-Way
  else if (state == 7) {
    blocks[py][px].mark = true;
    turnAround();
  }
}

void fw() {
  shortestPath += 'f';
  changePosition();
}

void rh() {
  shortestPath += 'r';
  if (direction == 'N')direction = 'E';
  else if (direction == 'E') direction = 'S';
  else if (direction == 'S') direction = 'W';
  else direction = 'N';
  changePosition();
}

void lf() {
  shortestPath += 'l';
  if (direction == 'N')direction = 'W';
  else if (direction == 'E') direction = 'N';
  else if (direction == 'S') direction = 'E';
  else direction = 'S';
  changePosition();
}

void decisionShortestPath() {
  uint8_t state = getStateFromWall();

  //0:left, 1:front, 2:right
  //0: y, 1: x
  Block left, front, right;
  bool sf[3] = {true, true, true};
  if (state == 0) {
    chooseBlock(sf, &left, &front, &right);
    if (front.value <= right.value && front.value <= left.value) {
      fw();
    }
    else if (right.value <= front.value && right.value <= left.value) {
      rh();
    }
    else lf();
  }
  //1 2-Way
  else if (state == 1) {
    sf[0] = false;
    //compare potential between front and right
    chooseBlock(sf, &left, &front, &right);
    if (front.value <= right.value) {
      fw();
    }
    else {
      rh();
    }
  }
  //2 2-Way
  else if (state == 2) {
    sf[1] = false;
    //compare potential between left and right
    chooseBlock(sf, &left, &front, &right);
    if (right.value <= left.value) {
      rh();
    }
    else {
      lf();
    }
  }
  //4 2-Way
  else if (state == 4) {
    sf[2] = false;
    //compare potential between front and left
    chooseBlock(sf, &left, &front, &right);
    if (front.value <= left.value) {
      fw();
    }
    else {
      lf();
    }
  }
  //3 1-Way
  else if (state == 3) {
    rh();
  }
  //5 1-Way
  else if (state == 5) {
    fw();
  }
  //6 1-Way
  else if (state == 6) {
    lf();
  }
  //  oled.drawString(String(state) + "\n" + String(py) + ", " + String(px) + "\n" + String(left.value) + "," + String(front.value) + "," + String(right.value), 2);
}
