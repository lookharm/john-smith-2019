#include <QueueArray.h>
#include "config.h"
#include "Oled.h"
#include "Motor.h"

#define LED_ON digitalWrite(LED_IR, HIGH)
#define LED_OFF digitalWrite(LED_IR, LOW)
#define SW1_PUSHED !digitalRead(SW_1)
#define SW2_PUSHED !digitalRead(SW_2)
#define SENSOR_LEFT_FRONT analogRead(SENSOR_1)
#define SENSOR_LEFT_SIDE analogRead(SENSOR_2)
#define SENSOR_RIGHT_SIDE analogRead(SENSOR_3)
#define SENSOR_RIGHT_FRONT analogRead(SENSOR_4)

#define ROW 16
#define COL 16
/***
   TODO
   -acc sigmoid
   -slide side infared
   -no switch debounce
   -no level shifter
   -no free wheeling diode
*/


/***
 * TODO FAST
 * -The God of Track Forward (1 block)
 * -Trackforward Check front wall to stop.
 * -Turn Left, Turn Right
 */

Oled oled;
Motor motorR;
Motor motorL;

short tLeftSide = 1500;
short tLeftFront = 3150;
short tRightSide = 1800;
short tRightFront = 3050;

byte step1Block = 215;// 7.7 V

byte _mode = 0;

byte verticalWalls[ROW][COL + 1];
byte horizontalWalls[ROW + 1][COL];
byte startY = 5;//15
byte startX = 0;//0
byte endPoints[4][2] = {{3, 2}, {3, 3}, {4, 2}, {4, 3}}; //{7,8},{7,9},{8,8},{8,9}

struct Block {
  byte y;
  byte x;
  byte value;
  byte count;
  bool flag;
  bool mark;
};
Block blocks[ROW][COL];

//current position of robot
byte px = 5;
byte py = 0;
char direction = 'N';
byte spd = 80;

void setup() {
  oled.init();
  motorR.init(MOTOR_R_INA, MOTOR_R_INB, MOTOR_R_OUTA, MOTOR_R_OUTB, MOTOR_R_CA, MOTOR_R_CB);
  motorL.init(MOTOR_L_INA, MOTOR_L_INB, MOTOR_L_OUTA, MOTOR_L_OUTB, MOTOR_L_CA, MOTOR_L_CB);

  Serial.begin(115200);
  Serial.println("Start");
  delay(1000);

  motorR.setCount(0);
  motorL.setCount(0);
  pinMode(LED_IR, OUTPUT);

  pinMode(SENSOR_1, INPUT);
  pinMode(SENSOR_2, INPUT);
  pinMode(SENSOR_3, INPUT);
  pinMode(SENSOR_4, INPUT);

  pinMode(SW_1, INPUT);
  pinMode(SW_2, INPUT);

  LED_ON;

  oled.drawString("John Smith");
  delay(1000);
  oled.drawString("Mode 0");
}

void loop() {
  //select mode
  if (SW1_PUSHED) {
    //Debouncing, wait 15 us.
    delayMicroseconds(15);
    if (SW1_PUSHED) {
      //Wait until release switch.
      while (SW1_PUSHED);
      _mode++;
      _mode %= 8;
      oled.drawString("Mode " + String(_mode));
    }
  }
  //choose mode[Enter]
  if (SW2_PUSHED) {
    //Debouncing, wait 15 us.
    delayMicroseconds(15);
    if (SW2_PUSHED) {
      //Wait until release switch.
      while (SW1_PUSHED);
      //mode
      Serial.println(_mode);
      if (_mode == 0) {
        char d[5] = "NESW";
        byte c = 0;
        while (true) {
          if (SW1_PUSHED) {
            delayMicroseconds(15);
            while (SW1_PUSHED);
            if (SW1_PUSHED) {
              c++;
              direction = d[c % 4];
            }
          }
          oled.drawString("D: " + String(direction) + "\nState: " + String(getState()));
        }
      }
      else if (_mode == 1) {
        delay(200);
        while (true) {
          oled.drawString(String(SENSOR_LEFT_SIDE) + "\n" + String(SENSOR_LEFT_FRONT) + "\n" + String(SENSOR_RIGHT_SIDE) + "\n" + String(SENSOR_RIGHT_FRONT));
          if (SW1_PUSHED) {
            delayMicroseconds(15);
            while (SW1_PUSHED);
            break;
          }
          if (SW2_PUSHED) {
            delayMicroseconds(15);
            while (SW2_PUSHED);
            sensorSolving();
            oled.drawString("OK!");
            delay(1000);
            oled.drawString("Mode 1");
            break;
          }
        }
      }
      else if (_mode == 2) {
        delay(1000);
        turnRight();
      }
      else if (_mode == 3) {
        delay(1000);
        turnLeft();
      }
      else if (_mode == 4) {
        delay(1000);
        mapping();
      }
      else if (_mode == 5) {
        delay(1000);
        trackForward(step1Block + step1Block * (5/100) );
      }
    }
  }
}

void sensorSolving() {
  short leftSide[10];
  short leftFront[10];
  short rightFront[10];
  short rightSide[10];
  int ls = 0;
  int lf = 0;
  int rf = 0;
  int rs = 0;
  for (int i = 0; i < 10; i++) {
    ls += SENSOR_LEFT_SIDE;
    lf += SENSOR_LEFT_FRONT;
    rf += SENSOR_RIGHT_FRONT;
    rs += SENSOR_RIGHT_SIDE;
    //    leftSide[i] = SENSOR_LEFT_SIDE;
    //    leftFront[i] = SENSOR_LEFT_FRONT;
    //    rightFront[i] = SENSOR_RIGHT_FRONT;
    //    rightFront[i] = SENSOR_RIGHT_SIDE;
    delay(10);
  }
  short tLeftSide = ls / 10;
  short tLeftFront = lf / 10;
  short tRightSide = rs / 10;
  short tRightFront = rf / 10;

}

void trackForward(int stepCount) {
  byte spdLeft = 60;
  byte spdRight = 60;
  motorL.setCount(0);
  motorR.setCount(0);
  motorL.forward(spdLeft);
  motorR.forward(spdRight);

  while (motorL.getCount() < stepCount && motorR.getCount() < stepCount) {
    if (SENSOR_LEFT_SIDE < tLeftSide - 100 &&  SENSOR_LEFT_SIDE < 3000) {
      spdLeft = 60;
      spdRight = 60 - 10;
    }
    else if (SENSOR_RIGHT_SIDE < tRightSide - 100 && SENSOR_RIGHT_SIDE < 3000) {
      spdLeft = 60 - 10;
      spdRight = 60;
    }
    else {
      spdLeft = 60;
      spdRight = 60;
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
}

void forward() {
  motorL.setCount(0);
  motorR.setCount(0);
  motorL.forward(spd);
  motorR.forward(spd);

  while (motorL.getCount() < step1Block && motorR.getCount() < step1Block) {

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

void turnRight() {
  short m = 80;
  byte spd = 40;
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
}

void turnLeft() {
  short m = 80;
  byte spd = 40;
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
}

void mapping() {
  bool foundTarget[4] = {false, false, false, false};

  while (!foundTarget[0] && !foundTarget[1] && !foundTarget[2] && !foundTarget[3]) {
    decistionFindTarget();
    resetBlockValue();
    floodFill();
  }
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

byte getState() {
  byte state = 0;
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

void chooseBlock(bool sf[], Block *left, Block *front, Block *right) {
  //sf = (left, front, right)
  //b[][0] = y
  //b[][1] = x
  if (direction == 'N') {
    if (sf[0]) {
      *left = blocks[py][px - 1];
    }
    if (sf[1]) {
      *front = blocks[py - 1][px];
    }
    if (sf[2]) {
      *right = blocks[py][px + 1];
    }
  }
  if (direction == 'E') {
    if (sf[0]) {
      *left = blocks[py - 1][px];
    }
    if (sf[1]) {
      *front = blocks[py][px + 1];
    }
    if (sf[2]) {
      *right = blocks[py + 1][px];
    }
  }
  if (direction == 'S') {
    if (sf[0]) {
      *left = blocks[py][px + 1];
    }
    if (sf[1]) {
      *front = blocks[py + 1][px];
    }
    if (sf[2]) {
      *right = blocks[py][px - 1];
    }
  }
  if (direction == 'W') {
    if (sf[0]) {
      *left = blocks[py + 1][px];
    }
    if (sf[1]) {
      *front = blocks[py][px - 1];
    }
    if (sf[2]) {
      *right = blocks[py - 1][px];
    }
  }
}

void decistionFindTarget() {
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

  byte state = getState();

  //  0 3-Way
  bool sf[3] = {true, true, true};
  //0:left, 1:front, 2:right
  //0: y, 1: x
  Block left, front, right;

  if (state == 0) {
    chooseBlock(sf, &left, &front, &right);
    //front
    if (front.flag == false && front.mark == false)
      forward();
    //right
    else if (right.flag == false && right.mark == false)
      turnRight();
    //left
    else if (left.flag == false && left.mark == false)
      turnLeft();
    else if (over && front.count <= right.count && front.count <= left.count)
      forward();
    else if (over && right.count <= front.count && right.count <= left.count)
      turnRight();
    else if (over && left.count <= front.count && left.count <= right.count)
      turnLeft();
    else if (front.value <= right.value && front.value <= left.value)
      forward();
    else if (right.value <= front.value && right.value <= left.value)
      turnRight();
    else
      turnLeft();
  }
  //1 2-Way
  else if (state == 1) {
    //compare potential between front and right
    sf[0] = false;
    chooseBlock(sf, &left, &front, &right);

    if (front.flag == false && front.mark == false)
      forward();
    else if (right.flag == false && right.mark == false)
      turnRight();
    else if (over && front.count <= right.count)
      forward();
    else if (over && right.count <= front.count)
      turnRight();
    else if (front.value <= right.value)
      forward();
    else
      turnRight();
  }
  //2 2-Way
  else if (state == 2) {
    //compare potential between left and right
    sf[1] = false;
    chooseBlock(sf, &left, &front, &right);

    if (right.flag == false && right.mark == false)
      turnRight();
    else if (left.flag == false && left.mark == false)
      turnLeft();
    else if (over && right.count <= left.count)
      turnRight();
    else if (over && left.count <= right.count)
      turnLeft();
    else if (right.value <= left.value)
      turnRight();
    else
      turnLeft();
  }
  //4 2-Way
  else if (state == 4) {
    //compare potential between front and left
    sf[2] = false;
    chooseBlock(sf, &left, &front, &right);

    if (front.flag == false && front.mark == false)
      forward();
    else if (left.flag == false && left.mark == false)
      turnLeft();
    else if (over && front.count <= left.count)
      forward();
    else if (over && left.count <= front.count)
      turnLeft();
    else if (front.value <= left.value)
      forward();
    else
      turnLeft();
  }
  //3 1-Way
  else if (state == 3) {
    turnRight();
  }
  //5 1-Way
  else if (state == 5) {
    forward();
  }
  //6 1-Way
  else if (state == 6) {
    turnLeft();
  }
  //7 0-Way
  else if (state == 7) {
    blocks[py][px].mark = true;
    turnRight();
    turnRight();
  }
}
