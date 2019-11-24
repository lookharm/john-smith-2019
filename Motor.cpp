#include <Arduino.h>
#include <FunctionalInterrupt.h>
#include "Motor.h"

Motor::Motor() {

}

Motor::~Motor() {
  detachInterrupt(this->OUTA);
}

void Motor::init(uint8_t INA, uint8_t INB, uint8_t OUTA, uint8_t OUTB, uint8_t channelA, uint8_t channelB) {
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);

  pinMode(OUTA, INPUT);
  pinMode(OUTB, INPUT);

  //stop motor from PWM on boot.
  digitalWrite(INA, LOW);
  digitalWrite(INB, LOW);

  //recommend detect falling
  //eye diagram analyze pulse
  attachInterrupt(OUTA, std::bind(&Motor::isr, this), FALLING);

  //attach to PIN to channel
  ledcAttachPin(INA, channelA);
  ledcAttachPin(INB, channelB);
  ledcSetup(channelA, 490, 8); // 490 Hz PWM, 8-bit resolution
  ledcSetup(channelB, 490, 8); // 490 Hz PWM, 8-bit resolution
  //low f -> control power very good
  this->INA = INA;
  this->INB = INB;
  this->OUTA = OUTA;
  this->OUTB = OUTB;
  this->channelA = channelA;
  this->channelB = channelB;
  this->count = 0;
};

void Motor::forward(int pwm) {
  ledcWrite(this->channelA, pwm);
  ledcWrite(this->channelB, 0);
  delay(1);
}

void Motor::backward(int pwm) {
  ledcWrite(this->channelA, 0);
  ledcWrite(this->channelB, pwm);
  delay(1);
}

void Motor::stop() {
  ledcWrite(this->channelA, 0);
  ledcWrite(this->channelB, 0);
  delay(1);
}

void IRAM_ATTR Motor::isr() {
  if (digitalRead(this->OUTB) == LOW) {
    this->count++;
  }
  else {
    this->count--;
  }
}

int32_t Motor::getCount() {
  return this->count;
}

void Motor::setCount(int32_t c) {
  this->count = 0;
}
