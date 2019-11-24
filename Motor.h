#ifndef MOTOR_H
#define MOTOR_H

class Motor
{
  public:
    Motor();
    ~Motor();
    void init(uint8_t INA, uint8_t INB, uint8_t OUTA, uint8_t OUTB, uint8_t channelA, uint8_t channelB);
    void forward(int pwm);
    void backward(int pwm);
    void stop();
    void IRAM_ATTR isr();
    int32_t getCount();
    void setCount(int32_t c);

  private:
    uint8_t INA;
    uint8_t INB;
    uint8_t OUTA;
    uint8_t OUTB;
    uint8_t channelA;
    uint8_t channelB;

    volatile int32_t count;
};

#endif
