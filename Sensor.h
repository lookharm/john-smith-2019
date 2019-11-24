#ifndef SENSOR_H
#define SENSOR_H

class Sensor
{
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
};

#endif
