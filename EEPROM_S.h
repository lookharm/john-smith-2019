#ifndef EEPROM_H
#define EEPROM_H

#include <Arduino.h>
#include "EEPROM.h"

void EEPROM_init(int size) {
  EEPROM.begin(size);
}

/***
   Read/Write EPROM
   By AJ.SOMSIN
*/
void EEPROM_write_int(int _addr, int _data) {
  EEPROM.write(_addr + 0 , (_data >> 0) & 0x000000FF);
  EEPROM.write(_addr + 1 , (_data >> 8) & 0x000000FF);
  EEPROM.write(_addr + 2 , (_data >> 16) & 0x000000FF);
  EEPROM.write(_addr + 3 , (_data >> 24) & 0x000000FF);
  EEPROM.commit();
}

int EEPROM_read_int(int _addr) {
  int _data = 0x00000000;
  _data = (EEPROM.read(_addr + 0) << 0) | (EEPROM.read(_addr + 1) << 8) | (EEPROM.read(_addr + 2) << 16) | (EEPROM.read(_addr + 3) << 24);
  return _data;
}

#endif
