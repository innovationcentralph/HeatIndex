#ifndef SHIFTREGISTER_H
#define SHIFTREGISTER_H

#include <Arduino.h>

class ShiftRegister {
  public:
    ShiftRegister(uint8_t dataPin, uint8_t latchPin, uint8_t clockPin, uint8_t numRegisters);
    void setBit(uint8_t bit, bool value);
    void setAll(bool value);
    void updateRegisters();
    void setDebuggingMode(bool mode);
    void setBitOrder(uint8_t bitOrder);
    uint8_t* registerValues;
    

  private:
    uint8_t _dataPin;
    uint8_t _latchPin;
    uint8_t _clockPin;
    uint8_t _numRegisters;
    uint8_t* _registerValues;
    bool DEBUG = false;
    uint8_t _bitOrder = LSBFIRST;
    
};

#endif
