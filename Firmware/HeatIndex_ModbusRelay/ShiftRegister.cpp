#include "ShiftRegister.h"

ShiftRegister::ShiftRegister(uint8_t dataPin, uint8_t latchPin, uint8_t clockPin, uint8_t numRegisters) {
  _dataPin = dataPin;
  _latchPin = latchPin;
  _clockPin = clockPin;
  _numRegisters = numRegisters;
  _registerValues = new uint8_t[numRegisters];
  memset(_registerValues, 0, numRegisters);
  pinMode(_dataPin, OUTPUT);
  pinMode(_latchPin, OUTPUT);
  pinMode(_clockPin, OUTPUT);
}

void ShiftRegister::setBit(uint8_t bit, bool value) {
  uint8_t registerNum = (bit - 1) / 8;
  uint8_t bitNum = (bit - 1) % 8;

  if (value) {
    _registerValues[registerNum] |= (1 << bitNum);
  } else {
    _registerValues[registerNum] &= ~(1 << bitNum);
  }
  
  // Debug output
  if(DEBUG){
    Serial.print("REGISTER NUM: ");
    Serial.println(registerNum);
    Serial.print("BIT NUM: ");
    Serial.println(bitNum);
    Serial.print("REG VALUE: ");
    Serial.println(_registerValues[registerNum], BIN);
  }

  registerValues = _registerValues;

  
}

void ShiftRegister::setAll(bool value) {
  uint8_t bitValue = value ? 0xFF : 0x00;
  for (int i = 0; i < _numRegisters; i++) {
    _registerValues[i] = bitValue;
  }
  updateRegisters();
}

void ShiftRegister::updateRegisters() {
  if(DEBUG){
    Serial.println("REGISTERS");
  }
  digitalWrite(_latchPin, LOW);
  for (int i = 0; i < _numRegisters; i++)  {
    shiftOut(_dataPin, _clockPin, _bitOrder, _registerValues[i]);
    if(DEBUG){
      Serial.print(_registerValues[i], BIN); Serial.print(" ");
    }
  }
  if(DEBUG){
    Serial.println();
  }
  digitalWrite(_latchPin, HIGH);
}

void ShiftRegister::setDebuggingMode(bool mode){
  DEBUG = mode;
}

void ShiftRegister::setBitOrder(uint8_t bitOrder){
  _bitOrder = bitOrder;
}
