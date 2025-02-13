#ifndef PCF8574_H
#define PCF8574_H

#include <Arduino.h>
#include <Wire.h>

class PCF8574 {
private:
  uint8_t address;
  uint8_t gpioState;

public:
  PCF8574(uint8_t addr) : address(addr), gpioState(0xFF) {}

  void begin() {

    Wire.beginTransmission(address);
    Wire.write(0xFF);
    Wire.endTransmission();
  }

  uint8_t readGPIO() {
    Wire.requestFrom(address, (uint8_t)1);
    if (Wire.available()) {
      gpioState = Wire.read();
    }
    return gpioState;
  }

  bool readPin(uint8_t pin) {
    if (pin > 7)
      return false;
    return (gpioState & (1 << pin)) != 0;
  }
};

#endif
