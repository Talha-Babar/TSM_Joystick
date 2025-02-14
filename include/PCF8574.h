#ifndef PCF8574_H
#define PCF8574_H

#include <Arduino.h>
#include <Wire.h>

class PCF8574 {
private:
  uint8_t address;
  uint8_t gpioState;

public:
  // Constructor
  PCF8574(uint8_t addr) : address(addr), gpioState(0xFF) {}

  // Initialize the device
  void begin() {
    Wire.begin();    // Ensure I2C is initialized
    writeGPIO(0xFF); // Set all pins as input by default (HIGH)
  }

  // Read full GPIO byte
  uint8_t readGPIO() {
    Wire.requestFrom(address, (uint8_t)1);
    if (Wire.available()) {
      gpioState = Wire.read();
    }
    return gpioState;
  }

  // Read specific pin state
  bool readPin(uint8_t pin) {
    if (pin > 7)
      return false;
    readGPIO(); // Ensure we get the latest state
    return (gpioState & (1 << pin)) != 0;
  }

  // Write full GPIO byte
  void writeGPIO(uint8_t value) {
    gpioState = value;
    Wire.beginTransmission(address);
    Wire.write(gpioState);
    Wire.endTransmission();
  }

  // Write specific pin state
  void writePin(uint8_t pin, bool value) {
    if (pin > 7)
      return;
    if (value) {
      gpioState |= (1 << pin); // Set pin HIGH
    } else {
      gpioState &= ~(1 << pin); // Set pin LOW
    }
    writeGPIO(gpioState); // Send updated state
  }
};

#endif
