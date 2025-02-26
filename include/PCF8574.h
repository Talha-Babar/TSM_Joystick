#ifndef PCF8574_H
#define PCF8574_H

#include <Arduino.h>
#include <Wire.h>

class PCF8574 {
private:
  uint8_t address;
  uint8_t gpioState; // Stores the current state of GPIO pins

public:
  // Constructor: Initializes the PCF8574 I2C address
  PCF8574(uint8_t addr) : address(addr), gpioState(0xFF) {}

  // Initializes communication and sets all pins as inputs with pull-ups
  void begin() {
    // Wire.begin();
    gpioState = 0xFF; // Default all pins HIGH (pull-up enabled)
    Wire.beginTransmission(address);
    Wire.write(gpioState);
    Wire.endTransmission();
  }

  // Reads the entire GPIO state from PCF8574
  uint8_t readGPIO() {
    Wire.requestFrom(address, (uint8_t)1);
    if (Wire.available()) {
      gpioState = Wire.read();
    }
    return gpioState;
  }

  // Reads the state of a specific pin (0-7)
  bool readPin(uint8_t pin) {
    if (pin > 7)
      return false; // Invalid pin number
    // return (gpioState & (1 << pin)) != 0;
    return bitRead(gpioState,pin);
  }

  // Writes a new GPIO state to the PCF8574
  void writeGPIO(uint8_t state) {
    gpioState = state; // Store the new state
    Wire.beginTransmission(address);
    Wire.write(gpioState);
    Wire.endTransmission();
  }

  // Writes a specific pin HIGH or LOW
  void writePin(uint8_t pin, bool state) {
    if (pin > 7)
      return; // Invalid pin number

    gpioState=0xFF;
    bitWrite(gpioState,pin,state);

    // if (state) {
    //   gpioState |= (1 << pin); // Set pin HIGH
    // } else {
    //   gpioState &= ~(1 << pin); // Set pin LOW
    // }

    // Send updated state to PCF8574
    Wire.beginTransmission(address);
    Wire.write(gpioState);
    Wire.endTransmission();
  }
};

#endif
