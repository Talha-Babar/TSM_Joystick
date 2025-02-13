#ifndef DWIN_H
#define DWIN_H

#include <Arduino.h>

class DwinController {
public:
  // HardwareSerial DWIN(1);

  DwinController(uint8_t txPin, uint8_t rxPin);
  void begin();
  void Rotate();
  int writeDwin(int x, uint8_t y);
  void setNeedlePosition(int inputAngle);

private:
  uint8_t txPin;
  uint8_t rxPin;
  float ii;
  int jj;
  bool dir;
};

HardwareSerial DWIN(1);

// Constructor
DwinController::DwinController(uint8_t rxPin, uint8_t txPin)
    : txPin(txPin), rxPin(rxPin), ii(0), jj(0), dir(false) {}

// Begin function
void DwinController::begin() { DWIN.begin(115200, SERIAL_8N1, rxPin, txPin); }

// Update values and handle logic
void DwinController::Rotate() {
  // Serial.println();
  // Serial.println();
  writeDwin(ii, 0x01);
  writeDwin(jj, 0x00);

  if (jj % 10 == 0) {
    dir ? ii += 0.06 : ii -= 0.06;
  }

  dir ? jj += 5 : jj -= 5;

  if (ii >= 600) {
    ii = 0;
  }
  if (ii < 0) {
    ii = 600;
  }
  if (jj >= 1000) {
    jj = 0;
  }
  if (jj < 0) {
    jj = 1000;
  }
  dir = 0;
  delay(10);
}

// Write function
int DwinController::writeDwin(int x, uint8_t y) {
  DWIN.write(0x5a);
  DWIN.write(0xa5);
  DWIN.write(0x05);
  DWIN.write(0x82);
  DWIN.write(0x10);
  DWIN.write(y);
  DWIN.write(highByte(x));
  DWIN.write(lowByte(x));
  return x;
}

void DwinController::setNeedlePosition(int inputAngle) {
  if (inputAngle >= 0 && inputAngle <= 36000) {
    int Needle_1;
    int Needle_2;
    int angle_1 = inputAngle / 100;
    int angle_2 = inputAngle % 100;

    Needle_1 = map((angle_1 + 180) % 360, 0, 360, 0, 600);
    Needle_2 = map(angle_2, 0, 100, 0, 1000);

    // Write both needles to the DWIN display
    writeDwin(Needle_1, 1);
    writeDwin(Needle_2, 0);
  }
}

#endif // DWIN_H