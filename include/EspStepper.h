#ifndef ESPSTEPPER_H
#define ESPSTEPPER_H

#include <Arduino.h>
#include <Ticker.h>
// #include <defines.h>

class EspStepper {
public:
  enum class MotorMode { POSITION, VELOCITY };

  EspStepper(int sPin, int dPin, int ePin, int ch, int stepsPerRev,
             int microStep, int gearRatio);
  void begin();
  void setMode(MotorMode mode);
  void setDirection(bool direction);
  void setRPM(int rpm);
  void step(int period);
  void stop(bool enable);
  void rotate_deg(float angle, float speedRPM = -1);
  void move_mm(int distance, float speedRPM = -1);
  float getCurrentRPM();
  void setRPMbyAcceleration(int rpm);

private:
  int stepPin, dirPin, enablePin;
  int channel;
  MotorMode currentMode;
  float currentRPM;
  float currentAngle_deg;
  float currentPos_mm;
  const int stepsPerRevolution;
  const int microStepping;
  const int gearRatio;
  unsigned long pulseWidth;
  bool rotating;
  Ticker rotationTimer;
  Ticker accelerationTimer;

  void setupPins();
  void enableMotor(bool enable);
  void rotate_tenth_deg(float angle, float speedRPM = -1);
  void onAccelerationStep();
  void onRotationComplete();
  static void accelerationStepCallback(EspStepper *instance);
  static void rotationCompleteCallback(EspStepper *instance);
};

EspStepper::EspStepper(int sPin, int dPin, int ePin, int ch, int stepsPerRev,
                       int microStep, int gearRatio)
    : stepPin(sPin), dirPin(dPin), enablePin(ePin), channel(ch),
      stepsPerRevolution(stepsPerRev), microStepping(microStep),
      gearRatio(gearRatio), currentMode(MotorMode::POSITION), currentRPM(0),
      currentAngle_deg(0), currentPos_mm(0), pulseWidth(0), rotating(false) {}

void EspStepper::begin() {
  setupPins();
  enableMotor(false);
  vTaskDelay(pdMS_TO_TICKS(10));
  enableMotor(true);
  ledcSetup(channel, 1000, 2);
  ledcWrite(channel, 1);
  // setRPM(0);
}

float EspStepper::getCurrentRPM() { return this->currentRPM; }

void EspStepper::setMode(MotorMode mode) {
  if (currentMode != mode) {
    currentMode = mode;
    if (mode == MotorMode::POSITION) {
      stop(true);
      ledcDetachPin(stepPin);
      pinMode(stepPin, OUTPUT);
    } else {
      ledcAttachPin(stepPin, channel);
      stop(true);
    }
  }
}

void EspStepper::setDirection(bool direction) {
  digitalWrite(dirPin, direction ? HIGH : LOW);
}

void EspStepper::setRPM(int rpm) {
  currentRPM = rpm;
  setMode(MotorMode::VELOCITY);
  uint32_t frequency =
      (rpm * stepsPerRevolution * microStepping * gearRatio) / 60;
  ledcWriteTone(channel, frequency);
  Serial.print("RPM set :");
  Serial.println(rpm);
}

void EspStepper::setRPMbyAcceleration(int RPM) {
  setMode(MotorMode::VELOCITY);

  if (RPM > 35) {
    RPM = 35;
  } else if (RPM < -30) {
    RPM = -30;
  }

  int rpm = map(RPM, 0, 100, 0, 1900);

  float maxRPMChangePerInterval = 25;
  float rpmIncrement =
      (rpm > currentRPM) ? maxRPMChangePerInterval : -maxRPMChangePerInterval;

  if (rpm > currentRPM) {
    while (currentRPM < rpm) {
      currentRPM += rpmIncrement;
      if (currentRPM >= rpm)
        currentRPM = rpm;
      setRPM(currentRPM);
      delay(150);
    }
  }

  else if (rpm < currentRPM) {
    while (currentRPM > rpm) {
      currentRPM += rpmIncrement;
      if (currentRPM <= rpm)
        currentRPM = rpm;
      setRPM(currentRPM);
      delay(100);
    }
    setRPM(rpm);
  }
}

void EspStepper::step(int period) {
  setMode(MotorMode::POSITION);
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(period);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(period);
}

void EspStepper::stop(bool enable) {
  if (currentMode == MotorMode::VELOCITY) {
    setRPM(0);
  }
  enableMotor(enable);
}

void EspStepper::setupPins() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
}

void EspStepper::enableMotor(bool enable) {
  digitalWrite(enablePin, enable ? LOW : HIGH);
}

void EspStepper::move_mm(int distance, float speedRPM) {
  if (rotating) {
    return;
  }
  Serial.printf("Rotating by %d mm at %.2f RPM\n", distance, speedRPM);
  currentPos_mm = distance;
  this->rotate_deg(360 * distance / 10.0, speedRPM);
}

void EspStepper::rotate_tenth_deg(float angle, float speedRPM) {
  float stepsPerDegree =
      (stepsPerRevolution * microStepping * gearRatio) / 360.0;
  uint32_t frequency =
      (speedRPM * stepsPerRevolution * microStepping * gearRatio) / 60;
  uint32_t totalDuration = (1.0 / 360.0) * (60000.0 / speedRPM);
  uint32_t temp = ledcWriteTone(channel, frequency);
  if (!temp) {
    ledcWriteTone(channel, 0);
    return;
  }
  accelerationTimer.once_ms(totalDuration, accelerationStepCallback, this);
}

void EspStepper::rotate_deg(float angle, float speedRPM) {
  if (rotating) {
    return;
  }
  Serial.printf("Rotating by %.2f deg at %.2f RPM\n", angle, speedRPM);
  rotating = true;

  currentAngle_deg = angle;

  float stepsPerDegree =
      (stepsPerRevolution * microStepping * gearRatio) / 360.0;
  setDirection(angle >= 0);
  uint32_t frequency =
      (speedRPM * stepsPerRevolution * microStepping * gearRatio) / 60;
  uint32_t totalDuration = (abs(angle) / 360.0) * (60000.0 / speedRPM);

  uint32_t temp = ledcWriteTone(channel, frequency);
  Serial.printf("frequency generated @ %d Hz\n", temp);
  if (!temp) {
    ledcWriteTone(channel, 0);
    return;
  }
  rotationTimer.once_ms(totalDuration, rotationCompleteCallback, this);
}

void EspStepper::rotationCompleteCallback(EspStepper *instance) {
  instance->onRotationComplete();
}

void EspStepper::onRotationComplete() {
  ledcWriteTone(channel, 0);
  rotating = false;
}

void EspStepper::accelerationStepCallback(EspStepper *instance) {
  instance->onAccelerationStep();
}

// void EspStepper::onAccelerationStep() { ledcWriteTone(channel, 0); }

// void EspStepper::rotate_deg(float angle, float speedRPM,
//                             float accelerationRPM) {
//   if (rotating) {
//     return;
//   }
//   Serial.printf(
//       "Rotating by %.2f deg at %.2f RPM with acceleration %.2f RPM/s\n",
//       angle, speedRPM, accelerationRPM);
//   rotating = true;
//   currentAngle_deg = angle;

//   float stepsPerDegree =
//       (stepsPerRevolution * microStepping * gearRatio) / 360.0;
//   setDirection(angle >= 0);

//   // Calculate total steps
//   uint32_t totalSteps = abs(angle) * stepsPerDegree;

//   // Total duration at max RPM without acceleration
//   uint32_t totalDuration = (abs(angle) / 360.0) * (60000.0 / speedRPM);

//   // Calculate acceleration steps per time
//   float stepIncrementPerMs = accelerationRPM / 1000.0; // RPM/ms

//   uint32_t currentRPM = 0; // Starting RPM
//   uint32_t frequency = 0;
//   uint32_t elapsedTime = 0; // Time elapsed since start

//   // Increment RPM smoothly from 0 to speedRPM based on acceleration
//   while (elapsedTime < totalDuration) {
//     if (currentRPM < speedRPM) {
//       currentRPM += stepIncrementPerMs;
//       if (currentRPM > speedRPM)
//         currentRPM = speedRPM; // Cap RPM
//     }

//     frequency =
//         (currentRPM * stepsPerRevolution * microStepping * gearRatio) / 60;

//     // Set the frequency for the PWM signal
//     ledcWriteTone(channel, frequency);

//     // Wait for the next step increment (you can tune the time for smoother
//     // acceleration)
//     delay(1); // Adjust delay for smoother increments (this will affect how
//               // smooth the acceleration is)
//     elapsedTime++;
//   }

//   // At this point, we should be at max RPM, continue the movement to
//   complete
//   // the rotation
//   rotationTimer.once_ms(totalDuration - elapsedTime,
//   rotationCompleteCallback,
//                         this);
// }

#endif
