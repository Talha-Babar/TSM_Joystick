#include "AiEsp32RotaryEncoder.h"
#include "EspStepper.h"
#include "GamepadDevice.h"
#include "PCF8574.h"
#include "USB.h"
#include "USBHID.h"
#include "defines.h"
#include "dwin.h"
#include "pcntEncoder.h"
#include <PubSubClient.h>
#include <WiFi.h>

USBHIDGamepad Gamepad;

WiFiClient espClient;
PubSubClient client(espClient);

// Encoder encoder(SCL, SDA);
EspStepper stepper(Pin_PUL, Pin_DIR, Pin_ENA, LEDC_CHAN_STP, STEPS_PER_REV,
                   MICROSTEP, GEARRATIO);
DwinController dwinController(DWIN_RX, DWIN_TX);

PCF8574 PCF1(0x20);
PCF8574 PCF2(0x21);
PCF8574 PCF3(0x22);
PCF8574 PCF4(0x23);
PCF8574 PCF5(0x25);

pcntEncoder Mechanical_ENC1(PCNT_UNIT_1, Mechanical_ENC1_A, Mechanical_ENC1_B);
pcntEncoder Mechanical_ENC2(PCNT_UNIT_0, Mechanical_ENC2_A, Mechanical_ENC2_B);
pcntEncoder DayScope_ENC1(PCNT_UNIT_3, DayScope_ENC2_A, DayScope_ENC2_B);
pcntEncoder NightScope_ENC1(PCNT_UNIT_2, NightScope_ENC1_A, NightScope_ENC1_B);

AiEsp32RotaryEncoder DayScope_ENC2(DayScope_ENC1_A, DayScope_ENC1_B, -1, -1,
                                   AIESP32ROTARYENCODER_DEFAULT_STEPS, false);
AiEsp32RotaryEncoder DayScope_ENC3(DayScope_ENC3_A, DayScope_ENC3_B, -1, -1,
                                   AIESP32ROTARYENCODER_DEFAULT_STEPS, false);

bool encoderFlag = false;
bool previousEncoderFlag = encoderFlag; // Store the previous state

void updateEncoders();
void reconnect();
void motorControlTask(void *pvParameters);
void readTask(void *pvparameters);
void setup_wifi();
void callback(char *topic, byte *message, unsigned int length);
int8_t limit(int value, int min, int max);

void IRAM_ATTR readEncoderISR_2() { DayScope_ENC2.readEncoder_ISR(); }
void IRAM_ATTR readEncoderISR_3() { DayScope_ENC3.readEncoder_ISR(); }

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA, SCL);
  Wire.setClock(100000);

  DayScope_ENC2.areEncoderPinsPulldownforEsp32 = false;
  DayScope_ENC2.begin();
  DayScope_ENC2.setup(readEncoderISR_2);
  DayScope_ENC2.setBoundaries(-999999, 999999, false);
  DayScope_ENC2.setAcceleration(1);

  DayScope_ENC3.areEncoderPinsPulldownforEsp32 = false;
  DayScope_ENC3.begin();
  DayScope_ENC3.setup(readEncoderISR_3);
  DayScope_ENC3.setBoundaries(-999999, 999999, false);
  DayScope_ENC3.setAcceleration(1);

  pinMode(YAW_PIN, INPUT);
  pinMode(PITCH_PIN, INPUT);

  PCF1.begin();
  PCF2.begin();
  PCF3.begin();
  PCF4.begin();
  PCF5.begin();
  dwinController.begin();
  stepper.begin();

  // dwinController.setNeedlePosition(800);
  // stepper.setRPMbyAcceleration(60);

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  vTaskDelay(5000);

  Mechanical_ENC1.begin();
  Mechanical_ENC2.begin();
  DayScope_ENC1.begin();
  NightScope_ENC1.begin();

  dwinController.setNeedlePosition(0);
  stepper.setRPMbyAcceleration(0);

  xTaskCreatePinnedToCore(motorControlTask, "MotorControl", 4096, NULL, 1, NULL,
                          1);
  xTaskCreatePinnedToCore(readTask, "readTask", 4096, NULL, 1, NULL, 1);

  USB.begin();
  Gamepad.begin();
}

void loop() {

  // Read Yaw (ADC channel 1)
  uint16_t result_adc1 = analogRead(YAW_PIN); // Read 12-bit ADC value (0-4095)
  float voltage1 = (result_adc1 / 4095.0) * 3.3; // Convert ADC value to voltage
  int8_t Yaw_value = constrain((((voltage1 - 1.5) / (3.1 - 1.5)) * 255.0), -128,
                               127); // Normalize & constrain to int8_t

  // Read Pitch (ADC channel 2)
  uint16_t result_adc2 = analogRead(PITCH_PIN);
  float voltage2 = (result_adc2 / 4095.0) * 3.3;
  int8_t Pitch_value =
      constrain((((voltage2 - 1.5) / (3.1 - 1.5)) * 255.0), -128, 127);

  if (debug) {

    Serial.print("Yaw_value: ");
    Serial.println(Yaw_value);
    Serial.print("voltage1: ");
    Serial.println(voltage1);
    Serial.print("Pitch_value: ");
    Serial.println(Pitch_value);
    Serial.print("voltage2: ");
    Serial.println(voltage2);
  }

  uint32_t buttons = 0;
  uint8_t Temp_DayNight = 0;
  uint8_t Temp_SmokeGernade_1 = 0;
  uint8_t Temp_SmokeGernade_2 = 0;
  uint8_t Temp_Mechanical = 0;
  uint8_t Temp_Electrical = 0;
  // Read GPIO values from the IO expanders and invert bits
  Temp_DayNight = ~PCF1.readGPIO();
  Temp_SmokeGernade_1 = ~PCF2.readGPIO();
  Temp_SmokeGernade_2 = ~PCF3.readGPIO();
  Temp_Mechanical = ~PCF4.readGPIO();
  Temp_Electrical = ~PCF5.readGPIO();

  if (debug) {
    Serial.print("Temp_DayNight: ");
    Serial.println(Temp_DayNight, BIN);
    Serial.print("Temp_Electrical: ");
    Serial.println(Temp_Electrical, BIN);
  }

  // Remove the 0th bit from Temp_Mechanical
  // Temp_Mechanical &= ~0x01; // Mask out bit 0 (keep bits 1-7)

  // Remove the 5th bit from Temp_Electrical while keeping the rest
  // Temp_Electrical &= ~0x20; // Mask out bit 5 (keep bits 0-4, 6-7)

  // Shift and store into a single 32-bit variable
  buttons = 0; // Ensure buttons starts empty

  buttons |= Temp_DayNight;                 // Bits 0-7 (8 bits)
  buttons |= (Temp_SmokeGernade_1 << 8);    // Bits 8-9 (2 bits)
  buttons |= ((Temp_SmokeGernade_2) << 10); // Bits 10-17 (8 bits)
  buttons |= ((Temp_Mechanical >> 1) << 18);
  buttons |=
      (((Temp_Electrical & 0x0F) | ((Temp_Electrical & 0xF0) >> 1)) << 25);
      PCF4.writePin(0, bitRead(buttons,28));
      // delay(10);
      // PCF5.writePin(0, bitRead(buttons,31));
      // if(bitRead(buttons,28)){  //K1
      //   PCF4.writePin(0, true) ;//Red LED
      // }else{
      //   PCF4.writePin(0, false) ;//Red LED
      // }

      // if(bitRead(buttons,31)){// Firing MG
      //   PCF5.writePin(5, false) ;//Green LED
      // }else{
      //   PCF5.writePin(5, true) ;//Green LED
      // }

    // Serial.println(Temp_Electrical & (1 << 4));
    // Serial.println(Temp_Mechanical & (1 << 1));


      // PCF5.writePin(0, 1) ;//Green LED
      // PCF4.writePin(0, false) ;//Red LED


      
     

  // Debugging Output
  if (1) {
    Serial.print("Buttons: ");
    Serial.println(buttons, BIN);
  }

  // Read first set of encoders
  int8_t Mechanical_1 = Mechanical_ENC1.getDifferential();
  Mechanical_1 = limit(Mechanical_1, Axes_Min, Axes_Max);
  int8_t Mechanical_2 = Mechanical_ENC2.getDifferential();
  Mechanical_2 = limit(Mechanical_2, Axes_Min, Axes_Max);
  int8_t DayScope_1 = DayScope_ENC1.getDifferential();
  DayScope_1 = limit(DayScope_1, Axes_Min, Axes_Max);
  int8_t NightScope = NightScope_ENC1.getDifferential();
  NightScope = limit(NightScope, Axes_Min, Axes_Max);

  if (debug) {
    Serial.print("Mechanical_1: ");
    Serial.println(Mechanical_1);
    Serial.print("NightScope: ");
    Serial.println(NightScope);
  }

  static int lastPosition_ENC2;
  int currentPosition_ENC2 = (DayScope_ENC2.readEncoder());
  int8_t DayScope_2 = currentPosition_ENC2 - lastPosition_ENC2;
  lastPosition_ENC2 = currentPosition_ENC2;

  static int lastPosition_ENC3;
  int currentPosition_ENC3 = (DayScope_ENC3.readEncoder());
  int8_t DayScope_3 = currentPosition_ENC3 - lastPosition_ENC3;
  lastPosition_ENC3 = currentPosition_ENC3;

  if (debug) {
    Serial.print("DayScope_3: ");
    Serial.println(DayScope_3);
  }

  // Send data to gamepad
  if (!Gamepad.send(Yaw_value, Pitch_value, Mechanical_1, Mechanical_2,
                    DayScope_1, DayScope_2, DayScope_3, NightScope, 0, buttons)) {
    Serial.println("Failed to send gamepad report.");
  }

  delay(10);
}

void readTask(void *pvparameters) {
  for (;;) {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
    dwinController.setNeedlePosition(TSM_Aux_value);
  }
}

void motorControlTask(void *pvParameters) {

  for (;;) {
    if (digitalRead(Pin_ALA)) {
      stepper.begin();
    }
    stepper.setRPMbyAcceleration(TSM_motor_value);
    vTaskDelay(30);
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client_Station")) {
      Serial.println("connected");
      client.subscribe(topic1);
      client.subscribe(topic2);
      client.subscribe(topic3);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char *topic, byte *message, unsigned int length) {
  if (debug) {
    Serial.print("Message arrived on topic: ");
    Serial.print(topic);
    Serial.print(". Message: ");
  }

  String messageTemp = "";
  for (int i = 0; i < length; i++) {
    messageTemp += (char)message[i];
  }

  if (debug) {
    Serial.println(messageTemp);
  }

  // Convert message to integer and process based on topic
  if (String(topic) == topic1) {
    TSM_motor_value = messageTemp.toInt(); // Convert to integer and store
    if (debug) {
      Serial.print("Stored TSM_motor_value: ");
      Serial.println(TSM_motor_value);
    }
  } else if (String(topic) == topic2) {
    TSM_Aux_value = messageTemp.toInt(); // Convert to integer and store
    if (debug) {
      Serial.print("Stored TSM_Aux_value: ");
      Serial.println(TSM_Aux_value);
    }
  } else if (String(topic) == topic3) {
    // Update encoderFlag based on received MQTT message
    int flagValue = messageTemp.toInt();
    if (flagValue == 0 || flagValue == 1) {
      encoderFlag = flagValue;
      if (debug) {

        Serial.print("Encoder flag updated to: ");
        Serial.println(encoderFlag);
      }
    } else {
      if (debug) {

        Serial.println("Invalid encoder flag value received.");
      }
    }
  }
}

int8_t limit(int value, int min, int max) {
  // Limit the value first
  if (value > max) {
    value = max;

  } else if (value < min) {
    value = min;
  }

  int8_t mapped_value = (int8_t)(((value - min) * 255 / (max - min)) - 128);

  return value;
}