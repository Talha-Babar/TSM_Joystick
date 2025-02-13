#include "GamepadDevice.h"
#include "PCF8574.h"
#include "USB.h"
#include "USBHID.h"
#include "encoder.h"
#include <EspStepper.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <defines.h>
#include <dwin.h>

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
PCF8574 PCF5(0x24);

Encoder Mechanical_ENC1(PCNT_UNIT_1, Mechanical_ENC1_A, Mechanical_ENC1_B);
Encoder Mechanical_ENC2(PCNT_UNIT_0, Mechanical_ENC2_A, Mechanical_ENC2_B);
Encoder DayScope_ENC1(PCNT_UNIT_3, DayScope_ENC1_A, DayScope_ENC1_B);
// Encoder DayScope_ENC2(PCNT_UNIT_3, DayScope_ENC2_A, DayScope_ENC2_B);
// Encoder DayScope_ENC3(PCNT_UNIT_1, DayScope_ENC3_A, DayScope_ENC3_B);
Encoder NightScope_ENC1(PCNT_UNIT_2, NightScope_ENC1_A, NightScope_ENC1_B);

bool encoderFlag = false;
bool previousEncoderFlag = encoderFlag; // Store the previous state

void updateEncoders();
void reconnect();
void motorControlTask(void *pvParameters);
void setup_wifi();
void callback(char *topic, byte *message, unsigned int length);

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA, SCL);

  pinMode(YAW_PIN, INPUT);
  pinMode(PITCH_PIN, INPUT);

  PCF1.begin();
  PCF2.begin();
  PCF3.begin();
  PCF4.begin();
  PCF5.begin();

  dwinController.begin();
  stepper.begin();
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  vTaskDelay(1000);

  Mechanical_ENC1.begin();
  Mechanical_ENC2.begin();
  DayScope_ENC1.begin();
  // DayScope_ENC2.begin();
  // DayScope_ENC3.begin();
  NightScope_ENC1.begin();

  dwinController.setNeedlePosition(550);
  stepper.setRPMbyAcceleration(10);
  vTaskDelay(10000);

  xTaskCreatePinnedToCore(motorControlTask, "MotorControl", 4096, NULL, 1, NULL,
                          1);

  USB.begin();
  Gamepad.begin();
}

void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  dwinController.setNeedlePosition(TSM_Aux_value);

  if (digitalRead(Pin_ALA)) {

    stepper.begin();
  }

  // if (encoderFlag != previousEncoderFlag) {
  //   updateEncoders();
  //   previousEncoderFlag = encoderFlag; // Update the previous state
  // }

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

  // Remove the 0th bit from Temp_Mechanical
  Temp_Mechanical &= ~0x01; // Mask out bit 0 (keep bits 1-7)

  // Remove the 5th bit from Temp_Electrical while keeping the rest
  Temp_Electrical &= ~0x20; // Mask out bit 5 (keep bits 0-4, 6-7)

  // Shift and store into a single 32-bit variable
  buttons = 0; // Ensure buttons starts empty

  buttons |= Temp_DayNight;                               // Bits 0-7 (8 bits)
  buttons |= (Temp_SmokeGernade_1 << 8);                  // Bits 8-9 (2 bits)
  buttons |= (((Temp_SmokeGernade_2 >> 6) & 0x03) << 10); // Bits 10-17 (8 bits)
  buttons |=
      ((Temp_Mechanical >> 1)
       << 18); // Bits 18-24 (shift right to remove bit 0, keeping 7 bits)
  buttons |= ((Temp_Electrical & 0xDF)
              << 25); // Bits 25-31 (mask bit 5, keeping 7 bits)

  // Debugging Output
  if (debug) {
    Serial.print("Buttons: ");
    Serial.println(buttons, BIN);
  }

  // Read first set of encoders
  int8_t Mechanical_1 = Mechanical_ENC1.getDifferential();
  int8_t Mechanical_2 = Mechanical_ENC2.getDifferential();
  int8_t DayScope_1 = DayScope_ENC1.getDifferential();
  int8_t NightScope = NightScope_ENC1.getDifferential();

  // Send data to gamepad
  if (!Gamepad.send(Yaw_value, Pitch_value, Mechanical_1, Mechanical_2,
                    DayScope_1, NightScope, 0, buttons)) {
    Serial.println("Failed to send gamepad report.");
  }

  delay(10);
}

// Function to switch between encoder groups
void updateEncoders() {
  // Pause and clear counters
  for (int unit = 0; unit < 4; unit++) {
    pcnt_counter_pause((pcnt_unit_t)unit);
    pcnt_counter_clear((pcnt_unit_t)unit);
  }

  if (!encoderFlag) {
    // First encoder group
    Mechanical_ENC1.setPins(PCNT_UNIT_0, PCNT_CHANNEL_0, Mechanical_ENC1_A,
                            Mechanical_ENC1_B);
    Mechanical_ENC2.setPins(PCNT_UNIT_1, PCNT_CHANNEL_0, Mechanical_ENC2_A,
                            Mechanical_ENC2_B);
    DayScope_ENC1.setPins(PCNT_UNIT_2, PCNT_CHANNEL_0, DayScope_ENC1_A,
                          DayScope_ENC1_B);
    NightScope_ENC1.setPins(PCNT_UNIT_3, PCNT_CHANNEL_0, NightScope_ENC1_A,
                            NightScope_ENC1_B);
  } else {
    // Second encoder group
    DayScope_ENC1.setPins(PCNT_UNIT_0, PCNT_CHANNEL_0, DayScope_ENC1_A,
                          DayScope_ENC1_B);
    // DayScope_ENC2.setPins(PCNT_UNIT_1, PCNT_CHANNEL_0, DayScope_ENC2_A,
    //                       DayScope_ENC2_B);
    // DayScope_ENC3.setPins(PCNT_UNIT_2, PCNT_CHANNEL_0, DayScope_ENC3_A,
    //                       DayScope_ENC3_B);
    NightScope_ENC1.setPins(PCNT_UNIT_3, PCNT_CHANNEL_0, NightScope_ENC1_A,
                            NightScope_ENC1_B);
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

void motorControlTask(void *pvParameters) {

  for (;;) {
    stepper.setRPMbyAcceleration(TSM_motor_value);
    vTaskDelay(30);
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