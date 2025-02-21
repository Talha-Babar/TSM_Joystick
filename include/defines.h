#ifndef DEFINES_H
#define DESINES_H

#define debug 0

#define NOISE_THRESHOLD 2
#define MAX_DELTA 1000

#define Axes_Min -127
#define Axes_Max 128

#define SCL 21
#define SDA 47
#define I2C_Frequency

// Electrical Joystick Button Pins

#define ADC_MAX 4095
#define JOYSTICK_MIN -128
#define JOYSTICK_MAX 127

#define YAW_PIN 11
#define PITCH_PIN 10

// Mechanical Joystick Button Pins
#define Mechanical_ENC1_A 3
#define Mechanical_ENC1_B 46
#define Mechanical_ENC2_A 18
#define Mechanical_ENC2_B 8

// NightScope Button Pins

#define NightScope_ENC1_A 37
#define NightScope_ENC1_B 38

// DayScope Button Pins

#define DayScope_ENC1_A 1
#define DayScope_ENC1_B 2
#define DayScope_ENC2_A 42
#define DayScope_ENC2_B 41
#define DayScope_ENC3_A 40
#define DayScope_ENC3_B 39

////////////////////////////////////////////////////////////////

#define STEPS_PER_REV 200
#define MICROSTEP 16
#define GEARRATIO 1
#define HOME_DIR true

// #define Pin_PUL 7
// #define Pin_DIR 6
// #define Pin_ENA 5
// #define Pin_ALA 4

#define Pin_PUL 4
#define Pin_DIR 5
#define Pin_ENA 6
#define Pin_ALA 7

#define LEDC_CHAN_STP 2

#define DWIN_TX 17
#define DWIN_RX -1

// WiFi Credentials
const char *ssid = "TSM_01";       // Replace with your WiFi network name
const char *password = "tsm2#123"; // Replace with your WiFi password

// const char *ssid_AP = "ESP32";        // Replace with your WiFi network name
// const char *password_AP = "12345678"; // Replace with your WiFi password

// MQTT Broker IP and Port
const char *mqtt_server = "192.168.41.4";
const int mqtt_port = 1883;

// MQTT Topics
const char *topic1 = "TSM_motor";
const char *topic2 = "TSM_Aux";
const char *topic3 = "Flag";

// Variables to store values from topics
uint16_t TSM_motor_value = 0;
uint16_t TSM_Aux_value = 0;

#endif // DEFINES_H
