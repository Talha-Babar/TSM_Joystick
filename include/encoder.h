#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include <driver/pcnt.h>
#include <soc/pcnt_struct.h>

#define PIN_NOT_USED (-1)

volatile int16_t currentCount = 0;
volatile int16_t highInterruptCount = 0;
volatile int16_t lowInterruptCount = 0;
uint32_t intrStatus;

class Encoder {
private:
  pcnt_unit_t pcnt_unit; // PCNT unit for this encoder
  int16_t position;      // Current position count
  int16_t last_position; // Previous position count
  int pin_a;             // Pin A for encoder
  int pin_b;             // Pin B for encoder (optional)
  bool quadrature;       // Is this a quadrature encoder?
  int var1 = 0;
  int var2 = 0;
  void configureSinglePin(int pin, pcnt_unit_t pcnt_unit) {
    pcnt_config_t pcnt_config = {.pulse_gpio_num = this->pin_a,
                                 .ctrl_gpio_num = PCNT_PIN_NOT_USED,
                                 .lctrl_mode = PCNT_MODE_KEEP,
                                 .hctrl_mode = PCNT_MODE_KEEP,
                                 .pos_mode = PCNT_COUNT_INC,
                                 .neg_mode = PCNT_COUNT_DIS,
                                 .counter_h_lim = INT16_MAX,
                                 .counter_l_lim = INT16_MIN,
                                 .unit = this->pcnt_unit,
                                 .channel = PCNT_CHANNEL_0};
    pcnt_unit_config(&pcnt_config);
  }

  void configureQuadrature(int pinA, int pinB, pcnt_unit_t pcnt_unit) {
    // Configure channel 0
    pcnt_config_t pcnt_config = {.pulse_gpio_num = this->pin_a,
                                 .ctrl_gpio_num = this->pin_b,
                                 .lctrl_mode = PCNT_MODE_REVERSE,
                                 .hctrl_mode = PCNT_MODE_KEEP,
                                 .pos_mode = PCNT_COUNT_INC,
                                 .neg_mode = PCNT_COUNT_DEC,
                                 .counter_h_lim = INT16_MAX,
                                 .counter_l_lim = INT16_MIN,
                                 .unit = this->pcnt_unit,
                                 .channel = PCNT_CHANNEL_0};
    pcnt_unit_config(&pcnt_config);

    // Configure channel 1
    // pcnt_config.pulse_gpio_num = pinB;
    // pcnt_config.ctrl_gpio_num = pinA;
    // pcnt_config.channel = PCNT_CHANNEL_1;
    // pcnt_unit_config(&pcnt_config);
  }

public:
  Encoder(pcnt_unit_t unit, int pinA, int pinB = PIN_NOT_USED)
      : pcnt_unit(unit), pin_a(pinA), pin_b(pinB), position(0),
        last_position(0), quadrature(pinB != PIN_NOT_USED) {}
  void begin();
  // Get current position
  int16_t getPosition();
  // Get differential output (change in position)
  int16_t getDifferential();
  // Reset the encoder position to zero
  void reset();
  void setPins(pcnt_unit_t unit, pcnt_channel_t channel, int pulse_io,
               int ctrl_io);

  bool intruptflag = false;

  void eventEnable(uint threshholdValue);
  static void IRAM_ATTR pcntISR(void *arg);
};
void Encoder::begin() {
  if (this->quadrature) {
    configureQuadrature(this->pin_a, this->pin_b, this->pcnt_unit);
  } else {
    configureSinglePin(this->pin_a, this->pcnt_unit);
  }

  // Initialize counter to 0
  pcnt_counter_pause(pcnt_unit);
  pcnt_counter_clear(pcnt_unit);
  pcnt_counter_resume(pcnt_unit);
}
void Encoder::setPins(pcnt_unit_t unit, pcnt_channel_t channel, int pulse_io,
                      int ctrl_io) {

  esp_err_t err = pcnt_set_pin(unit, channel, pulse_io, ctrl_io);
  if (err != ESP_OK) {
    Serial.println("Error configuring PCNT pins");
  }
}
int16_t Encoder::getPosition() {
  pcnt_get_counter_value(this->pcnt_unit, &(this->position));
  Serial.println("position" + String(this->position));
  return this->position;
}
int16_t Encoder::getDifferential() {
  if ((this->last_position > INT16_MAX >> 1) ||
      (this->last_position < INT16_MIN >> 1)) {
    this->reset();
  } else {
    this->last_position = this->position;
  }
  Serial.println(String(this->getPosition() - this->last_position));
  return this->getPosition() - this->last_position;
}
void Encoder::reset() {
  pcnt_counter_clear(this->pcnt_unit);
  this->position = 0;
  this->last_position = 0;
}

void Encoder::eventEnable(uint threshholdValue) {

  var1 = threshholdValue / INT16_MAX;
  Serial.println("var1 " + String(var1));
  var2 = threshholdValue % INT16_MAX;
  Serial.println("var2 " + String(var2));
  pcnt_isr_register(pcntISR, this, ESP_INTR_FLAG_EDGE, NULL);
  pcnt_set_event_value(this->pcnt_unit, PCNT_EVT_THRES_1, var2);
  Serial.println("threshholdValue" + String(threshholdValue));

  pcnt_event_enable(this->pcnt_unit, PCNT_EVT_THRES_1);
  pcnt_event_enable(this->pcnt_unit, PCNT_EVT_H_LIM);
  pcnt_intr_enable(this->pcnt_unit);
  pcnt_counter_clear(this->pcnt_unit);
}
void IRAM_ATTR Encoder::pcntISR(void *arg) {
  Encoder *encoder = static_cast<Encoder *>(arg);
  // intrStatus = PCNT.int_st.val;
  pcnt_get_event_status(encoder->pcnt_unit, &intrStatus);

  if (intrStatus & PCNT_EVT_H_LIM || intrStatus & PCNT_EVT_THRES_1) {
    if (intrStatus & PCNT_EVT_H_LIM) {
      highInterruptCount++;
    }
    if (encoder->var1 == highInterruptCount) {
      if (intrStatus & PCNT_EVT_THRES_1) {

        Serial.println("THRES_1");

        pcnt_intr_disable(encoder->pcnt_unit);

        highInterruptCount = 0;
        intrStatus = 0;
        // encoder->intruptflag = true;
      }

      Serial.println("H_LIM");
    }
  }

  PCNT.int_clr.val = BIT(encoder->pcnt_unit);
}

#endif