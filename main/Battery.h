#ifndef BATTERY_H
#define BATTERY_H

#include "driver/gpio.h"
#include "stdint.h"
#include <Arduino.h>

// Initialize battery module (load calibration from NVS)
void battery_init();
// battery voltage
float get_Vbatt(gpio_num_t pin);

uint8_t estimateSoC_filtered(float voltage);
// estimate State of Charge from 1S cell voltage
uint8_t estimateSoC(float voltage);
// calculate calibration factor from a known accurate voltage
void calculateVoltageCalibration(float accurateVoltage, gpio_num_t pin);
// save current calibration factor to NVS
void saveVoltageCalibration();

#endif // BATTERY_H