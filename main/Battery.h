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
// calibrate voltage reading with known accurate voltage
void calibrateVoltage(float accurateVoltage, gpio_num_t pin);

static double get_Vbatt_uncalibrated(gpio_num_t pin);

#endif // BATTERY_H