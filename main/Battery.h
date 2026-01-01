#ifndef BATTERY_H
#define BATTERY_H

#include "driver/gpio.h"
#include "stdint.h"
#include <Arduino.h>

// battery voltage
float get_Vbatt(gpio_num_t pin, uint8_t samples = 16);
// estimate State of Charge from 1S cell voltage
uint8_t estimateSoC(float voltage);

#endif // BATTERY_H