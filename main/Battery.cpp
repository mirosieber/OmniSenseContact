#include "Battery.h"
#include "Ewma.h"
#include "esp_log.h"
#include <Arduino.h>

static const char *TAG = "Battery";

// Moving average filter for ADC readings
Ewma adcFilter(0.3); // smoothing factor between 0 and 1 smaller = smoother

// battery voltage
float get_Vbatt(gpio_num_t pin, uint8_t samples) {
  uint32_t Voltage = 0;
  pinMode(pin, INPUT);
  for (int i = 0; i < samples; i++) {
    // Read and accumulate ADC voltage
    Voltage += analogReadMilliVolts(pin);
    delay(1);
  }
  // Apply EWMA filter and convert to volts
  double Vbattf =
      adcFilter.filter(static_cast<double>(Voltage) / samples) * 0.00152f;
  ESP_LOGI(TAG, "Vbatt: %.3f", Vbattf); // Output voltage to 3 decimal places
  return static_cast<float>(Vbattf);
}

uint8_t estimateSoC(float voltage) {
  // Stützpunkte: Spannung (V) → Ladezustand (%)
  const float voltages[] = {4.20, 4.15, 4.11, 4.08, 4.02, 3.98, 3.95,
                            3.91, 3.87, 3.83, 3.82, 3.81, 3.80, 3.77,
                            3.76, 3.75, 3.73, 3.71, 3.69, 3.67, 3.27};

  const uint8_t socs[] = {100, 95, 90, 85, 80, 75, 70, 65, 60, 55, 50,
                          45,  40, 35, 30, 25, 20, 15, 10, 5,  0};

  const int numPoints = sizeof(voltages) / sizeof(voltages[0]);

  // Begrenzen auf gültigen Bereich
  if (voltage >= voltages[0])
    return 100;
  if (voltage <= voltages[numPoints - 1])
    return 0;

  // Lineare Interpolation zwischen den Stützpunkten
  for (int i = 0; i < numPoints - 1; i++) {
    float v1 = voltages[i];
    float v2 = voltages[i + 1];
    if (voltage <= v1 && voltage >= v2) {
      float soc1 = socs[i];
      float soc2 = socs[i + 1];
      float fraction = (voltage - v2) / (v1 - v2);
      float result = soc2 + fraction * (soc1 - soc2);

      // Rundung und Begrenzung auf 0–100 %
      if (result < 0)
        result = 0;
      if (result > 100)
        result = 100;
      return (uint8_t)result;
    }
  }
  return 0; // sollte nie erreicht werden
}