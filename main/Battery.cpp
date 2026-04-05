#include "Battery.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <Arduino.h>

static const char *TAG = "Battery";
// Voltage divider resistors
constexpr float R1 = 510.0;
constexpr float R2 = 1000.0;
constexpr double DIVIDER = (R1 + R2) / R2;
constexpr double MV_TO_V = 0.001;
// k= soll Spannung(MultiM) / ist Spannung(ESP)
static float calibrationFactor = 1.0;
static double get_Vbatt_uncalibrated(gpio_num_t pin);

// Initialize battery module - load calibration factor from NVS
void battery_init() {
  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_open("battery", NVS_READONLY, &nvs_handle);
  if (err == ESP_OK) {
    float stored_factor;
    size_t size = sizeof(float);
    err = nvs_get_blob(nvs_handle, "cal_factor", &stored_factor, &size);
    if (err == ESP_OK) {
      calibrationFactor = stored_factor;
      ESP_LOGI(TAG, "Loaded calibration factor from NVS: %.5f",
               calibrationFactor);
    } else {
      ESP_LOGI(TAG, "No stored calibration factor, using default: %.5f",
               calibrationFactor);
    }
    nvs_close(nvs_handle);
  } else {
    ESP_LOGI(TAG,
             "Could not open NVS for battery, using default calibration: %.5f",
             calibrationFactor);
  }
}

// battery voltage
float get_Vbatt(gpio_num_t pin) {
  double Vbattf = get_Vbatt_uncalibrated(pin) * calibrationFactor;
  return static_cast<float>(Vbattf);
}

static double get_Vbatt_uncalibrated(gpio_num_t pin) {
  // nur einmal messen, da sehr hochohmig, adc Kapazität braucht Zeit zum
  // Aufladen
  analogReadMilliVolts(pin); // Dummy
  delay(20);                 // Aufladen
  uint32_t mv = analogReadMilliVolts(pin);
  return mv * DIVIDER * MV_TO_V;
}

uint8_t estimateSoC_filtered(float voltage) {
  static float soc_filt = -1;

  uint8_t soc = estimateSoC(voltage);

  if (soc_filt < 0)
    soc_filt = soc;
  else
    soc_filt = 0.8f * soc_filt + 0.2f * soc;

  return (uint8_t)(soc_filt + 0.5f);
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
      return (uint8_t)(result + 0.5f);
    }
  }
  return 0; // sollte nie erreicht werden
}

void calculateVoltageCalibration(float accurateVoltage, gpio_num_t pin) {
  // Calculate calibration factor
  calibrationFactor = accurateVoltage / get_Vbatt_uncalibrated(pin);
  ESP_LOGI(TAG, "Calibration factor set to: %.5f", calibrationFactor);
}

void saveVoltageCalibration() {
  // Save to NVS
  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_open("battery", NVS_READWRITE, &nvs_handle);
  if (err == ESP_OK) {
    err = nvs_set_blob(nvs_handle, "cal_factor", &calibrationFactor,
                       sizeof(float));
    if (err == ESP_OK) {
      err = nvs_commit(nvs_handle);
      if (err == ESP_OK) {
        ESP_LOGI(TAG, "Calibration factor saved to NVS");
      } else {
        ESP_LOGE(TAG, "Failed to commit calibration factor to NVS");
      }
    } else {
      ESP_LOGE(TAG, "Failed to write calibration factor to NVS");
    }
    nvs_close(nvs_handle);
  } else {
    ESP_LOGE(TAG, "Failed to open NVS for saving calibration factor");
  }
}