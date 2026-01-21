#include "Arduino.h"
#include "Battery.h"
#include "Zigbee.h"
#include "driver/rtc_io.h"
#include "esp_app_format.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"

// #define debug
//  zigbee debuging mus in menuconfig aktiviert werden

static const char *TAG = "OmniSenseContact";

#define CHARGER_CONNECTED_PIN                                                  \
  GPIO_NUM_0                     // Pin to detect if charger is connected
#define BATT_VOLT_PIN GPIO_NUM_1 // Pin to read battery voltage
#define CONTACT1_PIN GPIO_NUM_2  // Pin for contact sensor
#define CONTACT_SENSOR_ENDPOINT_NUMBER 10

#define CHARGER_CONNECTED_PIN_BITMASK (1ULL << CHARGER_CONNECTED_PIN)
#define CONTACT1_PIN_BITMASK (1ULL << CONTACT1_PIN)
/* Conversion factor for micro seconds to seconds */
#define uS_TO_S_FACTOR 1000000ULL
#define TIME_TO_SLEEP 86400 /* Sleep for max 1 day */
#define REPORT_TIMEOUT 1000 /* Timeout for response from coordinator in ms */
#define MAX_RETRIES 3       /* Max retries for sending data */

ZigbeeBinary zbContact = ZigbeeBinary(CONTACT_SENSOR_ENDPOINT_NUMBER);

uint8_t dataToSend = 2; // Binary and Battery values are reported in same
                        // endpoint, so 2 values are reported
bool resend = false;    // flag to indicate data resend is needed

// tiefer als 80MHz und light sleep geht nicht, da Zigbee sonst nicht mehr läuft
void cpu_power_config(void) {
  esp_pm_config_t pm_config = {
      .max_freq_mhz = 80, .min_freq_mhz = 80, .light_sleep_enable = false};
  esp_pm_configure(&pm_config);
}

/************************ Callbacks *****************************/
// Global response callback to handle responses for all endpoints
void onGlobalResponse(zb_cmd_type_t command, esp_zb_zcl_status_t status,
                      uint8_t endpoint, uint16_t cluster) {
  ESP_LOGI(TAG,
           "Global response command: %d, status: %s, endpoint: %d, "
           "cluster: 0x%04x",
           command, esp_zb_zcl_status_to_name(status), endpoint, cluster);
  if ((command == ZB_CMD_REPORT_ATTRIBUTE) &&
      (endpoint == CONTACT_SENSOR_ENDPOINT_NUMBER)) {
    switch (status) {
    case ESP_ZB_ZCL_STATUS_SUCCESS:
      dataToSend--;
      break;
    case ESP_ZB_ZCL_STATUS_FAIL:
      resend = true;
      break;
    default:
      break; // add more statuses like ESP_ZB_ZCL_STATUS_INVALID_VALUE,
             // ESP_ZB_ZCL_STATUS_TIMEOUT etc.
    }
  }
}

bool initialBoot() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  return (wakeup_reason == ESP_SLEEP_WAKEUP_UNDEFINED);
}

void chargingLoop() {
  pinMode(15, OUTPUT);    // Onboard LED pin
  digitalWrite(15, HIGH); // Turn the LED on
  // Initialize Serial communication at 115200 baud
  Serial.begin(115200);
  delay(1000); // Wait for Serial to be ready
  Serial.printf("Git Repository: %s\n", GIT_REMOTE_URL);
  Serial.printf("Git Version:    %s\n", GIT_VERSION);
  Serial.printf("Git Branch:     %s\n", GIT_BRANCH);
  Serial.println("Charger connected, entering charging loop");
  while (digitalRead(CHARGER_CONNECTED_PIN)) {
    float VBatt = get_Vbatt(BATT_VOLT_PIN);
    uint8_t SOC = estimateSoC_filtered(VBatt);
    Serial.println();
    Serial.printf("Vbatt:          %.3f V, SoC: %d%%\n", VBatt, SOC);
    Serial.println("To calibrate battery Voltage please type in the accurate "
                   "voltage with 3 decimal accuracy:");

    for (int i = 0; i < 100; i++) {
      // Check if user has typed something send infos every 10 seconds
      if (Serial.available() > 0) {
        String userInput = Serial.readStringUntil('\n');
        if (strcmp(userInput.c_str(), "s") == 0) {
          Serial.println("Skipping charging as per user request.");
          return;
        }
        float calibrationVoltage = userInput.toFloat();

        if (calibrationVoltage > 0 && calibrationVoltage < 5) {
          Serial.printf("Calibration voltage received: %.3f V\n",
                        calibrationVoltage);
          calibrateVoltage(calibrationVoltage, BATT_VOLT_PIN);
          Serial.println("Calibration complete.");
          break; // Exit the for loop after successful input
        } else {
          Serial.println(
              "Invalid voltage input. Please enter a value between 0 and 5V");
        }
      }
      delay(100); // check every 100ms for user input
    }
  }
  Serial.println("Charger disconnected / Battery full, exiting charging loop");
  Serial.end();
#ifndef debug
  // Disable UART0 (console)
  esp_log_level_set("*", ESP_LOG_NONE); // Disable all logs
  uart_driver_delete(UART_NUM_0);
  gpio_reset_pin(GPIO_NUM_16); // TX pin on ESP32-C6
  gpio_reset_pin(GPIO_NUM_17); // RX pin on ESP32-C6
#endif
}

/***************** Main application entry point ****************/

extern "C" void app_main(void) {
  cpu_power_config();
  initArduino();
#ifdef debug
  // Configure log levels for custom tags
  esp_log_level_set("*", ESP_LOG_INFO);
  esp_log_level_set("OmniSenseContact", ESP_LOG_INFO);
  esp_log_level_set("Battery", ESP_LOG_INFO);
#else
  // Disable UART0 (console)
  esp_log_level_set("*", ESP_LOG_NONE); // Disable all logs
  uart_driver_delete(UART_NUM_0);
  gpio_reset_pin(GPIO_NUM_16); // TX pin on ESP32-C6
  gpio_reset_pin(GPIO_NUM_17); // RX pin on ESP32-C6
#endif

  pinMode(CHARGER_CONNECTED_PIN, INPUT);
  pinMode(CONTACT1_PIN, INPUT_PULLUP);

  // Read binary sensor value
  bool contact = digitalRead(CONTACT1_PIN);
  ESP_LOGI(TAG, "Contact ist %s", contact ? "HIGH" : "LOW");

  battery_init(); // Initialize battery module (load calibration from NVS)

#ifdef debug
  pinMode(15, OUTPUT);    // Onboard LED pin
  digitalWrite(15, HIGH); // Turn the LED on
#endif

  // wenn device gets charged:
  if (digitalRead(CHARGER_CONNECTED_PIN)) {
    chargingLoop();
  }

  // set Zigbee device setup
  zbContact.setManufacturerAndModel("Miro Sieber", "OmniSenseContact");
  zbContact.addBinaryInput();
  zbContact.setPowerSource(ZB_POWER_SOURCE_BATTERY);
  zbContact.setBinaryInputApplication(
      BINARY_INPUT_APPLICATION_TYPE_SECURITY_INTRUSION_DETECTION);
  // Global callback for all endpoints with more params to determine the
  // endpoint and cluster in the callback function.
  Zigbee.onGlobalDefaultResponse(onGlobalResponse);
  // Add endpoint to Zigbee Core
  Zigbee.addEndpoint(&zbContact);

  // For battery powered devices, it can be better to set timeout for Zigbee
  // Begin to lower value to save battery If the timeout has been reached, the
  // network channel mask will be reset and the device will try to connect again
  // after reset (scanning all channels)
  // Set timeout for Zigbee Begin to 10s (default is 30s)
  Zigbee.setTimeout(10000);

  // When all EPs are registered, start Zigbee in End Device mode
  if (!Zigbee.begin(ZIGBEE_END_DEVICE, false)) {
    ESP_LOGE(TAG, "Zigbee failed to start!");
    ESP_LOGE(TAG, "Rebooting...");
    ESP.restart(); // If Zigbee failed to start, reboot the device and try again
  }

  while (!Zigbee.connected()) {
    ESP_LOGI(TAG, ".");
    delay(100);
  }
  ESP_LOGI(TAG, "Successfully connected to Zigbee network");
  if (initialBoot()) {
    ESP_LOGI(TAG,
             "Initial Boot detected wait 60 seconds until interview is done");
    delay(60000);
  } else {
    ESP_LOGI(TAG, "Wake up from Deep Sleep detected");
  }

  // Mesure Battery Voltage
  float VBatt = get_Vbatt(BATT_VOLT_PIN);
  uint8_t SOC = estimateSoC_filtered(VBatt);

  // Update values in the End Point
  zbContact.setBinaryInput(!contact);
  zbContact.setBatteryPercentage(SOC > 0 ? SOC : 1); // set min 1% to avoid 0%

  // Report values
  zbContact.reportBatteryPercentage();
  zbContact.reportBinaryInput();
  ESP_LOGI(TAG, "Reported Contact state: %s, Battery SOC: %d%%",
           contact ? "HIGH" : "LOW", SOC);

  unsigned long startTime = millis();
  const unsigned long timeout = REPORT_TIMEOUT;

  ESP_LOGI(TAG, "Waiting for data report to be confirmed");
  // Wait until data was successfully sent
  int tries = 0;
  while (dataToSend != 0 && tries < MAX_RETRIES) {
    if (resend) {
      ESP_LOGW(TAG, "Resending data on failure!");
      resend = false;
      dataToSend = 2;
      zbContact.reportBatteryPercentage();
      zbContact.reportBinaryInput(); // report again
    }
    if (millis() - startTime >= timeout) {
      ESP_LOGW(TAG, "Report timeout! Report Again");
      dataToSend = 2;
      zbContact.reportBatteryPercentage();
      zbContact.reportBinaryInput(); // report again
      startTime = millis();
      tries++;
    }
    // ESP_LOGI(TAG, ".");
    delay(10); // 50ms delay to avoid busy-waiting
  }

  if (SOC < 10) {
    ESP_LOGW(TAG, "Battery SOC is below 10%% Sleep forever to save battery");
  } else {
    // Determan Wake up level so next wake up is wen contact is closed/open
    esp_sleep_ext1_wakeup_mode_t level_mode;
    uint64_t CONTACT_WAKE_MASK;
    if (contact) {
      ESP_LOGI(TAG, "Contact ist HIGH");
      ESP_LOGI(TAG, "Window is closed");
      CONTACT_WAKE_MASK = (CONTACT1_PIN_BITMASK);
      level_mode = ESP_EXT1_WAKEUP_ANY_LOW;
    } else {
      ESP_LOGI(TAG, "Contact ist LOW");
      ESP_LOGI(TAG, "Window is opened");
      CONTACT_WAKE_MASK =
          (CHARGER_CONNECTED_PIN_BITMASK | CONTACT1_PIN_BITMASK);
      level_mode = ESP_EXT1_WAKEUP_ANY_HIGH;
    }
    // IO wake up setzen
    esp_sleep_enable_ext1_wakeup(CONTACT_WAKE_MASK, level_mode);
    // Configure the wake up source and set to wake up every x days to stay
    // online
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  }
  // Put device to deep sleep after data was sent successfully or timeout
  ESP_LOGI(TAG, "Going to sleep now");
  rtc_gpio_hold_en(
      (gpio_num_t)CONTACT1_PIN); // hold GPIO state during deep sleep
  esp_deep_sleep_start();

  for (;;) {
    // should never be reached
    ESP_LOGE(TAG, "Error: Reached unreachable code, restarting...");
    delay(1000);
    ESP.restart();
  }
}
