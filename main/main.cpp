#include "Arduino.h"
#include "Battery.h"
#include "Zigbee.h"
#include "driver/rtc_io.h"
#include "esp_app_format.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "OmniSenseContact";

#define BATT_VOLT_PIN GPIO_NUM_1
#define CONTACT_PIN GPIO_NUM_0
#define CONTACT_SENSOR_ENDPOINT_NUMBER 10

#define CONTACT1_PIN_BITMASK (1ULL << CONTACT_PIN) // GPIO 0 bitmask for ext1

/* Conversion factor for micro seconds to seconds */
#define uS_TO_S_FACTOR 1000000ULL
#define TIME_TO_SLEEP 86400 /* Sleep for max 1 day */
#define REPORT_TIMEOUT 1000 /* Timeout for response from coordinator in ms */
#define MAX_RETRIES 3       /* Max retries for sending data */

ZigbeeBinary zbContact = ZigbeeBinary(CONTACT_SENSOR_ENDPOINT_NUMBER);

uint8_t dataToSend = 2; // Binary and Battery values are reported in same
                        // endpoint, so 2 values are reported
bool resend = false;    // flag to indicate data resend is needed

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
// Read contact sensor state
bool readContact() {
  pinMode(CONTACT_PIN, INPUT_PULLDOWN); // GPIO 0 with pulldown
  bool contact = digitalRead(CONTACT_PIN);
  // pinMode(CONTACT_PIN, INPUT); // reset to normal input to save power
  return contact;
}

bool initialBoot() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  return (wakeup_reason == ESP_SLEEP_WAKEUP_UNDEFINED);
}

/***************** Main application entry point ****************/

extern "C" void app_main(void) {
  // Initialize Arduino runtime
  initArduino();

  // Configure log levels for custom tags
  esp_log_level_set("OmniSenseContact", ESP_LOG_INFO);
  esp_log_level_set("Battery", ESP_LOG_INFO);

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
  ESP_LOGI(TAG, "Connecting to network");
  while (!Zigbee.connected()) {
    ESP_LOGI(TAG, ".");
    delay(100);
  }
  ESP_LOGI(TAG, "Successfully connected to Zigbee network");
  if (initialBoot()) {
    ESP_LOGI(TAG,
             "Initial Boot detected wait 20 seconds until interview is done");
    delay(20000);
  } else {
    ESP_LOGI(TAG, "Wake up from Deep Sleep detected");
  }

  // Read binary sensor value
  bool contact = readContact();
  ESP_LOGI(TAG, "Contact ist %s", contact ? "HIGH" : "LOW");

  // Mesure Battery Voltage
  float VBatt = get_Vbatt(BATT_VOLT_PIN, 16);
  uint8_t SOC = estimateSoC(VBatt);

  // Update values in the End Point
  zbContact.setBinaryInput(contact);
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
    ESP_LOGI(TAG, ".");
    delay(50); // 50ms delay to avoid busy-waiting
  }

  if (SOC < 10) {
    ESP_LOGW(TAG, "Battery SOC is below 10%% Sleep forever to save battery");
  } else {
    // re read contact state to set correct wake up level
    bool contact = readContact();
    // Determan Wake up level so next wake up is wen contact is closed/open
    esp_sleep_ext1_wakeup_mode_t level_mode;
    if (contact) {
      ESP_LOGI(TAG, "Contact ist HIGH");
      level_mode = ESP_EXT1_WAKEUP_ANY_LOW;
    } else {
      ESP_LOGI(TAG, "Contact ist LOW");
      level_mode = ESP_EXT1_WAKEUP_ANY_HIGH;
    }
    // IO wake up setzen
    esp_sleep_enable_ext1_wakeup(CONTACT1_PIN_BITMASK, level_mode);
    // Configure the wake up source and set to wake up every x days to stay
    // online
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  }
  // Put device to deep sleep after data was sent successfully or timeout
  ESP_LOGI(TAG, "Going to sleep now");
  rtc_gpio_hold_en(CONTACT_PIN); // hold GPIO state during deep sleep
  esp_deep_sleep_start();

  for (;;) {
    // should never be reached
    ESP_LOGE(TAG, "Error: Reached unreachable code, restarting...");
    delay(1000);
    ESP.restart();
  }
}
