#include "Arduino.h"
#include "Battery.h"
#include "Zigbee.h"
#include "esp_app_format.h"
#include "esp_delta_ota.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_zigbee_ota.h"

// FreeRTOS event group used to signal OTA in-progress state
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

/* Zigbee temperature + humidity sensor configuration */
#define CONTACT_SENSOR_ENDPOINT_NUMBER 10

#define BUTTON_PIN_BITMASK (1ULL << GPIO_NUM_0) // GPIO 0 bitmask for ext1
/* Conversion factor for micro seconds to seconds */
#define uS_TO_S_FACTOR 1000000ULL
#define TIME_TO_SLEEP 86400 /* Sleep for max 1 day */
#define REPORT_TIMEOUT 1000 /* Timeout for response from coordinator in ms */

/* Zigbee OTA configuration */
// Increment this value when the running image is updated
#define OTA_UPGRADE_RUNNING_FILE_VERSION 0x4
// Increment this value when the downloaded image is updated
#define OTA_UPGRADE_DOWNLOADED_FILE_VERSION 0x5
// The hardware version, this can be used to differentiate between
#define OTA_UPGRADE_HW_VERSION 0x1
// different hardware versions

uint8_t button = BOOT_PIN;

ZigbeeBinary zbContact = ZigbeeBinary(CONTACT_SENSOR_ENDPOINT_NUMBER);

uint8_t dataToSend = 2; // Binary and Battery values are reported in same
                        // endpoint, so 2 values are reported
bool resend = false;

// Event group to track OTA state (created in app_main)
static EventGroupHandle_t ota_event_group = NULL;
static const EventBits_t OTA_IN_PROGRESS_BIT = (1 << 0);

static const char *TAG = "appinfo";

void print_running_version() {
  // easiest: pointer to running app's description (IDF provides this shortcut)
  const esp_app_desc_t *app_desc = esp_ota_get_app_description();
  if (app_desc != NULL) {
    ESP_LOGI(TAG, "Running version: %s (project: %s, idf: %s)",
             app_desc->version, app_desc->project_name, app_desc->idf_ver);
    return;
  }
}

/************************ Callbacks *****************************/
void onGlobalResponse(zb_cmd_type_t command, esp_zb_zcl_status_t status,
                      uint8_t endpoint, uint16_t cluster) {
  Serial.printf("Global response command: %d, status: %s, endpoint: %d, "
                "cluster: 0x%04x\r\n",
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

static void otaActiveCallback(bool state) {
  // Keep callback short: just signal OTA start/stop via an EventGroup bit.
  // This avoids blocking the Zigbee stack/task which drives the OTA transfer.
  if (state) {
    Serial.println("OTA Update started");
    if (ota_event_group) {
      xEventGroupSetBits(ota_event_group, OTA_IN_PROGRESS_BIT);
    }
  } else {
    Serial.println("OTA Update finished");
    if (ota_event_group) {
      xEventGroupClearBits(ota_event_group, OTA_IN_PROGRESS_BIT);
    }
  }
}

/***************** Main application entry point ****************/

extern "C" void app_main(void) {
  // Initialize Arduino runtime
  initArduino();
  // Setup serial communication
  Serial.begin(115200);

  delay(8000);
  print_running_version();

  // Init button switch
  pinMode(button, INPUT_PULLUP);
  // Configure the wake up source and set to wake up every 7 days
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

  // set Zigbee device name and model
  zbContact.setManufacturerAndModel("Miro Sieber", "OmniSenseContact");

  zbContact.addBinaryInput();

  zbContact.addOTAClient(OTA_UPGRADE_RUNNING_FILE_VERSION,
                         OTA_UPGRADE_DOWNLOADED_FILE_VERSION,
                         OTA_UPGRADE_HW_VERSION, 0x1001, 0x1012);
  zbContact.onOTAStateChange(otaActiveCallback);

  // Set power source to battery
  zbContact.setPowerSource(ZB_POWER_SOURCE_BATTERY);

  zbContact.setBinaryInputApplication(
      BINARY_INPUT_APPLICATION_TYPE_SECURITY_INTRUSION_DETECTION);

  // Global callback for all endpoints with more params to determine the
  // endpoint and cluster in the callback function.
  Zigbee.onGlobalDefaultResponse(onGlobalResponse);

  // Add endpoint to Zigbee Core
  Zigbee.addEndpoint(&zbContact);

  // Create a custom Zigbee configuration for End Device with keep alive 10s to
  // avoid interference with reporting data
  esp_zb_cfg_t zigbeeConfig = ZIGBEE_DEFAULT_ED_CONFIG();
  zigbeeConfig.nwk_cfg.zed_cfg.keep_alive = 10000;

  // For battery powered devices, it can be better to set timeout for Zigbee
  // Begin to lower value to save battery If the timeout has been reached, the
  // network channel mask will be reset and the device will try to connect again
  // after reset (scanning all channels)
  Zigbee.setTimeout(
      10000); // Set timeout for Zigbee Begin to 10s (default is 30s)

  // When all EPs are registered, start Zigbee in End Device mode
  if (!Zigbee.begin(&zigbeeConfig, false)) {
    Serial.println("Zigbee failed to start!");
    Serial.println("Rebooting...");
    ESP.restart(); // If Zigbee failed to start, reboot the device and try again
  }
  Serial.println("Connecting to network");
  while (!Zigbee.connected()) {
    Serial.print(".");
    delay(100);
  }
  Serial.println();
  Serial.println("Successfully connected to Zigbee network");
  // Create OTA event group and register OTA state callback so we can
  // pause main flow while an OTA is active (avoid deep-sleep during OTA).
  static EventGroupHandle_t local_ota_event_group = NULL;
  if (ota_event_group == NULL) {
    ota_event_group = xEventGroupCreate();
    local_ota_event_group = ota_event_group;
    if (ota_event_group == NULL) {
      Serial.println("Failed to create OTA event group");
    }
  }

  // Register the non-blocking OTA state callback
  zbContact.onOTAStateChange(otaActiveCallback);

  // Request OTA update from server/coordinator
  zbContact.requestOTAUpdate();

  // If the OTA bit is set, wait until it clears. If not set, wait a short
  // window to see whether an OTA starts (60s). This avoids going to deep
  // sleep while OTA is ongoing.
  if (ota_event_group != NULL) {
    EventBits_t bits = xEventGroupGetBits(ota_event_group);
    if (bits & OTA_IN_PROGRESS_BIT) {
      Serial.println("OTA in progress: waiting for completion...");
      while (xEventGroupGetBits(ota_event_group) & OTA_IN_PROGRESS_BIT) {
        vTaskDelay(pdMS_TO_TICKS(500));
      }
      Serial.println("OTA completed");
    } else {
      // Wait up to 60s for OTA to start. If it starts, wait until it finishes.
      EventBits_t started =
          xEventGroupWaitBits(ota_event_group, OTA_IN_PROGRESS_BIT, pdFALSE,
                              pdFALSE, pdMS_TO_TICKS(30000));
      if (started & OTA_IN_PROGRESS_BIT) {
        Serial.println(
            "OTA started within wait window: waiting for completion...");
        while (xEventGroupGetBits(ota_event_group) & OTA_IN_PROGRESS_BIT) {
          vTaskDelay(pdMS_TO_TICKS(500));
        }
        Serial.println("OTA completed");
      }
    }
  }

  // Start Temperature sensor reading and reporting

  // Read binary sensor value
  pinMode(0, INPUT);
  bool contact = digitalRead(0);

  // Determan Wake up level so next wake up is wen contact is closed/open
  esp_sleep_ext1_wakeup_mode_t level_mode;
  Serial.print("Contact ist ");
  if (contact) {
    Serial.println("HIGH");
    level_mode = ESP_EXT1_WAKEUP_ANY_LOW;
  } else {
    Serial.println("LOW");
    level_mode = ESP_EXT1_WAKEUP_ANY_HIGH;
  }

  // Mesure Battery Voltage
  float VBatt = get_Vbatt();

  uint8_t SOC = estimateSoC(VBatt);

  // Update values in the End Point
  zbContact.setBinaryInput(contact);
  zbContact.setBatteryPercentage(SOC);

  // Report values
  zbContact.reportBatteryPercentage();
  zbContact.reportBinaryInput();
  Serial.printf("Reported Contact state: %s, Battery SOC: %d%%\r\n",
                contact ? "HIGH" : "LOW", SOC);

  unsigned long startTime = millis();
  const unsigned long timeout = REPORT_TIMEOUT;

  Serial.printf("Waiting for data report to be confirmed \r\n");
  // Wait until data was successfully sent
  int tries = 0;
  const int maxTries = 3;
  while (dataToSend != 0 && tries < maxTries) {
    if (resend) {
      Serial.println("Resending data on failure!");
      resend = false;
      dataToSend = 2;
      zbContact.reportBatteryPercentage();
      zbContact.reportBinaryInput(); // report again
    }
    if (millis() - startTime >= timeout) {
      Serial.println("\nReport timeout! Report Again");
      dataToSend = 2;
      zbContact.reportBatteryPercentage();
      zbContact.reportBinaryInput(); // report again
      startTime = millis();
      tries++;
    }
    Serial.printf(".");
    delay(50); // 50ms delay to avoid busy-waiting
  }

  // IO wake up setzen
  esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK, level_mode);

  // Put device to deep sleep after data was sent successfully or timeout
  Serial.println("Going to sleep now");
  esp_deep_sleep_start();

  for (;;) {
    delay(1000);
  }
}
