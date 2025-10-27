#include "Arduino.h"
#include "Battery.h"
#include "Zigbee.h"
#include "esp_delta_ota.h"
#include "esp_zigbee_ota.h"

/* Zigbee temperature + humidity sensor configuration */
#define CONTACT_SENSOR_ENDPOINT_NUMBER 10

#define BUTTON_PIN_BITMASK (1ULL << GPIO_NUM_0) // GPIO 0 bitmask for ext1
#define uS_TO_S_FACTOR                                                         \
  1000000ULL /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 604800 /* Sleep for max 7 days */
#define REPORT_TIMEOUT 1000  /* Timeout for response from coordinator in ms */

/* Zigbee OTA configuration */
#define OTA_UPGRADE_RUNNING_FILE_VERSION                                       \
  0x1 // Increment this value when the running image is updated
#define OTA_UPGRADE_DOWNLOADED_FILE_VERSION                                    \
  0x2 // Increment this value when the downloaded image is updated
#define OTA_UPGRADE_HW_VERSION                                                 \
  0x1 // The hardware version, this can be used to differentiate between
      // different hardware versions

uint8_t button = BOOT_PIN;

ZigbeeBinary zbContact = ZigbeeBinary(CONTACT_SENSOR_ENDPOINT_NUMBER);

uint8_t dataToSend = 2; // Binary and Battery values are reported in same
                        // endpoint, so 2 values are reported
bool resend = false;

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

/************************ Contact sensor *****************************/
static void meausureReportSleep(void *arg) {

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

  Serial.println("bis hier geht");
  delay(10000);

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
}

/***************** Main application entry point ****************/

extern "C" void app_main(void) {
  // Initialize Arduino runtime
  initArduino();
  // Setup serial communication
  Serial.begin(115200);

  delay(8000);

  // Init button switch
  pinMode(button, INPUT_PULLUP);
  // Configure the wake up source and set to wake up every 7 days
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

  // set Zigbee device name and model
  zbContact.setManufacturerAndModel("Miro Sieber", "OmniSenseContact");

  zbContact.addBinaryInput();

  zbContact.addOTAClient(OTA_UPGRADE_RUNNING_FILE_VERSION,
                         OTA_UPGRADE_DOWNLOADED_FILE_VERSION,
                         OTA_UPGRADE_HW_VERSION);

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
  zbContact.requestOTAUpdate();

  // debuging halt
  delay(10000);

  // Start Temperature sensor reading task
  // xTaskCreate(meausureReportSleep, "temp_sensor_update", 2048, NULL, 10,
  // NULL);

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
