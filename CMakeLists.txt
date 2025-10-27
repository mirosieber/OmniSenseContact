# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# Enable USB mode and CDC on boot
add_definitions(
  -DARDUINO_USB_MODE
  -DARDUINO_USB_CDC_ON_BOOT
  -DCONFIG_ZB_DELTA_OTA=1
)


include_directories(${CMAKE_SOURCE_DIR}/managed_components/espressif__esp_delta_ota/include)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(OmniSensContact)