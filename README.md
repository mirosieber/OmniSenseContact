# OmniSenseContact

A battery-powered Zigbee window/door contact sensor based on the ESP32-C6 microcontroller. This project provides a low-power, reliable contact sensor solution for home automation systems with intelligent power management and battery monitoring.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Pin Configuration](#pin-configuration)
- [Software Requirements](#software-requirements)
- [Installation](#installation)
- [Configuration](#configuration)
- [Usage](#usage)
- [Battery Management](#battery-management)
- [Power Management](#power-management)
- [Troubleshooting](#troubleshooting)
- [Project Structure](#project-structure)
- [Development](#development)
- [License](#license)

## Overview

OmniSenseContact is a Zigbee-based window/door contact sensor designed for smart home automation. Built on the ESP32-C6 platform, it operates as a Zigbee End Device and communicates with your Zigbee coordinator (such as Home Assistant, Zigbee2MQTT, or other Zigbee hubs).

The device features:
- Ultra-low power consumption using ESP32-C6 deep sleep modes
- Battery voltage monitoring with State of Charge (SoC) estimation
- Automatic battery calibration during charging
- Intelligent wake-up on contact state changes
- Daily automatic reporting to maintain network connectivity
- USB charging support with real-time monitoring

## Features

### Core Functionality
- **Zigbee Communication**: Native Zigbee 3.0 support via ESP32-C6
- **Binary Contact Sensor**: Reports open/closed state changes instantly
- **Battery Monitoring**: Real-time battery voltage and SoC reporting
- **Low Power Design**: Deep sleep mode with wake-up on state change or timer
- **Automatic Reconnection**: Network failure handling with automatic retry
- **Security Application Type**: Configured as intrusion detection sensor

### Power Management
- **Deep Sleep Mode**: Ultra-low power consumption between events
- **Wake-up Sources**: 
  - Contact state change (GPIO interrupt)
  - Daily timer (configurable, default 24 hours)
  - Charger connection detection
- **Battery Protection**: Automatic shutdown at low battery (<10% SoC)
- **Smart Reporting**: Only reports changes to minimize power usage

### Battery System
- **1S Li-Ion/Li-Po Support**: Designed for single-cell batteries (3.0V - 4.2V)
- **Voltage Monitoring**: Analog voltage measurement with calibration
- **SoC Estimation**: Accurate State of Charge calculation using voltage curves
- **Filtered SoC**: Exponential filtering to prevent erratic readings
- **Calibration**: User-interactive calibration during charging

## Hardware Requirements

### Microcontroller
- **ESP32-C6**: RISC-V based MCU with native Zigbee support
  - 160 MHz single-core processor
  - 512 KB SRAM
  - 4 MB Flash (for OTA support)
  - Built-in Zigbee 3.0 IEEE 802.15.4 radio

### Components
- **Battery**: 1S Li-Ion or Li-Po battery (3.7V nominal)
- **Contact Sensor**: Magnetic reed switch or physical contact switch
- **Voltage Divider**: For battery monitoring
  - R1: 510Ω
  - R2: 1000Ω
- **USB Charging Circuit**: For battery charging and calibration
- **LED**: Status indicator (GPIO 15)

### Recommended Battery Capacity
- Minimum: 500 mAh
- Recommended: 1000-2000 mAh for extended operation
- Expected battery life: Several months to over a year (depending on usage)

## Pin Configuration

```cpp
// Pin Definitions
#define CHARGER_CONNECTED_PIN  GPIO_NUM_0   // Charger detection input
#define BATT_VOLT_PIN          GPIO_NUM_1   // Battery voltage ADC input
#define CONTACT1_PIN           GPIO_NUM_2   // Contact sensor input (with pull-up)
#define LED_PIN                15           // Status LED (onboard LED)
// UART pins (ESP32-C6)
#define TX_PIN                 GPIO_NUM_16  // UART TX
#define RX_PIN                 GPIO_NUM_17  // UART RX
```

### Pin Usage Details

| Pin | Function | Direction | Description |
|-----|----------|-----------|-------------|
| GPIO 0 | Charger Detection | Input | HIGH when USB charger connected |
| GPIO 1 | Battery Voltage | Analog Input | Voltage divider output |
| GPIO 2 | Contact Sensor | Input (Pull-up) | LOW = closed, HIGH = open |
| GPIO 15 | Status LED | Output | Activity indicator |
| GPIO 16 | UART TX | Output | Serial communication (debug only) |
| GPIO 17 | UART RX | Input | Serial communication (debug only) |

## Software Requirements

### Development Environment
- **ESP-IDF**: Version 4.1.0 or higher (tested with v5.x)
- **Arduino ESP32**: Version 3.3.2 (included as component)
- **CMake**: Version 3.5 or higher
- **Python**: 3.6 or higher (for ESP-IDF tools)

### Required Components
- **ESP Zigbee SDK**: Included as git submodule
- **ESP Delta OTA**: Version 1.1.2+ (managed component)
- **Arduino Framework**: Integrated for ease of development

### Zigbee Coordinator
- Home Assistant with ZHA or Zigbee2MQTT
- Any Zigbee 3.0 compatible coordinator
- Supported binding: IEEE 802.15.4

## Installation

### 1. Clone the Repository

```bash
git clone https://github.com/mirosieber/OmniSenseContact.git
cd OmniSenseContact
git submodule update --init --recursive
```

### 2. Install ESP-IDF

Follow the official ESP-IDF installation guide:
https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/get-started/

```bash
# Example for Linux/macOS
mkdir -p ~/esp
cd ~/esp
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32c6
. ./export.sh
```

### 3. Configure the Project

```bash
cd OmniSenseContact
idf.py set-target esp32c6
idf.py menuconfig
```

**Important Configuration:**
- Navigate to `Component config → FreeRTOS → Kernel`
- Set `CONFIG_FREERTOS_HZ` to **1000** (default is 100)
- This is **required** to avoid compilation errors

### 4. Build the Project

```bash
idf.py build
```

### 5. Flash the Device

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

Replace `/dev/ttyUSB0` with your serial port (e.g., `COM3` on Windows, `/dev/cu.usbserial-*` on macOS).

## Configuration

### Zigbee Configuration

The device is configured with the following Zigbee parameters:

```cpp
Manufacturer: "Miro Sieber"
Model: "OmniSenseContact"
Device Type: Binary Input (Contact Sensor)
Endpoint: 10
Power Source: Battery
Application Type: Security/Intrusion Detection
```

### Network Settings

- **Connection Timeout**: 10 seconds (reduced from default 30s to save battery)
- **Report Timeout**: 1000ms per attribute
- **Max Retries**: 3 attempts before giving up
- **Deep Sleep Duration**: 86400 seconds (24 hours)

### Debug Mode

To enable debug logging, uncomment the `#define debug` line in `main.cpp`:

```cpp
#define debug
```

**Debug Mode Effects:**
- Enables serial console logging
- Keeps UART active during operation
- LED remains on during operation
- Higher power consumption

**Production Mode** (debug undefined):
- Serial console disabled to save power
- UART drivers deleted
- GPIO pins reset
- Minimal power consumption

## Usage

### Initial Setup

1. **Flash the firmware** to your ESP32-C6 device
2. **Power on** the device (battery or USB)
3. The device will:
   - Boot up
   - Initialize Zigbee
   - Start searching for a network
   - Blink or show status on LED (if debug mode enabled)

### Pairing with Zigbee Coordinator

1. Put your Zigbee coordinator in **pairing mode**
   - Home Assistant: Configuration → Devices & Services → Add Device
   - Zigbee2MQTT: Enable permit join in the UI
2. Power on or reset the OmniSenseContact device
3. Wait up to 10 seconds for the device to connect
4. The device will appear as:
   - Manufacturer: "Miro Sieber"
   - Model: "OmniSenseContact"
5. **First boot**: The device waits 60 seconds for the interview process to complete
6. Subsequent boots: Immediate operation after connection

### Operation

Once paired:
- The device monitors the contact sensor continuously
- When the contact state changes (open/closed):
  - Device wakes from deep sleep
  - Reports new state to coordinator
  - Reports battery level
  - Returns to deep sleep
- Daily automatic wake-up to maintain network presence
- Instant response to state changes (<1 second)

### Factory Reset / Re-pairing

To force the device to re-pair with a new network:

```bash
# Erase all flash memory (including Zigbee network credentials)
idf.py erase-flash

# Flash the firmware again
idf.py flash
```

This ensures the Zigbee stack reconnects and reconfigures from scratch.

## Battery Management

### Battery Voltage Monitoring

The device uses a voltage divider to measure battery voltage:

```
Battery (+) ──[510Ω]──┬──[1000Ω]── GND
                      │
                   GPIO 1 (ADC)
```

**Voltage Range:**
- Maximum: 4.2V (fully charged Li-Ion)
- Nominal: 3.7V
- Minimum: 3.0V (protection threshold)
- Critical: Below 3.0V triggers permanent sleep

### State of Charge (SoC) Estimation

The firmware uses a 21-point voltage-to-SoC lookup table with linear interpolation:

| Voltage | SoC |
|---------|-----|
| 4.20V   | 100% |
| 4.08V   | 85% |
| 3.95V   | 70% |
| 3.83V   | 55% |
| 3.80V   | 50% |
| 3.75V   | 30% |
| 3.69V   | 10% |
| 3.27V   | 0% |

**Filtering:** Exponential moving average (80% old + 20% new) for stable readings.

### Battery Calibration

For accurate voltage readings, calibrate the ADC:

1. **Connect USB charger** to the device
2. Device enters **charging loop** automatically
3. **Open serial monitor** (115200 baud)
4. The device displays:
   ```
   Vbatt: X.XXX V, SoC: XX%
   To calibrate battery Voltage please type in the accurate voltage with 3 decimal accuracy:
   ```
5. **Measure actual battery voltage** with a multimeter
6. **Enter the measured voltage** (e.g., `3.847`) in the serial console
7. Press Enter
8. Calibration factor is calculated and saved to NVS (Non-Volatile Storage)
9. **Skip charging loop**: Type `s` and press Enter to exit early

**Calibration is persistent** across reboots and stored in flash memory.

### Low Battery Protection

When SoC drops below 10%:
- Device reports low battery status
- Logs warning: "Battery SOC is below 10% Sleep forever to save battery"
- Enters permanent deep sleep (no wake-up timer)
- Only wakes when charger is connected

## Power Management

### Deep Sleep Strategy

The device uses ESP32-C6 deep sleep mode to achieve ultra-low power consumption:

**Active Time:** ~2-5 seconds per event
- Zigbee connection: <1 second
- Data reporting: <1 second
- State verification: <1 second

**Sleep Time:** 
- Between contact changes: Until next state change (potentially days/weeks)
- Maximum sleep: 24 hours (then automatic wake for network maintenance)

**Wake-up Conditions:**
1. **Contact state change** (GPIO interrupt)
   - Window closed → wakes on CONTACT1_PIN going LOW
   - Window open → wakes on CONTACT1_PIN going HIGH or charger connected
2. **Daily timer** (86400 seconds)
3. **Charger connected** (GPIO 0 HIGH)

### Power Optimization

```cpp
// CPU Configuration
Max Frequency: 80 MHz (minimum for Zigbee operation)
Min Frequency: 80 MHz
Light Sleep: Disabled (incompatible with Zigbee)
```

**Power Consumption Estimates:**
- Deep Sleep: ~10-20 µA
- Active (Zigbee TX): ~50-100 mA
- Charging Detection: ~100 µA
- Average: Depends on event frequency

**Expected Battery Life:**
- Low activity (few events/day): 6-12 months
- Medium activity (10-20 events/day): 3-6 months
- High activity (50+ events/day): 1-3 months
- *Based on 1000 mAh battery*

### GPIO Hold During Sleep

The contact input pin (GPIO 2) maintains its pull-up state during deep sleep:

```cpp
rtc_gpio_hold_en((gpio_num_t)CONTACT1_PIN);
```

This ensures stable readings without additional power drain.

## Troubleshooting

### Common Issues

#### 1. Compilation Error: FreeRTOS Hz

**Error:** Compilation fails with timing-related errors

**Solution:**
```bash
idf.py menuconfig
# Navigate to: Component config → FreeRTOS → Kernel
# Set CONFIG_FREERTOS_HZ to 1000
```

#### 2. Device Won't Connect to Zigbee Network

**Possible Causes:**
- Coordinator not in pairing mode
- Too far from coordinator (weak signal)
- Network full (maximum devices reached)
- Old network credentials stored

**Solutions:**
1. Ensure coordinator is in pairing mode
2. Move device closer to coordinator
3. Check coordinator logs for errors
4. Erase flash and reflash:
   ```bash
   idf.py erase-flash
   idf.py flash
   ```

#### 3. Battery Percentage Incorrect

**Problem:** Reported battery percentage doesn't match actual state

**Solution:**
1. Connect charger to enter charging loop
2. Measure actual battery voltage with multimeter
3. Calibrate using serial console
4. Calibration factor is stored permanently

#### 4. Device Sleeps Forever / Won't Wake

**Possible Causes:**
- Battery below 10% (protection mode)
- Contact pin wiring incorrect
- Deep sleep wake-up not configured

**Solutions:**
1. Charge battery fully
2. Check contact sensor wiring (GPIO 2 + GND)
3. Verify pull-up resistor on contact pin
4. Reset device or reflash firmware

#### 5. High Power Consumption

**Problem:** Battery drains faster than expected

**Possible Causes:**
- Debug mode enabled
- Frequent contact state changes
- Poor Zigbee network quality (retries)
- UART not disabled

**Solutions:**
1. Ensure `#define debug` is commented out
2. Check contact sensor stability (debouncing)
3. Improve Zigbee network range/quality
4. Verify UART is deleted in production build

#### 6. Device Reboots During Operation

**Problem:** Unexpected restarts or watchdog resets

**Solutions:**
1. Check power supply stability
2. Verify battery voltage is adequate (>3.3V)
3. Enable debug mode to see logs
4. Check for stack overflows in logs

### Debug Information

When connected to serial console (debug mode):

```
Git Repository: [shows git remote]
Git Version: [shows git commit]
Git Branch: [shows current branch]
```

**Useful logs to check:**
- "Successfully connected to Zigbee network"
- "Contact ist HIGH/LOW"
- "Reported Contact state: X, Battery SOC: X%"
- "Going to sleep now"

## Project Structure

```
OmniSenseContact/
├── .devcontainer/           # VSCode development container config
├── .vscode/                 # VSCode settings
├── components/              # External components
│   └── arduino-esp32-3.3.2/ # Arduino framework
├── main/                    # Main application source
│   ├── main.cpp            # Main application logic
│   ├── Battery.cpp         # Battery monitoring implementation
│   ├── Battery.h           # Battery module interface
│   ├── CMakeLists.txt      # Component build configuration
│   └── idf_component.yml   # Component dependencies
├── CMakeLists.txt          # Root build configuration
├── partitions.csv          # Flash partition table
├── sdkconfig               # ESP-IDF configuration
├── dependencies.lock       # Component dependency lock
└── README.md               # This file
```

### Key Files

- **main.cpp**: Application entry point and main logic
  - Zigbee initialization and configuration
  - Contact sensor reading
  - Battery monitoring
  - Deep sleep management
  - Charging loop

- **Battery.cpp/h**: Battery management module
  - Voltage measurement with calibration
  - SoC estimation with lookup table
  - NVS storage for calibration
  - Filtering algorithms

- **sdkconfig**: ESP-IDF configuration
  - **Critical**: FreeRTOS Hz must be 1000
  - Zigbee stack configuration
  - Power management settings

- **partitions.csv**: Flash memory layout
  - NVS: 24KB for settings
  - OTA: Dual partition for updates
  - Zigbee Storage: 16KB for network data
  - SPIFFS: 424KB for file system

## Development

### Building for Production

1. **Disable debug mode:**
   ```cpp
   // In main.cpp, ensure this line is commented:
   // #define debug
   ```

2. **Optimize build:**
   ```bash
   idf.py menuconfig
   # Compiler options → Optimization Level → Optimize for size (-Os)
   ```

3. **Build and flash:**
   ```bash
   idf.py build flash
   ```

### OTA Updates

The project includes ESP Delta OTA support for future updates:

```
Partition Layout:
- app0: 1.75 MB (primary application)
- app1: 1.75 MB (OTA update slot)
```

**Note:** OTA functionality is included but not yet implemented in the current version.

### Adding Features

When adding new features:

1. **Consider power consumption** - minimize active time
2. **Test with battery** - verify deep sleep still works
3. **Update Zigbee attributes** - register new endpoints if needed
4. **Maintain wake-up logic** - ensure device can still sleep
5. **Document changes** - update README accordingly

### Git Workflow

```bash
# Create feature branch
git checkout -b feature/your-feature

# Make changes
git add .
git commit -m "Description of changes"

# Push to remote
git push origin feature/your-feature
```

### Testing Checklist

- [ ] Zigbee pairing works
- [ ] Contact state reported correctly
- [ ] Battery percentage accurate
- [ ] Deep sleep activates
- [ ] Wake on state change works
- [ ] Daily wake-up timer works
- [ ] Charging loop enters correctly
- [ ] Low battery protection works
- [ ] Power consumption acceptable
- [ ] OTA partition accessible

## Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch
3. Test your changes thoroughly
4. Submit a pull request with detailed description

## License

[Add your license information here]

## Author

**Miro Sieber**

## Acknowledgments

- Espressif Systems for ESP-IDF and ESP32-C6
- Arduino ESP32 community
- Zigbee Alliance for the Zigbee 3.0 specification

## Support

For issues, questions, or contributions:
- Open an issue on GitHub
- Check existing issues for solutions
- Refer to ESP-IDF documentation: https://docs.espressif.com/projects/esp-idf/

---

**Version Information:**
- Hardware: ESP32-C6
- Firmware: See git tags for releases
- Zigbee: 3.0 (IEEE 802.15.4)
- Build System: ESP-IDF + CMake
