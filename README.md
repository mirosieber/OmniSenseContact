# OmniSenseContact

A battery-powered Zigbee window/door contact sensor based on the ESP32-C6 Supermini with integrated 1S Li-Po charger.

## Overview

OmniSenseContact is a Zigbee-based window/door contact sensor for smart home automation. Built on the ESP32-C6 Supermini platform, it operates as a Zigbee End Device and communicates with your Zigbee coordinator (such as Home Assistant, Zigbee2MQTT, or other Zigbee hubs).

## Features

- **Zigbee Communication**: Native Zigbee 3.0 support via ESP32-C6
- **Binary Contact Sensor**: Reports open/closed state changes instantly
- **Battery Monitoring**: Real-time battery voltage and SoC reporting
- **Low Power Design**: Deep sleep mode with wake-up on state change or timer
- **Wake-up Sources**: Contact state change (GPIO interrupt), daily timer (24 hours), charger connection detection
- **Battery Protection**: Automatic shutdown at low battery (<10% SoC)

## Hardware Requirements

### Microcontroller
- **ESP32-C6 Supermini**: RISC-V based MCU with native Zigbee support and integrated 1S Li-Po charger
  - 160 MHz single-core processor
  - 512 KB SRAM
  - Built-in Zigbee 3.0 IEEE 802.15.4 radio

### Components
- **Battery**: 1S Li-Ion or Li-Po battery (3.7V nominal)
- **Contact Sensor**: **Normally Closed (NC)** magnetic reed switch or physical contact switch (required for energy efficiency)
- **Voltage Dividers**: For battery and charger detection
  - Battery monitoring: R1: 510kΩ, R2: 1MΩ
  - Charger detection: R1: 510kΩ, R2: 1MΩ

## Pin Configuration

| Pin | Function | Direction | Description |
|-----|----------|-----------|-------------|
| GPIO 0 | Charger Detection | Input | HIGH when USB charger connected |
| GPIO 1 | Battery Voltage | Analog Input | Voltage divider output |
| GPIO 2 | Contact Sensor | Input (Pull-up) | LOW = closed, HIGH = open |
| GPIO 15 | Status LED | Output | Activity indicator |

## Software Requirements

- **ESP-IDF**: Version 4.1.0 or higher (tested with v5.x)
- **Arduino ESP32**: Version 3.3.2 (included as component)
- **CMake**: Version 3.5 or higher

### Required Components
- **ESP Zigbee SDK**: Included as git submodule

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

### 3. Build and Flash

```bash
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

Replace `/dev/ttyUSB0` with your serial port.

## Configuration

### Zigbee Configuration

```cpp
Manufacturer: "Miro Sieber"
Model: "OmniSenseContact"
Device Type: Binary Input (Contact Sensor)
Endpoint: 10
Power Source: Battery
```

### Network Settings

- **Connection Timeout**: 10 seconds
- **Report Timeout**: 1000ms per attribute
- **Max Retries**: 3 attempts
- **Deep Sleep Duration**: 86400 seconds (24 hours)

## Usage

### Pairing with Zigbee Coordinator

1. Put your Zigbee coordinator in pairing mode
2. Power on or reset the OmniSenseContact device
3. Wait up to 10 seconds for connection
4. First boot: Device waits 60 seconds for interview process

### Operation

- Device monitors contact sensor continuously
- Reports state changes instantly to coordinator
- Returns to deep sleep between events
- Daily automatic wake-up to maintain network presence

### Factory Reset

To re-pair with a new network:

```bash
idf.py erase-flash
idf.py flash
```

## Battery Management

### Voltage Monitoring

The device uses voltage dividers (510kΩ/1MΩ) to measure battery voltage and detect charger connection.

**Voltage Range:**
- Maximum: 4.2V (fully charged)
- Nominal: 3.7V
- Minimum: 3.0V
- Critical: Below 3.0V triggers permanent sleep

### State of Charge (SoC) Estimation

21-point voltage-to-SoC lookup table with linear interpolation and exponential filtering (80% old + 20% new).

### Battery Calibration

For accurate voltage readings:

1. Connect USB charger
2. Open serial monitor (115200 baud)
3. Measure actual battery voltage with multimeter
4. Enter measured voltage in serial console (e.g., `3.847`)
5. Calibration factor is saved to NVS

Type `s` to skip charging loop.
Type `reset` to factory reset the device.

### Low Battery Protection

When SoC < 10%:
- Device enters permanent deep sleep
- Only wakes when charger connected

## Power Management

**Active Time:** ~2-5 seconds per event
**Sleep Time:** Until next state change or 24-hour timer

**Wake-up Conditions:**
1. Contact state change (GPIO interrupt)
2. Daily timer (86400 seconds)
3. Charger connected

**Power Consumption:**
- Deep Sleep: ~10-20 µA
- Active (Zigbee TX): ~50-100 mA

## License

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

## Author

Miro Sieber
