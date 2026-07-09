# Depth Sensor (ESP32‑C6 + Zigbee)

Water tank level sensor using an ESP32‑C6 and Gravity Industrial Submersible Pressure Level Sensors (0–5 m) SKU: KIT0139, via DFRobot Gravity Analog 4–20 mA to Voltage Converters (SEN0262).
Publishes two water depth readings (mm) and board temperature via Zigbee (Home Assistant through Zigbee2MQTT).
An onboard LED is exposed via Zigbee for basic status/identify.

## Features
- Configurable depth reporting from 4–20 mA pressure sensors (Analog Output cluster `presentValue` in mm).
- Temperature reporting (Temperature Measurement cluster, 0.01°C resolution).
- Identify effect blinks the onboard LED for visual feedback.

## Hardware
- MCU: ESP32‑C6-DevKitM-1
- Level sensors: Gravity Industrial Submersible Pressure Level Sensor 0–5 m (KIT0139)
- Converters: DFRobot Gravity Analog Current to Voltage Converter (SEN0262, 4–20 mA → 0.48–2.4 V across 120 Ω)
- Onboard LED: single LED (mono) or discrete RGB depending on board version
- Temperature sensor: on‑board sensor (software driver included)

### Wiring
- Power each KIT0139 per its datasheet and connect each 4–20 mA output to its own SEN0262 input.
- Tank 1 SEN0262 Vout → ESP32‑C6 GPIO6 (`ADC_CHANNEL_6` / ADC1 channel 6).
- Tank 2 SEN0262 Vout → ESP32‑C6 GPIO2 (`ADC_CHANNEL_2` / ADC1 channel 2).
- SEN0262 GND → ESP32‑C6 GND for every converter.
- SEN0262 VCC per your board (typically 5V); ensure common ground with the ESP32‑C6.
- Important: Each KIT0139 has a vented cable; keep the vent tube dry and open to atmosphere.

```mermaid
flowchart LR
    sensor1["Tank 1 KIT0139<br/>4-20 mA"] --> converter1["SEN0262 converter"]
    sensor2["Tank 2 KIT0139<br/>4-20 mA"] --> converter2["SEN0262 converter"]
    converter1 -->|"0.48-2.4 V Vout"| gpio6["ESP32-C6 GPIO6<br/>ADC_CHANNEL_6"]
    converter2 -->|"0.48-2.4 V Vout"| gpio2["ESP32-C6 GPIO2<br/>ADC_CHANNEL_2"]
```

Notes:
- This project reads each ADC voltage, converts to current using the 120 Ω sense resistor, then maps 4–20 mA to 0–5000 mm depth.
- Depth is clamped and reported as 0–5000 millimeters (mm) to Zigbee.
- ESP32‑C6-DevKitM-1 exposes ADC1 channels on GPIO0 through GPIO6. GPIO7 is not ADC-capable.
- Change `s_depth_sensors` in `main/depth_sensor.c` to add, remove, or remap tank sensors.

## Firmware behavior
- ADC oneshot configures every listed sensor channel with 12 dB attenuation and 12-bit reads.
- ADC calibration is used when ESP-IDF provides it; otherwise firmware falls back to a 3300 mV full-scale approximation.
- Each depth update reads 8 ADC samples per sensor, drops the minimum and maximum, converts the trimmed mean to depth in mm, and rounds it.
- Depth is sampled once per second, averaged over the last 20 rounded readings per sensor, and published to each sensor endpoint's Analog Output cluster.
- Temperature is sampled once per second and reported in 0.01°C units through the Temperature Measurement cluster.
- The Identify command toggles the onboard LED once per second for visual location feedback.

## Zigbee
- Depth endpoints: tank 1 on endpoint 1, tank 2 on endpoint 2.
- Manufacturer/model: `Acheta` / `Depth.Sensor`
- Endpoint 1 server clusters: Basic, Identify, Analog Output, Temperature Measurement, On/Off, Color Control, Scenes, Level, Groups
- Endpoint 2 server clusters: Analog Output
- Endpoint 1 client clusters: Identify
- Analog Output `presentValue`: depth in mm as a floating-point value.
- Temperature Measurement value: board temperature in 0.01°C units.
- Reporting:
  - Analog Output `presentValue`: periodic 1–10 s per depth endpoint, delta disabled.
  - Temperature Measurement: periodic 30–300 s or >=0.5°C change.
- Zigbee2MQTT exposes depth readings with endpoint-specific names from `z2m-external_definition.js`.

## ESP‑IDF Setup

Prerequisites:
- ESP‑IDF 6.0.x installed with required tools; verified with IDF 6.0.2.

Build-only verification used by CI:

```sh
. /path/to/esp-idf/export.sh
./scripts/verify-build.sh
```

The script runs `idf.py -B "${BUILD_DIR:-build}" build`. Set `BUILD_DIR=build-ci` to use a separate local build directory. GitHub Actions runs the same build gate on pushes and pull requests using the ESP-IDF 6.0.2 container.

Manual build, flash, and monitor:

```sh
. /path/to/esp-idf/export.sh
idf.py set-target esp32c6
idf.py build
idf.py -p /dev/ttyUSB0 flash
idf.py -p /dev/ttyUSB0 monitor
```

Hardware validation:

```sh
idf.py -p /dev/ttyUSB0 flash monitor
```

Adjust the serial device as needed, then verify depth readings, temperature reports, Zigbee join behavior, and LED attribute handling in the monitor logs. Use `Ctrl+]` to exit the IDF monitor.
