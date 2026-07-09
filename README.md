# Depth Sensor (ESP32‑C6 + Zigbee)

Water tank level sensor using an ESP32‑C6 and a Gravity Industrial Submersible Pressure Level Sensor (0–5 m) SKU: KIT0139, via the DFRobot Gravity Analog 4–20 mA to Voltage Converter (SEN0262).  
Publishes water depth (mm) and board temperature via Zigbee (Home Assistant through Zigbee2MQTT).
An onboard LED is exposed via Zigbee for basic status/identify.

## Features
- Depth reporting from 4–20 mA pressure sensor (Analog Output cluster presentValue in mm).
- Temperature reporting (Temperature Measurement cluster, 0.01°C resolution).
- Identify effect blinks the onboard LED for visual feedback.

## Hardware
- MCU: ESP32‑C6
- Level sensor: Gravity Industrial Submersible Pressure Level Sensor 0–5 m (KIT0139)
- Converter: DFRobot Gravity Analog Current to Voltage Converter (SEN0262, 4–20 mA → 0.48–2.4 V across 120 Ω)
- Onboard LED: single LED (mono) or discrete RGB depending on board version
- Temperature sensor: on‑board sensor (software driver included)

### Wiring
- Power the KIT0139 per its datasheet and connect its 4–20 mA output to the SEN0262 input.
- SEN0262 Vout → ESP32‑C6 GPIO7 (ADC1 channel resolved at runtime).
- SEN0262 GND → ESP32‑C6 GND.
- SEN0262 VCC per your board (typically 5V); ensure common ground with the ESP32‑C6.
- Important: The KIT0139 has a vented cable; keep the vent tube dry and open to atmosphere.

Notes:
- This project reads the ADC voltage, converts to current using the 120 Ω sense resistor, then maps 4–20 mA to 0–5000 mm depth.
- Depth is reported as 0–5000 millimeters (mm) to Zigbee.

## Firmware behavior
- ADC oneshot driver is used with 11 dB attenuation and calibration when available.
- Depth is sampled once per second, averaged over the last 10 samples, and published to the Analog Output cluster.
- Temperature is reported via the Temperature Measurement cluster with a default report interval.

## Zigbee
- Endpoint: 1
- Clusters (server): Basic, Identify, Analog Output (depth in mm), Temperature Measurement, On/Off, Color Control, Level, Scenes, Groups
- Reporting:
  - Analog Output presentValue: periodic 1–10 s
  - Temperature Measurement: periodic 30–300 s or ≥0.5 °C change

## ESP‑IDF Setup

Prerequisites:
- ESP‑IDF 5.5.x installed with required tools; verified with IDF 5.5.4.

Build and flash:
