# Repository Guidelines

## Project Structure & Module Organization

This is an ESP-IDF firmware project for an ESP32-C6 Zigbee depth sensor. Root-level build and configuration files include `CMakeLists.txt`, `sdkconfig`, `partitions.csv`, `dependencies.lock`, and `main/idf_component.yml`. Firmware source lives in `main/`:

- `main/depth_sensor.c` and `.h`: Zigbee endpoint, ADC depth reporting, commissioning, and attribute handling.
- `main/light_driver.c` and `.h`: onboard LED control exposed through Zigbee light clusters.
- `main/temp_sensor_driver.c` and `.h`: onboard temperature polling.
- `z2m-external_definition.js`: Zigbee2MQTT external converter definition.

## Build, Test, and Development Commands

Use ESP-IDF 5.x with the environment loaded before running commands.

- `idf.py set-target esp32c6`: configure the target MCU.
- `idf.py build`: compile firmware and resolve managed components.
- `idf.py -p /dev/ttyUSB0 flash monitor`: flash a connected board and stream logs; adjust the serial device as needed.
- `idf.py menuconfig`: update ESP-IDF configuration stored in `sdkconfig`.
- `idf.py clean`: remove build outputs while keeping configuration.

## Coding Style & Naming Conventions

Write C in the existing ESP-IDF style and match surrounding formatting when editing. Prefer `snake_case` for functions and local variables, uppercase `#define` constants, and `s_` prefixes for file-static state such as `s_adc_handle`. Keep headers guarded with project-specific include guards. Use ESP-IDF helpers such as `ESP_ERROR_CHECK`, `ESP_RETURN_ON_ERROR`, and `ESP_LOG*` consistently for error handling and observability.

## Testing Guidelines

No automated test suite is currently present. At minimum, run `idf.py build` before submitting changes. For behavior changes, flash hardware and verify logs with `idf.py -p /dev/ttyUSB0 flash monitor`. Check depth readings, temperature reports, Zigbee join behavior, and LED attribute handling. If adding tests later, place them in an ESP-IDF-compatible test component and document the exact command here.

## Commit & Pull Request Guidelines

Recent commits use short imperative subjects such as `Add README.md` and `Fix external definition`. Keep commit messages to one concise subject line unless a short body is necessary. Pull requests should include a brief summary, linked issue if one exists, hardware used for validation, build output status, and relevant monitor log excerpts for runtime behavior. Note any required Zigbee2MQTT converter or `sdkconfig` changes explicitly.

## Security & Configuration Tips

Do not commit local serial-port assumptions, private network details, or generated credentials. Treat `sdkconfig` changes as reviewable firmware behavior changes, especially radio, partition, flash, and Zigbee settings.
