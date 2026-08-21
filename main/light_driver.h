/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee light driver example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */


#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* light intensity level */
#define LIGHT_DEFAULT_ON  1
#define LIGHT_DEFAULT_OFF 0

/* LED strip configuration */
#define CONFIG_EXAMPLE_STRIP_LED_GPIO   8
#define CONFIG_EXAMPLE_STRIP_LED_NUMBER 1

/**
* @brief Set light power (on/off).
*
* @param  power  The light power to be set
*/
void light_driver_set_power(bool power);

/**
* @brief Temporarily override light power for Zigbee Identify without changing
*        the requested light state.
*
* @param active enable or disable the Identify override
* @param power  output power while the Identify override is active
*/
void light_driver_set_identify(bool active, bool power);

/**
* @brief color light driver init, be invoked where you want to use color light
*
* @param power power on/off
*/
void light_driver_init(bool power);

/**
* @brief Set light level
*
* @param  level  The light level to be set
*/
void light_driver_set_level(uint8_t level);

/**
* @brief Set light color from RGB
*
* @param  red    The red color to be set
* @param  green  The green color to be set
* @param  blue   The blue color to be set
*/
void light_driver_set_color_RGB(uint8_t red, uint8_t green, uint8_t blue);

/**
* @brief Set light color from color xy
*
* @param  color_currentx  The color x to be set
* @param  color_currenty  The color y to be set
*/
void light_driver_set_color_xy(uint16_t color_current_x, uint16_t color_current_y);

/**
* @brief Set light color from hue saturation
*
* @param  hue  The hue to be set
* @param  sat  The sat to be set
*/
void light_driver_set_color_hue_sat(uint8_t hue, uint8_t sat);

#ifdef __cplusplus
} // extern "C"
#endif
