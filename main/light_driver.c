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


#include "esp_log.h"
#include "led_strip.h"
#include "light_driver.h"

#include <math.h>

typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} rgb_color_t;

static const char *TAG = "LIGHT_DRIVER";
static led_strip_handle_t s_led_strip;
static uint8_t s_red = 255, s_green = 255, s_blue = 255, s_level = 255;
static bool s_power = false;

static uint8_t clamp_unit_to_u8(float value)
{
    if (!isfinite(value) || value <= 0.0f) {
        return 0;
    }
    if (value >= 1.0f) {
        return UINT8_MAX;
    }
    return (uint8_t)(value * (float)UINT8_MAX + 0.5f);
}

static bool xy_to_rgb(uint16_t color_current_x, uint16_t color_current_y, rgb_color_t *rgb)
{
    if (!rgb || color_current_y == 0) {
        return false;
    }

    const float color_x = (float)color_current_x / 65535.0f;
    const float color_y = (float)color_current_y / 65535.0f;
    const float color_z = 1.0f - color_x - color_y;

    if (color_z < 0.0f) {
        return false;
    }

    const float color_X = color_x / color_y;
    const float color_Z = color_z / color_y;
    const float red_f = 3.240479f * color_X - 1.537150f - 0.498535f * color_Z;
    const float green_f = -0.969256f * color_X + 1.875992f + 0.041556f * color_Z;
    const float blue_f = 0.055648f * color_X - 0.204043f + 1.057311f * color_Z;

    rgb->red = clamp_unit_to_u8(red_f);
    rgb->green = clamp_unit_to_u8(green_f);
    rgb->blue = clamp_unit_to_u8(blue_f);
    return true;
}

static rgb_color_t hsv_to_rgb(uint8_t hue, uint8_t sat, uint8_t value)
{
    const float v = (float)value / (float)UINT8_MAX;
    const float s = (float)sat / (float)UINT8_MAX;

    if (sat == 0) {
        const uint8_t grey = clamp_unit_to_u8(v);
        return (rgb_color_t){.red = grey, .green = grey, .blue = grey};
    }

    const float h = ((float)hue * 360.0f) / 256.0f;
    const float c = v * s;
    const float h_sector = h / 60.0f;
    const float x = c * (1.0f - fabsf(fmodf(h_sector, 2.0f) - 1.0f));
    const float m = v - c;
    float red = 0.0f;
    float green = 0.0f;
    float blue = 0.0f;

    if (h_sector < 1.0f) {
        red = c;
        green = x;
    } else if (h_sector < 2.0f) {
        red = x;
        green = c;
    } else if (h_sector < 3.0f) {
        green = c;
        blue = x;
    } else if (h_sector < 4.0f) {
        green = x;
        blue = c;
    } else if (h_sector < 5.0f) {
        red = x;
        blue = c;
    } else {
        red = c;
        blue = x;
    }

    return (rgb_color_t){
        .red = clamp_unit_to_u8(red + m),
        .green = clamp_unit_to_u8(green + m),
        .blue = clamp_unit_to_u8(blue + m),
    };
}

static inline void apply_current_output(void)
{
    if (!s_led_strip) return;
    if (s_power) {
        float ratio = (float)s_level / 255;
        ESP_ERROR_CHECK(led_strip_set_pixel(s_led_strip, 0,
                                            (uint8_t)((float)s_red * ratio),
                                            (uint8_t)((float)s_green * ratio),
                                            (uint8_t)((float)s_blue * ratio)));
    } else {
        // Fully off when power is false
        ESP_ERROR_CHECK(led_strip_set_pixel(s_led_strip, 0, 0, 0, 0));
    }
    ESP_ERROR_CHECK(led_strip_refresh(s_led_strip));
}

void light_driver_set_color_xy(uint16_t color_current_x, uint16_t color_current_y)
{
    rgb_color_t rgb;
    if (!xy_to_rgb(color_current_x, color_current_y, &rgb)) {
        ESP_LOGW(TAG, "Ignoring invalid XY color: x=0x%04x, y=0x%04x", color_current_x, color_current_y);
        return;
    }

    s_red = rgb.red;
    s_green = rgb.green;
    s_blue = rgb.blue;
    if (s_power) apply_current_output();
}

void light_driver_set_color_hue_sat(uint8_t hue, uint8_t sat)
{
    const rgb_color_t rgb = hsv_to_rgb(hue, sat, UINT8_MAX);
    s_red = rgb.red;
    s_green = rgb.green;
    s_blue = rgb.blue;
    if (s_power) apply_current_output();
}

void light_driver_set_color_RGB(uint8_t red, uint8_t green, uint8_t blue)
{
    s_red = red;
    s_green = green;
    s_blue = blue;
    if (s_power) apply_current_output();
}

void light_driver_set_power(bool power)
{
    s_power = power;
    apply_current_output();
}

void light_driver_set_level(uint8_t level)
{
    s_level = level;
    if (s_power) apply_current_output();
}

void light_driver_init(bool power)
{
    led_strip_config_t led_strip_conf = {
        .max_leds = CONFIG_EXAMPLE_STRIP_LED_NUMBER,
        .strip_gpio_num = CONFIG_EXAMPLE_STRIP_LED_GPIO,
    };
    led_strip_rmt_config_t rmt_conf = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&led_strip_conf, &rmt_conf, &s_led_strip));

    // Start from a known state
    s_red = 255; s_green = 255; s_blue = 255; s_level = 255;
    s_power = power;
    apply_current_output();
}
