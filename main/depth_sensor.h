//
// Created by stastny on 25.9.24.
//

#ifndef DEPTH_SENSOR_DEPTH_SENSOR_H
#define DEPTH_SENSOR_DEPTH_SENSOR_H

#include "esp_zigbee_core.h"
#include "light_driver.h"

/* Zigbee configuration */
#define MAX_CHILDREN                      10                                    /* the max amount of connected devices */
#define INSTALLCODE_POLICY_ENABLE       false   /* enable the install code policy for security */
#define HA_ESP_SENSOR_ENDPOINT          1
#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK    /* Zigbee primary channel mask use in the example */

#define ESP_DIST_SENSOR_UPDATE_INTERVAL (1)     /* Local sensor update interval (second) */
#define ESP_DIST_SENSOR_MAX_VALUE       (600)    /* Local sensor max measured value (cm) */

/* Attribute values in ZCL string format
 * The string should be started with the length of its own.
 */
#define MANUFACTURER_NAME               "\x06""Acheta"
#define MODEL_IDENTIFIER                "\x0C""Depth.Sensor"

#define ESP_ZB_ZR_CONFIG()                                                              \
    {                                                                                   \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER,                                       \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,                               \
        .nwk_cfg.zczr_cfg = {                                                           \
            .max_children = MAX_CHILDREN,                                               \
        },                                                                              \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()                           \
    {                                                           \
        .radio_mode = ZB_RADIO_MODE_NATIVE,                     \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                            \
    {                                                           \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,   \
    }

#endif //DEPTH_SENSOR_DEPTH_SENSOR_H
