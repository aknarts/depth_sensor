#include <sys/cdefs.h>
#include <stdio.h>
#include <stdbool.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ultrasonic.h>
#include <esp_err.h>
#include "depth_sensor.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "temp_sensor_driver.h"
#include "ha/esp_zigbee_ha_standard.h"

//TODO: https://github.com/Koenkk/zigbee2mqtt/issues/18321

#define TRIGGER_GPIO 7
#define ECHO_GPIO 14

#define MAX_VALUES 10

static ultrasonic_sensor_t sensor = {
		.trigger_pin = TRIGGER_GPIO,
		.echo_pin = ECHO_GPIO
};

static const char *TAG = "ESP_ZB_DIST_SENSOR";

static int16_t zb_temperature_to_s16(float temp)
{
	return (int16_t) (temp * 100);
}

float calculate_average(float values[], int count)
{
	float sum = 0.0;
	for (int i = 0; i < count; i++)
	{
		sum += values[i];
	}
	return sum / count;
}

_Noreturn void ultrasonic_task(void *pvParameters)
{
	float values[MAX_VALUES] = {0.0};
	int currentIndex = 0;
	int count = 0;

	while (true)
	{
		int32_t distance;
		esp_err_t res = ultrasonic_measure_cm(&sensor, ESP_DIST_SENSOR_MAX_VALUE, &distance);
		if (res != ESP_OK)
		{
			printf("Error %d: ", res);
			switch (res)
			{
				case ESP_ERR_ULTRASONIC_PING:
					ESP_LOGW(TAG, "Cannot ping (device is in invalid state)\n");
					break;
				case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
					ESP_LOGW(TAG, "Ping timeout (echo timeout)\n");
					break;
				case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
					ESP_LOGW(TAG, "Echo timeout (i.e. distance too big)\n");
					break;
				default:
					ESP_LOGE(TAG, "%s\n", esp_err_to_name(res));
			}
		} else
		{
			ESP_LOGI(TAG, "Distance: %ld cm", distance);
			values[currentIndex] = (float) distance;
			currentIndex = (currentIndex + 1) % MAX_VALUES;
			if (count < MAX_VALUES)
			{
				count++;
			}
			float fdistance = roundf(calculate_average(values, count));
			ESP_LOGI(TAG, "Distance Average: %f cm", fdistance);
			esp_zb_lock_acquire(portMAX_DELAY);
			esp_zb_zcl_set_attribute_val(HA_ESP_SENSOR_ENDPOINT,
										 ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
										 ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID, &fdistance, false);
			esp_zb_lock_release();
		}


		vTaskDelay(pdMS_TO_TICKS(ESP_DIST_SENSOR_UPDATE_INTERVAL * 1000));
	}
}


static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
	ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, ,
						TAG, "Failed to start Zigbee bdb commissioning");
}

static void esp_app_temp_sensor_handler(float temperature)
{
	int16_t measured_value = zb_temperature_to_s16(temperature);
	/* Update temperature sensor measured value */
	esp_zb_lock_acquire(portMAX_DELAY);
	esp_zb_zcl_set_attribute_val(HA_ESP_SENSOR_ENDPOINT,
								 ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
								 ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &measured_value, false);
	esp_zb_lock_release();
}

static esp_err_t deferred_driver_init(void)
{
	ultrasonic_init(&sensor);
	light_driver_init(LIGHT_DEFAULT_OFF);
	xTaskCreate(ultrasonic_task, "ultrasonic_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
	temperature_sensor_config_t temp_sensor_config =
			TEMPERATURE_SENSOR_CONFIG_DEFAULT(ESP_TEMP_SENSOR_MIN_VALUE, ESP_TEMP_SENSOR_MAX_VALUE);
	ESP_RETURN_ON_ERROR(
			temp_sensor_driver_init(&temp_sensor_config, ESP_TEMP_SENSOR_UPDATE_INTERVAL, esp_app_temp_sensor_handler),
			TAG,
			"Failed to initialize temperature sensor");
	return ESP_OK;
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
	uint32_t *p_sg_p = signal_struct->p_app_signal;
	esp_err_t err_status = signal_struct->esp_err_status;
	esp_zb_app_signal_type_t sig_type = *p_sg_p;
	switch (sig_type)
	{
		case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
			ESP_LOGI(TAG, "Initialize Zigbee stack");
			esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
			break;
		case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
		case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
			if (err_status == ESP_OK)
			{
				ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
				ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
				if (esp_zb_bdb_is_factory_new())
				{
					ESP_LOGI(TAG, "Start network steering");
					esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
				} else
				{
					ESP_LOGI(TAG, "Device rebooted");
				}
			} else
			{
				/* commissioning failed */
				ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
			}
			break;
		case ESP_ZB_BDB_SIGNAL_STEERING:
			if (err_status == ESP_OK)
			{
				esp_zb_ieee_addr_t extended_pan_id;
				esp_zb_get_extended_pan_id(extended_pan_id);
				ESP_LOGI(TAG,
						 "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
						 extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
						 extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
						 esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
			} else
			{
				ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
				esp_zb_scheduler_alarm((esp_zb_callback_t) bdb_start_top_level_commissioning_cb,
									   ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
			}
			break;
		case ESP_ZB_ZDO_SIGNAL_LEAVE:
			ESP_LOGI(TAG, "Leaving old network");
			esp_zb_nvram_erase_at_start(true);
			ESP_LOGI(TAG, "Start network steering");
			esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
			break;
		default:
			ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
					 esp_err_to_name(err_status));
			break;
	}
}

static void esp_zb_identify(void *pvParameters)
{
	bool light_state = false;
	for (int i = 0; i < 50; ++i)
	{
		light_state = !light_state;
		light_driver_set_power(light_state);

		vTaskDelay(pdMS_TO_TICKS(1000));
	};
	light_driver_set_power(false);
	vTaskDelay(pdMS_TO_TICKS(1000));
	vTaskDelete(NULL);
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
	esp_err_t ret = ESP_OK;
	bool light_state = 0;
	uint8_t light_level = 0;
	uint16_t light_color_x = 0;
	uint16_t light_color_y = 0;
	ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
	ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG,
						"Received message: error status(%d)",
						message->info.status);
	ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)",
			 message->info.dst_endpoint, message->info.cluster,
			 message->attribute.id, message->attribute.data.size);
	if (message->info.dst_endpoint == HA_ESP_SENSOR_ENDPOINT)
	{
		switch (message->info.cluster)
		{
			case ESP_ZB_ZCL_CLUSTER_ID_ON_OFF:
				if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID &&
					message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL)
				{
					light_state = message->attribute.data.value ? *(bool *) message->attribute.data.value : light_state;
					ESP_LOGI(TAG, "Light sets to %s", light_state ? "On" : "Off");
					light_driver_set_power(light_state);
				} else
				{
					ESP_LOGW(TAG, "On/Off cluster data: attribute(0x%x), type(0x%x)", message->attribute.id,
							 message->attribute.data.type);
				}
				break;
			case ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL:
				if (message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_X_ID &&
					message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16)
				{
					light_color_x = message->attribute.data.value ? *(uint16_t *) message->attribute.data.value
																  : light_color_x;
					light_color_y = *(uint16_t *) esp_zb_zcl_get_attribute(message->info.dst_endpoint,
																		   message->info.cluster,
																		   ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
																		   ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_Y_ID)
							->data_p;
					ESP_LOGI(TAG, "Light color x changes to 0x%x", light_color_x);
				} else if (message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_Y_ID &&
						   message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16)
				{
					light_color_y = message->attribute.data.value ? *(uint16_t *) message->attribute.data.value
																  : light_color_y;
					light_color_x = *(uint16_t *) esp_zb_zcl_get_attribute(message->info.dst_endpoint,
																		   message->info.cluster,
																		   ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
																		   ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_X_ID)
							->data_p;
					ESP_LOGI(TAG, "Light color y changes to 0x%x", light_color_y);
				} else
				{
					ESP_LOGW(TAG, "Color control cluster data: attribute(0x%x), type(0x%x)", message->attribute.id,
							 message->attribute.data.type);
				}
				light_driver_set_color_xy(light_color_x, light_color_y);
				break;
			case ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL:
				if (message->attribute.id == ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID &&
					message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8)
				{
					light_level = message->attribute.data.value ? *(uint8_t *) message->attribute.data.value
																: light_level;
					light_driver_set_level((uint8_t) light_level);
					ESP_LOGI(TAG, "Light level changes to %d", light_level);
				} else
				{
					ESP_LOGW(TAG, "Level Control cluster data: attribute(0x%x), type(0x%x)", message->attribute.id,
							 message->attribute.data.type);
				}
				break;
			case ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY:
				xTaskCreate(esp_zb_identify, "Identify", 4096, NULL, 5, NULL);
			default:
				ESP_LOGI(TAG, "Message data: cluster(0x%x), attribute(0x%x)  ", message->info.cluster,
						 message->attribute.id);
		}
	}
	return ret;
}


static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
	esp_err_t ret = ESP_OK;
	switch (callback_id)
	{
		case ESP_ZB_CORE_REPORT_ATTR_CB_ID:
//			ret = zb_attribute_reporting_handler((esp_zb_zcl_report_attr_message_t *)message);
			ESP_LOGI(TAG, "Report attribute callback");
			break;
		case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID:
//			ret = zb_read_attr_resp_handler((esp_zb_zcl_cmd_read_attr_resp_message_t *)message);
			ESP_LOGI(TAG, "Read attribute response callback");
			break;
		case ESP_ZB_CORE_CMD_REPORT_CONFIG_RESP_CB_ID:
			ESP_LOGI(TAG, "Configure report response callback");
//			ret = zb_configure_report_resp_handler((esp_zb_zcl_cmd_config_report_resp_message_t *)message);
			break;
		case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
			ESP_LOGI(TAG, "Set attribute value callback");
			ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *) message);
			break;
		case ESP_ZB_CORE_IDENTIFY_EFFECT_CB_ID:
			ESP_LOGI(TAG, "Identify effect callback");
			break;
		case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID:
			ESP_LOGI(TAG, "Default response callback");
			break;
		default:
			ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
			break;
	}
	return ret;
}

static esp_zb_cluster_list_t *
custom_distance_sensor_clusters_create(esp_zb_analog_output_cluster_cfg_t *distance_sensor,
									   esp_zb_temperature_meas_cluster_cfg_t *temperature_sensor,
									   esp_zb_color_dimmable_light_cfg_t *light)
{
	esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

	esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&light->basic_cfg);
	ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
												  MANUFACTURER_NAME));
	ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
												  MODEL_IDENTIFIER));
	ESP_ERROR_CHECK(
			esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

	ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(
			&light->identify_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
	ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_zcl_attr_list_create(
			ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));

	ESP_ERROR_CHECK(esp_zb_cluster_list_add_analog_output_cluster(cluster_list,
																  esp_zb_analog_output_cluster_create(
																		  distance_sensor),
																  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
	ESP_ERROR_CHECK(esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list,
																	 esp_zb_temperature_meas_cluster_create(
																			 temperature_sensor),
																	 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
	ESP_ERROR_CHECK(esp_zb_cluster_list_add_on_off_cluster(cluster_list,
														   esp_zb_on_off_cluster_create(
																   &light->on_off_cfg),
														   ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
	ESP_ERROR_CHECK(esp_zb_cluster_list_add_color_control_cluster(cluster_list,
																  esp_zb_color_control_cluster_create(
																		  &light->color_cfg),
																  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
	ESP_ERROR_CHECK(esp_zb_cluster_list_add_scenes_cluster(cluster_list,
														   esp_zb_scenes_cluster_create(
																   &light->scenes_cfg),
														   ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
	ESP_ERROR_CHECK(esp_zb_cluster_list_add_level_cluster(cluster_list,
														  esp_zb_level_cluster_create(
																  &light->level_cfg),
														  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
	ESP_ERROR_CHECK(esp_zb_cluster_list_add_groups_cluster(cluster_list,
														  esp_zb_groups_cluster_create(
																  &light->groups_cfg),
														  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
	return cluster_list;
}

static esp_zb_ep_list_t *
custom_distance_sensor_ep_create(esp_zb_analog_output_cluster_cfg_t *distance_sensor,
								 esp_zb_temperature_meas_cluster_cfg_t *temperature_sensor,
								 esp_zb_color_dimmable_light_cfg_t *light)
{
	esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
	esp_zb_endpoint_config_t endpoint_config = {
			.endpoint = HA_ESP_SENSOR_ENDPOINT,
			.app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
			.app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
			.app_device_version = 0
	};
	esp_zb_ep_list_add_ep(ep_list, custom_distance_sensor_clusters_create(distance_sensor, temperature_sensor, light),
						  endpoint_config);
	return ep_list;
}

static void esp_zb_task(void *pvParameters)
{
	/* Initialize Zigbee stack */
	esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZR_CONFIG();

	esp_zb_init(&zb_nwk_cfg);

	esp_zb_analog_output_cluster_cfg_t analog_cfg = {.out_of_service = false, .present_value = 0, .status_flags = 0};
	esp_zb_temperature_meas_cluster_cfg_t temp_cfg = {.measured_value = ESP_ZB_ZCL_TEMP_MEASUREMENT_MEASURED_VALUE_DEFAULT, .min_value = zb_temperature_to_s16(
			ESP_TEMP_SENSOR_MIN_VALUE), .max_value = zb_temperature_to_s16(ESP_TEMP_SENSOR_MAX_VALUE)};
	esp_zb_color_dimmable_light_cfg_t light_cfg = {
			.basic_cfg = {
					.zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
					.power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE,
			},
			.on_off_cfg = {
					.on_off = false,
			},
			.color_cfg = {
					.current_x = ESP_ZB_ZCL_COLOR_CONTROL_CURRENT_X_DEF_VALUE,
					.current_y = ESP_ZB_ZCL_COLOR_CONTROL_CURRENT_Y_DEF_VALUE,
					.color_mode = ESP_ZB_ZCL_COLOR_CONTROL_COLOR_MODE_DEFAULT_VALUE,
					.options = ESP_ZB_ZCL_COLOR_CONTROL_OPTIONS_DEFAULT_VALUE,
					.enhanced_color_mode = ESP_ZB_ZCL_COLOR_CONTROL_ENHANCED_COLOR_MODE_DEFAULT_VALUE,
					.color_capabilities = 0x0008,
			},
			.level_cfg =
					{
							.current_level = ESP_ZB_ZCL_LEVEL_CONTROL_CURRENT_LEVEL_DEFAULT_VALUE,
					},
			.scenes_cfg =
					{
							.scenes_count = ESP_ZB_ZCL_SCENES_SCENE_COUNT_DEFAULT_VALUE,
							.current_scene = ESP_ZB_ZCL_SCENES_CURRENT_SCENE_DEFAULT_VALUE,
							.current_group = ESP_ZB_ZCL_SCENES_CURRENT_GROUP_DEFAULT_VALUE,
							.scene_valid = ESP_ZB_ZCL_SCENES_SCENE_VALID_DEFAULT_VALUE,
							.name_support = ESP_ZB_ZCL_SCENES_NAME_SUPPORT_DEFAULT_VALUE,
					},
			.groups_cfg =
					{
							.groups_name_support_id = ESP_ZB_ZCL_GROUPS_NAME_SUPPORT_DEFAULT_VALUE,
					},
			.identify_cfg =
					{
							.identify_time = ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE,
					},

	};
	esp_zb_ep_list_t *esp_zb_sensor_ep = custom_distance_sensor_ep_create(&analog_cfg, &temp_cfg, &light_cfg);

	/* Register the device */
	esp_zb_device_register(esp_zb_sensor_ep);

	/* Config the reporting info  */
	esp_zb_zcl_reporting_info_t reporting_info = {
			.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
			.ep = HA_ESP_SENSOR_ENDPOINT,
			.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT,
			.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
			.dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
			.u.send_info.min_interval = 1,
			.u.send_info.max_interval = 0,
			.u.send_info.def_min_interval = 1,
			.u.send_info.def_max_interval = 0,
			.u.send_info.delta.u16 = 100,
			.attr_id = ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID,
			.manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
	};

	esp_zb_zcl_update_reporting_info(&reporting_info);

	esp_zb_core_action_handler_register(zb_action_handler);
	esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
	ESP_ERROR_CHECK(esp_zb_start(false));

	esp_zb_stack_main_loop();
}

void app_main(void)
{
	esp_zb_platform_config_t config = {
			.radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
			.host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
	};
	ESP_ERROR_CHECK(nvs_flash_init());
	ESP_ERROR_CHECK(esp_zb_platform_config(&config));
	xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
