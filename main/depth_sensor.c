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

float calculate_average(float values[], int count) {
	float sum = 0.0;
	for (int i = 0; i < count; i++) {
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
			if (count < MAX_VALUES) {
				count++;
			}
			float fdistance = calculate_average(values, count);
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

static esp_err_t deferred_driver_init(void)
{
	ultrasonic_init(&sensor);
	light_driver_init(LIGHT_DEFAULT_OFF);
	xTaskCreate(ultrasonic_task, "ultrasonic_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
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

	ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
	ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG,
						"Received message: error status(%d)",
						message->info.status);
	ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)",
			 message->info.dst_endpoint, message->info.cluster,
			 message->attribute.id, message->attribute.data.size);
	if (message->info.dst_endpoint == HA_ESP_SENSOR_ENDPOINT)
	{
		if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY)
		{
			xTaskCreate(esp_zb_identify, "Identify", 4096, NULL, 5, NULL);
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
custom_distance_sensor_clusters_create(esp_zb_analog_output_cluster_cfg_t *distance_sensor)
{
	esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

	esp_zb_basic_cluster_cfg_t basic_cfg =
			{
					.zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
					.power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE,
			};
	esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&basic_cfg);
	ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
												  MANUFACTURER_NAME));
	ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
												  MODEL_IDENTIFIER));


	ESP_ERROR_CHECK(
			esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

	esp_zb_identify_cluster_cfg_t identify_cfg = {
			.identify_time = ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE,
	};
	ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(
			&identify_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
	ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_zcl_attr_list_create(
			ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));

	ESP_ERROR_CHECK(esp_zb_cluster_list_add_analog_output_cluster(cluster_list,
																  esp_zb_analog_output_cluster_create(
																		  distance_sensor),
																  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
	return cluster_list;
}

static esp_zb_ep_list_t *
custom_distance_sensor_ep_create(esp_zb_analog_output_cluster_cfg_t *distance_sensor)
{
	esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
	esp_zb_endpoint_config_t endpoint_config = {
			.endpoint = HA_ESP_SENSOR_ENDPOINT,
			.app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
			.app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
			.app_device_version = 0
	};
	esp_zb_ep_list_add_ep(ep_list, custom_distance_sensor_clusters_create(distance_sensor), endpoint_config);
	return ep_list;
}

static void esp_zb_task(void *pvParameters)
{
	/* Initialize Zigbee stack */
	esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();

	esp_zb_init(&zb_nwk_cfg);

	esp_zb_analog_output_cluster_cfg_t analog_cfg = {.out_of_service = false, .present_value = 0, .status_flags = 0};
	esp_zb_ep_list_t *esp_zb_sensor_ep = custom_distance_sensor_ep_create(&analog_cfg);

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
	printf("Starting Zigbee task\n");
	xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
