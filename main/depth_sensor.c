#include <sys/cdefs.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include "depth_math.h"
#include "depth_sensor.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "temp_sensor_driver.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"

//TODO: https://github.com/Koenkk/zigbee2mqtt/issues/18321

#define SENSOR_ADC_ATTEN ADC_ATTEN_DB_12
#define SENSOR_ADC_BITWIDTH ADC_BITWIDTH_12
#define SENSOR_ADC_APPROX_FULL_SCALE_MV 3300

#define MAX_VALUES 20
#define DEPTH_ADC_SAMPLE_COUNT 8
#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))

typedef struct {
	const char *name;
	uint8_t endpoint;
	adc_channel_t adc_channel;
	float values[MAX_VALUES];
	int current_index;
	int count;
} depth_sensor_state_t;

static const char *TAG = "ESP_ZB_DIST_SENSOR";

// ADC oneshot driver and calibration handles
static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static adc_cali_handle_t s_adc_cali_handle = NULL;
static bool s_adc_cali_enabled = false;
static TaskHandle_t s_identify_task_handle = NULL;
static uint16_t s_identify_time = 0;
static portMUX_TYPE s_identify_lock = portMUX_INITIALIZER_UNLOCKED;

static depth_sensor_state_t s_depth_sensors[] = {
	{.name = "tank_1", .endpoint = HA_ESP_SENSOR_ENDPOINT, .adc_channel = ADC_CHANNEL_6},
	{.name = "tank_2", .endpoint = HA_ESP_SENSOR_ENDPOINT + 1, .adc_channel = ADC_CHANNEL_2},
};

static int16_t zb_temperature_to_s16(float temp)
{
	return (int16_t) (temp * 100);
}

static float calculate_average(const float values[], int count)
{
	float sum = 0.0;
	for (int i = 0; i < count; i++)
	{
		sum += values[i];
	}
	return sum / count;
}

static esp_err_t read_depth_sensor_raw(const depth_sensor_state_t *sensor, int *raw)
{
	ESP_RETURN_ON_FALSE(sensor && raw, ESP_ERR_INVALID_ARG, TAG, "Invalid depth sensor read request");

	int raw_first = 0;
	esp_err_t res = adc_oneshot_read(s_adc_handle, sensor->adc_channel, &raw_first);
	if (res != ESP_OK)
	{
		ESP_LOGW(TAG, "%s ADC read failed: %s", sensor->name, esp_err_to_name(res));
		return res;
	}

	int sum_raw = raw_first;
	int min_raw = raw_first;
	int max_raw = raw_first;
	for (int i = 1; i < DEPTH_ADC_SAMPLE_COUNT; ++i)
	{
		int sample_raw = 0;
		res = adc_oneshot_read(s_adc_handle, sensor->adc_channel, &sample_raw);
		if (res != ESP_OK)
		{
			ESP_LOGW(TAG, "%s ADC read failed (sample %d/%d): %s",
					 sensor->name, i + 1, DEPTH_ADC_SAMPLE_COUNT, esp_err_to_name(res));
			return res;
		}
		sum_raw += sample_raw;
		if (sample_raw < min_raw) min_raw = sample_raw;
		if (sample_raw > max_raw) max_raw = sample_raw;
	}

	*raw = (sum_raw - min_raw - max_raw) / (DEPTH_ADC_SAMPLE_COUNT - 2);
	return ESP_OK;
}

static int depth_sensor_raw_to_voltage_mv(const depth_sensor_state_t *sensor, int raw)
{
	int voltage_mv = 0;
	if (s_adc_cali_enabled)
	{
		if (adc_cali_raw_to_voltage(s_adc_cali_handle, raw, &voltage_mv) == ESP_OK)
		{
			return voltage_mv;
		}
		ESP_LOGW(TAG, "%s ADC calibration conversion failed, using approximation", sensor->name);
	}

	// Approximate linear conversion for 12 dB attenuation near a 3.3 V full-scale input.
	return depth_sensor_raw_to_voltage_mv_approx(raw, SENSOR_ADC_BITWIDTH, SENSOR_ADC_APPROX_FULL_SCALE_MV);
}

static esp_err_t update_depth_sensor(depth_sensor_state_t *sensor)
{
	int raw = 0;
	ESP_RETURN_ON_ERROR(read_depth_sensor_raw(sensor, &raw), TAG, "Failed to read depth sensor %s", sensor->name);

	int voltage_mv = depth_sensor_raw_to_voltage_mv(sensor, raw);
	float current_mA = depth_sensor_voltage_to_current_ma(voltage_mv);
	float depth_mm = depth_sensor_voltage_to_depth_mm(voltage_mv);
	float depth_mm_rounded = roundf(depth_mm);

	sensor->values[sensor->current_index] = depth_mm_rounded;
	sensor->current_index = (sensor->current_index + 1) % MAX_VALUES;
	if (sensor->count < MAX_VALUES) sensor->count++;

	float avg_mm = roundf(calculate_average(sensor->values, sensor->count));

	ESP_LOGI(TAG, "%s depth: raw=%d, %d mV, %.2f mA, %.0f mm (avg %.0f mm)",
			 sensor->name, raw, voltage_mv, current_mA, depth_mm_rounded, avg_mm);

	esp_zb_lock_acquire(portMAX_DELAY);
	esp_zb_zcl_set_attribute_val(sensor->endpoint,
								 ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
								 ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID, &avg_mm, false);
	esp_zb_lock_release();
	return ESP_OK;
}

_Noreturn void pressure_task(void *pvParameters)
{
	while (true)
	{
		for (size_t i = 0; i < ARRAY_SIZE(s_depth_sensors); ++i)
		{
			esp_err_t res = update_depth_sensor(&s_depth_sensors[i]);
			if (res != ESP_OK)
			{
				ESP_LOGW(TAG, "Skipping %s update: %s", s_depth_sensors[i].name, esp_err_to_name(res));
			}
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

static esp_err_t create_app_task(TaskFunction_t task_function, const char *task_name,
								 configSTACK_DEPTH_TYPE stack_depth, void *task_arg,
								 UBaseType_t priority, TaskHandle_t *task_handle)
{
	if (!task_function || !task_name)
	{
		ESP_LOGE(TAG, "Invalid task configuration");
		return ESP_ERR_INVALID_ARG;
	}

	BaseType_t task_created = xTaskCreate(task_function, task_name, stack_depth, task_arg, priority, task_handle);
	if (task_created != pdPASS)
	{
		ESP_LOGE(TAG, "Failed to start task %s: xTaskCreate returned %ld", task_name, (long) task_created);
		return ESP_ERR_NO_MEM;
	}

	return ESP_OK;
}

static esp_err_t validate_depth_sensor_config(void)
{
	ESP_RETURN_ON_FALSE(ARRAY_SIZE(s_depth_sensors) > 0, ESP_ERR_INVALID_STATE, TAG,
						"At least one depth sensor must be configured");

	for (size_t i = 0; i < ARRAY_SIZE(s_depth_sensors); ++i)
	{
		ESP_RETURN_ON_FALSE(s_depth_sensors[i].name, ESP_ERR_INVALID_ARG, TAG,
							"Depth sensor %u is missing a name", (unsigned) i);
		ESP_RETURN_ON_FALSE(s_depth_sensors[i].endpoint > 0, ESP_ERR_INVALID_ARG, TAG,
							"Depth sensor %s has invalid endpoint 0", s_depth_sensors[i].name);

		for (size_t j = i + 1; j < ARRAY_SIZE(s_depth_sensors); ++j)
		{
			ESP_RETURN_ON_FALSE(s_depth_sensors[i].endpoint != s_depth_sensors[j].endpoint,
								ESP_ERR_INVALID_ARG, TAG,
								"Depth sensors %s and %s share endpoint %u",
								s_depth_sensors[i].name, s_depth_sensors[j].name,
								s_depth_sensors[i].endpoint);
			ESP_RETURN_ON_FALSE(s_depth_sensors[i].adc_channel != s_depth_sensors[j].adc_channel,
								ESP_ERR_INVALID_ARG, TAG,
								"Depth sensors %s and %s share ADC channel %d",
								s_depth_sensors[i].name, s_depth_sensors[j].name,
								s_depth_sensors[i].adc_channel);
		}
	}

	return ESP_OK;
}

static esp_err_t configure_depth_sensor_adc_channels(void)
{
	adc_oneshot_chan_cfg_t chan_cfg = {
			.bitwidth = SENSOR_ADC_BITWIDTH,
			.atten = SENSOR_ADC_ATTEN,
	};

	for (size_t i = 0; i < ARRAY_SIZE(s_depth_sensors); ++i)
	{
		ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_adc_handle, s_depth_sensors[i].adc_channel, &chan_cfg),
							TAG, "Failed to configure ADC channel %d for %s",
							s_depth_sensors[i].adc_channel, s_depth_sensors[i].name);

		int gpio_num;
		ESP_RETURN_ON_ERROR(adc_oneshot_channel_to_io(ADC_UNIT_1, s_depth_sensors[i].adc_channel, &gpio_num),
							TAG, "Failed to resolve GPIO for ADC channel %d",
							s_depth_sensors[i].adc_channel);
		ESP_RETURN_ON_ERROR(gpio_pullup_dis((gpio_num_t) gpio_num), TAG,
							"Failed to disable pull-up on GPIO%d", gpio_num);
		ESP_RETURN_ON_ERROR(gpio_pulldown_en((gpio_num_t) gpio_num), TAG,
							"Failed to enable pull-down on GPIO%d", gpio_num);
		ESP_LOGI(TAG, "Configured %s on endpoint %u using ADC channel %d (GPIO%d, pull-down enabled)",
				 s_depth_sensors[i].name, s_depth_sensors[i].endpoint,
				 s_depth_sensors[i].adc_channel, gpio_num);
	}

	return ESP_OK;
}

static esp_err_t deferred_driver_init(void)
{
	light_driver_init(LIGHT_DEFAULT_OFF);
	ESP_RETURN_ON_ERROR(validate_depth_sensor_config(), TAG, "Invalid depth sensor configuration");

	// Initialize ADC Oneshot driver (Unit 1)
	adc_oneshot_unit_init_cfg_t init_config = {
			.unit_id = ADC_UNIT_1,
			.ulp_mode = ADC_ULP_MODE_DISABLE,
	};
	ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&init_config, &s_adc_handle),
						TAG, "Failed to initialize ADC oneshot unit");

	ESP_RETURN_ON_ERROR(configure_depth_sensor_adc_channels(), TAG, "Failed to configure depth sensor ADC channels");

	// Try to enable calibration (curve fitting scheme)
	adc_cali_curve_fitting_config_t cali_config = {
			.unit_id = ADC_UNIT_1,
			.atten = SENSOR_ADC_ATTEN,
			.bitwidth = SENSOR_ADC_BITWIDTH,
	};
	if (adc_cali_create_scheme_curve_fitting(&cali_config, &s_adc_cali_handle) == ESP_OK) {
		s_adc_cali_enabled = true;
		ESP_LOGI(TAG, "ADC calibration enabled (curve fitting)");
	} else {
		s_adc_cali_enabled = false;
		ESP_LOGW(TAG, "ADC calibration not available; using approximate conversion");
	}

	ESP_RETURN_ON_ERROR(create_app_task(pressure_task, "pressure_task",
										configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL),
						TAG, "Failed to start pressure sensor task");

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
				esp_err_t init_status = deferred_driver_init();
				if (init_status != ESP_OK)
				{
					ESP_LOGE(TAG, "Deferred driver initialization failed: %s", esp_err_to_name(init_status));
					break;
				}
				ESP_LOGI(TAG, "Deferred driver initialization successful");
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
	while (true)
	{
		taskENTER_CRITICAL(&s_identify_lock);
		uint16_t identify_time = s_identify_time;
		if (identify_time == 0)
		{
			s_identify_task_handle = NULL;
		}
		taskEXIT_CRITICAL(&s_identify_lock);
		if (identify_time == 0)
		{
			break;
		}

		light_state = !light_state;
		light_driver_set_identify(true, light_state);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
	light_driver_set_identify(false, false);
	vTaskDelete(NULL);
}

static bool read_local_u16_attr(uint8_t endpoint, uint16_t cluster_id, uint16_t attr_id, uint16_t *value)
{
	esp_zb_zcl_attr_t *attr = esp_zb_zcl_get_attribute(endpoint, cluster_id, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, attr_id);
	if (!value || !attr || attr->type != ESP_ZB_ZCL_ATTR_TYPE_U16 || !attr->data_p)
	{
		ESP_LOGW(TAG, "Missing local u16 attribute: endpoint(%d), cluster(0x%x), attribute(0x%x)",
				 endpoint, cluster_id, attr_id);
		return false;
	}

	*value = *(uint16_t *)attr->data_p;
	return true;
}

static bool read_message_u16_attr(const esp_zb_zcl_set_attr_value_message_t *message, uint16_t *value)
{
	if (!value || !message->attribute.data.value || message->attribute.data.size < sizeof(uint16_t))
	{
		ESP_LOGW(TAG, "Invalid u16 attribute payload: attribute(0x%x), data size(%d)",
				 message->attribute.id, message->attribute.data.size);
		return false;
	}

	*value = *(uint16_t *)message->attribute.data.value;
	return true;
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
	esp_err_t ret = ESP_OK;
	bool light_state = 0;
	uint8_t light_level = 0;
	uint16_t light_color_x = 0;
	uint16_t light_color_y = 0;
	uint16_t identify_time = 0;
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
					if (!read_message_u16_attr(message, &light_color_x) ||
						!read_local_u16_attr(message->info.dst_endpoint, message->info.cluster,
											 ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_Y_ID, &light_color_y))
					{
						break;
					}
					ESP_LOGI(TAG, "Light color x changes to 0x%x", light_color_x);
					light_driver_set_color_xy(light_color_x, light_color_y);
				} else if (message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_Y_ID &&
						   message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16)
				{
					if (!read_message_u16_attr(message, &light_color_y) ||
						!read_local_u16_attr(message->info.dst_endpoint, message->info.cluster,
											 ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_X_ID, &light_color_x))
					{
						break;
					}
					ESP_LOGI(TAG, "Light color y changes to 0x%x", light_color_y);
					light_driver_set_color_xy(light_color_x, light_color_y);
				} else
				{
					ESP_LOGW(TAG, "Color control cluster data: attribute(0x%x), type(0x%x)", message->attribute.id,
							 message->attribute.data.type);
				}
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
				if (message->attribute.id == ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID &&
					message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16 &&
					read_message_u16_attr(message, &identify_time))
				{
					taskENTER_CRITICAL(&s_identify_lock);
					s_identify_time = identify_time;
					bool start_identify_task = s_identify_time > 0 && s_identify_task_handle == NULL;
					taskEXIT_CRITICAL(&s_identify_lock);
					if (start_identify_task)
					{
						ret = create_app_task(esp_zb_identify, "Identify", 4096, NULL, 5,
										  &s_identify_task_handle);
					}
				}
				break;
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
custom_primary_endpoint_clusters_create(esp_zb_analog_output_cluster_cfg_t *distance_sensor,
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

static esp_zb_cluster_list_t *
custom_depth_endpoint_clusters_create(esp_zb_analog_output_cluster_cfg_t *distance_sensor)
{
	esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
	ESP_ERROR_CHECK(esp_zb_cluster_list_add_analog_output_cluster(cluster_list,
																  esp_zb_analog_output_cluster_create(
																		  distance_sensor),
																  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
	return cluster_list;
}

static esp_zb_ep_list_t *
custom_distance_sensor_ep_create(esp_zb_analog_output_cluster_cfg_t *distance_sensors,
								 esp_zb_temperature_meas_cluster_cfg_t *temperature_sensor,
								 esp_zb_color_dimmable_light_cfg_t *light)
{
	esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
	esp_zb_endpoint_config_t endpoint_config = {
			.endpoint = s_depth_sensors[0].endpoint,
			.app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
			.app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
			.app_device_version = 0
	};
	esp_zb_ep_list_add_ep(ep_list,
						  custom_primary_endpoint_clusters_create(&distance_sensors[0], temperature_sensor, light),
						  endpoint_config);

	for (size_t i = 1; i < ARRAY_SIZE(s_depth_sensors); ++i)
	{
		esp_zb_endpoint_config_t depth_endpoint_config = {
				.endpoint = s_depth_sensors[i].endpoint,
				.app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
				.app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
				.app_device_version = 0
		};
		esp_zb_ep_list_add_ep(ep_list,
							  custom_depth_endpoint_clusters_create(&distance_sensors[i]),
							  depth_endpoint_config);
	}
	return ep_list;
}

static void esp_zb_task(void *pvParameters)
{
	/* Initialize Zigbee stack */
	esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZR_CONFIG();

	esp_zb_init(&zb_nwk_cfg);

	esp_zb_analog_output_cluster_cfg_t analog_cfg[ARRAY_SIZE(s_depth_sensors)];
	for (size_t i = 0; i < ARRAY_SIZE(s_depth_sensors); ++i)
	{
		analog_cfg[i] = (esp_zb_analog_output_cluster_cfg_t) {
				.out_of_service = false,
				.present_value = 0,
				.status_flags = 0
		};
	}
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
	esp_zb_ep_list_t *esp_zb_sensor_ep = custom_distance_sensor_ep_create(analog_cfg, &temp_cfg, &light_cfg);

	/* Register the device */
	esp_zb_device_register(esp_zb_sensor_ep);

	for (size_t i = 0; i < ARRAY_SIZE(s_depth_sensors); ++i)
	{
		esp_zb_zcl_reporting_info_t reporting_info = {
				.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
				.ep = s_depth_sensors[i].endpoint,
				.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT,
				.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
				.dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
				/* Periodic reporting: every 1..10 seconds */
				.u.send_info.min_interval = 1,
				.u.send_info.max_interval = 10,
				.u.send_info.def_min_interval = 1,
				.u.send_info.def_max_interval = 10,
				/* Neutralize delta to avoid type mismatch on float attribute; rely on periodic updates */
				.u.send_info.delta.u16 = 0,
				.attr_id = ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID,
				.manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
		};

		esp_zb_zcl_update_reporting_info(&reporting_info);
	}

	/* Also configure periodic reporting for Temperature Measurement (s16: value = degC * 100) */
	esp_zb_zcl_reporting_info_t temp_reporting_info = {
			.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
			.ep = HA_ESP_SENSOR_ENDPOINT,
			.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
			.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
			.dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
			/* Report every 30..300 seconds, or when change >= 0.5°C (50 in 0.01°C units) */
			.u.send_info.min_interval = 30,
			.u.send_info.max_interval = 300,
			.u.send_info.def_min_interval = 30,
			.u.send_info.def_max_interval = 300,
			.u.send_info.delta.u16 = 50,
			.attr_id = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
			.manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
	};

	esp_zb_zcl_update_reporting_info(&temp_reporting_info);

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
	ESP_ERROR_CHECK(create_app_task(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL));
}
