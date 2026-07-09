#include "depth_math.h"

#include <stdint.h>

int depth_sensor_raw_to_voltage_mv_approx(int raw, int bitwidth, int full_scale_mv)
{
    if (raw <= 0 || bitwidth <= 0 || bitwidth >= 31 || full_scale_mv <= 0) {
        return 0;
    }

    const uint32_t full_scale_raw = (1U << bitwidth) - 1U;
    if ((uint32_t)raw >= full_scale_raw) {
        return full_scale_mv;
    }

    return (int)(((uint64_t)raw * (uint32_t)full_scale_mv + full_scale_raw / 2U) / full_scale_raw);
}

float depth_sensor_voltage_to_current_ma(int voltage_mv)
{
    return (float)voltage_mv / (float)DEPTH_SENSOR_SENSE_RESISTOR_OHMS;
}

float depth_sensor_voltage_to_depth_mm(int voltage_mv)
{
    const float current_ma = depth_sensor_voltage_to_current_ma(voltage_mv);
    float depth_mm = (current_ma - 4.0f) * ((float)DEPTH_SENSOR_PRESSURE_RANGE_MM / 16.0f);

    if (depth_mm < 0.0f) {
        return 0.0f;
    }
    if (depth_mm > (float)DEPTH_SENSOR_PRESSURE_RANGE_MM) {
        return (float)DEPTH_SENSOR_PRESSURE_RANGE_MM;
    }

    return depth_mm;
}
