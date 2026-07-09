#ifndef DEPTH_SENSOR_DEPTH_MATH_H
#define DEPTH_SENSOR_DEPTH_MATH_H

#define DEPTH_SENSOR_PRESSURE_RANGE_MM 5000
#define DEPTH_SENSOR_SENSE_RESISTOR_OHMS 120

int depth_sensor_raw_to_voltage_mv_approx(int raw, int bitwidth, int full_scale_mv);
float depth_sensor_voltage_to_current_ma(int voltage_mv);
float depth_sensor_voltage_to_depth_mm(int voltage_mv);

#endif // DEPTH_SENSOR_DEPTH_MATH_H
