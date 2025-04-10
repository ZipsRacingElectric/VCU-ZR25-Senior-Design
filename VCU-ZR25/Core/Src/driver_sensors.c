/*
 * driver_sensors.c
 *
 * This handles all functions relating to measuring driver input sensors.
 *
 *  Created on: Nov 13, 2024
 *      Author: bglen
 */

/*
 * TODO:
 * - why are there two apps and bps for one pedal
 * - calibrate_apps: Floor min/max
 * - init_driver_input: handle HAL Error, pass adc handle to sensor structs
 * - validate_apps: report different plausibility faults in the future
 * - BPS calibration
 * - Steering angle calibration
 * - Steering angle dead zone
 * - filtering
 * - unit testing
 * - better pedal algorithm
 */

// Includes
#include "driver_sensors.h"
#include "vehicle_fsm.h"
#include "am4096_encoder.h"
#include "vehicle_data.h"
#include <math.h>
#include "usbd_cdc_if.h"

// Static variables
APPSSensor_t s_apps = {.raw_value_1 = 0, .raw_value_2 = 0, .voltage_1 = 0, .voltage_2 = 0, .percent_1 = 0, .percent_2 = 0, .percent = 0, .pedal_travel = false, .plausible = false};
BPSSensor_t s_bps_front = {.raw_value = 0, .voltage = 0, .pressure = 0, .plausible = false, .brakes_engaged = false};
BPSSensor_t s_bps_rear = {.raw_value = 0, .voltage = 0, .pressure = 0, .plausible = false, .brakes_engaged = false};
SteeringAngleSensor_t s_steering_angle = {.angle = 0, .angular_velocity = 0, .plausible = false};

// Static calibration variables. These hold the 0% and 100% setpoints
static uint16_t apps_1_min = 0600; // V * 1000
static uint16_t apps_1_max = 4400; // V * 1000
static uint16_t apps_2_min = 0350; // V * 1000
static uint16_t apps_2_max = 2150; // V * 1000

// Private Function Prototypes
static uint16_t read_adc(ADC_HandleTypeDef *adc, uint32_t channel);
static uint16_t adc_to_voltage(uint16_t adc_value);
static void calc_apps_percent(APPSSensor_t *apps);
static void calc_bps_pressure(BPSSensor_t *bps);
static uint16_t abs_diff(uint16_t v1, uint16_t v2);
static bool validate_apps(APPSSensor_t apps);
static bool validate_bps(BPSSensor_t bps);
static bool validate_steering_angle(SteeringAngleSensor_t steering_angle);

// Public Functions
void update_driver_sensor_data(){
	osMutexAcquire(vdb_apps_lockHandle, osWaitForever);
	VehicleData.apps = s_apps;
	osMutexRelease(vdb_apps_lockHandle);

	osMutexAcquire(vdb_bps_front_lockHandle, osWaitForever);
	VehicleData.bps_front = s_bps_front;
	osMutexRelease(vdb_bps_front_lockHandle);

	osMutexAcquire(vdb_bps_rear_lockHandle, osWaitForever);
	VehicleData.bps_rear = s_bps_rear;
	osMutexRelease(vdb_bps_rear_lockHandle);

	osMutexAcquire(vdb_sas_lockHandle, osWaitForever);
	VehicleData.sas = s_steering_angle;
	osMutexRelease(vdb_sas_lockHandle);
}

void StartDriverSensorTask(
	void *void_args
){
	DriverSensorTaskArgs_t* args = (DriverSensorTaskArgs_t*)void_args;
	ADC_HandleTypeDef hadc1 = args->hadc1;
	while (1) {
		read_driver_input(&hadc1);
		print_driver_input();
		update_driver_sensor_data();
		fsm_sensor_callback();

		osDelay(50);
	}
}

/*
 * Initializes the driver input sensors
 */
void init_driver_input(I2C_HandleTypeDef *i2c)
{
	// Initialize the AM4096
	(void)am4096_init(&s_steering_angle.i2c_device, i2c);
}

/*
 * Reads in the raw sensor data and updates the sensor variables
 */
void read_driver_input(ADC_HandleTypeDef *adc)
{
    // Read Raw ADC Values
	s_apps.raw_value_1 = read_adc(adc, APPS_1_CHANNEL);
    s_apps.raw_value_2 = read_adc(adc, APPS_2_CHANNEL);
    s_bps_front.raw_value = read_adc(adc, BPS_FRONT_CHANNEL);
    s_bps_rear.raw_value = read_adc(adc, BPS_REAR_CHANNEL);

    // Get raw steering angle values
    (void)am4096_read_angle(&s_steering_angle.i2c_device);
    (void)am4096_read_angular_velocity(&s_steering_angle.i2c_device);

    // Convert Raw Values to Voltages
    s_apps.voltage_1 = adc_to_voltage(s_apps.raw_value_1);
    s_apps.voltage_2 = adc_to_voltage(s_apps.raw_value_2);
    s_bps_front.voltage = adc_to_voltage(s_bps_front.raw_value);
    s_bps_rear.voltage = adc_to_voltage(s_bps_rear.raw_value);

    // Convert Voltages to Physical Values
    (void)calc_apps_percent(&s_apps);
    (void)calc_bps_pressure(&s_bps_front);
    (void)calc_bps_pressure(&s_bps_rear);

    // Perform Plausibility Checking
    // Handling of plausibility is outside the scope of measuring driver input sensors
    s_apps.plausible = validate_apps(s_apps);
    s_bps_front.plausible = validate_bps(s_bps_front);
    s_bps_rear.plausible = validate_bps(s_bps_rear);
    s_steering_angle.plausible = validate_steering_angle(s_steering_angle);
}
// Format data to send over USB
char msg_buffer[4096];

void print_driver_input(void)
{
	GPSState_t gps;
	VCU_State_t fsm_state;

	osMutexAcquire(vdb_gps_lockHandle, osWaitForever);
	gps = VehicleData.gps;
	fsm_state = VehicleData.fsm_state;
	osMutexRelease(vdb_gps_lockHandle);

	int length = snprintf(msg_buffer, sizeof(msg_buffer),
		"\nAccelerator Pedal:\n"
		"- Raw Value 1: %u\n"
		"- Raw Value 2: %u\n"
		"- Voltage 1: %u mV\n"
		"- Voltage 2: %u mV\n"
		"- Pedal Percentage: %u percent * 10\n"
		"- Channel 1: %u percent * 10\n"
		"- Channel 2: %u percent * 10\n"
		"- APPS Plausibility: %d\n"
		"\n"
		"Brake Pressure:\n"
		"- Raw Value Front: %u\n"
		"- Raw Value Rear: %u\n"
		"- Voltage Front: %u mV\n"
		"- Voltage Rear: %u mV\n"
		"- Pressure Front: %u PSI\n"
		"- Pressure Rear: %u PSI\n"
		"- Plausibility Front: %d\n"
		"- Plausibility Rear: %d\n"
		"\n"
		"Steering Angle:\n"
		"- Device status: %u\n"
		"- Angle: %u radians * 1000\n"
		"- Plausibility Front: %d\n"
		"\n"
		"FSM\n"
		"- Vehicle state: %s\n"
		"\n"
		"GPS\n"
		"- Latitude: %f\n"
		"- Longitude: %f\n"
		"- Speed: %f km/h\n"
			,
		// APPS
		s_apps.raw_value_1,
		s_apps.raw_value_2,
		s_apps.voltage_1,
		s_apps.voltage_2,
		s_apps.percent,
		s_apps.percent_1,
		s_apps.percent_2,
		(uint8_t)s_apps.plausible,

		// BPS
		s_bps_front.raw_value,
		s_bps_rear.raw_value,
		s_bps_front.voltage,
		s_bps_rear.voltage,
		s_bps_front.pressure,
		s_bps_rear.pressure,
		(uint8_t)s_bps_front.plausible,
		(uint8_t)s_bps_rear.plausible,

		// SAS
		s_steering_angle.i2c_device.device_status,
		s_steering_angle.angle,
		(uint8_t)s_steering_angle.plausible,

		// FSM
		fsm_state_string(fsm_state),

		// GPS
		(float)gps.latitude * 1e-7,
		(float)gps.longitude * 1e-7,
		(float)gps.speed * 1e-3 / 36.0
	);

	// Ensure snprintf was successful and message length is valid
	if (length > 0 && length < sizeof(msg_buffer)) {
	  // Send only the formatted message length over USB
	  CDC_Transmit_FS((uint8_t *)msg_buffer, length);
	} else {
	  // Handle error in formatting or length (optional)
	}
}

void fsm_sensor_callback(void){
	uint8_t flag;
	uint8_t value;
		if ((s_bps_front.raw_value > BPS_MIN_VOLTAGE)
				| (s_bps_rear.raw_value > BPS_MIN_VOLTAGE)) {
			HAL_GPIO_WritePin(GPIOB, BRAKE_LIGHT_CONTROL_Pin, GPIO_PIN_SET);
			flag = FLAG_INDEX_BRAKE_PRESSED;
			value = 1;
			fsm_flag_callback(flag, value);
		}
		else{
			HAL_GPIO_WritePin(GPIOB, BRAKE_LIGHT_CONTROL_Pin, GPIO_PIN_RESET);
			flag = FLAG_INDEX_BRAKE_PRESSED;
			value = 0;
			fsm_flag_callback(flag, value);
		}
}
/*
 * Returns an APPSSensor_t struct with the current APPS data in it
 */
APPSSensor_t get_apps_data(void)
{
	return s_apps;
}

/*
 * Returns an APPSSensor_t struct with the current APPS data in it
 */
BPSSensor_t get_bps_front_data(void)
{
	return s_bps_front;
}

/*
 *Returns an BPSSensor_t struct with the current front BPS data in it
 */
BPSSensor_t get_bps_rear_data(void)
{
	return s_bps_rear;
}

/*
 * Returns an BPSSensor_t struct with the current rear BPS data in it
 */
SteeringAngleSensor_t get_steering_angle_data(void)
{
	return s_steering_angle;
}

/*
 * Calibrates the constants used to determine APPS position. Returns false if calibration failed.
 */
bool calibrate_apps(uint16_t apps_1_pedal_min, uint16_t apps_1_pedal_max, uint16_t apps_2_pedal_min, uint16_t apps_2_pedal_max)
{
	// Check if calibration values are in a valid range
	if (apps_1_pedal_min > apps_1_pedal_max || apps_2_pedal_min > apps_2_pedal_max)
	{
		return false;
	}

	// Apply limits that are 10% beyond what the driver recorded during calibration
	apps_1_min = apps_1_pedal_min;
	apps_2_min = apps_2_pedal_min;
	apps_1_max = apps_1_pedal_max;
	apps_2_max = apps_2_pedal_max;

	return true;
}

/*
 * Calibrates the constants used to determine BPS pressure, percent braking, brake light setpoint, and hard braking condition
 */
bool calibrate_bps(void)
{
	return false;
}

/*
 * Calibrates the constants used to determine steering angles
 */
bool calibrate_steering(void)
{
	return false;
}

// Private Functions

/*
 * Reads in the raw ADC values from hardware
 */
static uint16_t read_adc(ADC_HandleTypeDef *adc, uint32_t channel)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	// Set the ADC channel
	sConfig.Channel = channel;
	sConfig.Rank = 1; // Regular group rank
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;

	if (HAL_ADC_ConfigChannel(adc, &sConfig) != HAL_OK)
	{
		// Return error value
		return 0xFFFF;
	}

	// Read ADC value
	HAL_ADC_Start(adc);
	if (HAL_ADC_PollForConversion(adc, HAL_MAX_DELAY) != HAL_OK)
	{
		// Return error value
		return 0xFFFF;
	}

	return HAL_ADC_GetValue(adc);
}

/*
 * Convert raw 12-bit ADC readings to voltages in V * 1000
 */
static uint16_t adc_to_voltage(uint16_t adc_value)
{
	// Multiply adc_value by the reference voltage in millivolts
	// Then divide by the maximum ADC value to get the voltage
    uint32_t mv = ((uint32_t)adc_value * ADC_REF_VOLTAGE) / ADC_MAX_VALUE;

    // Convert from 3.3V to 5V caused by voltage dividers
    mv = (mv * 5000) / 3300;

	return (uint16_t)mv;
}

/*
 * Calculate individual APPS channel percentage and final pedal percentage
 */
static void calc_apps_percent(APPSSensor_t *apps)
{
	// grab apps voltages
	uint32_t v1 = (uint32_t)apps->voltage_1;
	uint32_t v2 = (uint32_t)apps->voltage_2;

	// Calculate voltage setpoints considering the pedal deadzone
	uint32_t apps_1_v_min = apps_1_min + ((uint32_t)(abs_diff(apps_1_max, apps_1_min) * APPS_DEADZONE) / 1000);
	uint32_t apps_1_v_max = apps_1_max - ((uint32_t)(abs_diff(apps_1_max, apps_1_min) * APPS_DEADZONE) / 1000);
	uint32_t apps_2_v_min = apps_2_min + ((uint32_t)(abs_diff(apps_2_max, apps_2_min) * APPS_DEADZONE) / 1000);
	uint32_t apps_2_v_max = apps_2_max - ((uint32_t)(abs_diff(apps_2_max, apps_2_min) * APPS_DEADZONE) / 1000);

	// Default percent is zero, accounts for <= apps_v_min condition
	uint32_t p1 = 0;
	uint32_t p2 = 0;

	// Condition: voltage is within the linear range
	if (v1 > apps_1_v_min && v1 < apps_1_v_max)
	{
		// Multiply by 1000 to get percentage * 10, then divide by voltage range defined by calibration
		p1 = ((v1 - apps_1_v_min) * 1000) / (apps_1_v_max - apps_1_v_min);
	}

	// Condition: voltage is within the linear range
	if (v2 > apps_2_v_min && v2 < apps_2_v_max)
	{
		// Multiply by 1000 to get percentage * 10, then divide by voltage range defined by calibration
		p2 = ((v2 - apps_2_v_min) * 1000) / (apps_2_v_max - apps_2_v_min);
	}

	// Condition: voltage is above upper limit and into/past the deadzone
	if(v1 > apps_1_v_max)
	{
		// Limit to 100%
		p1 = 1000;
	}

	// Condition: voltage is above upper limit and into/past the deadzone
	if(v2 > apps_2_v_max)
	{
		// Limit to 100%
		p2 = 1000;
	}

	// Averaging allows a usable pedal if one channel is implausible
	uint32_t pedal_percent = (p1 + p2) * 1000 / 2000;


	apps->percent_1 = (uint16_t)p1;
	apps->percent_2 = (uint16_t)p2;
	apps->percent = (uint16_t)pedal_percent;
}

/*
 * Calculate the pressure from the BPS sensor voltage
 */
static void calc_bps_pressure(BPSSensor_t *bps)
{
    // Condition: voltage < min voltage
	uint32_t pressure = 0;

	// Condition: voltage in linear range
    if ((bps->voltage - BPS_DEADZONE) > BPS_MIN_VOLTAGE && (bps->voltage + BPS_DEADZONE) < BPS_MAX_VOLTAGE)
    {
    	pressure = ((uint32_t)bps->voltage - (BPS_MIN_VOLTAGE + BPS_DEADZONE)) * (BPS_MAX_PRESSURE  * 1000 / (BPS_MAX_VOLTAGE - BPS_DEADZONE)) / 1000;
    }

    // Condition: voltage > max voltage
    if (bps->voltage > BPS_MAX_VOLTAGE)
    {
    	pressure = BPS_MAX_PRESSURE;
    }

    bps->pressure = (uint16_t)pressure;
}

/*
 * Calculates absolute difference of unsigned integer to avoid wrap around
 */
static uint16_t abs_diff(uint16_t v1, uint16_t v2)
{
    uint16_t delta;

    if (v1 > v2) {
        delta = v1 - v2;
    } else {
        delta = v2 - v1;
    }

    return delta;
}

/*
 * APPS Plausibility Check
 * - plausibility conditions are checked here
 * - disabling of the powertrain after 100 ms is not handled here, out of scope
 */
bool validate_apps(APPSSensor_t apps)
{
		// Check for short to ground
	    if (apps.voltage_1 < APPS_1_MIN_VOLTAGE || apps.voltage_2 < APPS_2_MIN_VOLTAGE)
	    {
	        return false;
	    }

	    // Check for short to power
	    if (apps.voltage_1 > APPS_1_MAX_VOLTAGE || apps.voltage_2 > APPS_2_MAX_VOLTAGE)
	    {
	        return false;
	    }

	    // Check for short to sensor output
	    if ((uint16_t)abs_diff(apps.voltage_1, apps.voltage_2) < APPS_MIN_DELTA_V)
	    {
	        return false;
	    }

	    // Check for out of range for APPS1
	    if (apps.voltage_1 < (apps_1_min - APPS_OUT_OF_RANGE) || apps.voltage_1 > (apps_1_max + APPS_OUT_OF_RANGE))
	    {
	        return false;
	    }

	    // Check for out of range for APPS2
	    if (apps.voltage_2 < (apps_1_min - APPS_OUT_OF_RANGE) || apps.voltage_2 > (apps_1_max + APPS_OUT_OF_RANGE))
	    {
	        return false;
	    }

	    // Check for a significant deviation between APPS1 and APPS2 calculated percentages
	    if ((uint16_t)abs_diff(apps.percent_1, apps.percent_2) > APPS_MAX_DELTA_P)
	    {
	    	return false;
	    }

	    // Check for pedal travel greater than 25%
	    if (apps.percent >= APPS_PEDAL_MAX){
	    	apps.pedal_travel = true;
	    	return false;
	    }
	    else if (apps.percent <= APPS_PEDAL_MIN)
	    {
	    	apps.pedal_travel = false;
	    }

	    // If none of the conditions are met, plausibility is valid
	    return true;
}

/*
 * BPS Plausibility Check
 */
bool validate_bps(BPSSensor_t bps)
{
	// Check if voltage out of expected range, with a deadzone for noise
	if ((bps.voltage + BPS_DEADZONE) >= BPS_MIN_VOLTAGE && (bps.voltage - BPS_DEADZONE) <= BPS_MAX_VOLTAGE)
	{
		return true;
	}

	if ((bps.voltage + BPS_DEADZONE) >= BPS_ENGAGED_VOLTAGE_THRESHOLD){
		bps.brakes_engaged = true;
		return false;
	}
	else {
		bps.brakes_engaged = false;
	}

    return false;
}

/*
 * Steering Angle Plausibility Check
 */
bool validate_steering_angle(SteeringAngleSensor_t steering_angle)
{
	// Check if the calculated angle is outside the expected range
	if (steering_angle.angle >= STEERING_MIN_ANGLE && steering_angle.angle <= STEERING_MAX_ANGLE)
	{
		return false;
	}

	return true;
}

