/*
 * tsdz_data.c
 *
 *  Created on: 2 set 2019
 *      Author: Max
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "tsdz_data.h"
#include "tsdz_nvs.h"
#include "tsdz_uart.h"
#include "tsdz_utils.h"

#define TAG "tsdz_data"

struct_tsdz_status tsdz_status = {
    .ui8_riding_mode = OFF_MODE,
    .ui8_assist_level = 0,
    .ui16_wheel_speed_x10 = 0,
    .ui8_pedal_cadence_RPM = 0,
	.ui16_motor_temperaturex10 = 220,
    .ui16_pedal_power_x10 = 0,
    .ui16_battery_voltage_x1000 = 36000,
    .ui8_battery_current_x10 = 0,
    .ui8_controller_system_state = 0,
    .ui8_braking = 0
};

struct_tsdz_debug tsdz_debug = {
    .ui8_adc_throttle = 0,
    .ui8_throttle = 0,
    .ui16_adc_pedal_torque_sensor = 0,
    .ui8_duty_cycle = 0,
    .ui16_motor_speed_erps = 0,
    .ui8_foc_angle = 0,
	.ui16_pedal_torque_x100 = 0,
	.ui16_cadence_sensor_pulse_high_percentage_x10 = 500
};

const struct_tsdz_cfg tsdz_default_cfg = {
  .ui8_motor_type = 1,
  .ui8_motor_temperature_min_value_to_limit = 65,
  .ui8_motor_temperature_max_value_to_limit = 80,
  .ui8_motor_acceleration = 0,
  .ui8_cadence_sensor_mode = STANDARD_MODE,
  .ui16_cadence_sensor_pulse_high_percentage_x10 = 500,
  .ui8_pedal_torque_per_10_bit_ADC_step_x100 = 67,
  .ui8_optional_ADC_function = 0,
  .ui8_assist_without_pedal_rotation_threshold = 0,
  .ui16_wheel_perimeter = 2300,
  .ui8_oem_wheel_divisor = 125,
  .ui16_battery_voltage_reset_wh_counter_x10 = 416, // 41.6V
  .ui8_battery_max_current = 15,
  .ui8_target_max_battery_power_div25 = 10,
  .ui8_battery_cells_number = 10,
  .ui16_battery_pack_resistance_x1000 = 180,
  .ui16_battery_low_voltage_cut_off_x10 = 290,
  .ui8_li_io_cell_overvolt_x100 = 225, // mV*10-200 (225 -> 200+225 -> 425 -> 4.25V)
  .ui8_li_io_cell_full_bars_x100 = 200,
  .ui8_li_io_cell_one_bar_x100 = 130,
  .ui8_li_io_cell_empty_x100 = 100,
  .ui8_street_mode_enabled = 0,
  .ui8_street_mode_power_limit_enabled = 0,
  .ui8_street_mode_throttle_enabled = 0,
  .ui8_street_mode_power_limit_div25 = 10,
  .ui8_street_mode_speed_limit = 25,
  .ui8_eMTB_assist_sensitivity = 10,
  .ui8_power_assist_level = {5,10,20,30},
  .ui8_torque_assist_level = {15,40,65,90},
  .ui8_walk_assist_level = {20,30,40,48},
  .ui8_esp32_temp_control = 0
};


// System Configuration parameters
struct_tsdz_cfg tsdz_cfg;

// OEM LCD values
static uint8_t ui8_oem_wheel_diameter;
static uint8_t ui8_oem_lights;
static uint8_t ui8_oem_wheel_max_speed;

// global system variables
static uint16_t   	ui16_battery_power_filtered_x10 = 100;
static uint32_t   	ui32_wh_sum_x10 = 0;
uint32_t      		ui32_wh_x10 = 0;
uint32_t      		ui32_wh_x10_offset = 0;

volatile uint8_t ui8_cadence_sensor_calibration = 0;

static uint8_t ui8_message_ID = 0;

static bool lcdMessageReceived = false;

uint32_t filter(uint32_t ui32_new_value, uint32_t ui32_old_value, uint8_t ui8_alpha);
int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
uint8_t battery_level(uint8_t* error_code, const uint16_t ui16_cell_voltage_x100);
void energy(void);

// called every 100 ms
void tsdz_data_update() {
	static uint32_t lastUpdateValue;

	// calculate Wh consumption
	energy();

	// save Wh consumed every 1 Wh increment
	if ((lastUpdateValue/10) != (ui32_wh_x10/10)) {
		lastUpdateValue = ui32_wh_x10;
		tsdz_nvs_update_whOffset();
	}
}


void processLcdMessage(const uint8_t lcd_oem_message[]) {
    switch(lcd_oem_message[1] & 0x5E) {
        case OEM_ASSIST_LEVEL4:
			tsdz_status.ui8_assist_level = 4;
			break;
        case OEM_ASSIST_LEVEL3:
			tsdz_status.ui8_assist_level = 3;
			break;
        case OEM_ASSIST_LEVEL2:
			tsdz_status.ui8_assist_level = 2;
			break;
        case OEM_ASSIST_LEVEL1:
			tsdz_status.ui8_assist_level = 1;
			break;
        case OEM_ASSIST_LEVEL0:
			tsdz_status.ui8_assist_level = 0;
			break;
    }

    // wheel diameter
    ui8_oem_wheel_diameter = lcd_oem_message[3];

    if (ui8_cadence_sensor_calibration) {
    	tsdz_status.ui8_riding_mode = CADENCE_SENSOR_CALIBRATION_MODE;
		goto skip;
    }

    // check if walk assist is set
    if (lcd_oem_message[1] & 0x20) {
		tsdz_status.ui8_riding_mode = WALK_ASSIST_MODE;
		goto skip;
    }

    // check the riding mode
	switch (ui8_oem_wheel_diameter) {
		case 27:
			tsdz_status.ui8_riding_mode = TORQUE_ASSIST_MODE;
			break;
		case 29:
			tsdz_status.ui8_riding_mode = eMTB_ASSIST_MODE;
			break;
		case 26:
		default:
			tsdz_status.ui8_riding_mode = POWER_ASSIST_MODE;
	}


    skip:

    // light status
	ui8_oem_lights = lcd_oem_message[1] & 0x01;

	// max speed
	ui8_oem_wheel_max_speed = lcd_oem_message[5];
	lcdMessageReceived = true;
}

void getLCDMessage(uint8_t ct_oem_message[]) {

	// used to set the Temerature Error blink speed according to the temperature
	static uint8_t temperatureError = 0;
	static uint8_t temperatureErrorOn = 0;
	static uint8_t temperatureErrorCounter = 0;

	uint16_t ui16_temp;
	uint8_t  ui8_working_status = 0;
	uint8_t  ui8_error_code;

	// start up byte
	ct_oem_message[0] = CT_MSG_ID;

	// battery level
	ui16_temp = (tsdz_status.ui16_battery_voltage_x1000 / 10 / (uint16_t)tsdz_cfg.ui8_battery_cells_number)-200;
	ct_oem_message[1] = battery_level(&ui8_error_code, ui16_temp);

	// undervoltage flag
	if (ui8_error_code == UNDERVOLTAGE) {
		ui8_error_code = OEM_NO_FAULT;
		ui8_working_status |= 0x01;
	}
	// hold display on flag
	if (tsdz_status.ui8_battery_current_x10 || tsdz_status.ui16_wheel_speed_x10)
		ui8_working_status |= 0x04;
	ct_oem_message[2] = ui8_working_status;

	// reserved
	ct_oem_message[3] = 0x46;
	ct_oem_message[4] = 0x46;

	// system status
	if(tsdz_status.ui8_controller_system_state == ERROR_MOTOR_BLOCKED)
		ui8_error_code = OEM_ERROR_MOTOR_BLOCKED;
	else if (tsdz_status.ui8_controller_system_state == ERROR_TORQUE_SENSOR)
		ui8_error_code = OEM_ERROR_TORQUE_SENSOR;
	else if (tsdz_status.ui8_controller_system_state == ERROR_CADENCE_SENSOR_CALIBRATION)
		ui8_error_code = OEM_ERROR_CADENCE_SENSOR_CALIBRATION;
	if (tsdz_cfg.ui8_esp32_temp_control || tsdz_cfg.ui8_optional_ADC_function == TEMPERATURE_CONTROL) {
		if (tsdz_status.ui16_motor_temperaturex10 >= tsdz_cfg.ui8_motor_temperature_max_value_to_limit*10) {
			// Temperature Error fixed
			temperatureError = 1;
			temperatureErrorOn = 1;
			temperatureErrorCounter = 0;
			ui8_error_code = OEM_ERROR_OVERTEMPERATURE;
		} else if (tsdz_status.ui16_motor_temperaturex10 > tsdz_cfg.ui8_motor_temperature_min_value_to_limit*10) {
			// Temperature Error blinking slow to fast
			if (temperatureError) {
				temperatureErrorCounter++;
				int counter = map(tsdz_status.ui16_motor_temperaturex10,
					 tsdz_cfg.ui8_motor_temperature_min_value_to_limit*10,
					 tsdz_cfg.ui8_motor_temperature_max_value_to_limit*10,
					 16,
					 4);
				if (temperatureErrorCounter > counter) {
					temperatureErrorCounter = 0;
					temperatureErrorOn = !temperatureErrorOn;
				}
				if (temperatureErrorOn)
					ui8_error_code = OEM_ERROR_OVERTEMPERATURE;
			} else {
				temperatureError = 1;
				temperatureErrorOn = 1;
				temperatureErrorCounter = 0;
				ui8_error_code = OEM_ERROR_OVERTEMPERATURE;
			}
		} else {
			temperatureError = 0;
		}
	}
	ct_oem_message[5] = ui8_error_code;

	// wheel speed
	if (tsdz_status.ui16_wheel_speed_x10 == 0) {
		ct_oem_message[6] = 0x07;
		ct_oem_message[7] = 0x07;
	} else {
		// calculate the nr. of clock ticks of OEM LCD for one wheel revolution (1 tick is 1/500 sec)
		// (3600/(ui16_wheel_speed_x10 * 100000)) * (ui8_oem_wheel_diameter * 25.4 * pi)  *    500)
		//              (sec/mm)                  *          (mm/rev)                 * (ticks/sec) = ticks/rev
		uint32_t tmp = (36 * ui8_oem_wheel_diameter * 798 * 5) / (tsdz_status.ui16_wheel_speed_x10 *100);
		ct_oem_message[6] = (uint8_t) tmp;
		ct_oem_message[7] = (uint8_t) (tmp >> 8);
	}

	ct_oem_message[8] = crc8(ct_oem_message, CT_OEM_MSG_BYTES - 1);
}


void processControllerMessage(const uint8_t ct_os_message[]) {
    // update motor controller status

    // battery voltage
    tsdz_status.ui16_battery_voltage_x1000 = (((uint16_t) ct_os_message[2]) << 8) + ((uint16_t) ct_os_message[1]);

    // battery current x10
    tsdz_status.ui8_battery_current_x10 = ct_os_message[3];

    // calculate battery Power filterd for Wh calcualtion
    uint32_t ui32_battery_power_temp_x10 = ((uint32_t) tsdz_status.ui16_battery_voltage_x1000 * tsdz_status.ui8_battery_current_x10) / 1000;
    ui16_battery_power_filtered_x10 = filter(ui32_battery_power_temp_x10, ui16_battery_power_filtered_x10, 72);

    // wheel speed
    tsdz_status.ui16_wheel_speed_x10 = (((uint16_t) ct_os_message[5]) << 8) + ((uint16_t) ct_os_message[4]);

    // brake state
    tsdz_status.ui8_braking = ct_os_message[6] & 1;

    // value from optional ADC channel
    tsdz_debug.ui8_adc_throttle = ct_os_message[7];

    // throttle or temperature control mapped from 0 to 255
    tsdz_debug.ui8_throttle = ct_os_message[8];

    // ADC pedal torque
    tsdz_debug.ui16_adc_pedal_torque_sensor = (((uint16_t) ct_os_message[10]) << 8) + ((uint16_t) ct_os_message[9]);

    // pedal cadence
    tsdz_status.ui8_pedal_cadence_RPM = ct_os_message[11];

    // PWM duty_cycle
    tsdz_debug.ui8_duty_cycle = ct_os_message[12];

    // motor speed in ERPS
    tsdz_debug.ui16_motor_speed_erps = (((uint16_t) ct_os_message[14]) << 8) + ((uint16_t) ct_os_message[13]);

    // FOC angle
    tsdz_debug.ui8_foc_angle = ct_os_message[15];

    // controller system state
    tsdz_status.ui8_controller_system_state = ct_os_message[16];

    // motor temperature
    if (!tsdz_cfg.ui8_esp32_temp_control)
    	tsdz_status.ui16_motor_temperaturex10 = ct_os_message[17]*10;

    if (tsdz_status.ui8_controller_system_state == NO_ERROR &&
    		(tsdz_cfg.ui8_esp32_temp_control || tsdz_cfg.ui8_optional_ADC_function == TEMPERATURE_CONTROL)) {
    	if (tsdz_status.ui16_motor_temperaturex10 >= tsdz_cfg.ui8_motor_temperature_max_value_to_limit*10)
    		tsdz_status.ui8_controller_system_state = ERROR_TEMPERATURE_MAX;
    	else if (tsdz_status.ui16_motor_temperaturex10 > tsdz_cfg.ui8_motor_temperature_min_value_to_limit*10) {
    		tsdz_status.ui8_controller_system_state = ERROR_TEMPERATURE_LIMIT;
    	}
    }

    // wheel_speed_sensor_tick_counter
    //tsdz_debug.ui32_wheel_speed_sensor_tick_counter = (((uint32_t) ct_os_message[20]) << 16) + (((uint32_t) ct_os_message[19]) << 8) + ((uint32_t) ct_os_message[18]);

    // pedal torque x100
    tsdz_debug.ui16_pedal_torque_x100 = (((uint16_t) ct_os_message[22]) << 8) + ((uint16_t) ct_os_message[21]);

    // human power x10
    tsdz_status.ui16_pedal_power_x10 = (((uint16_t) ct_os_message[24]) << 8) + ((uint16_t) ct_os_message[23]);

    // cadence sensor pulse high percentage
    tsdz_debug.ui16_cadence_sensor_pulse_high_percentage_x10 = (((uint16_t) ct_os_message[26]) << 8) + ((uint16_t) ct_os_message[25]);
}

bool getControllerMessage(uint8_t lcd_os_message[]) {
	//if (!lcdMessageReceived)
	//	return false;

    // start up byte
    lcd_os_message[0] = LCD_MSG_ID;

    // message ID
    lcd_os_message[1] = ui8_message_ID;

    // riding mode
    lcd_os_message[2] = tsdz_status.ui8_riding_mode;

    // riding mode parameter
    switch (tsdz_status.ui8_riding_mode) {
		case POWER_ASSIST_MODE:
			if (tsdz_status.ui8_assist_level > 0) {
				lcd_os_message[3] = tsdz_cfg.ui8_power_assist_level[tsdz_status.ui8_assist_level - 1];
			} else {
				lcd_os_message[3] = 0;
			}
			break;
		case TORQUE_ASSIST_MODE:
			if (tsdz_status.ui8_assist_level > 0) {
				lcd_os_message[3] = tsdz_cfg.ui8_torque_assist_level[tsdz_status.ui8_assist_level - 1];
			} else {
				lcd_os_message[3] = 0;
			}
			break;
		case eMTB_ASSIST_MODE:
			lcd_os_message[3] = tsdz_cfg.ui8_eMTB_assist_sensitivity;
			break;
		case WALK_ASSIST_MODE:
			if (tsdz_status.ui8_assist_level > 0) {
				lcd_os_message[3] = tsdz_cfg.ui8_walk_assist_level[tsdz_status.ui8_assist_level - 1];
			} else {
				lcd_os_message[3] = 0;
			}
			break;
		default:
			lcd_os_message[3] = 0;
			break;
    }

    // set lights state
    lcd_os_message[4] = ui8_oem_lights;

    switch (ui8_message_ID) {
		case 0:
			// battery low voltage cut off x10
			lcd_os_message[5] = (uint8_t) (tsdz_cfg.ui16_battery_low_voltage_cut_off_x10 & 0xff);
			lcd_os_message[6] = (uint8_t) (tsdz_cfg.ui16_battery_low_voltage_cut_off_x10 >> 8);

			// wheel max speed
			if (tsdz_cfg.ui8_street_mode_enabled) {
				lcd_os_message[7] = tsdz_cfg.ui8_street_mode_speed_limit;
			} else {
				lcd_os_message[7] = ui8_oem_wheel_max_speed;
			}
			break;
		case 1:
			// wheel perimeter
			lcd_os_message[5] = (uint8_t) (tsdz_cfg.ui16_wheel_perimeter & 0xff);
			lcd_os_message[6] = (uint8_t) (tsdz_cfg.ui16_wheel_perimeter >> 8);

			// optional ADC function, disable throttle if set to be disabled in Street Mode
			if (tsdz_cfg.ui8_street_mode_enabled && !tsdz_cfg.ui8_street_mode_throttle_enabled && tsdz_cfg.ui8_optional_ADC_function == THROTTLE_CONTROL) {
				lcd_os_message[7] = 0;
			} else {
				lcd_os_message[7] = tsdz_cfg.ui8_optional_ADC_function;
			}
			break;
		case 2:
			// set motor type
			lcd_os_message[5] = tsdz_cfg.ui8_motor_type;
			// motor over temperature min value limit
			lcd_os_message[6] = tsdz_cfg.ui8_motor_temperature_min_value_to_limit;
			// motor over temperature max value limit
			lcd_os_message[7] = tsdz_cfg.ui8_motor_temperature_max_value_to_limit;
			break;

		case 3:
			lcd_os_message[5] = 0;
			lcd_os_message[6] = 0;
			lcd_os_message[7] = 0;
			break;
		case 4:
			// lights configuration
			lcd_os_message[5] = tsdz_cfg.ui8_lights_configuration;

			// assist without pedal rotation threshold
			lcd_os_message[6] = tsdz_cfg.ui8_assist_without_pedal_rotation_threshold;

			// motor acceleration adjustment
			lcd_os_message[7] = tsdz_cfg.ui8_motor_acceleration;
			break;
		case 5:
			// pedal torque conversion
			lcd_os_message[5] = tsdz_cfg.ui8_pedal_torque_per_10_bit_ADC_step_x100;

			// max battery current in amps
			if (tsdz_cfg.ui8_esp32_temp_control) {
				lcd_os_message[6] = map((uint32_t) tsdz_status.ui16_motor_temperaturex10,
						(uint32_t) tsdz_cfg.ui8_motor_temperature_min_value_to_limit * 10,
						(uint32_t) tsdz_cfg.ui8_motor_temperature_max_value_to_limit * 10,
						(uint32_t) tsdz_cfg.ui8_battery_max_current,
						(uint32_t) 0);
			} else
				lcd_os_message[6] = tsdz_cfg.ui8_battery_max_current;

			// battery power limit
			if (tsdz_cfg.ui8_street_mode_enabled && tsdz_cfg.ui8_street_mode_power_limit_enabled) {
				lcd_os_message[7] = tsdz_cfg.ui8_street_mode_power_limit_div25;
			} else {
				lcd_os_message[7] = tsdz_cfg.ui8_target_max_battery_power_div25;
			}
			break;
		case 6:
			// cadence sensor mode
			if (ui8_cadence_sensor_calibration)
				lcd_os_message[5] = CALIBRATION_MODE;
			else
				lcd_os_message[5] = tsdz_cfg.ui8_cadence_sensor_mode;

			// cadence sensor pulse high percentage
			uint16_t ui16_temp = tsdz_cfg.ui16_cadence_sensor_pulse_high_percentage_x10;
			lcd_os_message[6] = (uint8_t) (ui16_temp & 0xff);
			lcd_os_message[7] = (uint8_t) (ui16_temp >> 8);
			break;
		default:
			ui8_message_ID = 0;
			break;
    }

    // prepare crc of the package
    uint16_t ui16_crc_tx = 0xffff;
    uint8_t ui8_i;

    for (ui8_i = 0; ui8_i < LCD_OS_MSG_BYTES-2; ui8_i++) {
    	crc16 (lcd_os_message[ui8_i], &ui16_crc_tx);
    }

    lcd_os_message[LCD_OS_MSG_BYTES-2] = (uint8_t) (ui16_crc_tx & 0xff);
    lcd_os_message[LCD_OS_MSG_BYTES-1] = (uint8_t) (ui16_crc_tx >> 8) & 0xff;

    if (++ui8_message_ID > 6) { ui8_message_ID = 0; }

    lcdMessageReceived = false;
    return true;
}


int tsdz_update_cfg(struct_tsdz_cfg *new_cfg) {
	if (memcmp(new_cfg, &tsdz_cfg, sizeof(struct _tsdz_cfg)) == 0 ) {
		ESP_LOGI(TAG,"tsdz_update_cfg NO CHANGE");
		return 0;
	} else {
		if ((new_cfg->ui8_motor_type > 3) ||
			(new_cfg->ui8_cadence_sensor_mode > CALIBRATION_MODE) ||
			(new_cfg->ui8_optional_ADC_function > 2) ||
			(new_cfg->ui8_battery_cells_number > 15) ||
			(new_cfg->ui8_street_mode_enabled > 1) ||
			(new_cfg->ui8_street_mode_power_limit_enabled > 1) ||
			(new_cfg->ui8_street_mode_power_limit_enabled > 1)) {
			ESP_LOGI(TAG,"tsdz_update_cfg VALUES OUT OF RANGE");
			return 1;
		}
		memcpy(&tsdz_cfg, new_cfg, sizeof(struct _tsdz_cfg));
		tsdz_nvs_update_cfg();
		ESP_LOGI(TAG,"tsdz_update_cfg OK");
		return 0;
	}
}


void energy(void)
{
	static uint8_t ui8_wh_reset;

	// reset watt-hour value if battery voltage is over threshold set from user, but only do this once for every power on
	if (((tsdz_status.ui16_battery_voltage_x1000) > (tsdz_cfg.ui16_battery_voltage_reset_wh_counter_x10 * 100)) && (!ui8_wh_reset))
	{
		ui8_wh_reset = 1;
		ui32_wh_x10_offset = 0;
	}

	ui32_wh_sum_x10 += ui16_battery_power_filtered_x10;

	// calculate watt-hours since last full charge
	ui32_wh_x10 = ui32_wh_x10_offset + (ui32_wh_sum_x10 / 36000);
	tsdz_status.ui16_battery_wh = (uint16_t)(ui32_wh_x10 / 10);
}


uint8_t battery_level(uint8_t* error_code, const uint16_t ui16_cell_voltage_x100) {
	*error_code = 0;
	if (ui16_cell_voltage_x100 > tsdz_cfg.ui8_li_io_cell_overvolt_x100) {
		// level full + overvoltage
		*error_code = OEM_ERROR_OVERVOLTAGE;
		return 0x0C;
	} else if (ui16_cell_voltage_x100 > tsdz_cfg.ui8_li_io_cell_full_bars_x100) {
		// level full
		return 0x0C;
	} else if (ui16_cell_voltage_x100 < tsdz_cfg.ui8_li_io_cell_empty_x100) {
		// level 0 + undervoltage
		*error_code = UNDERVOLTAGE; // battery undervoltage
		return 0x00;
	} else if (ui16_cell_voltage_x100 < tsdz_cfg.ui8_li_io_cell_one_bar_x100) {
		// level 1
		return 0x01;
	} else {
		return map(ui16_cell_voltage_x100,
			  tsdz_cfg.ui8_li_io_cell_one_bar_x100,
			  tsdz_cfg.ui8_li_io_cell_full_bars_x100,
			  2,
			  12);
	}
}