/*
	Copyright 2019 - 2020 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC BMS firmware.

	The VESC BMS firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC BMS firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "hw_luna_bms.h"
#include "pwr.h"
#include "terminal.h"
#include "commands.h"
#include "bq76940.h"
#include "bms_if.h"
#include "sleep.h"

static void terminal_cmd_shipmode(int argc, const char **argv);
static void terminal_cmd_connect(int argc, const char **argv);

static THD_WORKING_AREA(precharge_thread_wa, 512);
static THD_FUNCTION(precharge_thread, arg);

void hw_luna_init(void){
	bq76940_init();
	HW_SET_CAN_ENABLE_LINE();
	HW_CAN_ON();
	terminal_register_command_callback("shipmode", "Shipmode = turn off bq 76940", 0, terminal_cmd_shipmode);	
	terminal_register_command_callback("Connect", "Connect=turn on big mosfets", 0, terminal_cmd_connect);

	// Config precharge pins
	palSetLineMode(ADC_PRECHARGE_I_LINE, PAL_MODE_INPUT_ANALOG);    // hardware under process, uncomment for hardware V3
	palSetLineMode(ADC_PRECH_RES_TEMP_LINE, PAL_MODE_INPUT_ANALOG);
	palSetLineMode(PRECHARGE_ENABLE_LINE, PAL_MODE_OUTPUT_PUSHPULL);
	//PWR->PUCRB |= PWR_PUCRB_PB0; //PB0 (precharge pin pull up during standby
	//PWR->CR3 |= PWR_CR3_APC;	// apply pull up configuration
	chThdCreateStatic(precharge_thread_wa, sizeof(precharge_thread_wa), NORMALPRIO +3 , precharge_thread, NULL);
}

float hw_luna_get_cell_temp_max(void) {
	float temp_max = -100.0;

	for(int i = 0; i<4 ;i++) {
		float temp = hw_luna_get_temp(i);
		if(temp > temp_max) {
			temp_max = temp;
		}
	}
	return temp_max;
}

float hw_luna_get_cell_temp_min(void) {
	float temp_min = 200.0;

	for(int i = 0; i<4 ;i++) {
		float temp = hw_luna_get_temp(i);
		if(temp < temp_min) {
			temp_min = temp;
		}
	}
	return temp_min;
}

float hw_luna_get_temp(int sensors){
// hardware has 8 temperature sensors (plus internal AFE sensor):
// T[0]: cell temperature TC1
// T[1]: cell temperature TC2
// T[2]: cell temperature TC3
// T[3]: cell temperature TC4
// T[4]: Negative Connector terminal temperature
// T[5]: Positive Connector terminal temperature
// T[6]: MOSFET temperature
// T[7]: Linear Voltage regulator temperature

	float temp = -1;

	if(sensors <= 6) {
		// temperatures measured by the MCU ADC
		temp = pwr_get_temp(sensors);
	} else {
		if(sensors == 7) {
			// temperature measured by the AFE ADC
			temp = bq_get_temp(2);  //Linear reg temp
			}
	}
	return temp;
}
//return the highest temp between the internal sensor of AFE and pre regulator
float hw_luna_get_bal_temp (void)
{
	float reg_temp,AFE_temp;

	AFE_temp = bq_get_temp_ic();
	reg_temp = bq_get_temp(2);

	if(AFE_temp>reg_temp)
		return AFE_temp;
	else
		return reg_temp;
}

static void terminal_cmd_shipmode(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	bq_shutdown_bq76940();
	
	return;
}

static void terminal_cmd_connect(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	bq_discharge_enable();
	bq_charge_enable();

	return;
}

float hw_luna_get_connector_temp(){// return the higest temperature between connector temp

	float temp_aux = 0;

	temp_aux = hw_luna_get_temp(4);
	if(hw_luna_get_temp(5) > temp_aux){
		temp_aux = hw_luna_get_temp(5);
	}

	return temp_aux;
}

float precharge_current = 0;
float precharge_temp = 0;
bool flag_precharge_temp = 0;
bool flag_precharge_OK = 0;
enum {
	WAIT_DISCHARGE = 0,
	WAIT_CURRENT_THRESHOLD,
	WAIT_FOR_IDLE,
	TEMP_FAULT
};
int PRECHARGE_STATUS = WAIT_DISCHARGE;
uint16_t precharge_timeout;
static THD_FUNCTION(precharge_thread, arg) {
	(void)arg;
	chRegSetThreadName("PRECHARGE_MONITOR");

	while ( !chThdShouldTerminateX() ) {

		precharge_temp = 0;//pwr_get_adc_ch16(); // todo: temp transfer function

		if( bms_if_get_bms_state() != BMS_FAULT ) {

			switch(PRECHARGE_STATUS) {

				case WAIT_DISCHARGE:
					bq_allow_discharge(false);
					PRECHARGE_ON();
					chThdSleepMilliseconds(1);// time to ADC convert
					precharge_current = pwr_get_adc_ch1() * 0.5; // V to I // commented, until the hardware is finished

					if ( precharge_current > (PRECHARGE_CURRENT_THRESHOLD *0.5) && precharge_temp < PRECHARGE_TEMP_MAX ) {
						PRECHARGE_STATUS = WAIT_CURRENT_THRESHOLD;
					} else {
						if ( precharge_temp >= PRECHARGE_TEMP_MAX ) {
							PRECHARGE_STATUS = TEMP_FAULT;
						}
					}
				break;

				case WAIT_CURRENT_THRESHOLD:
					chThdSleepMilliseconds(1);// time to ADC convert
					precharge_current = pwr_get_adc_ch1() * 0.5; // V to I // commented, until the hardware is finished
					if( precharge_current < PRECHARGE_CURRENT_THRESHOLD && precharge_temp < PRECHARGE_TEMP_MAX ) {
						PRECHARGE_STATUS = WAIT_FOR_IDLE;
					} else {
						if ( precharge_temp >= PRECHARGE_TEMP_MAX ) {
							PRECHARGE_STATUS = TEMP_FAULT;
						}
					}
				break;

				case WAIT_FOR_IDLE:
					bq_allow_discharge(true);
					chThdSleepMilliseconds(300); // wait for bms to detect the discarging and change state
					if( bms_if_get_bms_state() == BMS_IDLE && precharge_temp < PRECHARGE_TEMP_MAX ) {
						PRECHARGE_STATUS = WAIT_DISCHARGE;
					} else {
						if ( precharge_temp >= PRECHARGE_TEMP_MAX ) {
							PRECHARGE_STATUS = TEMP_FAULT;
						}
						if(bms_if_get_bms_state() == (BMS_CHARGING || BMS_DISCHARGIN)){
							PRECHARGE_OFF();
						}
					}
				break;

				case TEMP_FAULT:
					bq_allow_discharge(false);
					PRECHARGE_OFF();
					sleep_reset();
					for ( precharge_timeout = (RECONNECTION_TIMEOUT * 10) ; precharge_timeout > 0 ; precharge_timeout-- ) {
						chThdSleepMilliseconds(100);
						sleep_reset();
					}
					if(precharge_temp < (PRECHARGE_TEMP_MAX * PRECHARGE_TEMP_HYST)) {
						PRECHARGE_STATUS = WAIT_DISCHARGE;
					}
				break;
			}

		} else {
			while(bms_if_get_bms_state() == BMS_FAULT) {// the pre charge must not interfiere in the fault handle
				PRECHARGE_OFF(); // the precahrge must be off in order to let the AFE to detect for example a short chircuit
				bq_allow_discharge(true); // let the AFE to decide if connect or not
				PRECHARGE_STATUS = WAIT_DISCHARGE; // when the bms recovers from the fault, return to the wait for discharge state
				chThdSleepMilliseconds(300); // let time to the AFE and charge_discharge task to handle the fault
										// the AFE thread will be executed after 250ms (worst case)
			}
		}
	}
}
