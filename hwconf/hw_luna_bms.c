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

#ifdef USE_PRECHARGE
static THD_WORKING_AREA(precharge_thread_wa, 1024);
static THD_FUNCTION(precharge_thread, arg);
#endif

void hw_luna_init(void){
	bq76940_init();
	HW_SET_CAN_ENABLE_LINE();
	HW_CAN_ON();
	palSetLineMode(LINE_CHG_DETECTION, PAL_MODE_INPUT);
	terminal_register_command_callback("shipmode", "Shipmode = turn off bq 76940", 0, terminal_cmd_shipmode);	
	terminal_register_command_callback("Connect", "Connect=turn on big mosfets", 0, terminal_cmd_connect);
#ifdef USE_PRECHARGE
	// Config precharge pins
	PRECHARGE_ON();
	palSetLineMode(ADC_PRECHARGE_I_LINE, PAL_MODE_INPUT_ANALOG);
	palSetLineMode(ADC_PRECH_RES_TEMP_LINE, PAL_MODE_INPUT_ANALOG);
	palSetLineMode(PRECHARGE_ENABLE_LINE, PAL_MODE_OUTPUT_PUSHPULL);

	PWR->PDCRB |= PWR_PDCRB_PB0; //PB0 precharge pin pull down during standby
	PWR->CR3 |= PWR_CR3_APC;	// apply pull down configuration

	chThdCreateStatic(precharge_thread_wa, sizeof(precharge_thread_wa), NORMALPRIO + 3 , precharge_thread, NULL);
#else
	bq_allow_discharge(true); // if precharge is nor used, precharge condition have to be always true
#endif

#ifdef HW_USE_WKP2
	PWR->PUCRC |= PWR_PUCRC_PC13; //wkp2 pin (PC13) pull up
#endif
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

#ifdef USE_PRECHARGE
float hw_luna_get_precharge_current(){
	return pwr_get_adc_ch1() * 0.5;
}

enum {
	PRECH_IDLE = 0,
	PRECH_WAIT_DISCHARGE,
	PRECH_WAIT_CURRENT_THRESHOLD,
	PRECH_WAIT_FOR_IDLE,
	PRECH_TEMP_FAULT,
	PRECH_OC_FAULT
	};

float precharge_current = 0.0;
float precharge_temp = 0.0;
int PRECHARGE_STATUS = PRECH_IDLE;
systime_t prech_thresold_time = 0;
uint16_t precharge_reconnect_timeout = 0;
uint8_t precharge_reconnect_atempt = 0;

static THD_FUNCTION(precharge_thread, arg) {
	(void)arg;
	chRegSetThreadName("PRECHARGE_MONITOR");

	PRECHARGE_ON();
	bq_allow_discharge(false);

	while ( !chThdShouldTerminateX() ) {
		chThdSleepMilliseconds(5);// time to ADC convert
		
		precharge_temp = HW_GET_PRECH_TEMP();
		precharge_current = HW_GET_PRECH_CURRENT();

		if( bms_if_get_bms_state() != BMS_FAULT ) {

			switch(PRECHARGE_STATUS) {

				case PRECH_IDLE:
					bq_allow_discharge(false);
					if ( precharge_current > PRECHARGE_CURRENT_THRESHOLD && precharge_temp < PRECHARGE_TEMP_MAX 
					&& precharge_current < PRECHARGE_OC) {
						prech_thresold_time = chVTGetSystemTimeX();
						PRECHARGE_STATUS = PRECH_WAIT_DISCHARGE;
						sleep_reset();
						
					} else {
						if (bms_if_get_bms_state() == BMS_CHARGING && precharge_temp < PRECHARGE_TEMP_MAX 
						&& precharge_current < PRECHARGE_OC){
							bq_allow_discharge(true);
						}else{
							if ( precharge_temp >= PRECHARGE_TEMP_MAX ) {
								PRECHARGE_STATUS = PRECH_TEMP_FAULT;
								bq_allow_discharge(false);
								PRECHARGE_OFF();
								bms_if_fault_report(FAULT_CODE_PRECH_OT);
							} else {
								if ( precharge_current >= PRECHARGE_OC ) {
									PRECHARGE_STATUS = PRECH_OC_FAULT;
									bq_allow_discharge(false);
									PRECHARGE_OFF();
									bms_if_fault_report(FAULT_CODE_PRECH_OC);
								}
							}
						}
					}
										
				break;

				case PRECH_WAIT_DISCHARGE:

					sleep_reset();
					if( UTILS_AGE_S(prech_thresold_time) > PRECHARGE_TIMEOUT && precharge_temp < PRECHARGE_TEMP_MAX 
					&& precharge_current < PRECHARGE_OC) {
						PRECHARGE_STATUS = PRECH_WAIT_FOR_IDLE;
						bq_allow_discharge(true);
						chThdSleepMilliseconds(500);
					} else {
						if ( precharge_temp >= PRECHARGE_TEMP_MAX ) {
							PRECHARGE_STATUS = PRECH_TEMP_FAULT;
							bq_allow_discharge(false);
							PRECHARGE_OFF();
							bms_if_fault_report(FAULT_CODE_PRECH_OT);
						} else {
							if ( precharge_current >= PRECHARGE_OC ) {
								PRECHARGE_STATUS = PRECH_OC_FAULT;
								bq_allow_discharge(false);
								PRECHARGE_OFF();
								bms_if_fault_report(FAULT_CODE_PRECH_OC);
							}	
						}
					}
				break;

				case PRECH_WAIT_FOR_IDLE:

					if( bms_if_get_bms_state() == BMS_IDLE && precharge_temp < PRECHARGE_TEMP_MAX 
					&& precharge_current < PRECHARGE_OC) {
						PRECHARGE_STATUS = PRECH_IDLE;
						bq_allow_discharge(false);			
					} else {
						if ( precharge_temp >= PRECHARGE_TEMP_MAX ) {
							PRECHARGE_STATUS = PRECH_TEMP_FAULT;
							bq_allow_discharge(false);
							PRECHARGE_OFF();
							bms_if_fault_report(FAULT_CODE_PRECH_OT);
						} else {
							if ( precharge_current >= PRECHARGE_OC ) {
								PRECHARGE_STATUS = PRECH_OC_FAULT;
								bq_allow_discharge(false);
								PRECHARGE_OFF();
								bms_if_fault_report(FAULT_CODE_PRECH_OC);
							}
						}
					}
					chThdSleepMilliseconds(300); // wait for bms to detect the discarging and change state

				break;

				case PRECH_TEMP_FAULT:
										
					if(precharge_temp < (PRECHARGE_TEMP_MAX * PRECHARGE_TEMP_HYST)) {
						PRECHARGE_STATUS = PRECH_IDLE;
						PRECHARGE_ON();
					}

					for ( precharge_reconnect_timeout = (RECONNECTION_TIMEOUT * 10) ; precharge_reconnect_timeout > 0 ; 
						precharge_reconnect_timeout-- ) {
						sleep_reset();
						chThdSleepMilliseconds(100);
					}

				break;

				case PRECH_OC_FAULT:
					
					if(precharge_current < PRECHARGE_OC) {
						PRECHARGE_STATUS = PRECH_IDLE;
						PRECHARGE_ON();
						precharge_reconnect_atempt = 0;
					}

					for ( precharge_reconnect_timeout = (RECONNECTION_TIMEOUT * 10) ; precharge_reconnect_timeout > 0 ; 
						precharge_reconnect_timeout-- ) {
						sleep_reset();
						chThdSleepMilliseconds(100);
					}

					PRECHARGE_ON();
					precharge_reconnect_atempt++;
					
					if(precharge_reconnect_atempt > 3){
						while(HW_LOAD_DETECTION()){			// if max attempt reached, wait for load removal. Is considered a
							sleep_reset();					// critical fault so bms must be stay here until load
							precharge_reconnect_atempt = 0;	// is removed.
							chThdSleepMilliseconds(1000);	// if no load detection is implemented, the bms
															//will continue to attempt connection every RECONNECTION_TIMEOUT
						}
					}

				break;

				default:
					
					PRECHARGE_STATUS = PRECH_TEMP_FAULT; // if uknow state, go to temp fault
					bq_allow_discharge(false);
					PRECHARGE_OFF();

				break;
			}
			
		} else {
			while(bms_if_get_bms_state() == BMS_FAULT) {// the pre charge must not interfiere in the fault handle
				PRECHARGE_OFF(); // the precahrge must be off in order to let the AFE to detect for example a short chircuit
				bq_allow_discharge(true); // let the AFE to decide if connect or not
				chThdSleepMilliseconds(300); // let time to the AFE and charge_discharge task to handle the fault
										// the AFE thread will be executed after 250ms (worst case)
			}
			PRECHARGE_STATUS = PRECH_IDLE; // when the bms recovers from the fault, return to the wait for discharge state
			PRECHARGE_ON();
			bq_allow_discharge(false);
		}
	}
}
#endif