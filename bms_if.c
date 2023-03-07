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

#include "ltc6813.h"
#include "bq76940.h"
#include "main.h"
#include "bms_if.h"
#include "pwr.h"
#include "utils.h"
#include "hdc1080.h"
#include "sht30.h"
#include "comm_can.h"
#include "timeout.h"
#include "sleep.h"
#include "terminal.h"
#include "flash_helper.h"


#include <math.h>

// Settings
#define I_IN_FILTER_CONST			0.006
#define I_IN_FILTER_CONST_IC		0.006
#define IC_ISENSE_I_GAIN_CORR		0.997 // This gain correction is a hack and should probably be set in config or in hw config


// Private variables
static volatile float m_i_in_filter = 0.0;
static volatile float m_i_in_filter_ic = 0.0;
static volatile bool m_charge_allowed = true;
static volatile bool m_is_charging = false;
static volatile bool m_is_balancing = false;
static volatile float m_voltage_cell_min = 0.0;
static volatile float m_voltage_cell_max = 0.0;
static volatile int m_balance_override[HW_CELLS_SERIES] = {0};
static volatile bool m_bal_ok = false;
static volatile bool m_was_charge_overcurrent = false;
static float m_soc_filtered = 0.0;
static bool m_soc_filter_init_done = false;

bms_if_fault_cb m_fault_cb = NULL;

// Threads
static THD_WORKING_AREA(if_thd_wa, 2048);
static THD_FUNCTION(if_thd, p);
#ifndef HW_BIDIRECTIONAL_SWITCH
static THD_WORKING_AREA(charge_thd_wa, 2048);
static THD_FUNCTION(charge_thd, p);
#else
static THD_WORKING_AREA(charge_discharge_thd_wa, 2048);
static THD_FUNCTION(charge_discharge_thd, p);
#endif
static THD_WORKING_AREA(balance_thd_wa, 2048);
static THD_FUNCTION(balance_thd, p);

void bms_if_init(void) {
	chThdCreateStatic(if_thd_wa, sizeof(if_thd_wa), NORMALPRIO, if_thd, 0);
#ifndef HW_BIDIRECTIONAL_SWITCH
	chThdCreateStatic(charge_thd_wa, sizeof(charge_thd_wa), NORMALPRIO, charge_thd, 0);
#else
	chThdCreateStatic(charge_discharge_thd_wa, sizeof(charge_discharge_thd_wa), NORMALPRIO-1, charge_discharge_thd, 0);
#endif
	chThdCreateStatic(balance_thd_wa, sizeof(balance_thd_wa), NORMALPRIO, balance_thd, 0);
}

bool bms_if_charge_ok(void) {
	float max = m_is_charging ? backup.config.vc_charge_end : backup.config.vc_charge_start;
	return m_voltage_cell_min > backup.config.vc_charge_min &&
			m_voltage_cell_max < max &&
			(!backup.config.t_charge_mon_en || (HW_TEMP_CELLS_MAX() < backup.config.t_charge_max &&
					HW_TEMP_CELLS_MAX() > backup.config.t_charge_min));
}

#ifndef HW_BIDIRECTIONAL_SWITCH
static THD_FUNCTION(charge_thd, p) {
	(void)p;
	chRegSetThreadName("Charge");

	int no_charge_cnt = 0;

	for (;;) {
		if (m_is_charging && HW_TEMP_CELLS_MAX() >= backup.config.t_charge_max &&
				backup.config.t_charge_mon_en) {
			bms_if_fault_report(FAULT_CODE_CHARGE_OVERTEMP);
		}

		bms_soc_soh_temp_stat *msg;
		bool chg_can_ok = true;
		for (int i = 0;i < CAN_BMS_STATUS_MSGS_TO_STORE;i++) {
			msg = comm_can_get_bms_soc_soh_temp_stat_index(i);

			if (msg->id >= 0) {
				if (!msg->is_charge_ok) {
					chg_can_ok = false;
					break;
				}
			} else {
				break;
			}
		}

		if (chg_can_ok && HW_GET_V_CHARGE() > backup.config.v_charge_detect &&
				bms_if_charge_ok() && m_charge_allowed && !m_was_charge_overcurrent) {
			if (!m_is_charging) {
				sleep_reset();
				chThdSleepMilliseconds(2000);
				if (bms_if_charge_ok() && HW_GET_V_CHARGE() > backup.config.v_charge_detect) {
					m_is_charging = true;
					CHARGE_ENABLE();
				}
			}
		} else {
			m_is_charging = false;
			CHARGE_DISABLE();
		}

		chThdSleepMilliseconds(10);

		if (m_i_in_filter > -0.5 && m_is_charging && !HW_CHARGER_DETECTED()) {
			no_charge_cnt++;

			if (no_charge_cnt > 100) {
				no_charge_cnt = 0;
				m_is_charging = false;
				CHARGE_DISABLE();
				chThdSleepMilliseconds(5000);
			}
		} else {
			no_charge_cnt = 0;
		}

		if (m_is_charging) {
			if (fabsf(m_i_in_filter) > backup.config.max_charge_current) {
				m_was_charge_overcurrent = true;
				m_is_charging = false;
				CHARGE_DISABLE();
				bms_if_fault_report(FAULT_CODE_CHARGE_OVERCURRENT);
			}

			sleep_reset();
		}

		// Charger must be disconnected and reconnected on charge overcurrent events
		if (m_was_charge_overcurrent && HW_GET_V_CHARGE() < backup.config.v_charge_detect) {
			m_was_charge_overcurrent = false;
		}

		// Store data and counters to flash every time charger is disconnected
		static bool charger_connected_last = false;
		if (charger_connected_last && HW_GET_V_CHARGE() < backup.config.v_charge_detect) {
			flash_helper_store_backup_data();
		}
		charger_connected_last = HW_GET_V_CHARGE() > backup.config.v_charge_detect;
	}
}
#else
uint8_t BMS_state = 0;
uint16_t FAULT_BITS = 0;
bms_fault_code FAULT_CODE = FAULT_CODE_NONE;
float fabs_in_current = 0;
bool flag_temp_OT_cell_fault = 0;
bool flag_temp_UT_cell_fault = 0;
bool flag_temp_hardware_fault = 0;
bool flag_temp_Vreg_fault = 0;
bool flag_UV_fault = 0;
bool flag_OV_fault = 0;
bool flag_SC_fault = 0;
bool flag_OC_fault = 0;
bool flag_I_charge_fault = 0;
bool flag_SC_discharge_fault = 0;
bool flag_OC_discharge_fault = 0;
bool flag_global_fault = 0;
bool allow_ot_cell_fault_clear = 1;
bool allow_ut_cell_fault_clear = 1;
bool allow_temp_hw_fault_clear = 1;
bool allow_temp_Vreg_fault_clear = 1;
bool allow_OV_fault_clear = 1;
uint8_t oc_sc_count_attempt = 0;
uint8_t oc_charge_count_attempt = 0;
uint16_t timeout = 0;
bool charger_connected = false;
static int blink = 0;
static int blink_count = 0;
float aux_temp_limit = 0.0;
bool BMS_was_chargin = 0;
static THD_FUNCTION(charge_discharge_thd,p){
	(void)p;
	chRegSetThreadName("Charge_Discharge");
	chThdSleepMilliseconds(100);// time to acquire ADCs
	for (;;) {
		HW_WAIT_AFE();
		// check can status
		bms_soc_soh_temp_stat *msg;
		bool chg_can_ok = true;
		for (int i = 0;i < CAN_BMS_STATUS_MSGS_TO_STORE;i++) {
			msg = comm_can_get_bms_soc_soh_temp_stat_index(i);

			if (msg->id >= 0) {
				if (!msg->is_charge_ok) {
					chg_can_ok = false;//TODO: what should i do if chg_can_ok = FALSE?
					break;
				}
			} else {
				break;
			}
		}

		// check and clear faults

		//check mcu V regulator temp
		if ( (bms_if_get_vreg_temp() > HW_MAX_VREG_TEMP) && (flag_temp_Vreg_fault == 0)) {
			bms_if_fault_report(FAULT_CODE_VREG_OVERTEMP);
			flag_temp_Vreg_fault = 1;
			allow_temp_Vreg_fault_clear = 0;
		}else {
			if(allow_temp_Vreg_fault_clear){
				flag_temp_Vreg_fault = 0;
			}
		}

		if ( (HW_TEMP_CELLS_MIN() <= backup.config.t_charge_min && backup.config.t_charge_mon_en) && (flag_temp_UT_cell_fault == 0)) {
			bms_if_fault_report(FAULT_CODE_CELL_UNDERTEMP);
			flag_temp_UT_cell_fault = 1;
			allow_ut_cell_fault_clear = 0;
			blink_count = 0;
		} else {
			if( allow_ut_cell_fault_clear ) {
				flag_temp_UT_cell_fault = 0;
			}
		}

		if(BMS_state == BMS_CHARGING){
			aux_temp_limit = HW_MAX_CELL_TEMP_CHARG;
		}
		else{
			aux_temp_limit = HW_MAX_CELL_TEMP_DISCH;	
		}
		
		if ( (HW_TEMP_CELLS_MAX() >= aux_temp_limit) && (flag_temp_OT_cell_fault == 0)) {
			bms_if_fault_report(FAULT_CODE_CELL_OVERTEMP);
			if(BMS_state == BMS_CHARGING){
				BMS_was_chargin = TRUE; // we need to know if the pack was chargin to avoid using discharge temp limit during the fault recover
			}else{
				BMS_was_chargin = FALSE;
			}
			flag_temp_OT_cell_fault = 1;
			allow_ot_cell_fault_clear = 0;
			blink_count = 0;
		} else {
			if( allow_ot_cell_fault_clear ) {
				flag_temp_OT_cell_fault = 0;
			}
		}

		//check hardware temp
		if ( (((bms_if_get_temp_ic() > HW_MAX_TEMP_IC) || (bms_if_get_temp_mosfet() > HW_MAX_MOSFET_TEMP )
		|| (bms_if_get_connector_temp() > HW_MAX_CONNECTOR_TEMP))) && (flag_temp_hardware_fault == 0)){
			bms_if_fault_report(FAULT_CODE_HARDWARE_OVERTEMP);
			flag_temp_hardware_fault = 1;
			allow_temp_hw_fault_clear = 0;
			blink_count = 0;
		} else {
			if(allow_temp_hw_fault_clear) {
				flag_temp_hardware_fault = 0;
			}
		}

		//check over current charge
		if ( (HW_GET_I_IN() > backup.config.max_charge_current) && (flag_I_charge_fault == 0) ) {
			bms_if_fault_report(FAULT_CODE_CHARGE_OVERCURRENT);
			flag_I_charge_fault = 1;
			blink_count = 0;
		} else {
			if(HW_GET_I_IN() < backup.config.max_charge_current){
				flag_I_charge_fault = 0;
				oc_charge_count_attempt = 0;
			}
		}

		// check short circuit discharge
		if ( HW_SC_DETECTED() && (flag_SC_discharge_fault == 0) ) {
			bms_if_fault_report(FAULT_CODE_DISCHARGE_SHORT_CIRCUIT);
			flag_SC_discharge_fault = 1;
			blink_count = 0;
		} else {
			flag_SC_discharge_fault = 0;
		}

		// check over current discharge
		if ( HW_OC_DETECTED() && (flag_OC_discharge_fault == 0)) {
			bms_if_fault_report(FAULT_CODE_DISCHARGE_OVERCURRENT);
			flag_OC_discharge_fault = 1;
			blink_count = 0;
		} else {
			flag_OC_discharge_fault = 0;
		}

		//check cell under voltage
		if ( HW_UV_DETECTED() && (flag_UV_fault == 0) ) {
			bms_if_fault_report(FAULT_CODE_CELL_UNDERVOLTAGE);
			flag_UV_fault = 1;
			blink_count = 0;
		} else {
			flag_UV_fault = 0;
		}

		//check cell over voltage
		if ( HW_OV_DETECTED() && (flag_OV_fault == 0) ) {
			bms_if_fault_report(FAULT_CODE_CELL_OVERVOLTAGE);
			flag_OV_fault = 1;
			allow_OV_fault_clear = 0;
			blink_count = 0;
		} else {
			if( allow_OV_fault_clear == 1 ) {
				flag_OV_fault = 0;
			}

		}

		//set FAULT_CODE according to priorities to be handle
		if ( flag_temp_Vreg_fault ) {
			FAULT_CODE = FAULT_CODE_VREG_OVERTEMP;
		} else {
			if ( flag_temp_OT_cell_fault ) {
				FAULT_CODE = FAULT_CODE_CELL_OVERTEMP;
			} else {
				if ( flag_temp_UT_cell_fault ) {
					FAULT_CODE = FAULT_CODE_CELL_UNDERTEMP;
				} else {
					if ( flag_temp_hardware_fault ) {
						FAULT_CODE = FAULT_CODE_HARDWARE_OVERTEMP;
					} else {
						if ( flag_I_charge_fault ) {
							FAULT_CODE = FAULT_CODE_CHARGE_OVERCURRENT;
						} else {
							if ( flag_OC_discharge_fault || flag_SC_discharge_fault ) {
								FAULT_CODE = FAULT_CODE_DISCHARGE_OVERCURRENT;
							} else {
								if ( flag_UV_fault ) {
									FAULT_CODE = FAULT_CODE_CELL_UNDERVOLTAGE;
								} else {
									if ( flag_OV_fault ) {
										FAULT_CODE = FAULT_CODE_CELL_OVERVOLTAGE;
									} else {
										FAULT_CODE = FAULT_CODE_NONE;
									}
								}
							}
						}
					}
				}
			}
		}

		//check minumum sleep current
		fabs_in_current = fabs(HW_GET_I_IN());
		if( fabs_in_current > backup.config.min_current_sleep ) {
			sleep_reset();// prevent sleeping
		}

		// check current direction
		if ( (HW_GET_I_IN() > 0.03) && (FAULT_CODE == FAULT_CODE_NONE) ) {// incoming current, pack is charging
			BMS_state = BMS_CHARGING;
		} else {
			if ( (HW_GET_I_IN() < -0.03) && (FAULT_CODE == FAULT_CODE_NONE) ) { // out going current, pack is dischargin
				BMS_state = BMS_DISCHARGIN;
			} else {
				if ( FAULT_CODE != FAULT_CODE_NONE ) {
					BMS_state = BMS_FAULT;
				} else {
					BMS_state = BMS_IDLE;//between -0.03 and 0.03 is considered adc noise, with no faults and no current bms is IDLE
				}
			}
		}
//commands_printf("%d %d %d %d %d %d %d %d", flag_temp_Vreg_fault, flag_temp_OT_cell_fault, flag_temp_UT_cell_fault, flag_temp_hardware_fault, flag_I_charge_fault, flag_OC_discharge_fault , flag_UV_fault, flag_OV_fault );
//commands_printf("%d",BMS_state);
		switch(BMS_state){

			case BMS_CHARGING:	// de activate discharge port
				m_is_charging = true;
				HW_PACK_CONN_ONLY_CHARGE(false);
				HW_PACK_CONNECT();
				if( !charger_connected ) {
					flash_helper_store_backup_data(); // Store data and counters to flash every time charger is disconnected
				}
				charger_connected = true;

			break;

			case BMS_DISCHARGIN:// de activate charge port
				HW_PACK_CONN_ONLY_CHARGE(false);
				HW_PACK_CONNECT();
				charger_connected = false;
				m_is_charging = false;
			break;

			case BMS_FAULT:// fault handler
				sleep_reset();
				charger_connected = false;
				m_is_charging = false;
				switch(FAULT_CODE){
					case FAULT_CODE_VREG_OVERTEMP:
						HW_PACK_DISCONNECT();
						if(HW_VREGULATOR_TEMP() < (HW_MAX_VREG_TEMP - HW_HYSTERESIS_TEMP)){
							allow_temp_Vreg_fault_clear = 1;
						}else{
							chThdSleepMilliseconds(500); //give time to the AFE to response to the disconnect request
							force_sleep(); // force the mcu to sleep to cool down the regulator
						}
					break;

					case FAULT_CODE_CELL_OVERTEMP:
						HW_PACK_DISCONNECT();// disconnect pack until the temp is acceptable
						if(BMS_was_chargin){
							aux_temp_limit = HW_MAX_CELL_TEMP_CHARG; // if the pack was chargin before the temp fault
						}else{											// apply the charge temp limit
							aux_temp_limit = HW_MAX_CELL_TEMP_DISCH;
						}
						if(HW_TEMP_CELLS_MAX() < (aux_temp_limit - HW_HYSTERESIS_TEMP)){
							allow_ot_cell_fault_clear = 1;
						}
					break;

					case FAULT_CODE_CELL_UNDERTEMP:
						HW_PACK_DISCONNECT();// disconnect pack until the temp is acceptable
						if(HW_TEMP_CELLS_MIN()  > (backup.config.t_charge_min + HW_HYSTERESIS_TEMP)){
							allow_ut_cell_fault_clear = 1;
						}
					break;

					case FAULT_CODE_HARDWARE_OVERTEMP:
						HW_PACK_DISCONNECT();// disconnect pack until the temp is acceptable
						if ( (bms_if_get_temp_ic() < (HW_MAX_TEMP_IC - HW_HYSTERESIS_TEMP)) &&
							(bms_if_get_temp_mosfet() < (HW_MAX_MOSFET_TEMP - HW_HYSTERESIS_TEMP)) &&
							(bms_if_get_connector_temp() < (HW_MAX_CONNECTOR_TEMP - HW_HYSTERESIS_TEMP ))) {
							allow_temp_hw_fault_clear = 1;
						}
					break;

					case FAULT_CODE_CHARGE_OVERCURRENT: // charge over current or short circuit
						HW_PACK_DISCONNECT(); // disconnect pack and wait for reconnection time out, at this point the AFE should have disconnected the pack, this is redundant
						if ( oc_charge_count_attempt >= MAX_RECONNECT_ATTEMPT ) {
							oc_charge_count_attempt = 0;
							while ( !HW_CHARGER_DETECTED() ) {	// if max attempt reached, wait for charger removal. Is considered a
								sleep_reset();					// critical fault so bms must be stay here until charged
								oc_charge_count_attempt = 0;	// is removed
								chThdSleepMilliseconds(1000);	// if no charger removal detection is implemented, the bms
																//will continue to attempt connection every RECONNECTION_TIMEOUT
							}
							flag_I_charge_fault = 0;
						} else {
							//wait for reconnection timeout
							for ( timeout = (RECONNECTION_TIMEOUT * 100) ; timeout > 0 ; timeout-- ) {
								chThdSleepMilliseconds(100);
								sleep_reset();
							}

							HW_PACK_CONNECT();
							oc_charge_count_attempt++;
						}
					break;

					case FAULT_CODE_DISCHARGE_OVERCURRENT:
						HW_PACK_DISCONNECT(); // disconnect pack and wait for reconnection time out, at this point the AFE should have disconnected the pack, this is redundant
						if ( oc_sc_count_attempt >= MAX_RECONNECT_ATTEMPT ) {
																// wait for the current to ramp down
							while(HW_LOAD_DETECTION()){			// if max attempt reached, wait for load removal. Is considered a
								sleep_reset();					// critical fault so bms must be stay here until load
								oc_sc_count_attempt = 0;		// is removed.
								chThdSleepMilliseconds(1000);	// if no load detection is implemented, the bms
																//will continue to attempt connection every RECONNECTION_TIMEOUT
							}
							flag_OC_discharge_fault = 0;
							flag_SC_discharge_fault = 0;
							HW_SC_OC_RESTORE();
						} else {
							//wait for reconnection timeout
							for ( timeout = (RECONNECTION_TIMEOUT * 10) ; timeout > 0 ; timeout-- ) {
								chThdSleepMilliseconds(100);
								sleep_reset();
							}
							HW_SC_OC_RESTORE();
							HW_PACK_CONNECT();
							oc_sc_count_attempt++;
						}
					break;

					case FAULT_CODE_CELL_UNDERVOLTAGE:
						// disconnect load
						HW_PACK_CONN_ONLY_CHARGE(true);// allow only charging
						HW_PACK_CONNECT();//connect pack, the discharge will be disconnected

						// check for in current
						if(HW_GET_I_IN() > 0.1){// incoming current, pack is charging
							HW_PACK_CONN_ONLY_CHARGE(false);// connect discharge and charge ports
							HW_PACK_CONNECT();//connect pack
							} else {
								if(HW_GET_I_IN() < -0.1 ){ // out going current, pack is discharging
									HW_PACK_CONN_ONLY_CHARGE(true);// allow only charging
									HW_PACK_CONNECT();//connect pack
							}
						}

						float v_min_aux = 100.0;
						float cell_min = 0;
						// acquire min cell
						for (int i = backup.config.cell_first_index;i <
						(backup.config.cell_num + backup.config.cell_first_index);i++) {
							if (HW_LAST_CELL_VOLTAGE(i) < v_min_aux) {
								v_min_aux = HW_LAST_CELL_VOLTAGE(i);
							}
						}
						cell_min = v_min_aux;
						if(cell_min > ( HW_MIN_CELL + HW_HYSTEREIS_MIN_CELL) ){
							for(timeout = (RECONNECTION_TIMEOUT * 10); timeout > 0 ; timeout--){
								chThdSleepMilliseconds(100); // wait for time out before reconnect
								sleep_reset();
							}
							HW_UV_RESTORE_FAULT(); // restore fault, back to normal mode
							HW_PACK_CONN_ONLY_CHARGE(false);// allow charge and discharge
						}
					break;

					case FAULT_CODE_CELL_OVERVOLTAGE:
						HW_PACK_DISCONNECT();//at this point the AFE should have disconnected the pack, this is redundant
						//wait before measure the cells
						for ( timeout = (RECONNECTION_TIMEOUT * 10) ; timeout > 0 ; timeout-- ) {
							chThdSleepMilliseconds(100);
							sleep_reset();
						}
						float v_max_aux = 0.0;
						float cell_max = 0;
						// acquire max cell
						for (int i = backup.config.cell_first_index;i < (backup.config.cell_num + backup.config.cell_first_index) ;i++ ) {
							if (HW_LAST_CELL_VOLTAGE(i) > v_max_aux) {
								v_max_aux = HW_LAST_CELL_VOLTAGE(i);
							}
						}
						cell_max = v_max_aux;
						if(cell_max < ( HW_MAX_CELL - HW_HYSTEREIS_MAX_CELL) ){
							HW_OV_RESTORE_FAULT();
							allow_OV_fault_clear = 1;
						}
					break;

					default:
					break;

				}

			break;// end faults handler

			case BMS_IDLE:// activate charge and discharge ports, waiting for in or out current
				m_is_charging = false;
				charger_connected = false;
				HW_PACK_CONN_ONLY_CHARGE(false);
				HW_PACK_CONNECT();

			break;

			default:
				BMS_state = BMS_FAULT; // if unknown state, declare fault
				sleep_reset();// prevent sleeping
			break;

		}

	}
}

#endif
static THD_FUNCTION(balance_thd, p) {
	(void)p;
	chRegSetThreadName("Balance");

	systime_t last_charge_time = 0.0;

	while (!chThdShouldTerminateX()) {
		float v_min = 10.0;
		float v_max = 0.0;

		// Allow some time to start balancing after unplugging the charger. This is useful if it is unplugged
		// while the current was so high that balancing was prevented.
		float time_since_charge = 1000.0;
		if (UTILS_AGE_S(0) > 1.0) {
			if (m_is_charging) {
				last_charge_time = chVTGetSystemTimeX();
			}

			time_since_charge = UTILS_AGE_S(last_charge_time);
		}

		switch (backup.config.balance_mode) {
		case BALANCE_MODE_DISABLED:
			m_bal_ok = false;
			break;

		case BALANCE_MODE_CHARGING_ONLY:
			if (m_is_charging) {
				m_bal_ok = true;
			} else {
				m_bal_ok = false;
			}
			break;

		case BALANCE_MODE_DURING_AND_AFTER_CHARGING:
			if (time_since_charge < 2.0) {
				m_bal_ok = true;
			}
			break;

		case BALANCE_MODE_ALWAYS:
			m_bal_ok = true;
			break;
		}
#ifdef HW_BIDIRECTIONAL_SWITCH
		if (flag_temp_OT_cell_fault || flag_temp_UT_cell_fault || flag_temp_hardware_fault || flag_UV_fault ) { // if any temp fault or UV
			m_bal_ok = false;																					// force disable balance
		}else{
			if(flag_OV_fault){
				m_bal_ok = true;	// if no temp fault and OV fault force balance
			}
		}
#endif
		for (int i = backup.config.cell_first_index;i <
		(backup.config.cell_num + backup.config.cell_first_index);i++) {
			if (HW_LAST_CELL_VOLTAGE(i) > v_max) {
				v_max = HW_LAST_CELL_VOLTAGE(i);
			}
			if (HW_LAST_CELL_VOLTAGE(i) < v_min) {
				v_min = HW_LAST_CELL_VOLTAGE(i);
			}
		}

		m_voltage_cell_min = v_min;
		m_voltage_cell_max = v_max;
		m_is_balancing = false;

		bool is_balance_override = false;
		int bal_ch = 0;
		for (int i = backup.config.cell_first_index;
				i < (backup.config.cell_num + backup.config.cell_first_index);i++) {
			if (m_balance_override[i] == 1) {
				is_balance_override = true;
				HW_SET_DSC(i, false);
			} else if (m_balance_override[i] == 2) {
				is_balance_override = true;
				HW_SET_DSC(i, true);
				bal_ch++;
				m_is_balancing = true;
			}
		}

		if (backup.config.dist_bal) {
			bms_soc_soh_temp_stat *msg = comm_can_get_bms_stat_v_cell_min();

			if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < 10.0 && msg->v_cell_min < v_min) {
				v_min = msg->v_cell_min;
			}
		}

		if (v_min > backup.config.vc_balance_min &&
				m_bal_ok &&
				!is_balance_override &&
				fabsf(bms_if_get_i_in_ic()) < backup.config.balance_max_current) {

			bal_ch = 0;
			for (int i = backup.config.cell_first_index;i <
			(backup.config.cell_num + backup.config.cell_first_index);i++) {
				float limit = HW_GET_DSC(i) ? backup.config.vc_balance_end : backup.config.vc_balance_start;
				limit += v_min;

				if (HW_LAST_CELL_VOLTAGE(i) >= limit) {
					HW_SET_DSC(i, true);
					bal_ch++;
					m_is_balancing = true;
				} else {
					HW_SET_DSC(i, false);
				}
			}
		}

		float t_bal = HW_GET_BAL_TEMP();
		float t_bal_start = backup.config.t_bal_lim_start;
		float t_bal_end = backup.config.t_bal_lim_end;
		int bal_ch_max = backup.config.max_bal_ch;

		if (t_bal > (t_bal_end - 0.5)) {
			bal_ch_max = 0;
		} else if (t_bal > t_bal_start) {
			bal_ch_max = utils_map_int(t_bal, t_bal_start, t_bal_end, bal_ch_max, 0);
		}

		// Limit number of simultaneous balancing channels by disabling
		// balancing on the cells with the highest voltage.
		while (bal_ch > bal_ch_max) {
			float v_min = 100.0;
			int v_min_cell = 0;
			for (int i = backup.config.cell_first_index;i <
			(backup.config.cell_num + backup.config.cell_first_index);i++) {
				if (HW_LAST_CELL_VOLTAGE(i) < v_min && HW_GET_DSC(i)) {
					v_min = HW_LAST_CELL_VOLTAGE(i);
					v_min_cell = i;
				}
			}

			HW_SET_DSC(v_min_cell, false);
			bal_ch--;
		}

		if (m_is_balancing) {
			sleep_reset();
		} else {
			m_bal_ok = false;

			for (int i = 0;i < HW_CELLS_SERIES;i++) {
				HW_SET_DSC(i, false);
			}
		}

		timeout_feed_WDT(THREAD_BAL);
		chThdSleepMilliseconds(50);
	}
}

static THD_FUNCTION(if_thd, p) {
	(void)p;
	chRegSetThreadName("IfThd");

	systime_t tick_last = chVTGetSystemTimeX();

	chThdSleepMilliseconds(2000);

	for(;;) {

		float i_bms_ic = 0;
		HW_GET_I_IN_AFE;
		float i_adc = HW_GET_I_IN();

		if (backup.config.i_measure_mode == I_MEASURE_MODE_VESC) {
			for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
				can_status_msg_4 *msg = comm_can_get_status_msg_4_index(i);

				if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < 2.0) {
					i_bms_ic += msg->current_in;
				}
			}
		}

		UTILS_LP_FAST(m_i_in_filter, i_adc, I_IN_FILTER_CONST);
		UTILS_LP_FAST(m_i_in_filter_ic, i_bms_ic, I_IN_FILTER_CONST_IC);

		double time = (double)TIME_I2US(chVTTimeElapsedSinceX(tick_last)) *
				(double)1.0e-6 * ((double)1.0 / (double)60.0) * ((double)1.0 / (double)60.0);
		tick_last = chVTGetSystemTimeX();

		if (fabsf(m_i_in_filter_ic) > backup.config.min_current_ah_wh_cnt) {
			double d_ah = (double)bms_if_get_i_in_ic() * time;
			double d_wh = (double)bms_if_get_i_in_ic() * (double)bms_if_get_v_tot() * time;

			backup.ah_cnt += d_ah;
			backup.wh_cnt += d_wh;

			if (m_i_in_filter_ic > 0.0) {
				backup.ah_cnt_dis_total += d_ah;
				backup.wh_cnt_dis_total += d_wh;
			} else {
				backup.ah_cnt_chg_total -= d_ah;
				backup.wh_cnt_chg_total -= d_wh;
			}
		}

		if (fabsf(bms_if_get_i_in_ic()) > backup.config.min_current_sleep) {
			sleep_reset();
		}

		float soc_now = utils_batt_liion_norm_v_to_capacity(utils_map(m_voltage_cell_min, 3.2, 4.2, 0.0, 1.0));
		if (!m_soc_filter_init_done) {
			m_soc_filter_init_done = true;
			m_soc_filtered = soc_now;
		} else {
			UTILS_LP_FAST(m_soc_filtered, soc_now, backup.config.soc_filter_const);
		}

		// RED LED
#ifdef HW_BIDIRECTIONAL_SWITCH

		if( BMS_state == BMS_FAULT ) {
			switch(FAULT_CODE) {

				case FAULT_CODE_CELL_OVERTEMP:
					blink_count++;
					if( blink_count > 1100) {
						blink_count = 0;
					}

					if( blink_count < 600 ) {
						blink++;
						if (blink > 300) {
							blink = 0;
						}

						if (blink < 150) {
							LED_ON(LINE_LED_RED);
						} else {
							LED_OFF(LINE_LED_RED);
						}
					} else {
						LED_OFF (LINE_LED_RED);
						blink = 0;
					}
				break;

				case FAULT_CODE_CELL_UNDERTEMP:
					blink_count++;
					if( blink_count > 1250) {
						blink_count = 0;
					}

					if( blink_count < 750 ) {
						blink++;
						if (blink > 300) {
							blink = 0;
						}

						if (blink < 150) {
							LED_ON(LINE_LED_RED);
						} else {
							LED_OFF(LINE_LED_RED);
						}
					} else {
						LED_OFF (LINE_LED_RED);
						blink = 0;
					}
				break;

				case FAULT_CODE_HARDWARE_OVERTEMP:
					blink_count++;
					if( blink_count > 1850) {
						blink_count = 0;
					}

					if( blink_count < 1350 ) {
						blink++;
						if (blink > 300) {
							blink = 0;
						}

						if (blink < 150) {
							LED_ON(LINE_LED_RED);
						} else {
							LED_OFF(LINE_LED_RED);
						}
					} else {
						LED_OFF (LINE_LED_RED);
						blink = 0;
					}
				break;

				case FAULT_CODE_CHARGE_OVERCURRENT:
					blink_count++;
					if( blink_count > 2150) {
						blink_count = 0;
					}

					if( blink_count < 1650 ) {
						blink++;
						if (blink > 300) {
							blink = 0;
						}

						if (blink < 150) {
							LED_ON(LINE_LED_RED);
						} else {
							LED_OFF(LINE_LED_RED);
						}
					} else {
						LED_OFF (LINE_LED_RED);
						blink = 0;
					}
				break;

				case FAULT_CODE_DISCHARGE_OVERCURRENT:
					blink_count++;
					if( blink_count > 2450) {
						blink_count = 0;
					}

					if( blink_count < 1950 ) {
						blink++;
						if (blink > 300) {
							blink = 0;
						}

						if (blink < 150) {
							LED_ON(LINE_LED_RED);
						} else {
							LED_OFF(LINE_LED_RED);
						}
					} else {
						LED_OFF (LINE_LED_RED);
						blink = 0;
					}
				break;

				case FAULT_CODE_CELL_UNDERVOLTAGE:
					blink_count++;
					if( blink_count > 2750) {
						blink_count = 0;
					}

					if( blink_count < 2250 ) {
						blink++;
						if (blink > 300) {
							blink = 0;
						}

						if (blink < 150) {
							LED_ON(LINE_LED_RED);
						} else {
							LED_OFF(LINE_LED_RED);
						}
					} else {
						LED_OFF (LINE_LED_RED);
						blink = 0;
					}
				break;

				case FAULT_CODE_CELL_OVERVOLTAGE:
					blink_count++;
					if( blink_count > 3000) {
						blink_count = 0;
					}

					if( blink_count < 2550 ) {
						blink++;
						if (blink > 300) {
							blink = 0;
						}

						if (blink < 150) {
							LED_ON(LINE_LED_RED);
						} else {
							LED_OFF(LINE_LED_RED);
						}
					} else {
						LED_OFF (LINE_LED_RED);
						blink = 0;
					}
				break;

				default:
				break;


			}


#else
		if (m_was_charge_overcurrent) {
			// Prevent sleeping to keep the charge input disabled (as long as the battery does not run too low)
			if (bms_if_get_soc() > 0.3) {
				sleep_reset();
			}

			// Blink out fault on red LED
			static int blink = 0;
			blink++;
			if (blink > 200) {
				blink = 0;
			}

			if (blink < 100) {
				LED_ON(LINE_LED_RED);
			} else {
				LED_OFF(LINE_LED_RED);
			}
#endif
		} else {
			if (m_is_balancing) {
				LED_ON(LINE_LED_RED);
			} else {
				LED_OFF(LINE_LED_RED);
			}
		}

		chThdSleepMilliseconds(1);
	}
}

float bms_if_get_i_in(void) {
	return m_i_in_filter;
}

float bms_if_get_i_in_ic(void) {
	return m_i_in_filter_ic;
}

float bms_if_get_v_cell(int cell) {
	return HW_LAST_CELL_VOLTAGE(cell);
}

float bms_if_get_v_cell_min(void) {
	return m_voltage_cell_min;
}

float bms_if_get_v_cell_max(void) {
	return m_voltage_cell_max;
}

float bms_if_get_v_tot(void) {
	float ret = 0.0;
	for (int i = backup.config.cell_first_index;i <
	(backup.config.cell_num + backup.config.cell_first_index);i++) {
		ret += bms_if_get_v_cell(i);
	}
	return ret;
}

float bms_if_get_v_charge(void) {
	return HW_GET_V_CHARGE();
}

float bms_if_get_temp(int sensor) {
	return HW_GET_TEMP(sensor);//pwr_get_temp(sensor);
}

float bms_if_get_temp_ic(void) {
	return HW_GET_TEMP_IC(); //ltc_last_temp();
}

bool bms_if_is_balancing_cell(int cell) {
	return HW_GET_DSC(cell);
}

double bms_if_get_ah_cnt(void) {
	return backup.ah_cnt;
}

double bms_if_get_wh_cnt(void) {
	return backup.wh_cnt;
}

double bms_if_get_ah_cnt_chg_total(void) {
	return backup.ah_cnt_chg_total;
}

double bms_if_get_wh_cnt_chg_total(void) {
	return backup.wh_cnt_chg_total;
}

double bms_if_get_ah_cnt_dis_total(void) {
	return backup.ah_cnt_dis_total;
}

double bms_if_get_wh_cnt_dis_total(void) {
	return backup.wh_cnt_dis_total;
}

bool bms_if_is_charge_allowed(void) {
	return m_charge_allowed;
}

void bms_if_set_charge_allowed(bool allowed) {
	m_charge_allowed = allowed;
}

bool bms_if_is_charging(void) {
	return m_is_charging;
}

bool bms_if_is_balancing(void) {
	return m_is_balancing;
}

/**
 * Override balancing.
 *
 * cell
 * Cell number
 *
 * override
 * Override setting.
 * 0: Do not override balancing
 * 1: Override and disable balancing on cell
 * 2: Override and enable balancing on cell
 */
void bms_if_set_balance_override(int cell, int override) {
	if (cell >= 0 && cell < HW_CELLS_SERIES) {
		m_balance_override[cell] = override;
	}
}

void bms_if_reset_counter_ah(void) {
	backup.ah_cnt = 0.0;
}

void bms_if_reset_counter_wh(void) {
	backup.wh_cnt = 0.0;
}

void bms_if_force_balance(bool bal_en) {
	if (bal_en) {
		m_bal_ok = true;
	} else {
		m_bal_ok = false;
		for (int i = 0;i < HW_CELLS_SERIES;i++) {
			HW_SET_DSC(i, false);
		}
	}
}

void bms_if_zero_current_offset(void) {
	float ofs_avg = 0.0;
	float samples = 0.0;

	for (int i = 0;i < 20;i++) {
		ofs_avg -= HW_ZERO_CURRENT_OFFSET;//ltc_last_gpio_voltage(LTC_GPIO_CURR_MON) - 1.65;
		samples += 1.0;
		chThdSleepMilliseconds(100);
	}

	backup.ic_i_sens_v_ofs = ofs_avg / samples;
	flash_helper_store_backup_data();
}

float bms_if_get_humsens_hum_pcb(void) {
#ifdef HDC1080_SDA_GPIO
	return hdc1080_get_hum();
#elif defined(SHT30_SDA_GPIO)
	return sht30_get_hum();
#else
	return 0.0;
#endif
}

float bms_if_get_humsens_temp_pcb(void) {
#ifdef HDC1080_SDA_GPIO
	return hdc1080_get_temp();
#elif defined(SHT30_SDA_GPIO)
	return sht30_get_temp();
#else
	return 0.0;
#endif
}

float bms_if_get_humsens_hum_ext(void) {
#ifdef 	SHT30_SDA_GPIO
	return sht30_get_hum();
#else
	return 0.0;
#endif
}

float bms_if_get_humsens_temp_ext(void) {
#ifdef	SHT30_SDA_GPIO
	return sht30_get_temp();
#else
	return 0.0;
#endif
}

float bms_if_get_soc(void) {
	// TODO: Estimate and compensate for ESR
	if (HW_SOC_OVERRIDE() >= 0.0) {
		return HW_SOC_OVERRIDE();
	} else {
		return m_soc_filtered;
	}
}

float bms_if_get_soh(void) {
	// TODO!
	return 1.0;
}

void bms_if_sleep(void) {
	HW_AFE_SLEEP();//ltc_sleep();
}

void bms_if_fault_report(bms_fault_code fault) {
	fault_data f;

	f.fault = fault;
	f.fault_time = chVTGetSystemTimeX();
	f.current = bms_if_get_i_in();
	f.current_ic = bms_if_get_i_in_ic();
	f.temp_batt = HW_TEMP_CELLS_MAX();
	f.temp_pcb = HW_PCB_TEMP();
	f.temp_ic = bms_if_get_temp_ic();
	f.v_cell_min = bms_if_get_v_cell_min();
	f.v_cell_max = bms_if_get_v_cell_max();
	f.pcb_humidity = bms_if_get_humsens_hum_pcb();
	f.temp_batt_min = HW_TEMP_CELLS_MIN();
	f.temp_connector = HW_CONNECTOR_TEMP();
	f.temp_mosfets = HW_MOSFET_SENSOR();
	terminal_add_fault_data(&f);

	if (m_fault_cb)
		m_fault_cb(&f);
}

bms_fault_code bms_if_fault_now(void) {
	bms_fault_code res = FAULT_CODE_NONE;

	if (m_was_charge_overcurrent) {
		res = FAULT_CODE_CHARGE_OVERCURRENT;
	}

	return res;
}

void bms_if_register_fault_cb(const bms_if_fault_cb cb) {
	m_fault_cb = cb;
}

float bms_if_get_temp_mosfet () {
	return HW_MOSFET_SENSOR();
}

float bms_if_get_connector_temp() {
	return HW_CONNECTOR_TEMP();
}

float bms_if_get_vreg_temp () {
	return HW_VREGULATOR_TEMP();
}
