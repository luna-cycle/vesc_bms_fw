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

#ifndef HWCONF_HW_LUNA_BMS_H_
#define HWCONF_HW_LUNA_BMS_H_

#include "bq76940.h"

#define HW_NAME					"luna_bms"

// HW-specific
#define HW_HAS_BQ76940
#define HW_BACK_TO_BACK_MOSFETS
#define HW_HAL_USE_I2C2
#define HW_USE_HSI16
#define WH_CAN_USE_SLEEP_MODE
#define USE_AFE_WD
#define HW_BIDIRECTIONAL_SWITCH

#define HW_MAX_TEMP_IC 			75.0	// AFE temp [°C]
#define HW_MAX_MOSFET_TEMP		75.0	// MOSFET TEMP [°C]
#define HW_MAX_CONNECTOR_TEMP	75.0	// power connectors and regulator max temp [°C]
#define HW_MAX_VREG_TEMP 		95.0	// max pre regulator temp [°C]
#define HW_MAX_SC_DISCHARGE_I	44.0	// short circuit current [A]
#define HW_MAX_OC_DISCHARGE_I	16.0	// overcurrent current [A]
#define HW_HYSTERESIS_TEMP		5.0		// hysteresis to avoid reconnecto on temp fault [°C]
#define MAX_RECONNECT_ATTEMPT 	3		// max reconnection attempt after a short circuit or overcurrent
#define RECONNECTION_TIMEOUT	3		// seconds to wait before reconnection attempt
#define HW_MIN_CELL				2.8		//min cell voltage [V]
#define HW_HYSTEREIS_MIN_CELL	0.05	//min cell hysteresis to clear undervoltage [V]
#define HW_MAX_CELL				4.2		//max cell voltage [V]
#define HW_HYSTEREIS_MAX_CELL	0.05	//max cell hysteresis to clear undervoltage [V]

// Macros
#define HW_INIT_HOOK()

#define HW_AFE_INIT()				hw_luna_init();
#define HW_PACK_CONNECT()			bq_request_connect_pack(true)
#define HW_PACK_DISCONNECT()		bq_request_connect_pack(false)
#define CHARGE_ENABLE()				bq_request_connect_pack(true)
#define CHARGE_DISABLE()			bq_request_connect_pack(false)
#define HW_GET_TEMP(sensors)		hw_luna_get_temp(sensors)
#define HW_GET_TEMP_IC()			bq_get_temp_ic()
#define HW_SET_DSC(cell, set)		bq_set_dsc(cell, set)
#define HW_GET_DSC(cell)			bq_get_dsc(cell)
#define HW_LAST_CELL_VOLTAGE(cell)	bq_last_cell_voltage(cell)
#define HW_GET_V_TOTAL()			bq_last_pack_voltage()
#define HW_GET_V_CHARGE()			bq_last_pack_voltage()	//until we implement charge voltage measurement in hw
#define HW_GET_I_IN()				bq_get_current()		//this hw wont read current using the mcu adc for the forseeable future
#define HW_GET_I_IN_AFE				i_bms_ic = bq_get_current()
#define HW_ZERO_CURRENT_OFFSET		bq_get_CC_raw()
#define HW_AFE_SLEEP()				//sleep_bq76940()
#define HW_GET_BAL_TEMP()			hw_luna_get_bal_temp()
#define HW_MOSFET_SENSOR()			hw_luna_get_temp(6)		// return mosfet temp
#define HW_CONNECTOR_TEMP()			hw_luna_get_connector_temp()
#define HW_VREGULATOR_TEMP()		hw_luna_get_temp(7)
#define HW_LOAD_DETECTION()			bq_get_load_status()
#define HW_CHARGER_DETECTION()		1
#define HW_PACK_CONN_ONLY_CHARGE(request)	bq_connect_only_charger(request)
#define HW_SC_DETECTED()			bq_sc_detected()
#define HW_OC_DETECTED()			bq_oc_detected()
#define HW_SC_OC_RESTORE()			bq_restore_oc_sc_fail()
#define HW_OV_DETECTED()			bq_ov_detected()
#define HW_UV_DETECTED()			bq_uv_detected()
#define HW_OV_RESTORE_FAULT()		bq_restore_ov_fault()
#define HW_UV_RESTORE_FAULT() 		bq_restore_uv_fault()
#define HW_WAIT_AFE()				bq_semaphore()

// Settings
#define HW_ADC_TEMP_SENSORS		8
#define HW_CELLS_SERIES			14
#define HW_SHUNT_AMP_GAIN		(20.0)
#define V_REG					3.3
#define R_CHARGE_TOP			(520e3 + 2.5e3 + 100.0)
#define R_CHARGE_BOTTOM			(10e3)
#define AFE						bq76940
#define WOLF_REV3
// Define the current with the shunt resistor value
#define HW_SHUNT_RES			(0.0005)

//More abstraction to setup the threshold shortcircuit current
//Only if only the resistor shunt is 0.5mOhm
#define CURRENT_44A		BQ_SCP_22mV
#define CURRENT_66A		BQ_SCP_33mV
#define CURRENT_88A		BQ_SCP_44mV
#define CURRENT_112A	BQ_SCP_56mV
#define CURRENT_134A	BQ_SCP_67mV
#define CURRENT_156A	BQ_SCP_78mV
#define CURRENT_178A	BQ_SCP_89mV
#define CURRENT_200A	BQ_SCP_100mV
#define CURRENT_222A	BQ_SCP_111mV
#define CURRENT_266A	BQ_SCP_133mV
#define CURRENT_310A	BQ_SCP_155mV
#define CURRENT_356A	BQ_SCP_178mV
#define CURRENT_400A	BQ_SCP_200mV

//Values for OverCurrent setting
#define CURRENT_16A		BQ_OCP_8mV
#define CURRENT_22A		BQ_OCP_11mV
#define CURRENT_28A		BQ_OCP_14mV
#define CURRENT_34A		BQ_OCP_17mV
#define CURRENT_38A		BQ_OCP_19mV
#define CURRENT_50A		BQ_OCP_25mV
#define CURRENT_56A		BQ_OCP_28mV
#define CURRENT_62A		BQ_OCP_31mV
#define CURRENT_72A		BQ_OCP_36mV
#define CURRENT_78A		BQ_OCP_39mV
#define CURRENT_84A		BQ_OCP_42mV
#define CURRENT_94A		BQ_OCP_47mV
#define CURRENT_100A	BQ_OCP_50mV
#define CURRENT_122A	BQ_OCP_61mV
#define CURRENT_144A	BQ_OCP_72mV
#define CURRENT_166A	BQ_OCP_83mV
#define CURRENT_188A	BQ_OCP_94mV

// LEDs

#define LINE_LED_RED			PAL_LINE(GPIOA, 0)
#define LINE_LED_GREEN			PAL_LINE(GPIOA, 1)

#define LINE_LED_RED_DEBUG			PAL_LINE(GPIOA, 0)
#define LINE_LED_GREEN_DEBUG		PAL_LINE(GPIOA, 1)
#define LED_RED_DEBUG_OFF()			palClearLine(LINE_LED_RED_DEBUG)
#define LED_RED_DEBUG_ON()			palSetLine(LINE_LED_RED_DEBUG)
#define LED_GREEN_DEBUG_OFF()		palClearLine(LINE_LED_GREEN_DEBUG)
#define LED_GREEN_DEBUG_ON()		palSetLine(LINE_LED_GREEN_DEBUG)

// BQ76200
#define LINE_BQ_CHG_EN			PAL_LINE(GPIOB, 15)
#define LINE_BQ_CP_EN			PAL_LINE(GPIOB, 13)
#define LINE_BQ_DSG_EN			PAL_LINE(GPIOB, 14)
#define LINE_BQ_PMON_EN			PAL_LINE(GPIOB, 11)
#define LINE_BQ_PCHG_EN			PAL_LINE(GPIOB, 12)

// LTC6813
#define LINE_LTC_CS				PAL_LINE(GPIOA, 4)
#define LINE_LTC_SCLK			PAL_LINE(GPIOA, 5)
#define LINE_LTC_MISO			PAL_LINE(GPIOA, 6)
#define LINE_LTC_MOSI			PAL_LINE(GPIOA, 7)
#define LTC_GPIO_CURR_MON		5

// CAN
#define LINE_CAN_RX				PAL_LINE(GPIOB, 8)
#define LINE_CAN_TX				PAL_LINE(GPIOB, 9)
#define HW_CAN_DEV				CAND1
#define HW_CAN_AF				9

// BQ76940
#define BQ76940_SDA_GPIO		GPIOB
#define BQ76940_SDA_PIN			11
#define BQ76940_SCL_GPIO		GPIOB
#define BQ76940_SCL_PIN			10
#define BQ76940_ALERT_GPIO		GPIOA
#define BQ76940_ALERT_PIN		2
#define BQ76940_LRD_GPIO		GPIOB
#define BQ76940_LRD_PIN			0

// Analog

#ifdef HDC1080_SDA_GPIO
#define LINE_V_CHARGE			PAL_LINE(GPIOC, 2)
#endif
//LINE_CURRENT not used in this hardware
#define LINE_CURRENT			PAL_LINE(GPIOC, 3)
#define LINE_TEMP_0				PAL_LINE(GPIOA, 3)
#define LINE_TEMP_1				PAL_LINE(GPIOA, 4)
#define LINE_TEMP_2				PAL_LINE(GPIOA, 5)
#define LINE_TEMP_3				PAL_LINE(GPIOA, 6)
#define LINE_TEMP_4				PAL_LINE(GPIOA, 7)
#define LINE_TEMP_5				PAL_LINE(GPIOC, 1)
#define LINE_TEMP_6 			PAL_LINE(GPIOC, 3)

//CANbus
#define HW_SET_CAN_ENABLE_LINE() palSetLineMode(LINE_CAN_EN, PAL_MODE_OUTPUT_PUSHPULL)
#define LINE_CAN_EN				PAL_LINE(GPIOB, 7)
#define HW_CAN_ON()				palSetLine(LINE_CAN_EN)
#define HW_CAN_OFF()			palClearLine(LINE_CAN_EN)

// Enable thermistor bank A
#define LINE_TEMP_0_EN			PAL_LINE(GPIOB, 1)

// Enable thermistor bank B. All repeadted until we make it more abstract
#define LINE_TEMP_1_EN			PAL_LINE(GPIOC, 2)
#define LINE_TEMP_2_EN			PAL_LINE(GPIOC, 2)
#define LINE_TEMP_3_EN			PAL_LINE(GPIOC, 2)
#define LINE_TEMP_4_EN			PAL_LINE(GPIOC, 2)
#define LINE_TEMP_5_EN			PAL_LINE(GPIOC, 2)
#define LINE_TEMP_6_EN			PAL_LINE(GPIOC, 2)

#define NTC_RES(adc)			(10000.0 * (float)adc / ( 4095.0 - (float)adc))
//#define NTC_RES(adc)			(10000.0 / ((4095.0 / (float)adc) - 1.0))
#define NTC_TEMP(adc)			(1.0 / ((logf(NTC_RES(adc) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15)

// Highest and lower cell temp
#define HW_TEMP_CELLS_MAX()		hw_luna_get_cell_temp_max()
#define HW_TEMP_CELLS_MIN()		hw_luna_get_cell_temp_min()

// ADC Channels
#define ADC_CH_V_CHARGE			ADC_CHANNEL_IN3
#define ADC_CH_CURRENT			ADC_CHANNEL_IN4
#define ADC_CH_TEMP0			ADC_CHANNEL_IN8  // Cell temp 1
#define ADC_CH_TEMP1			ADC_CHANNEL_IN9  // Cell temp 2
#define ADC_CH_TEMP2			ADC_CHANNEL_IN10 // Cell temp 3
#define ADC_CH_TEMP3			ADC_CHANNEL_IN11 // Cell temp 4
#define ADC_CH_TEMP4			ADC_CHANNEL_IN12 // Negative Connector terminal temp
#define ADC_CH_TEMP5			ADC_CHANNEL_IN2  // Positive Connector terminal temp
#define ADC_CH_TEMP6			ADC_CHANNEL_IN4  // MOSFET temp

void hw_luna_init(void);
float hw_luna_get_temp(int sensors);
float hw_luna_get_cell_temp_max(void);
float hw_luna_get_cell_temp_min(void);
float hw_luna_get_bal_temp (void);
float hw_luna_get_connector_temp(void);
#endif /* HWCONF_HW_LUNA_BMS_H_ */
