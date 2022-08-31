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

#ifndef HWCONF_HW_H_
#define HWCONF_HW_H_

#include <math.h>

#include HW_HEADER

// Default configuration overrides
#ifndef CONF_CELL_NUM
#define CONF_CELL_NUM 			HW_CELLS_SERIES
#endif

#ifndef CONF_CELL_FIRST_INDEX
#define CONF_CELL_FIRST_INDEX 	0
#endif

#ifndef CONF_EXT_SHUNT_RES
#define CONF_EXT_SHUNT_RES		HW_SHUNT_RES
#endif

#ifndef CONF_EXT_SHUNT_GAIN
#define CONF_EXT_SHUNT_GAIN		HW_SHUNT_AMP_GAIN
#endif

// Macros
#ifndef LED_OFF
#define LED_OFF(led)			palClearLine(led)
#endif
#ifndef LED_ON
#define LED_ON(led)				palSetLine(led)
#endif
#define LED_TOGGLE(led)			palToggleLine(led)

#ifndef HW_DEFAULT_ID
#define HW_DEFAULT_ID			(CONF_CONTROLLER_ID >= 0 ? CONF_CONTROLLER_ID : hw_id_from_uuid())
#endif

#ifndef HW_ADC_TEMP_SENSORS
#define HW_ADC_TEMP_SENSORS		6
#endif

#ifndef HW_MCU_ADC_TEMP_SENSOR
#define HW_MCU_ADC_TEMP_SENSOR HW_ADC_TEMP_SENSORS
#endif

#ifndef ADC_CH_TEMP6
#define ADC_CH_TEMP6			ADC_CH_TEMP5
#define LINE_TEMP_6				LINE_TEMP_5
#define LINE_TEMP_6_EN			LINE_TEMP_5_EN
#endif

#ifndef ADC_CH_V_FUSE
#define ADC_CH_V_FUSE			ADC_CH_V_CHARGE
#ifdef HDC1080_SDA_GPIO
#define LINE_V_FUSE				LINE_V_CHARGE
#endif
#endif

#ifndef HW_INIT_HOOK
#define HW_INIT_HOOK()
#endif

#ifndef LINE_CAN_EN
#define HW_CAN_ON()
#define HW_CAN_OFF()
#endif

#ifndef LINE_CURR_MEASURE_EN
#define CURR_MEASURE_ON()
#define CURR_MEASURE_OFF()
#endif

#ifndef HW_GET_V_CHARGE
#define HW_GET_V_CHARGE()		pwr_get_vcharge()
#endif

#ifndef HW_GET_BAL_TEMP
#define HW_GET_BAL_TEMP()		bms_if_get_humsens_temp_pcb()
#endif

#ifndef HW_SOC_OVERRIDE
#define HW_SOC_OVERRIDE()		-1.0
#endif

#ifndef HW_SEND_DATA
#define HW_SEND_DATA(send_func)
#endif

#ifndef HW_SEND_CAN_DATA
#define HW_SEND_CAN_DATA()
#endif

#ifndef HW_CHARGER_DETECTED
#define HW_CHARGER_DETECTED()	false
#endif

#ifndef NTC_TEMP_WITH_IND
#define NTC_TEMP_WITH_IND(adc, ind)	NTC_TEMP(adc)
#endif

#ifndef HW_MOSFET_SENSOR
#define HW_MOSFET_SENSOR()		0.0		// if not mosfet temp measurement return 0
#endif

#ifndef HW_CONNECTOR_TEMP
#define HW_CONNECTOR_TEMP()		0.0		// if not connector temp measurement return 0
#endif

#ifndef HW_VREGULATOR_TEMP
#define HW_VREGULATOR_TEMP()	0.0		// if not regulator temp measurement return 0
#endif

#ifndef HW_MAX_TEMP_IC
#define HW_MAX_TEMP_IC			120.0	// max internal AFE sensor temperature [°C]
#endif

#ifndef HW_MAX_MOSFET_TEMP
#define HW_MAX_MOSFET_TEMP		120.0	// max switch  temperature [°C]
#endif

#ifndef HW_MAX_CONNECTOR_TEMP
#define HW_MAX_CONNECTOR_TEMP	120.0	// max charge/discharge port temperature [°C]
#endif

#ifndef HW_MAX_VREG_TEMP
#define HW_MAX_VREG_TEMP		120.0	// max regulator temperature [°C]
#endif

#ifndef HW_MAX_OC_DISCHARGE_I
#define HW_MAX_OC_DISCHARGE_I	9.0 	// overcurrent current [A]
#endif

#ifndef HW_MAX_SC_DISCHARGE_I
#define HW_MAX_SC_DISCHARGE_I	44.0	// short circuit current [A]
#endif

#ifndef HW_HYSTERESIS_TEMP
#define HW_HYSTERESIS_TEMP		5.0		// temp hysteresis to clear over temp or unter temp [°C]
#endif

#ifndef MAX_RECONNECT_ATTEMPT
#define MAX_RECONNECT_ATTEMPT	3		// max reconnection attempts after a short circuit or overcurrent event
#endif

#ifndef RECONNECTION_TIMEOUT
#define RECONNECTION_TIMEOUT	3		// seconds to wait before reconnection attempt
#endif

#ifndef HW_LOAD_DETECTION
#define HW_LOAD_DETECTION()		1		//if no load detection is implemented, asume that the load is always present
#endif

#ifndef HW_CHARGER_DETECTION
#define HW_CHARGER_DETECTION()	1		//if no charger detecion is implemented assume that charger is always present
#endif

#ifndef HW_MIN_CELL
#define HW_MIN_CELL				2.8		//min cell voltage [V]
#endif

#ifndef HW_HYSTEREIS_MIN_CELL
#define HW_HYSTEREIS_MIN_CELL	0.05	//min cell hysteresis to clear undervoltage [V]
#endif

#ifndef HW_MAX_CELL
#define HW_MAX_CELL				4.2	//max cell voltage [V]
#endif

#ifndef HW_HYSTEREIS_MAX_CELL
#define HW_HYSTEREIS_MAX_CELL	0.05	//max cell hysteresis to clear undervoltage [V]
#endif

#ifndef HW_WAIT_AFE
#define HW_WAIT_AFE() 			__NOP();//null
#endif
// Functions
uint8_t hw_id_from_uuid(void);

#endif /* HWCONF_HW_H_ */
