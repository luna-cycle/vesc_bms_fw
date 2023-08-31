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

#include "i2c_bb.h"
#include "bq76940.h"
#include "bms_if.h"
#include "string.h"
#include "stdbool.h"
#include "timeout.h"
#include "utils.h"
#include "main.h"
#include "math.h"
#include "stm32l4xx_hal.h"
#include "sleep.h"
#include "conf_general.h"
#include "chbsem.h"
#include "ch.h"
#include "chtime.h"
#include "stdlib.h"
#ifdef HW_HAS_BQ76940

#define MAX_CELL_NUM		15
#define MAX_TEMP_SENSORS_NUM	3
#define BQ_I2C_ADDR			0x08
#define CC_REG_TO_AMPS_FACTOR   0.00000844
#define CC_VBAT_TO_VOLTS_FACTOR 0.001532

#define PROTECT1_SEL		(CURRENT_112A | BQ_SCP_70us)
#define PROTECT2_SEL		(CURRENT_72A | BQ_OCP_640ms)
#define PROTECT3_SEL		(BQ_UV_DELAY_1s | BQ_OV_DELAY_1s)
#define OV_TRIP_SEL			tripVoltage(HW_MAX_CELL)
#define UV_TRIP_SEL			tripVoltage(HW_MIN_CELL)

// Private variables
static i2c_bb_state  m_i2c;
static float m_v_cell[MAX_CELL_NUM];
static volatile bool m_discharge_state[MAX_CELL_NUM] = {false};
static volatile bool m_discharge[MAX_CELL_NUM] = {false};
typedef struct {
	i2c_bb_state m_i2c;
	bool mutex;
	bool initialized;
	uint8_t status;
	int lrd_pin;
	float shunt_res;
	float gain;
	float offset;
	float CC;
	float temp[MAX_TEMP_SENSORS_NUM];
	float temp_ic;
	float cell_mv[MAX_CELL_NUM];
	uint32_t padding;
	float pack_mv;
	bool m_discharge_state[MAX_CELL_NUM];
	bool discharge_enabled;
	bool charge_enabled;
	bool is_connected_pack;
	bool request_connection_pack;
	//fault_data fault;
	bool is_load_present;
	bool oc_detected;
	bool sc_detected;
	bool UV_detected;
	bool OV_detected;
	bool connect_only_charger;
	bool discharge_allowed;
	uint8_t re_init_retry;
	float fault_current_in;
	float fault_v_min;
	float fault_v_max;
} bq76940_t;

// The config is stored in the backup struct so that it is stored while sleeping.
static volatile bq76940_t *bq76940 = (bq76940_t*)&backup.hw_config[0];

//static void fault_data_cb(fault_data * const fault) {
	//bq76940->fault = *fault;
//}

static I2CConfig i2cfg1 = {
	STM32_TIMINGR_PRESC(4U) | // Uing 16MHZ HSI clock to source I2CCLK.
	STM32_TIMINGR_SCLDEL(4U) | STM32_TIMINGR_SDADEL(2U) |
	STM32_TIMINGR_SCLH(15U) | STM32_TIMINGR_SCLL(13U),
						// Just scale bits 28..31 if not 8MHz clock
	0,					// CR1
	0,					// CR2
};

// Threads
static THD_WORKING_AREA(sample_thread_wa, 512);
static THD_FUNCTION(sample_thread, arg);

// Private functions
float bq_read_CC(void);
int8_t bq_read_gain(float *gain);
int8_t bq_read_offsets(float *offset);
void bq_read_temps(volatile float *temps);
float bq_read_temp_ic(void);
void iin_measure(float *value_iin);
uint8_t write_reg(uint8_t reg, uint16_t val);
static void read_cell_voltages(float *m_v_cell);
static void read_v_batt(volatile float *v_bat);
uint8_t read_reg(uint8_t reg);
uint8_t CRC8(unsigned char *ptr, unsigned char len,unsigned char key);
void bq_balance_cells(volatile bool *m_discharge_state, volatile float *m_v_cell, volatile bool *m_discharge_filtering);
uint8_t tripVoltage(float voltage);
void bq_disconnect_battery(bool disconnect);
bool status_load_present(void);
binary_semaphore_t bq_alert_semph;

//Macros
#define READ_ALERT()	palReadPad(BQ76940_ALERT_GPIO, BQ76940_ALERT_PIN)

uint8_t bq76940_init(void) {
	float gain;
	float offset;

	palSetLineMode(LINE_LED_RED_DEBUG, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_LED_GREEN_DEBUG, PAL_MODE_OUTPUT_PUSHPULL);

	bq76940->shunt_res = HW_SHUNT_RES;

	palSetPadMode(BQ76940_ALERT_GPIO, BQ76940_ALERT_PIN, PAL_MODE_INPUT);

	memset(&m_i2c, 0, sizeof(i2c_bb_state));
	m_i2c.sda_gpio = BQ76940_SDA_GPIO;
	m_i2c.sda_pin = BQ76940_SDA_PIN;
	m_i2c.scl_gpio = BQ76940_SCL_GPIO;
	m_i2c.scl_pin = BQ76940_SCL_PIN;

	palSetPadMode(GPIOB, 10, PAL_STM32_OTYPE_OPENDRAIN
							| PAL_MODE_ALTERNATE(4) );
	palSetPadMode(GPIOB, 11, PAL_STM32_OTYPE_OPENDRAIN
							| PAL_MODE_ALTERNATE(4) );
	i2cStart(&I2CD2, &i2cfg1);
	//This variable is to detect any error in the bq's initialized
	//error = 0 -> succeful
	//error = 1 -> problem
	uint8_t error = 0;

	// make sure the bq is booted up--->set TS1 to 3.3V and back to VSS
	// maybe set temp-mode for internal or external temp here

	// these AFE registers should only be initialized once when the board is powered up
	// the MCU undergoes a full reset every 10 seconds or so, check ram4 to see if the
	// AFE was already initialized / i can't get this to work yet /
	
	error |= bq_read_gain(&gain);
	error |= bq_read_offsets(&offset);

	bq76940->gain = gain;
	bq76940->offset = offset;	// for some reason direct pointer reference crashes it
	// If SYS_CTRL1 and SYS_CTRL2 are 0x00 it means that the AFE is not configured. Lets configure it.
	// if we want to change the AFE configuration we have to reset it to default values and then it will be
	// rećonfigured on the next MCU reset.
	//TODO: #define registers values

	// enable countinous reading of the Coulomb Counter
	uint8_t val = read_reg(BQ_SYS_CTRL2);
	val = val | CC_EN;
	write_reg(BQ_SYS_CTRL2, val);	// sets ALERT at 250ms interval
	// write 0x19 to CC_CFG according to datasheet page 39
	write_reg(BQ_CC_CFG, 0x19);

	//Connect the pack
	bq76940->request_connection_pack  = true;
	bq76940->discharge_allowed = false;
	if((PWR->SR1 & 0x1) ){
		sleep_reset();
	}

	//bq76940_wfe();// sleep until alert pin is high

	if( ( (read_reg(BQ_PROTECT1)) != PROTECT1_SEL)  || (read_reg(BQ_PROTECT2) != PROTECT2_SEL) ||
			(read_reg(BQ_PROTECT3) != PROTECT3_SEL) || (read_reg(BQ_OV_TRIP) != OV_TRIP_SEL) ||
			(read_reg(BQ_UV_TRIP) != UV_TRIP_SEL) || (read_reg(BQ_SYS_CTRL1) == 0x0) ) {

		// enable ADC and thermistors
		error |= write_reg(BQ_SYS_CTRL1, (ADC_EN | TEMP_SEL));

		// write 0x19 to CC_CFG according to datasheet page 39
		error |= write_reg(BQ_CC_CFG, 0x19);

		//OverVoltage threshold
		error |= write_reg(BQ_OV_TRIP, OV_TRIP_SEL);
		//UnderVoltage threshold
		error |= write_reg(BQ_UV_TRIP, UV_TRIP_SEL);
		error |= write_reg(BQ_PROTECT3, PROTECT3_SEL);

		// Overcurrent protection
		error |= write_reg(BQ_PROTECT2, PROTECT2_SEL);
		// Short Circuit Protection
		//You can set the shortcircuit protection with ...
		//The delay time could be 70us, 100us, 200us or 400 us
		error |= write_reg(BQ_PROTECT1, PROTECT1_SEL);

		// clear SYS-STAT for init
		write_reg(BQ_SYS_STAT,0xFF);

		// enable countinous reading of the Coulomb Counter
		error |= write_reg(BQ_SYS_CTRL2, CC_EN);	// sets ALERT at 250ms interval

		chThdSleepMilliseconds(10);
		write_reg(BQ_SYS_STAT,0xFF);
		chThdSleepMilliseconds(10);

		//Connect the pack
		bq_connect_pack(true);

		// Mark the AFE as initialized for the next post-sleep resets
		if (error == 0) {
			bq76940->initialized = true;
		} else {
			bq76940->initialized = false;
		}

	}
	//init semaphore as taken
	chBSemObjectInit(&bq_alert_semph, false);
	chThdCreateStatic(sample_thread_wa, sizeof(sample_thread_wa), NORMALPRIO +2 , sample_thread, NULL);

	return error;
}

// This function will be executed when the Alert pin is driven to '1' by the AFE
void bq76940_Alert_handler(void) {

	// Read Status Register
	uint8_t sys_stat = read_reg(BQ_SYS_STAT);
	float v_aux = 0.0;

	// Report fault codes
	if ( sys_stat & SYS_STAT_DEVICE_XREADY ) {
		//handle error
		bq76940->request_connection_pack = false;
	}

	if ( sys_stat & SYS_STAT_OVRD_ALERT ) {
		//handle error
		bq76940->request_connection_pack = false;
	}

	if ( sys_stat & SYS_STAT_UV ) {
		//bms_if_fault_report(FAULT_CODE_CELL_UNDERVOLTAGE);
		bq76940->UV_detected = true;
		bq76940->request_connection_pack = false;
		read_cell_voltages(m_v_cell);

		// acquire min cell
		v_aux = 100.0;
		for (int i = backup.config.cell_first_index;i <
		(backup.config.cell_num + backup.config.cell_first_index);i++) {
			if (HW_LAST_CELL_VOLTAGE(i) < v_aux) {
				v_aux = HW_LAST_CELL_VOLTAGE(i);
			}
		}
		bq76940->fault_v_min = v_aux;
	}

	if ( sys_stat & SYS_STAT_OV ) {
		//bms_if_fault_report(FAULT_CODE_CELL_OVERVOLTAGE);
		bq76940->OV_detected = true;
		bq76940->request_connection_pack = false;
		read_cell_voltages(m_v_cell);

		// acquire max cell
		v_aux = 0.0;
		for (int i = backup.config.cell_first_index;i < (backup.config.cell_num + backup.config.cell_first_index) ;i++ ) {
			if (HW_LAST_CELL_VOLTAGE(i) > v_aux) {
				v_aux = HW_LAST_CELL_VOLTAGE(i);
			}
		}
		bq76940->fault_v_max = v_aux;
	}

	if ( sys_stat & SYS_STAT_SCD ) {
		//bms_if_fault_report(FAULT_CODE_DISCHARGE_SHORT_CIRCUIT);
		bq76940->sc_detected =  true;
		bq76940->request_connection_pack = false;

		bq76940->fault_current_in = bq_read_CC();
	}

	if ( sys_stat & SYS_STAT_OCD ) {
		//bms_if_fault_report(FAULT_CODE_DISCHARGE_OVERCURRENT);
		bq76940->oc_detected =  true;
		bq76940->request_connection_pack = false;

		bq76940->fault_current_in = bq_read_CC();
	}

	if( bq76940->initialized == false ) {
		//bms_if_fault_report(FAULT_CODE_DONT_INIT_AFE);
		bq76940->request_connection_pack = false;
	}
	// Clear Status Register. This will clear the Alert pin so its ready
	// for the next event in 250ms
	write_reg(BQ_SYS_STAT,0xFF);

	//Read pack current
	bq76940->CC = bq_read_CC();

	//Here execute the connection or disconnection of back to back mosfet
	bq_connect_pack(bq76940->request_connection_pack);
	
	// Every 1 second make the long read
	static uint8_t i = 0;
	//Here I'll select the sequence of no balance, measure, and then balance.
	if( i == 7 ){ //>1,75 seconds -> no balance
		write_reg(BQ_CELLBAL1, 0x00);
		write_reg(BQ_CELLBAL2, 0x00);
		write_reg(BQ_CELLBAL3, 0x00);		
	}
	if ( i++ == 10 ){	//>2,5 seconds -> measure
		read_cell_voltages(m_v_cell); 	//read cell voltages
		read_v_batt(&bq76940->pack_mv);		
		i = 0;
	}
	if( i <= 6 ){ //<1,5 seconds -> balance
		bq_balance_cells(m_discharge_state, m_v_cell, m_discharge);	//configure balancing bits over i2c
	}
	
	//read external temp for 2.5 sec, then internal temp for 2.5sec and repeat
	static uint8_t temp_sensing_state = 1;
	if( temp_sensing_state == 0 ) {
			//read internal temperature
			bq76940->temp_ic = bq_read_temp_ic();
			//configure AFE so after 2 seconds the external temperature will become available
			write_reg(BQ_SYS_CTRL1, (ADC_EN | TEMP_SEL));
	}

	if( temp_sensing_state == 10 ) {
			//read external temperatures
			bq_read_temps(bq76940->temp);
			//configure AFE so after 2 seconds the internal temperature will become available
			write_reg(BQ_SYS_CTRL1, ADC_EN);
	}
	temp_sensing_state++;

	if( temp_sensing_state == 20 ) {
		temp_sensing_state = 0;
	}

	bq76940->is_load_present = status_load_present();

}

uint16_t afe_pool_count = 0;
static THD_FUNCTION(sample_thread, arg) {
	(void)arg;
	chRegSetThreadName("BQ76940");

	while ( !chThdShouldTerminateX() ) {

		m_i2c.has_error = 0;
		afe_pool_count = 0;

		while( !palReadPad(GPIOA,2U) ){

			if( afe_pool_count > 20 ) {// if code reach here, the AFE is not active
				write_reg(BQ_SYS_STAT,0xFF);
				bms_if_fault_report(FAULT_CODE_DONT_INIT_AFE);
				afe_pool_count = 0;
				// reinit bq76940
				bq76940_init();

				while( 1 ) {chThdSleepMilliseconds(1000);}// loop here and wait for WD for restart
			}
			timeout_feed_WDT(THREAD_AFE);
			chThdSleepMilliseconds(25);
			afe_pool_count++;
		}
		bq76940_Alert_handler();
		chBSemSignal(&bq_alert_semph); // give semaphore
	}
}

uint8_t write_reg(uint8_t reg, uint16_t val) {

	uint8_t error;
	uint8_t txbuf[3];
	uint8_t buff[4];
	uint8_t retry = 0;
	uint8_t key = 0x7;

	buff[0] = BQ_I2C_ADDR << 1;
	buff[1] = reg;
	buff[2] = val;
	txbuf[0] = reg;
	txbuf[1] = val;
	txbuf[2] = CRC8(buff, 3, key);
	i2cMasterTransmit(&I2CD2, BQ_I2C_ADDR, txbuf, 3, NULL, 0);
	error = i2cGetErrors(&I2CD2);

	while( error != 0 ){// handle errors

		buff[0] = BQ_I2C_ADDR << 1;
		buff[1] = reg;
		buff[2] = val;
		txbuf[0] = reg;
		txbuf[1] = val;
		key = 0x7;
		txbuf[2] = CRC8(buff, 3, key);
		i2cMasterTransmit(&I2CD2, BQ_I2C_ADDR, txbuf, 3, NULL, 0);
		error = i2cGetErrors(&I2CD2);
		retry++;

		if(retry > I2C_MAX_RETRY){
			// enter in shipmode
			// re write buffers for safety
			buff[0] = BQ_I2C_ADDR << 1;
			buff[1] = BQ_SYS_CTRL1;
			buff[2] = 0x00;
			txbuf[0] = BQ_SYS_CTRL1;
			txbuf[1] = 0x00;
			key = 0x7;
			txbuf[2] = CRC8(buff, 3, key);
			i2cMasterTransmit(&I2CD2, BQ_I2C_ADDR, txbuf, 3, NULL, 0);

			buff[2] = 0x01;
			txbuf[1] = 0x01;
			txbuf[2] = CRC8(buff, 3, key);
			i2cMasterTransmit(&I2CD2, BQ_I2C_ADDR, txbuf, 3, NULL, 0);

			buff[2] = 0x02;
			txbuf[1] = 0x02;
			txbuf[2] = CRC8(buff, 3, key);
			i2cMasterTransmit(&I2CD2, BQ_I2C_ADDR, txbuf, 3, NULL, 0);

			// if ship mode atempt fails, reset MCU
			backup.ah_cnt_init_flag = 0; // force to re load backup data from flash after reboot
			backup.ah_cnt_init_flag  = 0;
			backup.wh_cnt_init_flag  = 0;
			backup.ic_i_sens_v_ofs_init_flag = 0;
			backup.controller_id_init_flag = 0;
			backup.send_can_status_rate_hz_init_flag = 0;
			backup.can_baud_rate_init_flag = 0;
			backup.conf_flash_write_cnt_init_flag = 0;
			backup.usb_cnt_init_flag = 0;
			backup.hw_config_init_flag = 0;
			backup.ah_cnt_chg_total_init_flag = 0;
			backup.wh_cnt_chg_total_init_flag = 0;
			backup.ah_cnt_dis_total_init_flag = 0;
			backup.wh_cnt_dis_total_init_flag = 0;
			NVIC_SystemReset();
		}
	}

	return error;
}

uint8_t read_reg(uint8_t reg){
	uint8_t data[2];
	uint8_t buff[3];
	uint8_t txbuf[3];
	uint8_t crc;
	uint8_t key = 0x7;
	uint8_t error = 0;
	uint8_t retry = 0;

	i2cMasterTransmit(&I2CD2, BQ_I2C_ADDR, &reg, 1, data, 2);
	buff[0] = (BQ_I2C_ADDR << 1) + 1;
	buff[1] = data[0];
	crc = CRC8(buff,2,key);
	error = i2cGetErrors(&I2CD2);

	while((crc != data[1]) || (error != 0)){
		//handle CRC error

		i2cMasterTransmit(&I2CD2, BQ_I2C_ADDR, &reg, 1, data, 2);
		buff[0] = (BQ_I2C_ADDR << 1) + 1;
		buff[1] = data[0];
		crc = CRC8(buff,2,key);
		error = i2cGetErrors(&I2CD2);
		retry++;

		if(retry > I2C_MAX_RETRY){

			// enter in shipmode
			buff[0] = BQ_I2C_ADDR << 1;
			buff[1] = BQ_SYS_CTRL1;
			buff[2] = 0x00;
			txbuf[0] = BQ_SYS_CTRL1;
			txbuf[1] = 0x00;
			key = 0x7;
			txbuf[2] = CRC8(buff, 3, key);
			i2cMasterTransmit(&I2CD2, BQ_I2C_ADDR, txbuf, 3, NULL, 0);

			buff[2] = 0x01;
			txbuf[1] = 0x01;
			txbuf[2] = CRC8(buff, 3, key);
			i2cMasterTransmit(&I2CD2, BQ_I2C_ADDR, txbuf, 3, NULL, 0);

			buff[2] = 0x02;
			txbuf[1] = 0x02;
			txbuf[2] = CRC8(buff, 3, key);
			i2cMasterTransmit(&I2CD2, BQ_I2C_ADDR, txbuf, 3, NULL, 0);

			// if ship mode atempt fails, reset MCU
			backup.ah_cnt_init_flag = 0; // force to re load backup data from flash after reboot
			backup.ah_cnt_init_flag  = 0;
			backup.wh_cnt_init_flag  = 0;
			backup.ic_i_sens_v_ofs_init_flag = 0;
			backup.controller_id_init_flag = 0;
			backup.send_can_status_rate_hz_init_flag = 0;
			backup.can_baud_rate_init_flag = 0;
			backup.conf_flash_write_cnt_init_flag = 0;
			backup.usb_cnt_init_flag = 0;
			backup.hw_config_init_flag = 0;
			backup.ah_cnt_chg_total_init_flag = 0;
			backup.wh_cnt_chg_total_init_flag = 0;
			backup.ah_cnt_dis_total_init_flag = 0;
			backup.wh_cnt_dis_total_init_flag = 0;
			NVIC_SystemReset();
		}
	}
	return data[0];
}

int8_t bq_read_gain(float *gain){
	int8_t error = 0;
	uint8_t reg1 = read_reg(BQ_ADCGAIN1);
	uint8_t reg2 = read_reg(BQ_ADCGAIN2);

	reg1 &= 0x0C;
	*gain = (365.0 + ((reg1 << 1) | (reg2 >> 5)));

	if ((*gain < 365) | (*gain > 396)) {
		error = BQ76940_FAULT_GAIN;
	}
	return error;
}

// convert a voltage into the format used by the trip registers
uint8_t tripVoltage(float threshold) {
	uint32_t reg_val = (uint16_t)(threshold * 1000.0);
	reg_val -= (bq76940->offset * 1000);
	reg_val /= ( bq76940->gain / 1e3);
	reg_val >>= 4;
	reg_val &= 0x00FF;
	return ((uint8_t)reg_val);
}

int8_t bq_read_offsets(float *offset){
	int8_t error = 0;
	*offset = ((float)read_reg(BQ_ADCOFFSET) / 1000.0);
//this will never be false. its also comparing a float with a hex int?
	if( (*offset < 0x00) | (*offset > 0xFF) ) {
		error = BQ76940_FAULT_OFFSET;
	}
	return error;
}

uint8_t CRC8(uint8_t *ptr, uint8_t len,uint8_t key){
	uint8_t  i;
	uint8_t  crc=0;

	while( len-- != 0 ) {
		for( i=0x80 ; i!=0; i/=2 ) {
			if ((crc & 0x80) != 0 ) {
				crc *= 2;
				crc ^= key;
			} else {
				crc *= 2;
			}
			if ( (*ptr & i) != 0 ) {
				crc ^= key;
			}
		}
		ptr++;
	}

	return(crc);
}

static void read_cell_voltages(float *m_v_cell) {
	float 	cell_voltages[MAX_CELL_NUM];

	for ( int i=0 ; i < MAX_CELL_NUM ; i++) {
		uint16_t VCx_lo = read_reg(BQ_VC1_LO + i * 2);
		uint16_t VCx_hi = read_reg(BQ_VC1_HI + i * 2);
		cell_voltages[i] = (((((float)(VCx_lo | (VCx_hi << 8))) * bq76940->gain) / 1e6)) + bq76940->offset;
	}
	
	// For 14s setups, handle the special case of cell 14 connected to VC15
	cell_voltages[13] = cell_voltages[14];
	
	memcpy( m_v_cell, cell_voltages, sizeof(cell_voltages) );
}

float bq_last_cell_voltage(int cell) {
	if ( cell < 0 || cell >= MAX_CELL_NUM ) {
		return -1.0;
	}

	return m_v_cell[cell];
}

float bq_last_pack_voltage(void) {
	return  bq76940->pack_mv;
}

// BQ76940 updates the temperatur registers every 2 seconds
void bq_read_temps(volatile float *temps) {
	// only update TS3, the others are unused in this particular hardware
	// make sure TEMP_SEL bit has been set to 1 for at leat 2 seconds

	for( int i = 2 ; i < 3 ; i++ ) {
		uint16_t BQ_TSx_hi = read_reg(BQ_TS1_HI + i * 2 );
		uint16_t BQ_TSx_lo = read_reg(BQ_TS1_LO + i * 2);
		float vtsx = (float)((BQ_TSx_hi << 8) | BQ_TSx_lo) * 0.000382;
		float R_ts = ( vtsx * 1e4 ) / ( 3.3 - vtsx );
		temps[i] = (1.0 / ((logf(R_ts / 10000.0) / 3455.0) + (1.0 / 298.15)) - 273.15);
	}
	return;
}

float bq_read_temp_ic(void) {
	//we are only reading TS2, but internal die temp is available for TS1, TS2 and TS3 dies.
	// make sure TEMP_SEL bit has been set to 0 for at leat 2 seconds
	uint16_t BQ_TSx_hi = read_reg(BQ_TS1_HI + 2);
	uint16_t BQ_TSx_lo = read_reg(BQ_TS1_LO + 2);

	float vtsx = (float)((BQ_TSx_hi << 8) | BQ_TSx_lo) * 0.000382;
	float temp_die = 25.0 - ((vtsx - 1.2) / 0.0042);
	return temp_die;
}

float bq_get_temp(int sensor) {
	if ( sensor < 0 || sensor >= 5 ) {
			return -1.0;
	}
	return bq76940->temp[sensor];
}

float bq_get_temp_ic(void) {
	return bq76940->temp_ic;
}

// read Coloumb Counter. This should be called every 250ms by the Alert interrupt
float bq_read_CC(void) {
	uint16_t CC_hi = read_reg(BQ_CC_HI);
	uint16_t CC_lo = read_reg(BQ_CC_LO);
	int16_t CC_reg = (int16_t)(CC_lo | CC_hi << 8);
	
	return (float)CC_reg * CC_REG_TO_AMPS_FACTOR / bq76940->shunt_res;
}

float bq_get_current(void){
	return bq76940->CC;
}

void bq_request_connect_pack(bool flag) {
	bq76940->request_connection_pack = flag;
}

void bq_connect_pack(bool request) {
	if(request) {
		if( bq76940->connect_only_charger ) {
			bq_discharge_disable();
		} else {
			if( bq76940->discharge_allowed ) { //ask for precharge condition
				bq_discharge_enable();
			} else {
				bq_discharge_disable();
			}
			bq_charge_enable();
		}
	} else {
		bq_discharge_disable();
		bq_charge_disable();
	}
}

void bq_discharge_enable(void) {
	uint8_t data = read_reg(BQ_SYS_CTRL2);
	data = data | 0x02;
	write_reg(BQ_SYS_CTRL2, data);
	bq76940->discharge_enabled = true;
	return;
}

void bq_discharge_disable(void) {
	uint8_t data = read_reg(BQ_SYS_CTRL2);
	data = data & 0xFD;
	write_reg(BQ_SYS_CTRL2, data);
	bq76940->discharge_enabled = false;
	return;
}

void bq_charge_enable(void) {
	uint8_t data = read_reg(BQ_SYS_CTRL2);
	data = data | 0x01;
	write_reg(BQ_SYS_CTRL2, data);
	bq76940->charge_enabled = true;
	return;
}

void bq_charge_disable(void) {
	uint8_t data = read_reg(BQ_SYS_CTRL2);
	data = data & 0xFE;
	write_reg(BQ_SYS_CTRL2, data);
	bq76940->charge_enabled = false;
}

void bq_set_dsc(int cell, bool set) {
	if (cell < 0 || cell >= MAX_CELL_NUM) {
		return;
	}

	m_discharge_state[cell] = set;
}

bool bq_get_dsc(int cell) {
	if (cell < 0 || cell >= MAX_CELL_NUM) {
		return false;
	}

	return m_discharge[cell];
}

void bq_balance_cells(volatile bool *m_discharge_state, volatile float *m_v_cell, volatile bool *m_discharge) {
	uint8_t buffer[3]= {0,0,0};
	int i, j;
	int sorted_indexes[MAX_CELL_NUM];

	// Initialize the sorted indices to the original indices
	for (i = 0; i < MAX_CELL_NUM; i++) {
		sorted_indexes[i] = i;
	}

	// Sort the cells that need balancing by voltage (highest to lowest)
	for (i = 0; i < MAX_CELL_NUM - 1; i++) {
		for (j = i + 1; j < MAX_CELL_NUM; j++) {
			if (m_v_cell[sorted_indexes[i]] < m_v_cell[sorted_indexes[j]]) {
				int temp = sorted_indexes[i];
				sorted_indexes[i] = sorted_indexes[j];
				sorted_indexes[j] = temp;
			}
		}
	}

	// Copy the array
	for (i = 0; i < MAX_CELL_NUM ; i++) {
		m_discharge[i] = m_discharge_state[i];
	}

	// Disable a cell balancing if a higher priority cell is contiguous and is scheduled for balancing
	for (i = 1; i < MAX_CELL_NUM ; i++) {
		for (j = 0; j < i; j++) {
			if (abs(sorted_indexes[i] - sorted_indexes[j]) <= 1 && (m_discharge_state[sorted_indexes[j]] == 1)) {
				m_discharge[sorted_indexes[i]] = 0;
			}
		}
	}

	if (m_discharge[0]) buffer[0] = buffer[0] | 0x01;
	if (m_discharge[1]) buffer[0] = buffer[0] | 0x02;
	if (m_discharge[2]) buffer[0] = buffer[0] | 0x04;
	if (m_discharge[3]) buffer[0] = buffer[0] | 0x08;
	if (m_discharge[4]) buffer[0] = buffer[0] | 0x10;
	write_reg(BQ_CELLBAL1, buffer[0]);

	if (m_discharge[5]) buffer[1] = buffer[1] | 0x01;
	if (m_discharge[6]) buffer[1] = buffer[1] | 0x02;
	if (m_discharge[7]) buffer[1] = buffer[1] | 0x04;
	if (m_discharge[8]) buffer[1] = buffer[1] | 0x08;
	if (m_discharge[9]) buffer[1] = buffer[1] | 0x10;
	write_reg(BQ_CELLBAL2, buffer[1]);

	if (m_discharge[10]) buffer[2] = buffer[2] | 0x01;
	if (m_discharge[11]) buffer[2] = buffer[2] | 0x02;
	if (m_discharge[12]) buffer[2] = buffer[2] | 0x0C; // Special case for a 14s setup
	if (m_discharge[13]) buffer[2] = buffer[2] | 0x10;
	write_reg(BQ_CELLBAL3, buffer[2]);

	return;
}

bool status_load_present() {
	uint8_t load_present = ((read_reg(BQ_SYS_CTRL1)) & 0X80 );
	if( load_present == 0x80 ) {
		return true;
	} else {
		return false;
	}
}

static void read_v_batt(volatile float *v_bat) {
	uint16_t BAT_hi = read_reg(BQ_BAT_HI);
	uint16_t BAT_lo = read_reg(BQ_BAT_LO);

	*v_bat = (float)(((uint16_t)(BAT_lo | BAT_hi << 8)) * CC_VBAT_TO_VOLTS_FACTOR )-(14 * bq76940->offset);
}

void sleep_bq76940() {

	while(read_reg(BQ_SYS_CTRL2) != 0x0){// read CTR2 reg to be sure that CHG is disable
		write_reg(BQ_SYS_CTRL2, 0x0);
	}
	while( !palReadPad(GPIOA,2U) ){

	}
	write_reg(BQ_SYS_STAT,0xFF);

}

void bq_shutdown_bq76940(void) {
	//Shutdown everything frontend
	write_reg(BQ_SYS_CTRL1, 0x00);
	write_reg(BQ_SYS_CTRL1, 0x01);
	write_reg(BQ_SYS_CTRL1, 0x02);
}

float bq_get_CC_raw(void) {
	return bq_get_current() * bq76940->shunt_res / CC_REG_TO_AMPS_FACTOR;
}

bool bq_get_load_status(void) {
	return bq76940->is_load_present;
}

bool bq_oc_detected (void) {
	return bq76940->oc_detected;
}

bool bq_sc_detected (void) {
	return bq76940->sc_detected;
}

void bq_restore_oc_sc_fail() {
	bq76940->oc_detected = false;
	bq76940->sc_detected = false;
}

bool bq_ov_detected(void) {
	return bq76940->OV_detected;
}

bool bq_uv_detected(void) {
	return bq76940->UV_detected;
}

void bq_restore_ov_fault() {
	bq76940->OV_detected = false;
}

void bq_restore_uv_fault() {
	bq76940->UV_detected = false;
}

void bq_connect_only_charger (bool request) {
	bq76940->connect_only_charger = request;
}

void bq_semaphore (void){
	chBSemWaitTimeout(&bq_alert_semph, chTimeMS2I(500));// time out 500mS
}

void bq_allow_discharge(bool set) {
	bq76940->discharge_allowed = set;
}

float bq_get_fault_data_current(void){
	return bq76940->fault_current_in;
}

float bq_get_fault_data_UV(void){
	return bq76940->fault_v_min;
}

float bq_get_fault_data_OV(void){
	return bq76940->fault_v_max;
}

void bq76940_wfe(void){
	//PWR->CR4 |= PWR_CR4_WP4;//wkp event rising edge
	PWR->CR3 |= PWR_CR3_EWUP4;// Enable Wakeup pin WKUP4
	PWR->SCR |= PWR_SCR_CWUF; // clear wkp flags
	PWR->CR1 |= PWR_CR1_LPMS_STOP2;// select lowpower mode STOP2, SRAM1, SRAM2 and register are preserved
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;// enable low power mode
	__WFE();
}
#endif
