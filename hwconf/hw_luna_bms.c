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

static void terminal_cmd_set_initial_assist_level(int argc, const char **argv);

void hw_luna_init(void){
    bq76940_init();
	terminal_register_command_callback("shipmode", "Shipmode = turn off bq 76940", 0, terminal_cmd_set_initial_assist_level);	
//    terminal_register_command_callback("Level trip", "Shunt Resistors", 0, terminal_cmd_set_initial_assist_level);	
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


    
static void terminal_cmd_set_initial_assist_level(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	bq_shutdown_bq76940();
	
	return;
}

static void terminal_cmd_read_initial_assist_level(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	commands_printf("BBSHD initial assist level is set at %i", hw_read_initial_assist_level());
	commands_printf(" ");
	return;
}
