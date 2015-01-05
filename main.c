/*
 * main.c
 *
 *  Created on: Sep 11, 2014
 *      Author: jcobb
 */

#define F_CPU	8000000

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/twi.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include "i2c/i2c.h"
#include "util/log.h"
#include "util/clock.h"
#include "util/config.h"
#include "imu/imu.h"
#include "imu/gyro.h"
//#include "pwm/pwm.h"
#include "eeprom/eeprom.h"
//#include "blc/blc.h"
#include "gimbal/gimbal.h"



// log debugging
static const char _tag[] PROGMEM = "main: ";
volatile char term_in = 0;

// local porototypes
void balance();
int32_t calc_pid();

void terminal_in_cb(uint8_t c)
{
	term_in = c;


	//LOG("input=%c\r\n", c);


}

// https://github.com/TKJElectronics/KalmanFilter/tree/master/examples
// https://github.com/TKJElectronics/BalancingRobotArduino/blob/master/BalancingRobotArduino.ino
// https://github.com/TKJElectronics/KalmanFilter

void main()
{

	debug_init(terminal_in_cb);
	clock_init();
	sei();

	LOG("\r\n\r\nspike_328p_imu starting...\r\n");

	/*
	 * load configuration
	 */
	LOG("config_init...\r\n");
	config_init();


	/* --- i2c/gyro/imu initialization ---*/
	// First we need to join the I2C bus
	LOG("joining i2c bus...\r\n");
	i2c_begin();
	LOG("initializing resolution divider...\r\n");
	init_resolution_divider();
	LOG("imu_init...\r\n");
	imu_init();
	// Make sure we are able to communicate with imu
	LOG("running imu test...\r\n");
	if(imu_test()){
		LOG("imu test pass.\r\n");
		if(config.gyro_calibrate == true){
			LOG("running gyro offset calibration...\r\n");
			gyro_offset_calibration();
			LOG("config.gyro_offset[x,y,z]: %d %d %d\r\n", config.gyro_offset_x, config.gyro_offset_y, config.gyro_offset_z);
		}
	} else {
		LOG("imu test failed!\r\n");
	}

	/* --- end i2c/gyro/imu initialization ---*/


	// set sensor orientation
	LOG("init_sensor_orientation...\r\n");
	init_sensor_orientation();

	/*
	 * init pid parameters
	 */
	LOG("init_pids...\r\n");
	init_pids();

	// TODO: Not implemented
	// Init rc variables
	//init_rc();
	// Init rc-input
	//init_rc_pins();

	/*
	 * gimbal is the main state machine
	 * for processing stabilization
	 */
	LOG("gimbal_init...\r\n");
	gimbal_init();


	LOG("starting gimbal loop...\r\n");

	int16_t axis_rotation[3];
	while(1)
	{
		if(term_in == (char)'+') {
			set_flt_alpha(.01f);
			term_in = 0;
		}
		else if (term_in == (char)'-') {
			set_flt_alpha(-.01f);
			term_in = 0;
		}

		gimbal_tick();
		balance();
	}
}


// pid defines
#define k_p		 7
#define k_i		 1
#define k_d		-5

int32_t center_val = 0;


int32_t error = 0;
int32_t proportion = 0;
int32_t integral = 0;
int32_t derivative = 0;
int32_t output = 0;
int32_t prev_error = 0;


void balance()
{
	output = calc_pid();

	LOG("pid_loop_out:%d\r\n", output);
}

int32_t calc_pid()
{
	error = center_val - gimbal_angle.X;
	proportion = error;
	integral += error;
	derivative = (error - prev_error);

	int32_t out = (k_p * proportion) + (k_i * integral) + (k_d * derivative);
	prev_error = error;

	return out;
}


/*
void i2c_template()
{
	// TODO: Initial research for basic i2c driver

	i2c_begin_transmission(0x68);
	i2c_write_byte(0x02);

	i2c_end_transmission(0);

	_delay_ms(70); // delay 70 milliseconds


	i2c_request_from(11, 2, 0);




	while(i2c_available() > 2)
	{
		reading = i2c_read();
		reading = reading <<8;
		reading |= i2c_read();
		LOG("i2c_reading: %c\r\n", reading);

		_delay_ms(100);
	}

}
*/


