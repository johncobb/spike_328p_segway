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
#include "pwm/pwm.h"




// log debugging
static const char _tag[] PROGMEM = "main: ";
volatile char term_in = 0;


// pid variables
clock_time_t pid_timer = 0;
clock_time_t last_pid_read = 0;
float pid_val = 0.0f;
// local porototypes
void balance();
void calc_pid();

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

	last_pid_read = clock_time();

	LOG("\r\n\r\nspike_328p_segway starting...\r\n");

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

	/*
	 * gimbal is the main state machine
	 * for processing stabilization
	 */
	LOG("gimbal_init...\r\n");
	gimbal_init();
	//kalman_init();
	pwm_init();




	LOG("starting segway loop...\r\n");

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


#define duty_min 185
#define duty_max 255
#define duty_range (duty_max-duty_min)

uint8_t pwm_out = duty_min + (duty_range/2);




void balance()
{


//	kal_angle_x = 0.0f;
//	kal_angle_y = 0.0f;

	calc_pid();
	int pid_out_i = (int) pid_val;


	//int pwm_output = pwm_out + pid_output;


	//LOG("pid_output_f %f pwm_output_i: %d\r\n", pid_val, pid_out_i);
	//LOG("pid_val2: %f\r\n", pid_val);
	//pwm_out = constrain(output, duty_min, duty_max);

	pwm_setval(pwm_out, 2);


	//LOG("pid_loop_out:%d\r\n", pwm_out);
}



// pid defines
//#define k_p		 7
//#define k_i		 1
//#define k_d		-5

#define k_p		 9
#define k_i		 2
#define k_d		 3

float center_val = 0.0f;

int32_t error = 0;
int32_t proportion = 0;
float integral = 0.0f;
int32_t derivative = 0;
int32_t output = 0;
int32_t prev_error = 0;
float last_error = 0.0f;

void calc_pid()
{
	// guard against initial windup
//	if(last_pid_read == 0) {
//		last_pid_read = clock_time();
//		return;
//	}
	clock_time_t t_now = clock_time();

	float delta_t = (t_now-last_pid_read)/1000.0f;

	float pid_error = (center_val - kal_angle_x);
	float p_term = k_p * pid_error;
	integral += pid_error * delta_t;
	integral = constrain(integral, -1.0, 1.0); // limit the error
	float i_term = (k_i * 100.0) * integral;
	float d_term = (k_d * 100.0) * (pid_error - last_error)/delta_t;
	last_error = pid_error;

	pid_val = p_term + i_term + d_term;

	last_pid_read = t_now;

	//LOG("pid err int dlt: %f %f %f %f\r\n", pid_val, err, integral, delta_t);
	LOG("pid %f err %f int %f dlt %f\r\n", pid_val, pid_error, integral, delta_t);
	//LOG("pid_val: %f\r\n", pid_val);



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


