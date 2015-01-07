/*

 * gimbal.c
 *
 *  Created on: Oct 28, 2014
 *      Author: jcobb
 */
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include "../util/config.h"
#include "../util/clock.h"
#include "../util/log.h"
#include "../math/fast_math.h"
#include "../imu/imu.h"
#include "../imu/gyro.h"
#include "../kalman/kalman2.h"
//#include "../kalman/kalman.h"
#include "gimbal.h"

// https://github.com/sparkfun/MPU-9150_Breakout/blob/master/firmware/MPU6050/Examples/MPU9150_AHRS.ino

static const char _tag[] PROGMEM = "gimbal: ";

volatile int8_t gimbal_state = GIM_IDLE;


// task management
static int8_t task_id = 0;
static void task_handler();
static void enter_task(int8_t index);

// overall state management
static void state_handler();
static void enter_state(int8_t state);

// state timeout management
static volatile clock_time_t future = 0;
static bool timeout();
static void set_timer(clock_time_t timeout);

// logging timeouts
static clock_time_t f_timeout = 0;
static clock_time_t f_log_timeout = 0;

// local prototypes
void gimbal_accel_angle();
void gimbal_complementary_angle();
void gimbal_kalman_angle();
void log_application_data();

uint16_t mag_count = 0; // used to control display output rate
uint8_t mag_rate = 10;     // read rate for magnetometer data

pid_data_t pitch_pid_par;
pid_data_t roll_pid_par;

clock_time_t _last_time_read = 0;

float _last_angle_x = 0.0f;
float _last_angle_y = 0.0f;
float _last_angle_z = 0.0f;
float _last_gyro_angle_x = 0.0f;
float _last_gyro_angle_y = 0.0f;
float _last_gyro_angle_z = 0.0f;

const float accel_alpha = 0.5f;
const float g_sensitivity = 131.0f; // for 250 deg/s, check datasheet
float flt_alpha = .95;
t_fp_angle_t gimble_angle = {0.0f, 0.0f, 0.0f};

enum gimbal_task {
	READACC = 0,
	UPDATEACC = 1,
	VOLTAGECOMP = 2
};

void set_flt_alpha(float a)
{

	flt_alpha += a;
	LOG("flt_alpha:%f\r\n", flt_alpha);
}
static void enter_task(int8_t index)
{
	task_id = index;
}

static void enter_state(int8_t state)
{
	gimbal_state = state;

	// only GIM_IDLE and GIM_UNLOCKED have timeouts
	if(state == GIM_IDLE) {
		set_timer(1000);
	}
	else if (state == GIM_UNLOCKED) {
		set_timer(LOCK_TIME_SEC);
	}
}

void set_acc_time_constant(int16_t acc_time_constant){
	acc_compl_filter_const = (float)DT_FLOAT/(acc_time_constant + DT_FLOAT);
}

kalman_state kalman_roll;

void gimbal_init()
{

	//kalman_init2(&kalman_roll, 0.001f, 0.003f,  0.03f, 0.0f, 0.0f);
	//kalman_init();

	// initial tunable variables
	kalman_roll.q_angle = 0.001f;
	kalman_roll.q_bias = 0.003f;
	kalman_roll.r_measure = 0.03f;

	kalman_roll.k_angle = 0.0f; // reset the angle
	kalman_roll.bias = 0.0f; // reset bias


	// Since we assume that the bias is 0 and we know the starting angle (use setAngle),
	// the error covariance matrix is set like so -
	// see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical

	kalman_roll.P[0][0] = 0.0f;
	kalman_roll.P[0][1] = 0.0f;
	kalman_roll.P[1][0] = 0.0f;
	kalman_roll.P[1][1] = 0.0f;




	// resolution=131, scale = 0.000133
	//gyro_scale = 1.0 / resolution_divider/ 180.0 * PI * DT_FLOAT;
	//LOG("gyro_scale: %d\r\n", gyro_scale*1000);
	//set_acc_time_constant(config.acc_time_constant);
	//acc_mag = ACC_1G*ACC_1G; // magnitude of 1G initially

	//est_g.V.X = 0.0f;
	//est_g.V.Y = 0.0f;
	//est_g.V.Z = ACC_1G;

	//LOG("enter_task: READACC\r\n");
	//LOG("enter_state: GIM_IDLE\r\n");
	LOG("flt_alpha:%f\r\n", flt_alpha);
	//enter_task(READACC);
	//enter_state(GIM_IDLE);
	_last_time_read = clock_time();

}

void gimbal_tick()
{
	gimbal_kalman_angle();
	//gimbal_complementary_angle();
	//gimbal_accel_angle();
}



void gimbal_accel_angle()
{

	if(clock_time() >= f_timeout) {
		f_timeout = clock_time() + 33;
	}
	else {
		return;
	}

	float fXg = 0;
	float fYg = 0;
	float fZg = 0;
	int16_t Xg = 0;
	int16_t Yg = 0;
	int16_t Zg = 0;
	float pitch = 0;
	float roll = 0;
	float yaw = 0;


	// read the accelerometer
	imu_get_acceleration(&Xg, &Yg, &Zg);

	// net out offsets
	Xg -= config.acc_offset_x;
	Yg -= config.acc_offset_y;
	Zg -= config.acc_offset_z;

	fXg = Xg *2.0f/16384.0f;
	fYg = Yg *2.0f/16384.0f;
	fZg = Zg *2.0f/16384.0f;


	// low pass filter
	fXg = fXg * accel_alpha + (fXg * (1.0f - accel_alpha));
	fYg = fYg * accel_alpha + (fYg * (1.0f - accel_alpha));
	fZg = fZg * accel_alpha + (fZg * (1.0f - accel_alpha));

	// calc roll and pitch
	roll = (atan2(fYg, fZg)*180.0)/PI;
	pitch = (atan2(fXg, sqrt(pow(fYg,2) + pow(fZg,2)))*180.0)/PI;

	// Throttle output to 10x per second
	if(clock_time() >= f_log_timeout) {
		//LOG("roll/pitch: %f:%f\r\n", roll, pitch);
		LOG("roll/pitch/yaw %f:%f:%f\r\n", roll, pitch, 0);
		//LOG("%f:%f\r\n", roll, pitch);
		f_log_timeout = clock_time() + 100;
	}

}

void read_sensor_data(int16_t aX, int16_t aY, int16_t aZ, int16_t gX, int16_t gY, int16_t gZ, int16_t mX, int16_t mY, int16_t mZ);




void read_sensor_data(int16_t aX, int16_t aY, int16_t aZ, int16_t gX, int16_t gY, int16_t gZ, int16_t mX, int16_t mY, int16_t mZ)
{

	mag_count++;
	// *** ACCEL ***

	imu_get_acceleration(aX, aY, aZ);

	// apply calibration offsets
	aX -= config.acc_offset_x;
	aY -= config.acc_offset_y;
	aZ -= config.acc_offset_z;

	aX = aX*2/32768;
	aY = aY*2/32768;
	aZ = aZ*2/32768;


	imu_get_rotation(gX, gY, gZ);

	gX = (gX - config.gyro_offset_x) / g_sensitivity;
	gY = (gY - config.gyro_offset_y) / g_sensitivity;
	gZ = (gZ - config.gyro_offset_z) / g_sensitivity;

	if (mag_count > 1000/mag_rate) {
		imu_get_mag(mX, mY, mZ);
		mX = mX*10*1229/4096 + 18; // milliGauss (1229 microTesla per 2^12 bits, 10 mG per microTesla)
		mY = mY*10*1229/4096 + 70; // apply calibration offsets in mG that correspond to your environment and magnetometer
		mZ = mZ*10*1229/4096 + 270;
		mag_count = 0;
	}

}

void gimbal_complementary_angle()
{
	int16_t aX=0, aY=0, aZ=0, gX=0, gY=0, gZ=0, mX=0, mY=0, mZ=0;

	//read_sensor_data();
	read_sensor_data(&aX, &aY, &aZ, &gX, &gY, &gZ, &mX, &mY, &mZ);
	//LOG("\r\n\r\nstart\r\n");

	//LOG("ax, ay, az: %d %d %d\r\n", aX, aY, aZ); // accel only
	//LOG("gx, gy, gz: %d %d %d\r\n", gX, gY, gZ); // gyro only
	//LOG("ax, ay, az, gx, gy, gz: %d %d %d %d %d %d\r\n", aX, aY, aZ, gX, gY, gZ); // both

	clock_time_t t_now = clock_time();



	float gyro_x = (float)gX;
	float gyro_y = (float)gY;
	float gyro_z = (float)gZ;
	float accel_x = (float)aX;
	float accel_y = (float)aY;
	float accel_z = (float)aZ;
	float mag_x = (float) mX;
	float mag_y = (float) mY;
	float mag_z = (float) mZ;

	gyro_x = ((float) gX)/131*M_PI/180.0f;
	gyro_y = ((float) gY)/131*M_PI/180.0f;
	gyro_z = ((float) gZ)/131*M_PI/180.0f;

	mag_x = ((float) mX)*M_PI/180.0f;
	mag_y = ((float) mY)*M_PI/180.0f;
	mag_z = ((float) mZ)*M_PI/180.0f;


	//LOG("accel x,y,z gyro x,y,z: %f %f %f %f %f %f\r\n", accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)
	//LOG("gyro x,y,z: %f %f %f\r\n", gyro_x, gyro_y, gyro_z)
	//return;

	float accel_angle_y = atan2(accel_x, sqrt( pow(accel_y, 2) + pow(accel_z, 2))) * 180.0f / M_PI;
	float accel_angle_x = atan2(accel_y, sqrt( pow(accel_x, 2) + pow(accel_z, 2))) * 180.0f / M_PI;
	float accel_angle_z = 0;

	/*
	LOG("roll/pitch/yaw %f:%f:%f\r\n", accel_angle_x, accel_angle_y, accel_angle_z);
	return;
	*/

	//LOG("accel angle x,y,z: %f %f %f\r\n", accel_angle_x, accel_angle_y, accel_angle_z);



	// Compute filtered angles
	clock_time_t delta_t = (t_now-_last_time_read);

	float dt = ((float)delta_t)/1000.0f;
	//LOG("t_now %lu, t_last %lu t_delta %lu dt %f\r\n", t_now, _last_time_read, delta_t, dt);

	float gyro_angle_x = gyro_x * dt + _last_angle_x;
	float gyro_angle_y = gyro_y * dt + _last_angle_y;
	float gyro_angle_z = gyro_z * dt + _last_angle_z;

	//LOG("gyro_angle_x,y,z: %f %f %f\r\n", gyro_angle_x, gyro_angle_y, gyro_angle_z);

	// Compute the drifting gyro angles
	float unfiltered_gyro_angle_x = gyro_x*dt + _last_gyro_angle_x;
	float unfiltered_gyro_angle_y = gyro_y*dt + _last_gyro_angle_y;
	float unfiltered_gyro_angle_z = gyro_z*dt + _last_gyro_angle_z;

	//LOG("unflt angles x, y, z: %f %f %f\r\n", unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);

	// Apply complementry filter
//	const float alpha = 0.95;
//	float angle_x = alpha*gyro_angle_x + (1.0f-alpha)*accel_angle_x;
//	float angle_y = alpha*gyro_angle_y + (1.0f-alpha)*accel_angle_y;
//	float angle_z = alpha*gyro_angle_z + (1.0f-alpha)*accel_angle_z;

	float angle_x = flt_alpha*gyro_angle_x + (1.0f-flt_alpha)*accel_angle_x;
	float angle_y = flt_alpha*gyro_angle_y + (1.0f-flt_alpha)*accel_angle_y;
	float angle_z = flt_alpha*gyro_angle_z + (1.0f-flt_alpha)*accel_angle_z;

	gimbal_angle.X = angle_x;
	gimbal_angle.Y = angle_y;
	gimbal_angle.Z = angle_z;

	_last_time_read = t_now;
	_last_angle_x = angle_x;
	_last_angle_y = angle_y;
	_last_angle_z = angle_z;
	_last_gyro_angle_x = unfiltered_gyro_angle_x;
	_last_gyro_angle_y = unfiltered_gyro_angle_y;
	_last_gyro_angle_z = unfiltered_gyro_angle_z;


	// Throttle output to .1x per second
	if(clock_time() >= f_log_timeout) {
		f_log_timeout = clock_time() + 10;
		//LOG("%f %f %f %f\r\n", angle_x, angle_y, angle_z, dt);
		LOG("roll/pitch/yaw %f:%f:%f\r\n", angle_x, angle_y, angle_z);
	}
}



float kal_angle_x = 0.0f;

//void gimbal_kalman_angle()
//{
//	int16_t aX=0, aY=0, aZ=0, gX=0, gY=0, gZ=0, mX=0, mY=0, mZ=0;
//
//	read_sensor_data(&aX, &aY, &aZ, &gX, &gY, &gZ, &mX, &mY, &mZ);
//
//	clock_time_t t_now = clock_time();
//
//
//	clock_time_t delta_t = (t_now-_last_time_read);
//
//	float dt = ((float)delta_t)/1000.0f;
//
//	float accel_x = (float)aX;
//	float accel_y = (float)aY;
//	float accel_z = (float)aZ;
//
//	float gyro_x = (float)gX;
//	float gyro_y = (float)gY;
//	float gyro_z = (float)gZ;
//
//	gyro_x = ((float) gX)*DEG_TO_RAD;
//	gyro_y = ((float) gY)*DEG_TO_RAD;
//	gyro_z = ((float) gZ)*DEG_TO_RAD;
//
//
//	// restricting pitch
//	float roll = atan2(accel_y, accel_z) * RAD_TO_DEG;
//	float pitch = atan(-accel_x / sqrt(pow(accel_y, 2) + pow(accel_z, 2))) * RAD_TO_DEG;
//
//	if((roll < -90.0f && kal_angle_x > 90.0f) || (roll > 90.0f && kal_angle_x < -90.0f)) {
//		set_angle(roll);
//		kal_angle_x = roll;
//
//	} else {
//		kal_angle_x = get_angle(roll, gyro_x, dt);
//	}
//
//	_last_time_read = t_now;
//
//	// Throttle output to .1x per second
//	if(clock_time() >= f_log_timeout) {
//		f_log_timeout = clock_time() + 10;
//		//LOG("%f %f %f %f\r\n", angle_x, angle_y, angle_z, dt);
//		LOG("roll/pitch/yaw %f:%f:%f\r\n", kal_angle_x, 0.0f, 0.0f);
//	}
//
//
//}

void gimbal_kalman_angle()
{
	int16_t aX=0, aY=0, aZ=0, gX=0, gY=0, gZ=0, mX=0, mY=0, mZ=0;

	read_sensor_data(&aX, &aY, &aZ, &gX, &gY, &gZ, &mX, &mY, &mZ);

	clock_time_t t_now = clock_time();


	clock_time_t delta_t = (t_now-_last_time_read);

	float dt = ((float)delta_t)/1000.0f;

	float accel_x = (float)aX;
	float accel_y = (float)aY;
	float accel_z = (float)aZ;

	float gyro_x = (float)gX;
	float gyro_y = (float)gY;
	float gyro_z = (float)gZ;

	gyro_x = ((float) gX)*DEG_TO_RAD;
	gyro_y = ((float) gY)*DEG_TO_RAD;
	gyro_z = ((float) gZ)*DEG_TO_RAD;


	// restricting pitch
	float roll = atan2(accel_y, accel_z) * RAD_TO_DEG;
	float pitch = atan(-accel_x / sqrt(pow(accel_y, 2) + pow(accel_z, 2))) * RAD_TO_DEG;

	if((roll < -90.0f && kal_angle_x > 90.0f) || (roll > 90.0f && kal_angle_x < -90.0f)) {
		kalman_roll.k_angle = roll;
		kal_angle_x = roll;

	} else {
		//kal_angle_x = get_angle(roll, gyro_x, dt);

		kal_angle_x = get_angle2(&kalman_roll, roll, gyro_x, dt);
	}

	_last_time_read = t_now;

	// Throttle output to .1x per second
	if(clock_time() >= f_log_timeout) {
		f_log_timeout = clock_time() + 10;
		//LOG("%f %f %f %f\r\n", angle_x, angle_y, angle_z, dt);
		LOG("roll/pitch/yaw %f:%f:%f\r\n", kal_angle_x, 0.0f, 0.0f);
	}


}


//	if(force_magnitude > 8192 && force_magnitude < 32768) {
//		// angles based on accelerometer
//		float accelRoll = atan2(accelX, sqrt( pow(accelY, 2) + pow(accelZ, 2))) * 180.0f / M_PI;
//		_roll = _roll * 0.98f + accelRoll * 0.02f;
//
//		float accelPitch = atan2(accelY, sqrt( pow(accelX, 2) + pow(accelZ, 2))) * 180.0f / M_PI;
//		_pitch = _pitch * 0.98f + accelPitch * 0.02f;
//
//	}

void log_application_data()
{

	/*
	LOG("%f:", ax*1000);
	LOG("%f:", ay*1000);
	LOG("%f:", az*1000);
	LOG("%f:", gx);
	LOG("%f:", gy);
	LOG("%f:", gz);
	LOG("%d:", (int)mx);
	LOG("%d:", (int)my);
	LOG("%d:", (int)mz);
	LOG("%f:", q[1]);
	LOG("%f:", q[2]);
	LOG("%f:", q[3]);
	LOG("%f:", pitch); // pitch
	LOG("%f:", roll); // pitch
	LOG("%f", yaw); // roll
	LOG("\r\n");
	*/

}

static void set_timer(clock_time_t timeout)
{
	future = clock_time() + timeout;
}

// timeout routine to demonstrate clock_time
// being kept by pwm isr interrupt
static bool timeout()
{
	bool timeout = false;

	if(clock_time() >= future)
	{
		set_timer(1000);
		timeout = true;

	}

	return timeout;
}


