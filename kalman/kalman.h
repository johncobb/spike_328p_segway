/*
 * kalman.h
 *
 *  Created on: Jan 6, 2015
 *      Author: jcobb
 */


//https://github.com/TKJElectronics/KalmanFilter

#ifndef KALMAN_H_
#define KALMAN_H_

#include "../util/defines.h"

float q_angle;		// process noise variance for the accel
float q_bias;		// process noise variance for the gyro bias
float r_measure;	// measurement noise variance - this is actually the variance of the measurement noise

float k_angle;		// The angle calculated by the Kalman filter - part of the 2x1 state vector
float bias;			// The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
float rate;			// Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

float P[2][2];		// Error covariance matrix - This is a 2x2 matrix


void kalman_init();
void set_angle(float new_angle) { k_angle=new_angle; }; // Used to set angle, this should be set as the starting angle
float get_rate() {return rate; };  // Return the unbiased rate

// Kalman tuning accessors
void set_qangle(float newq_angle) {q_angle = newq_angle; };
void set_qbias(float newq_bias) {q_bias = newq_bias; };
void set_rmeasure(float newr_measure) {r_measure = newr_measure; };

float get_qangle() {return q_angle; };
float get_qbias() {return q_bias; };
float get_rmeasure() {return r_measure; };


void kalman_init()
{
	// initial tunable variables
	q_angle = 0.001f;
	q_bias = 0.003f;
	r_measure = 0.03f;

	k_angle = 0.0f; // reset the angle
	bias = 0.0f; // reset bias


	// Since we assume that the bias is 0 and we know the starting angle (use setAngle),
	// the error covariance matrix is set like so -
	// see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical

	P[0][0] = 0.0f;
	P[0][1] = 0.0f;
	P[1][0] = 0.0f;
	P[1][1] = 0.0f;

}

float get_angle(float new_angle, float new_rate, float dt)
{
	// KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
	// Modified by Kristian Lauszus
	// See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

	// Discrete Kalman filter time update equations - Time Update ("Predict")
	// Update xhat - Project the state ahead
	/* Step 1 */
	rate = new_rate - bias;

	k_angle += dt * rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
	P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + q_angle);
	P[0][1] -= dt * P[1][1];
	P[1][0] -= dt * P[1][1];
	P[1][1] += q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
	float s = P[0][0] + r_measure; // Etimate error

	// Step 5
	float K[2]; // Kalman gain - This is a 2x1 vector
	K[0] = P[0][0]/s;
	K[1] = P[1][0]/s;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
	float y = new_angle - k_angle; // angle difference
	// Step 6
	k_angle += K[0] * y;
	bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */

	float p00_temp = P[0][0];
	float p01_temp = P[0][1];

	P[0][0] -= K[0] * p00_temp;
	P[0][1] -= K[0] * p01_temp;
	P[1][0] -= K[1] * p00_temp;
	P[1][1] -= K[1] * p01_temp;

	return k_angle;

}




#endif /* KALMAN_H_ */
