/*
 * kalman2.h
 *
 *  Created on: Jan 7, 2015
 *      Author: jcobb
 */

#ifndef KALMAN2_H_
#define KALMAN2_H_
//https://github.com/TKJElectronics/KalmanFilter

#include "../util/defines.h"

typedef struct {
	float q_angle;
	float q_bias;
	float r_measure;
	float k_angle;
	float bias;
	float rate;
	float P[2][2];

} kalman_state;

void kalman_init2(kalman_state * state, float q_angle, float q_bias, float r_measure, float k_angle, float bias)
{
	// initial tunable variables
	state->q_angle = q_angle;
	state->q_bias = q_bias;
	state->r_measure = r_measure;
	state->k_angle = k_angle; // reset the angle
	state->bias = bias; // reset bias

	// Since we assume that the bias is 0 and we know the starting angle (use setAngle),
	// the error covariance matrix is set like so -
	// see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical

	state->P[0][0] = 0.0f;
	state->P[0][1] = 0.0f;
	state->P[1][0] = 0.0f;
	state->P[1][1] = 0.0f;

}

float get_angle2(kalman_state * state, float new_angle, float new_rate, float dt)
{
	// KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
	// Modified by Kristian Lauszus
	// See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

	// Discrete Kalman filter time update equations - Time Update ("Predict")
	// Update xhat - Project the state ahead
	/* Step 1 */
	state->rate = new_rate - state->bias;

	state->k_angle += dt * state->rate;



    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
	state->P[0][0] += dt * (dt*state->P[1][1] - state->P[0][1] - state->P[1][0] + state->q_angle);
	state->P[0][1] -= dt * state->P[1][1];
	state->P[1][0] -= dt * state->P[1][1];
	state->P[1][1] += state->q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
	float s = state->P[0][0] + state->r_measure; // Etimate error

	// Step 5
	float K[2]; // Kalman gain - This is a 2x1 vector
	K[0] = state->P[0][0]/s;
	K[1] = state->P[1][0]/s;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
	float y = new_angle - state->k_angle; // angle difference
	// Step 6
	state->k_angle += K[0] * y;
	state->bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */

	float p00_temp = state->P[0][0];
	float p01_temp = state->P[0][1];

	state->P[0][0] -= K[0] * p00_temp;
	state->P[0][1] -= K[0] * p01_temp;
	state->P[1][0] -= K[1] * p00_temp;
	state->P[1][1] -= K[1] * p01_temp;

	return state->k_angle;
}

#endif /* KALMAN2_H_ */
