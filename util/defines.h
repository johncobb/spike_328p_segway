/*
 * defines.h
 *
 *  Created on: Oct 20, 2014
 *      Author: jcobb
 */

#ifndef DEFINES_H_
#define DEFINES_H_

#include <stddef.h>
#include <stdbool.h>
#include <inttypes.h>
//#include "config.h"

typedef bool boolean;
typedef uint8_t byte;

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

//void constrain_32(int32_t amt, int32_t low, int32_t high){ ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));}

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))

//long map(long x, long in_min, long in_max, long out_min, long out_max)
//{
//  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
//}

#endif /* DEFINES_H_ */
