/*
 * clock.h
 *
 *  Created on: Oct 20, 2014
 *      Author: jcobb
 */

#ifndef CLOCK_H_
#define CLOCK_H_

#include <stdlib.h>
#include <stdint.h>

typedef uint32_t clock_time_t;
extern volatile clock_time_t clock_millis;

#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )

// the prescaler is set so that timer0 ticks every 64 clock cycles, and the
// the overflow handler is called every 256 ticks.
#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))

// the whole number of milliseconds per timer0 overflow
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)

// the fractional number of milliseconds per timer0 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)


void clock_init();
clock_time_t clock_time();
clock_time_t clock_time_micros();
void delay_millis(clock_time_t millis);
void isr_tick();

#endif /* CLOCK_H_ */
