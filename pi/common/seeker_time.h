#ifndef SEEKER_TIME_H
#define SEEKER_TIME_H

#include <stdint.h>

/* IF-1 clock: CLOCK_MONOTONIC in ms. Never compare across machines. */
int64_t seeker_now_ms(void);
/* The same clock in microseconds, for timing stages inside one frame. */
int64_t seeker_now_us(void);
void seeker_sleep_ms(int64_t ms);

#endif
