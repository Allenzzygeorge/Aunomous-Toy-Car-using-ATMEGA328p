#ifndef TIMER_MOTOR_H
#define TIMER_MOTOR_H

#include <stdint.h>
#include <avr/io.h>

// --- Millis ---
void timer_motor_init(void);
uint32_t millis(void);

// --- Motor ---
void motor_set_speed(uint8_t speed);
void motor_stop(void);
void motor_forward(void);
void motor_backward(void);

#endif
