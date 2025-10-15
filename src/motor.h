#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

// Initializes the hardware pins and Timer0 for PWM speed control.
void motor_init(void);

// Sets the motor's desired cruising speed.
void motor_set_speed(uint8_t speed);

// Starts the motor moving forward with a 1-second kickstart.
void motor_forward(void);

// Starts the motor moving backward with a 1-second kickstart.
void motor_backward(void);

// Stops the motor immediately.
void motor_stop(void);

// This function must be called continuously in your main while() loop.
// It manages the state of the kickstart timer.
void motor_update(void);

#endif // MOTOR_H