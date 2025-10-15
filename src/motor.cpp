#include <avr/io.h>
#include "motor.h"
#include "timer.h" // ⭐ We need access to the millis() timer

// --- Private Pin Defines ---
#define ENA_PIN     PD6
#define IN1_PIN     PD2
#define IN2_PIN     PB0

// --- Private Module State Variables ---
// 'static' makes these variables private to this file.
static uint8_t cruise_speed = 0;        // The desired speed after the kickstart
static bool is_kickstarting = false;    // Flag to track if we're in the kickstart phase
static uint32_t kickstart_start_time = 0; // Timestamp of when the kickstart began

// --- Function Implementations ---

void motor_init(void) {
    DDRD |= (1 << ENA_PIN) | (1 << IN1_PIN);
    DDRB |= (1 << IN2_PIN);
    TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);
    TCCR0B = (1 << CS01) | (1 << CS00);
    motor_stop();
}

// This function now just saves the desired speed. The kickstart logic applies it.
void motor_set_speed(uint8_t speed) {
    cruise_speed = speed;
    // If we aren't in a kickstart, apply the new speed immediately.
    // This allows for changing speed while already moving.
    if (!is_kickstarting) {
        OCR0A = cruise_speed;
    }
}

void motor_forward(void) {
    PORTD |= (1 << IN1_PIN);
    PORTB &= ~(1 << IN2_PIN);

    // Apply the high-power kickstart pulse
    OCR0A = 140;
    
    // Set the state flags for the motor_update() function
    is_kickstarting = true;
    kickstart_start_time = millis(); // Record the start time
}

void motor_backward(void) {
    PORTD &= ~(1 << IN1_PIN);
    PORTB |= (1 << IN2_PIN);

    // Apply the same kickstart logic for reverse
    OCR0A = 140;
    is_kickstarting = true;
    kickstart_start_time = millis();
}

void motor_stop(void) {
    PORTD &= ~(1 << IN1_PIN);
    PORTB &= ~(1 << IN2_PIN);
    
    // Immediately stop the motor and cancel any kickstart
    OCR0A = 0;
    is_kickstarting = false;
}

// This is the new "heartbeat" for the motor module.
void motor_update(void) {
    // Only check the timer if we are in the kickstart state
    if (is_kickstarting) {
        // Check if 1000 milliseconds (1 second) have passed
        if (millis() - kickstart_start_time >= 1000) {
            // The kickstart period is over.
            // Settle the motor down to the desired cruising speed.
            OCR0A = cruise_speed;
            
            // Exit the kickstart state
            is_kickstarting = false;
        }
    }
}