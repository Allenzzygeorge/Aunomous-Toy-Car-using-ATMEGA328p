#include <avr/io.h>
#include "motor.h"
#include "timer.h" // For millis()

// --- Private Pin Defines ---
#define ENA_PIN   PD6
#define IN1_PIN   PD2
#define IN2_PIN   PB0

// --- Private Module State Variables ---
static uint8_t cruise_speed = 0;
static bool is_kickstarting = false;
static uint32_t kickstart_start_time = 0;

// --- Defines ---
#define KICKSTART_DURATION_MS 500 // MODIFIED: 2 seconds
#define KICKSTART_PWM 200         // MODIFIED: 100% PWM

void motor_init(void) {
    DDRD |= (1 << ENA_PIN) | (1 << IN1_PIN);
    DDRB |= (1 << IN2_PIN);
    // Fast PWM, Non-inverting, Prescaler 64
    TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);
    TCCR0B = (1 << CS01) | (1 << CS00);
    motor_stop();
}

void motor_set_speed(uint8_t speed) {
    cruise_speed = speed;
    if (!is_kickstarting) {
        OCR0A = cruise_speed;
    }
}

void motor_forward(void) {
    PORTD |= (1 << IN1_PIN);
    PORTB &= ~(1 << IN2_PIN);

    // Apply the high-power kickstart pulse
    OCR0A = KICKSTART_PWM;
    
    // Set the state flags for the motor_update() function
    is_kickstarting = true;
    kickstart_start_time = millis(); // Record the start time
}

void motor_backward(void) {
    PORTD &= ~(1 << IN1_PIN);
    PORTB |= (1 << IN2_PIN);

    // Apply the same kickstart logic for reverse
    OCR0A = KICKSTART_PWM;
    is_kickstarting = true;
    kickstart_start_time = millis();
}

void motor_suddenbreak(void) {
    PORTD &= ~(1 << IN1_PIN);
    PORTB |= (1 << IN2_PIN);
    OCR0A = 255;
}

void motor_stop(void) {
    PORTD &= ~(1 << IN1_PIN);
    PORTB &= ~(1 << IN2_PIN);
    
    // Immediately stop the motor and cancel any kickstart
    OCR0A = 0;
    is_kickstarting = false;
}

// This is the "heartbeat" for the motor module.
void motor_update(void) {
    // Only check the timer if we are in the kickstart state
    if (is_kickstarting) {
        // Check if the kickstart duration has passed
        if (millis() - kickstart_start_time >= KICKSTART_DURATION_MS) {
            // The kickstart period is over.
            // Settle the motor down to the desired cruising speed.
            OCR0A = cruise_speed;
            
            // Exit the kickstart state
            is_kickstarting = false;
        }
    }
}
