#include "timer_motor.h"
#include <avr/interrupt.h>

// --- Motor Direction Pins (from your PID example) ---
#define MOTOR_IN1 PD2
#define MOTOR_IN2 PB0
#define MOTOR_PWM_PIN PD6 // OC0A

// --- Millis Counter ---
volatile uint32_t timer0_overflow_count = 0;

/**
 * @brief This ISR increments the overflow count for millis()
 */
ISR(TIMER0_OVF_vect) {
    timer0_overflow_count++;
}

/**
 * @brief Initializes Timer0 for BOTH millis() and Fast PWM.
 * This is the unified function that solves the timer conflict.
 */
void timer_motor_init(void) {
    // --- Motor Pin Setup ---
    DDRD |= (1 << MOTOR_IN1) | (1 << MOTOR_PWM_PIN);
    DDRB |= (1 << MOTOR_IN2);
    
    // --- Timer0 Setup ---
    // Mode 3: Fast PWM, 8-bit (WGM01, WGM00)
    // Output: Non-inverting PWM on OC0A (COM0A1)
    TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);
    
    // Prescaler: 64 (CS01, CS00)
    // This gives PWM freq = 16MHz / 64 / 256 = 976 Hz
    // This gives millis() tick = (64 / 16MHz) * 256 = 1.024ms
    TCCR0B = (1 << CS01) | (1 << CS00);
    
    // Enable Timer0 Overflow Interrupt (for millis)
    TIMSK0 |= (1 << TOIE0); 
    
    OCR0A = 0; // Start with motor off
}

/**
 * @brief Provides a millisecond counter using Timer0.
 */
uint32_t millis(void) {
    uint32_t m;
    uint8_t oldSREG = SREG;
    cli();
    m = timer0_overflow_count;
    SREG = oldSREG;
    
    // (1.024ms per overflow) * 1000 = 1024
    return (m * 1024) / 1000; 
}

// --- Motor Control Functions ---

void motor_set_speed(uint8_t speed) {
    OCR0A = speed;
}

void motor_stop(void) {
    PORTD &= ~(1 << MOTOR_IN1);
    PORTB &= ~(1 << MOTOR_IN2);
    OCR0A = 0;
}

void motor_forward(void) {
    PORTD |= (1 << MOTOR_IN1);
    PORTB &= ~(1 << MOTOR_IN2);
}

void motor_backward(void) {
    PORTD &= ~(1 << MOTOR_IN1);
    PORTB |= (1 << MOTOR_IN2);
}
