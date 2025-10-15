#include <avr/io.h>
#include "servo.h"

// --- Private Position Defines for BOTH Servos ---
// Steering Servo (PB1 / OC1A)
#define STEER_POS_CENTER 150
#define STEER_POS_RIGHT  120
#define STEER_POS_LEFT   180

// Rotation Servo (PB2 / OC1B)
#define USS_ROT_INITIAL 115
#define USS_ROT_FIRST   170
#define USS_ROT_SECOND  70

// --- The Correct, Unified Initialization Function ---
void servos_init(void) {
    // 1. Set BOTH servo pins (PB1 and PB2) as outputs.
    DDRB |= (1 << PB1) | (1 << PB2);

    // 2. Configure Timer1 for Fast PWM.
    //  FIX: Enable output on BOTH OC1A (PB1) and OC1B (PB2) in one line.
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
    
    // Set common timer settings (Mode 14, Prescaler 64)
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11) | (1 << CS10);
    ICR1 = 4999; // 50Hz period for the 64 prescaler

    // 3. Set the initial positions for both servos.
    OCR1A = STEER_POS_CENTER; // Steering servo
    OCR1B = USS_ROT_INITIAL;  // Rotation servo
}

// --- Steering Servo Functions ---
void steer_center(void) { OCR1A = STEER_POS_CENTER; }
void steer_right(void) { OCR1A = STEER_POS_RIGHT; }
void steer_left(void) { OCR1A = STEER_POS_LEFT; }

// --- Rotation Servo Functions ---
void uss_rot_initial(void) { OCR1B = USS_ROT_INITIAL; }
void uss_rot_first(void) { OCR1B = USS_ROT_FIRST; }
void uss_rot_second(void) { OCR1B = USS_ROT_SECOND; }