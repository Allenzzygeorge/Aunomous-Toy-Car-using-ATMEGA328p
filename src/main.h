#ifndef MAIN_H
#define MAIN_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

// --- Include all project modules ---
#include "timer_motor.h"
#include "servo.h" 
#include "sensing.h"

// ======================================================================
// --- 1. System Defines ---
// ======================================================================

// --- Car State Defines ---
#define OBSTACLE_DISTANCE_CM 50
#define TURN_DURATION_MS     650
#define REVERSE_DURATION_MS  1100     

// --- MOTOR SPEED DEFINES  ---
#define SPEED_CRUISE 120
#define SPEED_TURN   160 

// --- Ultrasonic Sensor Defines ---
#define MIN_RELIABLE_DISTANCE_CM 2
#define MAX_RELIABLE_DISTANCE_CM 400
#define TRIGGER_PIN PB3
#define ECHO_PIN    PB4
#define MAX_TIMER_OVERFLOWS 8
#define TIMEOUT_PULSE_COUNT 0xFFFF

// ======================================================================
// --- 2. Global Enums & Variables ---
// ======================================================================

// --- Ultrasonic Sensor State ---
typedef enum {
    STATE_IDLE,
    STATE_WAITING_FOR_ECHO,
    STATE_MEASUREMENT_COMPLETE
} MeasurementState;

// --- Car State Machine ---
typedef enum {
    STATE_INITIAL_WAIT,
    STATE_DRIVING_FORWARD,
    STATE_OBSTACLE_REVERSE,
    STATE_SCAN_RIGHT,
    STATE_WAIT_FOR_RIGHT_SCAN,
    STATE_SCAN_LEFT,
    STATE_WAIT_FOR_LEFT_SCAN,
    STATE_TURN,
    STATE_RECOVER_FORWARD,
    STATE_RECOVER_STRAIGHTEN
} CarState;

typedef enum { TURN_LEFT, TURN_RIGHT } TurnDirection;

// --- Extern declarations for global variables defined in main.cpp ---
extern volatile MeasurementState sensor_state;
extern volatile uint16_t pulse_count;
extern volatile uint8_t timer_overflow_count;
extern volatile CarState current_state;
extern TurnDirection planned_turn;
extern uint16_t last_measured_distance;

// ======================================================================
// --- 3. Function Prototypes (from main.cpp) ---
// ======================================================================

// --- From timer_motor.h (adding for visibility) ---
// REMOVED: uint8_t motor_get_speed(void); - This was causing the linker error.

// --- From main.cpp ---
void Serial_begin(int BAUD);
void Serial_print(const char *s);
void Serial_println(const char *s);
void Serial_print(int num);
void Serial_println(int num);
void Serial_printFloat(double num, int precision);
uint8_t Serial_available(void);
char Serial_read(void);
float Serial_parseFloat(void);
void ultrasonic_init(void);
void trigger_ping(void);

#endif // MAIN_H

