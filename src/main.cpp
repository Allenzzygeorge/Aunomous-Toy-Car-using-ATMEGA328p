#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdlib.h>

// Assumed to be present from your project structure
#include "timer.h"
#include "servo.h"
#include "motor.h"

// --- System Defines ---
#define OBSTACLE_DISTANCE_CM 80
#define PING_INTERVAL_MS     100
#define TURN_DURATION_MS     1500
#define REVERSE_DURATION_MS  700 // Dedicated constant for reversing time

// --- Robust Ultrasonic Sensor Integration ---

// --- Physical Sensor Limits & Configuration ---
#define MIN_RELIABLE_DISTANCE_CM 2
#define MAX_RELIABLE_DISTANCE_CM 400

// --- Pin and Timeout Defines ---
#define TRIGGER_PIN PB3
#define ECHO_PIN    PB4
// Timeout for Timer2 with 256 prescaler. Max echo time for 400cm is ~24ms.
// Timer2 overflows every 4.096ms. Timeout after ~32ms (8 overflows).
#define MAX_TIMER_OVERFLOWS 8

// --- A Strict State Machine for Measurement ---
typedef enum {
    STATE_IDLE,
    STATE_WAITING_FOR_ECHO,
    STATE_MEASUREMENT_COMPLETE
} MeasurementState;

volatile MeasurementState sensor_state = STATE_IDLE;

// --- Global Volatile Variables ---
#define TIMEOUT_PULSE_COUNT 0xFFFF
volatile uint16_t pulse_count = 0;
volatile uint8_t timer_overflow_count = 0;
// --- End of Ultrasonic Integration Block ---


// --- Car State Machine (MODIFIED) ---
typedef enum {
    STATE_INITIAL_WAIT,
    STATE_DRIVING_FORWARD,
    STATE_OBSTACLE_REVERSE, // NEW: State for immediate reverse
    STATE_SCAN_RIGHT,
    STATE_WAIT_FOR_RIGHT_SCAN,
    STATE_SCAN_LEFT,
    STATE_WAIT_FOR_LEFT_SCAN,
    STATE_TURN,
    STATE_RECOVER_FORWARD,
    STATE_RECOVER_STRAIGHTEN
} CarState;
volatile CarState current_state = STATE_INITIAL_WAIT;

// --- ADDED: Variable to store turn decision ---
typedef enum { TURN_LEFT, TURN_RIGHT } TurnDirection;
TurnDirection planned_turn = TURN_RIGHT; // Default to right

// --- Function Prototypes ---
void uart_init(void);
void printString(const char* str);
void printNumber(int number);
void ultrasonic_init(void);
void trigger_ping(void);

int main(void) {
    // --- Hardware Initialization ---
    uart_init();
    timer_init(); // Assuming this initializes millis()
    motor_init();
    ultrasonic_init();
    servos_init();
    sei(); // Enable global interrupts

    printString("\n--- Smart Car Initializing ---\n");

    // --- Set Initial Conditions ---
    motor_stop();

    // --- Timestamps and Variables for Logic ---
    uint32_t last_ping_time = 0;
    uint32_t maneuver_start_time = 0;
    uint16_t right_distance = 0;
    uint16_t left_distance = 0;

    // --- Main Control Loop ---
    while (1) {
        uint32_t current_time = millis();
        motor_update();

        // --- Car State Machine Logic ---
        switch (current_state) {
            case STATE_INITIAL_WAIT:
                if (current_time >= 3000) {
                    printString("Initialization complete. Starting motor.\n");
                    motor_set_speed(80);
                    motor_forward();
                    current_state = STATE_DRIVING_FORWARD;
                    printString("Status: Moving Forward\n");
                }
                break;

            case STATE_DRIVING_FORWARD:
                if (current_time - last_ping_time >= PING_INTERVAL_MS) {
                    if (sensor_state == STATE_IDLE) {
                        last_ping_time = current_time;
                        trigger_ping();
                    }
                }
                break;

            // NEW STATE: Car reverses for a set duration after detecting an obstacle.
            case STATE_OBSTACLE_REVERSE:
                if (current_time - maneuver_start_time >= REVERSE_DURATION_MS) {
                    motor_stop();
                    printString("Status: Scanning right...\n");
                    uss_rot_first();
                    current_state = STATE_SCAN_RIGHT;
                    maneuver_start_time = current_time;
                }
                break;

            case STATE_SCAN_RIGHT:
                // Give servo time to move before pinging
                if (current_time - maneuver_start_time >= 500) {
                    if (sensor_state == STATE_IDLE) {
                        trigger_ping();
                        current_state = STATE_WAIT_FOR_RIGHT_SCAN;
                    }
                }
                break;

            case STATE_WAIT_FOR_RIGHT_SCAN:
                // Wait for the measurement to complete
                break;

            case STATE_SCAN_LEFT:
                // Give servo time to move before pinging
                if (current_time - maneuver_start_time >= 1000) {
                     if (sensor_state == STATE_IDLE) {
                        trigger_ping();
                        current_state = STATE_WAIT_FOR_LEFT_SCAN;
                    }
                }
                break;

            case STATE_WAIT_FOR_LEFT_SCAN:
                // Wait for the measurement to complete
                break;

            // MODIFIED: This state now happens AFTER reversing.
            case STATE_TURN:
                motor_stop(); // Ensure motor is stopped before turning
                if (planned_turn == TURN_RIGHT) {
                    printString("Action: Steering right.\n");
                    steer_right();
                } else {
                    printString("Action: Steering left.\n");
                    steer_left();
                }
                current_state = STATE_RECOVER_FORWARD;
                maneuver_start_time = current_time;
                break;

            case STATE_RECOVER_FORWARD:
                // A small delay to allow servo to turn before moving forward
                if (current_time - maneuver_start_time >= 200) {
                    motor_forward();
                    // Check total time for the forward part of the turn
                    if (current_time - maneuver_start_time >= (200 + TURN_DURATION_MS)) {
                         current_state = STATE_RECOVER_STRAIGHTEN;
                         maneuver_start_time = current_time;
                    }
                }
                break;

            case STATE_RECOVER_STRAIGHTEN:
                if (current_time - maneuver_start_time >= 500) {
                    printString("Maneuver complete. Resuming drive.\n");
                    steer_center();
                    current_state = STATE_DRIVING_FORWARD;
                    printString("Status: Moving Forward\n");
                }
                break;
        }

        // --- Process Sensor Data with Integrated Robust Logic ---
        if (sensor_state == STATE_MEASUREMENT_COMPLETE) {
            bool is_valid_measurement = false;
            uint16_t distance_cm = 0;

            if (pulse_count != TIMEOUT_PULSE_COUNT) {
                // Combine overflows with the final timer count for total duration
                uint32_t total_ticks = ((uint32_t)timer_overflow_count * 256) + pulse_count;
                // Calculate distance for Timer2 with prescaler 256: (total_ticks * 256 / 16) / 58
                distance_cm = (uint16_t)((total_ticks * 16) / 58);
                
                if (distance_cm >= MIN_RELIABLE_DISTANCE_CM && distance_cm <= MAX_RELIABLE_DISTANCE_CM) {
                    is_valid_measurement = true;
                }
            }

            // --- Part 1: Update logic based on the current state ---
            if (current_state == STATE_DRIVING_FORWARD) {
                if (is_valid_measurement) {
                    printString("Distance: "); printNumber(distance_cm); printString(" cm\n");
                    if (distance_cm < OBSTACLE_DISTANCE_CM) {
                        printString("Obstacle Detected! Status: Reversing\n");
                        motor_backward(); // Start reversing NOW
                        current_state = STATE_OBSTACLE_REVERSE; // Go to the new state
                        maneuver_start_time = current_time;
                    }
                }
            }
            else if (current_state == STATE_WAIT_FOR_RIGHT_SCAN) {
                if (is_valid_measurement) {
                    right_distance = distance_cm;
                    printString("Scan Right Distance: "); printNumber(right_distance); printString(" cm\n");
                } else {
                    // MODIFIED: Treat out of range as a very large, clear distance.
                    right_distance = 500; 
                    printString("Scan Right: Out of range (Clear Path)\n");
                }
                // ALWAYS advance the state and move the servo, even if measurement was invalid
                printString("Status: Scanning left...\n");
                uss_rot_second();
                current_state = STATE_SCAN_LEFT;
                maneuver_start_time = current_time;
            }
            else if (current_state == STATE_WAIT_FOR_LEFT_SCAN) {
                if (is_valid_measurement) {
                    left_distance = distance_cm;
                    printString("Scan Left Distance: "); printNumber(left_distance); printString(" cm\n");
                } else {
                     // MODIFIED: Treat out of range as a very large, clear distance.
                     left_distance = 500;
                     printString("Scan Left: Out of range (Clear Path)\n");
                }
                
                // This comparison now correctly handles out-of-range values
                if (right_distance > left_distance) {
                    planned_turn = TURN_RIGHT;
                    printString("Decision: Path right is clearer.\n");
                } else {
                    planned_turn = TURN_LEFT;
                    printString("Decision: Path left is clearer.\n");
                }
                uss_rot_initial(); // Center the sensor
                
                // Transition directly to the turning state
                current_state = STATE_TURN;
                maneuver_start_time = current_time;
            }

            // Reset the sensor state to allow a new measurement
            sensor_state = STATE_IDLE;
        }
    }
    return 0;
}

// --- ISRs and Helper Functions (Updated for Robust Sensor) ---

ISR(PCINT0_vect) {
    if (PINB & (1 << ECHO_PIN)) {
        if (sensor_state == STATE_WAITING_FOR_ECHO) {
            TCNT2 = 0;
            timer_overflow_count = 0;
            // Start Timer2 w/ prescaler 256
            TCCR2B = (1 << CS22);
        }
    } else {
        if (TCCR2B != 0) {
            TCCR2B = 0; // Stop timer
            pulse_count = TCNT2;
            sensor_state = STATE_MEASUREMENT_COMPLETE;
        }
    }
}

ISR(TIMER2_OVF_vect) {
    timer_overflow_count++;
    if (timer_overflow_count >= MAX_TIMER_OVERFLOWS) {
        TCCR2B = 0; // Stop the timer
        pulse_count = TIMEOUT_PULSE_COUNT;
        sensor_state = STATE_MEASUREMENT_COMPLETE;
    }
}

void trigger_ping(void) {
    sensor_state = STATE_WAITING_FOR_ECHO;
    PORTB |= (1 << TRIGGER_PIN);
    _delay_us(10);
    PORTB &= ~(1 << TRIGGER_PIN);
}

void ultrasonic_init(void) {
    DDRB |= (1 << TRIGGER_PIN);
    DDRB &= ~(1 << ECHO_PIN);
    PCICR |= (1 << PCIE0);
    PCMSK0 |= (1 << PCINT4); // Use PCINT4 for PB4
    // Enable Timer2 Overflow Interrupt
    TIMSK2 |= (1 << TOIE2);
}

// --- UART Functions (Unchanged) ---
void uart_init(void) {
    uint16_t ubrr = (F_CPU / (16UL * 9600UL)) - 1;
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)(ubrr);
    UCSR0B = (1 << TXEN0);
    UCSR0C = (3 << UCSZ00);
}

void printString(const char* s) {
    while (*s) {
        while (!(UCSR0A & (1 << UDRE0)));
        UDR0 = *s++;
    }
}

void printNumber(int n) {
    char buf[10];
    itoa(n, buf, 10);
    printString(buf);
}

