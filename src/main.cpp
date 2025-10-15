#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdlib.h>

//  FIX: Include the new unified servos module
#include "timer.h"
#include "servo.h" 
#include "motor.h"

// --- System Defines ---
#define OBSTACLE_DISTANCE_CM 80
#define PING_INTERVAL_MS     100
#define TURN_DURATION_MS     1200

// --- Pin Defines ---
#define TRIGGER_PIN PB3
#define ECHO_PIN    PB4

// --- Global Volatile Variables ---
volatile uint16_t pulse_count = 0;
volatile bool new_measurement_ready = false;

// --- State Machine ---
typedef enum {
    STATE_INITIAL_WAIT,
    STATE_DRIVING_FORWARD,
    STATE_OBSTACLE_STOP,
    STATE_SCAN_RIGHT,
    STATE_WAIT_FOR_RIGHT_SCAN,
    STATE_SCAN_LEFT,
    STATE_WAIT_FOR_LEFT_SCAN,
    STATE_DECIDE_AND_REVERSE,
    STATE_TURN,
    STATE_RECOVER_FORWARD,
    STATE_RECOVER_STRAIGHTEN
} CarState;
volatile CarState current_state = STATE_INITIAL_WAIT;

// --- Function Prototypes ---
void uart_init(void);
void printString(const char* str);
void printNumber(int number);
void ultrasonic_init(void);
void trigger_ping(void);

int main(void) {
    // --- Hardware Initialization ---
    uart_init();
    timer_init();
    motor_init();
    ultrasonic_init();
    
    // ⭐ FIX: Call the single, unified servo initialization function
    servos_init();
    
    printString("\n--- Smart Car Initializing ---\n");

    // --- Set Initial Conditions ---
    motor_stop();
    // The servos are already centered by servos_init()

    // --- Timestamps and Variables for Logic ---
    uint32_t last_ping_time = 0;
    uint32_t maneuver_start_time = 0;
    uint16_t right_distance = 0;
    uint16_t left_distance = 0;

    // --- Main Control Loop (Logic is unchanged) ---
    while (1) {
        uint32_t current_time = millis();
        motor_update();

        switch (current_state) {
            case STATE_INITIAL_WAIT:
                if (current_time >= 3000) {
                    printString("Initialization complete. Starting motor.\n");
                    motor_set_speed(80);
                    motor_forward();
                    current_state = STATE_DRIVING_FORWARD;
                }
                break;

            case STATE_DRIVING_FORWARD:
                if (current_time - last_ping_time >= PING_INTERVAL_MS) {
                    last_ping_time = current_time;
                    trigger_ping();
                }
                break;
            
            // ... (The rest of your state machine logic remains exactly the same) ...
            case STATE_OBSTACLE_STOP:
                if (current_time - maneuver_start_time >= 200) {
                    printString("Scanning right...\n");
                    uss_rot_first();
                    current_state = STATE_SCAN_RIGHT;
                    maneuver_start_time = current_time;
                }
                break;
            case STATE_SCAN_RIGHT:
                if (current_time - maneuver_start_time >= 500) {
                    trigger_ping();
                    current_state = STATE_WAIT_FOR_RIGHT_SCAN;
                }
                break;
            case STATE_WAIT_FOR_RIGHT_SCAN:
                break;
            case STATE_SCAN_LEFT:
                if (current_time - maneuver_start_time >= 1000) {
                    trigger_ping();
                    current_state = STATE_WAIT_FOR_LEFT_SCAN;
                }
                break;
            case STATE_WAIT_FOR_LEFT_SCAN:
                break;
            case STATE_DECIDE_AND_REVERSE:
                if (current_time - maneuver_start_time >= 500) {
                    printString("Reversing...\n");
                    motor_backward();
                    current_state = STATE_TURN;
                    maneuver_start_time = current_time;
                }
                break;
            case STATE_TURN:
                if (current_time - maneuver_start_time >= 500) {
                    motor_stop();
                    if (right_distance > left_distance) {
                        printString("Turning right (clearer path).\n");
                        steer_right();
                    } else {
                        printString("Turning left (clearer path).\n");
                        steer_left();
                    }
                    current_state = STATE_RECOVER_FORWARD;
                    maneuver_start_time = current_time;
                }
                break;
            case STATE_RECOVER_FORWARD:
                if (current_time - maneuver_start_time >= TURN_DURATION_MS) {
                    motor_forward();
                    current_state = STATE_RECOVER_STRAIGHTEN;
                    maneuver_start_time = current_time;
                }
                break;
            case STATE_RECOVER_STRAIGHTEN:
                if (current_time - maneuver_start_time >= 500) {
                    printString("Maneuver complete. Resuming drive.\n");
                    steer_center();
                    current_state = STATE_DRIVING_FORWARD;
                }
                break;
        }

        if (new_measurement_ready) {
            new_measurement_ready = false;
            uint16_t distance_cm = (uint32_t)pulse_count * 64 / 58;

            if (current_state == STATE_DRIVING_FORWARD) {
                printString("Distance: "); printNumber(distance_cm); printString(" cm\n");
                if (distance_cm < OBSTACLE_DISTANCE_CM) {
                    printString("Obstacle Detected! Stopping.\n");
                    motor_stop();
                    current_state = STATE_OBSTACLE_STOP;
                    maneuver_start_time = current_time;
                }
            }
            else if (current_state == STATE_WAIT_FOR_RIGHT_SCAN) {
                right_distance = distance_cm;
                printString("Right distance: "); printNumber(right_distance); printString(" cm\n");
                printString("Scanning left...\n");
                uss_rot_second();
                current_state = STATE_SCAN_LEFT;
                maneuver_start_time = current_time;
            }
            else if (current_state == STATE_WAIT_FOR_LEFT_SCAN) {
                left_distance = distance_cm;
                printString("Left distance: "); printNumber(left_distance); printString(" cm\n");
                printString("Deciding direction...\n");
                uss_rot_initial();
                current_state = STATE_DECIDE_AND_REVERSE;
                maneuver_start_time = current_time;
            }
        }
    }
    return 0;
}

// --- ISR and Helper Functions (Unchanged) ---
ISR(PCINT0_vect) {
    if (PINB & (1 << ECHO_PIN)) {
        TCNT2 = 0;
        TCCR2B = (1 << CS22);
    } else {
        pulse_count = TCNT2;
        TCCR2B = 0;
        new_measurement_ready = true;
    }
}

void trigger_ping(void) {
    PORTB |= (1 << TRIGGER_PIN);
    _delay_us(10);
    PORTB &= ~(1 << TRIGGER_PIN);
}

void ultrasonic_init(void) {
    DDRB |= (1 << TRIGGER_PIN);
    DDRB &= ~(1 << ECHO_PIN);
    PCICR |= (1 << PCIE0);
    PCMSK0 |= (1 << PCINT4);
}

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