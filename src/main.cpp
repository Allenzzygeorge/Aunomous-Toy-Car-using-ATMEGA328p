#include "main.h"

// ======================================================================
// --- 1. Global Variable Definitions ---
// ======================================================================

volatile MeasurementState sensor_state = STATE_IDLE;
volatile uint16_t pulse_count = 0;
volatile uint8_t timer_overflow_count = 0;
volatile CarState current_state = STATE_INITIAL_WAIT;
TurnDirection planned_turn = TURN_RIGHT; 
uint16_t last_measured_distance = 0;
uint8_t current_pwm_speed = 0; 

// Runtime cruise override (defaults to the compiled SPEED_CRUISE)
volatile uint8_t cruise_pwm = SPEED_CRUISE;

// Serial parser variables
static unsigned long _timeout_ms = 500;
static int pushed_back_char = -1;

// --- Define Software Reset Function ---
void (*soft_reset)(void) = 0;

// ======================================================================
// --- 2. MAIN FUNCTION ---
// ======================================================================

int main(void) {
    // --- Hardware Initialization ---
    Serial_begin(9600);
    timer_motor_init(); 
    ultrasonic_init();  
    servos_init();      
    adc_init();         
    sei();

    Serial_println("\n--- Smart Car Ready ---");

    // --- Set Initial Conditions ---
    motor_stop();
    current_pwm_speed = 0; 
    
    // --- Calibrate Current Sensor ---
    Serial_println("Calibrating current sensor...");
    _delay_ms(1000); 
    calibrate_current_sensor();
    Serial_print("Calibration complete. Zero mV: ");
    Serial_println((int)zero_current_mV);
    
    // --- Timestamps and Variables ---
    uint32_t maneuver_start_time = 0;
    uint16_t right_distance = 0;
    uint16_t left_distance = 0;
    
    uint32_t previousMillisPrint = 0;
    uint32_t last_ping_time = 0; 
    
    const uint16_t PING_INTERVAL_MS = 100; 
    const uint16_t PRINT_INTERVAL = 100; 
    
    int motCurrent = 0;
    const int STALL_CURRENT_THRESHOLD = 2500; // Limit in mA
    uint8_t over_current_counter = 0;         // <-- NEW: Counter for consecutive high readings

    // --- Main Control Loop ---
    while (1) {
        uint32_t current_time = millis();

        // ==============================================================
        // --- INDEPENDENT SAFETY CHECK (Stall Protection) ---
        // ==============================================================
        int safety_current = abs(read_current_mA());
        
        if (safety_current > STALL_CURRENT_THRESHOLD) {
             // Increment counter if current is too high
             over_current_counter++;
        } else {
             // Reset counter if current drops back to normal (must be consecutive)
             over_current_counter = 0;
        }

        // Trigger Stop only if we have 5 consecutive bad readings
        if (over_current_counter >= 56) {
             motor_stop();
             current_pwm_speed = 0; 
             Serial_print("!!! STALL DETECTED (");
             Serial_print(safety_current);
             Serial_println("mA) -> EMERGENCY STOP !!!");
             
             over_current_counter = 0; // Reset counter after tripping
             _delay_ms(1000);          // Cool down delay
        }

        // ==============================================================

        // --- Serial Command Parser ---
        if (Serial_available()) {
             char c = Serial_read();
             
             if (c == 'x') {
                 motor_stop();
                 current_pwm_speed = 0; 
                 Serial_println("CMD: Motor Stop.");
             }
             else if (c == 's') {
                 Serial_println("CMD: FORCE OBSTACLE SEQUENCE!");
                 
                 TCCR2B = 0; 
                 sensor_state = STATE_IDLE;
                 
                 // --- BRAKE FOR 300ms ---
                 Serial_println("Braking (300ms)...");
                 motor_set_speed(SPEED_TURN);
                 motor_backward();   
                 _delay_ms(300);     
                 motor_stop();       
                 current_pwm_speed = 0;
                 
                 Serial_println("Looking Right...");
                 uss_rot_first(); 
                 
                 current_state = STATE_SCAN_RIGHT; 
                 maneuver_start_time = current_time;
             }
             else if (c == 'r') {
                 Serial_println("CMD: Resetting...");
                 motor_stop();
                 _delay_ms(100); 
                 soft_reset();   
             }
             else if (c == 'h') {
                 // Set cruise PWM to 150
                 cruise_pwm = 150;
                 Serial_print("CMD: Set cruise PWM -> ");
                 Serial_println(cruise_pwm);

                 // If currently driving, apply immediately
                 if (current_state == STATE_DRIVING_FORWARD ||
                     current_state == STATE_RECOVER_FORWARD ||
                     current_state == STATE_RECOVER_STRAIGHTEN) {
                     motor_set_speed(cruise_pwm);
                     current_pwm_speed = cruise_pwm;
                 }
             }
             else if (c == 'l') {
                 // Set cruise PWM to 100
                 cruise_pwm = 100;
                 Serial_print("CMD: Set cruise PWM -> ");
                 Serial_println(cruise_pwm);

                 // If currently driving, apply immediately
                 if (current_state == STATE_DRIVING_FORWARD ||
                     current_state == STATE_RECOVER_FORWARD ||
                     current_state == STATE_RECOVER_STRAIGHTEN) {
                     motor_set_speed(cruise_pwm);
                     current_pwm_speed = cruise_pwm;
                 }
             }
        } 

        // --- Car State Machine Logic ---
        switch (current_state) {
            case STATE_INITIAL_WAIT:
                if (current_time >= 3000) {
                    Serial_println("Init done. Forward.");
                    motor_set_speed(cruise_pwm); 
                    current_pwm_speed = cruise_pwm; 
                    motor_forward();
                    current_state = STATE_DRIVING_FORWARD;
                }
                break;

            case STATE_DRIVING_FORWARD:
                // Checks every 100ms
                if (current_time - last_ping_time >= PING_INTERVAL_MS) {
                    if (sensor_state == STATE_IDLE) {
                        last_ping_time = current_time;
                        trigger_ping();
                    }
                }
                break;

            // --- 1. SCAN RIGHT ---
            case STATE_SCAN_RIGHT:
                if (current_time - maneuver_start_time >= 1000) {
                    if (sensor_state == STATE_IDLE) { 
                        trigger_ping();
                        current_state = STATE_WAIT_FOR_RIGHT_SCAN;
                    }
                }
                break;

            case STATE_WAIT_FOR_RIGHT_SCAN: break; 

            // --- 2. SCAN LEFT ---
            case STATE_SCAN_LEFT:
                if (current_time - maneuver_start_time >= 1000) {
                     if (sensor_state == STATE_IDLE) { 
                        trigger_ping();
                        current_state = STATE_WAIT_FOR_LEFT_SCAN;
                    }
                }
                break;

            case STATE_WAIT_FOR_LEFT_SCAN: break; // Logic handled in Sensor Block

            // --- 3. REVERSE WITH COUNTER STEER ---
            case STATE_OBSTACLE_REVERSE:
                
                motor_set_speed(SPEED_TURN);
                current_pwm_speed = SPEED_TURN;
                motor_backward();

                // Counter Steer Logic
                if (planned_turn == TURN_RIGHT) {
                    steer_left();
                } else {
                    steer_right();
                }

                if (current_time - maneuver_start_time >= REVERSE_DURATION_MS) {
                    motor_stop();
                    uss_rot_initial(); // Center Sensor
                    current_state = STATE_TURN;
                    maneuver_start_time = current_time;
                }
                break;

            // --- 4. FORWARD WITH CORRECT STEER ---
            case STATE_TURN:
                motor_set_speed(SPEED_TURN); 
                current_pwm_speed = SPEED_TURN; 
                
                // Correct Steer Logic
                if (planned_turn == TURN_RIGHT) {
                    Serial_println("Action: Turning right.");
                    steer_right();
                } else {
                    Serial_println("Action: Turning left.");
                    steer_left();
                }
                motor_forward();

                current_state = STATE_RECOVER_FORWARD;
                maneuver_start_time = current_time;
                break;

            case STATE_RECOVER_FORWARD:
                // Checks every 100ms
                if (current_time - last_ping_time >= PING_INTERVAL_MS) {
                    if (sensor_state == STATE_IDLE) {
                        last_ping_time = current_time;
                        trigger_ping();
                    }
                }
                
                if (current_time - maneuver_start_time >= TURN_DURATION_MS) {
                     current_state = STATE_RECOVER_STRAIGHTEN;
                     maneuver_start_time = current_time;
                }
                break;

            case STATE_RECOVER_STRAIGHTEN:
                // Checks every 100ms
                if (current_time - last_ping_time >= PING_INTERVAL_MS) {
                    if (sensor_state == STATE_IDLE) {
                        last_ping_time = current_time;
                        trigger_ping();
                    }
                }

                if (current_time - maneuver_start_time >= 500) {
                    Serial_println("Resuming Forward.");
                    steer_center();
                    motor_set_speed(cruise_pwm); 
                    current_pwm_speed = cruise_pwm; 
                    current_state = STATE_DRIVING_FORWARD;
                }
                break;
        }

        // --- Process Sensor Data ---
        if (sensor_state == STATE_MEASUREMENT_COMPLETE) {
            bool is_valid_measurement = false;
            uint16_t distance_cm = 0;

            if (pulse_count != TIMEOUT_PULSE_COUNT) {
                uint32_t total_ticks = ((uint32_t)timer_overflow_count * 256) + pulse_count;
                distance_cm = (uint16_t)((total_ticks * 16) / 58);
                
                if (distance_cm >= MIN_RELIABLE_DISTANCE_CM && distance_cm <= MAX_RELIABLE_DISTANCE_CM) {
                    is_valid_measurement = true;
                    last_measured_distance = distance_cm;
                } else {
                    last_measured_distance = 999; 
                }
            } else {
                last_measured_distance = 999;
            }
            
            // --- GLOBAL OBSTACLE CHECK ---
            if (is_valid_measurement && distance_cm < OBSTACLE_DISTANCE_CM)
            {
                if (current_state == STATE_DRIVING_FORWARD ||
                    current_state == STATE_RECOVER_FORWARD ||
                    current_state == STATE_RECOVER_STRAIGHTEN)
                {
                    Serial_println("!!! OBSTACLE DETECTED !!!");
                    
                    // --- ACTIVE BRAKE (300ms) ---
                    Serial_println("Action: Active Brake (300ms)!");
                    motor_set_speed(SPEED_TURN); 
                    motor_backward();            
                    _delay_ms(300); 
                    motor_stop();                
                    current_pwm_speed = 0;
                    
                    Serial_println("Looking Right...");
                    uss_rot_first(); 
                    
                    current_state = STATE_SCAN_RIGHT; 
                    maneuver_start_time = current_time;
                    
                    sensor_state = STATE_IDLE; 
                    continue; 
                }
            }

            // --- State-Specific Sensor Logic ---
            if (current_state == STATE_WAIT_FOR_RIGHT_SCAN) {
                if (is_valid_measurement) {
                    right_distance = distance_cm;
                    Serial_print("Right: "); Serial_println(right_distance);
                } else {
                    right_distance = 500; 
                    Serial_println("Right: Clear");
                }
                Serial_println("Looking Left...");
                uss_rot_second(); // Look Left
                
                current_state = STATE_SCAN_LEFT;
                maneuver_start_time = current_time;
            }
            else if (current_state == STATE_WAIT_FOR_LEFT_SCAN) {
                if (is_valid_measurement) {
                    left_distance = distance_cm;
                    Serial_print("Left: "); Serial_println(left_distance);
                } else {
                     left_distance = 500;
                     Serial_println("Left: Clear");
                }
                
                // --- DECISION LOGIC ---
                if (right_distance > left_distance) {
                    planned_turn = TURN_RIGHT;
                    Serial_println("Decision: Go Right");
                } else {
                    planned_turn = TURN_LEFT;
                    Serial_println("Decision: Go Left");
                }
                
                Serial_println("Centering Servo...");
                uss_rot_initial(); // Center Servo
                _delay_ms(500); 
                
                // --- START REVERSE WITH COUNTER STEER ---
                Serial_println("Status: Reversing to pivot...");
                current_state = STATE_OBSTACLE_REVERSE;
                maneuver_start_time = current_time;
            }

            sensor_state = STATE_IDLE;
        }

        // --- Serial Print Task ---
        if (current_time - previousMillisPrint >= PRINT_INTERVAL) {
            previousMillisPrint = current_time;
            motCurrent = abs(read_current_mA());
            Serial_print("D:");
            if (last_measured_distance == 999) Serial_print("---"); 
            else Serial_print(last_measured_distance);
            
            Serial_print(" C:"); Serial_print(motCurrent);
            Serial_print(" PWM:"); Serial_println(current_pwm_speed); 
        }
    }
    return 0; 
}

// ======================================================================
// --- 3. ISRs (Ultrasonic) (Unchanged) ---
// ======================================================================
ISR(PCINT0_vect) {
    if (PINB & (1 << ECHO_PIN)) {
        if (sensor_state == STATE_WAITING_FOR_ECHO) {
            TCNT2 = 0;
            timer_overflow_count = 0;
            TCCR2B = (1 << CS22); 
        }
    } else {
        if (TCCR2B != 0) {
            TCCR2B = 0; 
            pulse_count = TCNT2;
            sensor_state = STATE_MEASUREMENT_COMPLETE;
        }
    }
}

ISR(TIMER2_OVF_vect) {
    timer_overflow_count++;
    if (timer_overflow_count >= MAX_TIMER_OVERFLOWS) {
        TCCR2B = 0; 
        pulse_count = TIMEOUT_PULSE_COUNT;
        sensor_state = STATE_MEASUREMENT_COMPLETE;
    }
}

// ======================================================================
// --- 4. Helper Functions (Ultrasonic) (Unchanged) ---
// ======================================================================
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
    PCMSK0 |= (1 << PCINT4); 
    TIMSK2 |= (1 << TOIE2); 
}

// ======================================================================
// --- 5. Helper Functions (Serial Parser) (Unchanged except additions) ---
// ======================================================================
void Serial_begin(int BAUD) {
    uint16_t ubrr = (F_CPU / (16UL * BAUD)) - 1;
    UBRR0H = (ubrr >> 8); 
    UBRR0L = ubrr;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0); 
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}
void uart_putchar(char c) { while (!(UCSR0A & (1 << UDRE0))); UDR0 = c; }
void uart_print_string(const char *s) { while (*s) uart_putchar(*s++); }
void uart_print_int(long num) {
    char buf[12]; int i = 0; bool neg = false;
    if (num == 0) { uart_putchar('0'); return; }
    if (num < 0) { neg = true; num = -num; }
    while (num > 0) { buf[i++] = (num % 10) + '0'; num /= 10; }
    if (neg) buf[i++] = '-';
    while (i > 0) uart_putchar(buf[--i]);
}
void uart_print_float(double num, int precision) {
    if (num < 0) { uart_putchar('-'); num = -num; }
    long int_part = (long)num;
    double frac = num - int_part;
    uart_print_int(int_part);
    uart_putchar('.');
    for (int i = 0; i < precision; i++) {
        frac *= 10;
        int digit = (int)frac;
        uart_putchar('0' + digit);
        frac -= digit;
    }
}
uint8_t Serial_available(void) { return (UCSR0A & (1 << RXC0)); }
char Serial_read(void) { while (!(UCSR0A & (1 << RXC0))); return UDR0; }

static int timedRead(void) {
    if (pushed_back_char != -1) { int c = pushed_back_char; pushed_back_char = -1; return c; }
    unsigned long start_time = millis();
    while ((millis() - start_time) < _timeout_ms) { if (Serial_available()) return Serial_read(); }
    return -1; 
}
float Serial_parseFloat(void) {
    bool is_negative = false; bool has_started = false; bool in_fraction = false;
    float value = 0.0f; float decimal_divisor = 10.0f; int c;
    while (1) {
        c = timedRead(); if (c < 0) break;
        if (c >= '0' && c <= '9') {
            has_started = true;
            if (in_fraction) { value = value + (float)(c - '0') / decimal_divisor; decimal_divisor *= 10.0f; }
            else { value = value * 10.0f + (c - '0'); }
        }
        else if (c == '-' && !has_started) { is_negative = true; has_started = true; }
        else if (c == '.' && !in_fraction) { has_started = true; in_fraction = true; }
        else if (has_started) { pushed_back_char = c; break; }
    }
    return is_negative ? -value : value;
}

void Serial_print(const char *s) { uart_print_string(s); }
void Serial_print(int num) { uart_print_int(num); }
void Serial_println(const char *s) { uart_print_string(s); uart_putchar('\r'); uart_putchar('\n'); }
void Serial_println(int num) { uart_print_int(num); uart_putchar('\r'); uart_putchar('\n'); }
void Serial_printFloat(double num, int precision) { uart_print_float(num, precision); }
