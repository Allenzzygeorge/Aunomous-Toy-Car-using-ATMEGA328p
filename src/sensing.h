#ifndef SENSING_H
#define SENSING_H

#include <avr/io.h>
#include <stdbool.h>

// --- Global variable from sensing.cpp ---
// This holds the calibrated "zero current" reading in millivolts
extern volatile uint16_t zero_current_mV;

// --- Function Prototypes ---

/**
 * @brief Initializes the ADC for reading the current sensor on ADC0.
 */
void adc_init(void);

/**
 * @brief Reads the specified ADC channel multiple times and returns the average.
 * @param channel The ADC channel to read (0-5).
 * @param samples The number of samples to average.
 * @return The 10-bit averaged ADC value.
 */
uint16_t analogReadSmooth(uint8_t channel, uint8_t samples);

/**
 * @brief Calibrates the current sensor by reading the zero-current (no load) voltage.
 * This function takes 500ms to run.
 */
void calibrate_current_sensor(void);

/**
 * @brief Reads the current sensor and returns the value in milliamps (mA).
 * @return The current in mA (can be negative).
 */
int read_current_mA(void);


#endif // SENSING_H

