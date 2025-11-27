#include "main.h" // Includes sensing.h indirectly
#include "timer_motor.h" // For millis()
#include <util/delay.h>

// --- Global variable definition ---
volatile uint16_t zero_current_mV = 2500; // Default, will be calibrated

// ======================================================================
// --- 7. Helper Functions (ADC & Current) ---
// ======================================================================

void adc_init(void) {
    ADMUX = (1 << REFS0); // AVcc (5V)
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Enable, Prescaler 128
}

uint16_t analogReadSmooth(uint8_t channel, uint8_t samples) {
    uint32_t sum = 0;
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F); // Select channel
    for (uint8_t i = 0; i < samples; i++) {
        ADCSRA |= (1 << ADSC); 
        while (ADCSRA & (1 << ADSC));
        sum += ADC;
    }
    return (uint16_t)(sum / samples);
}

void calibrate_current_sensor(void) {
    uint32_t sum_mV = 0;
    // Take 250 samples over 500ms for a very stable reading
    for (int i = 0; i < 250; i++) {
        // Read ADC0, 10 samples averaged
        sum_mV += (uint16_t)((analogReadSmooth(0, 10) * 5000UL) / 1023);
        _delay_ms(2);
    }
    zero_current_mV = (uint16_t)(sum_mV / 250);
}

int read_current_mA(void) {
    // Read ADC0, 10 samples averaged
    uint16_t adc_millivolts = (uint16_t)((analogReadSmooth(0, 10) * 5000UL) / 1023);
    // ACS712 20A version has 100mV/A (0.1mV/mA) sensitivity
    float voltage_offset_mV = (float)adc_millivolts - (float)zero_current_mV;
    return (int)(voltage_offset_mV / 0.1f);
}

