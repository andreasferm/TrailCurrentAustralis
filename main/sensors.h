#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

// ---------------------------------------------------------------------------
// Hardware configuration
// ---------------------------------------------------------------------------

#define TCA9548A_ADDR     0x70   // I²C multiplexer (A0=A1=A2=GND)
#define SCD41_ADDR        0x62   // Fixed address on every mux channel
#define MAX_SCD41_SENSORS 8      // TCA9548A has 8 channels

// ---------------------------------------------------------------------------
// Sensor result type
// ---------------------------------------------------------------------------

typedef struct {
    uint16_t co2_ppm;        // 0–5000 ppm (NDIR)
    float    temperature_c;  // °C
    float    humidity;       // %RH (0–100)
    bool     valid;
} scd41_data_t;

// ---------------------------------------------------------------------------
// Multi-sensor state (opaque to caller — managed internally)
// ---------------------------------------------------------------------------

/**
 * Initialize the I²C bus with TCA9548A multiplexer.
 * Probes each mux channel for an SCD41 and records which are present.
 * Returns ESP_OK if the bus initializes successfully (even if zero sensors found).
 */
esp_err_t sensors_init(int sda_pin, int scl_pin);

/**
 * Returns the number of SCD41 sensors discovered during sensors_init().
 */
uint8_t sensors_get_count(void);

/**
 * Select a mux channel and issue the SCD41 single-shot measurement command.
 * The measurement takes ~5000 ms to complete.
 * Returns ESP_OK if the command was sent successfully.
 */
esp_err_t scd41_trigger_single_shot(uint8_t sensor_index);

/**
 * Select a mux channel and read the SCD41 measurement result.
 * Returns valid=false if data is not yet ready or if the read fails.
 */
scd41_data_t scd41_read(uint8_t sensor_index);

/**
 * Select a mux channel and set ambient pressure compensation.
 * Optional — improves accuracy if barometric pressure is known.
 */
esp_err_t scd41_set_ambient_pressure(uint8_t sensor_index, uint16_t pressure_hpa);
