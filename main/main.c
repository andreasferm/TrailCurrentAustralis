#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "can_common.h"
#include "wifi_config.h"
#include "ota.h"
#include "discovery.h"
#include "sensors.h"

static const char *TAG = "australis";

// Waveshare ESP32-S3-RS485-CAN pin assignments
#define I2C_SDA_PIN   5
#define I2C_SCL_PIN   6
#define CAN_RX_PIN    16
#define CAN_TX_PIN    15

// CAN protocol IDs
#define CAN_ID_OTA                0x00
#define CAN_ID_WIFI_CONFIG        0x01
#define CAN_ID_DISCOVERY_TRIGGER  0x02
#define CAN_ID_SENSOR_DATA        0x30   // Australis sensor data (indexed)
#define CAN_ID_ALARM              0x31   // Australis CO2 alarm (indexed, edge-triggered)

// Timing
#define CAN_STATUS_PERIOD_MS      1000
#define TX_PROBE_INTERVAL_MS      2000
#define SCD41_MEASUREMENT_TIME_MS 5000

// Alarm threshold
#define CO2_ALARM_PPM             2000

// ---------------------------------------------------------------------------
// Per-sensor state
// ---------------------------------------------------------------------------

typedef struct {
    scd41_data_t last_reading;
    int64_t      trigger_time_us;  // when single-shot was issued
    bool         triggered;        // waiting for measurement to complete
    bool         alarm_active;     // edge-trigger state: true if CO2 ≥ threshold
} sensor_state_t;

static sensor_state_t s_sensors[MAX_SCD41_SENSORS];
static uint8_t s_sensor_count = 0;

// Index map: maps slot 0..N-1 to the actual mux channel with a sensor present.
// This lets the scheduler iterate over a dense array without gaps.
static uint8_t s_index_map[MAX_SCD41_SENSORS];

// ---------------------------------------------------------------------------
// Shared sensor data for CAN task (latest reading to transmit)
// Written by main task, read by TWAI task.
// ---------------------------------------------------------------------------

typedef struct {
    uint8_t  sensor_index;
    int8_t   temp_c;
    int8_t   temp_f;
    uint16_t humidity_scaled;
    uint16_t co2_ppm;
    uint8_t  sensor_count;
    bool     pending;           // main task sets true, CAN task clears after TX
} can_env_frame_t;

typedef struct {
    uint8_t  sensor_index;
    uint8_t  alarm_flags;
    uint16_t co2_ppm;
    bool     pending;
} can_alarm_frame_t;

static volatile can_env_frame_t   s_can_env   = { .pending = false };
static volatile can_alarm_frame_t s_can_alarm = { .pending = false };

// ---------------------------------------------------------------------------
// TWAI (CAN) task — independent FreeRTOS task followin the canonical
// TX_ACTIVE / TX_PROBING state machine used across all TrailCurrent modules.
// ---------------------------------------------------------------------------

static void twai_task(void *arg)
{
    // Configure alerts BEFORE any bus activity so no error transitions are missed.
    twai_reconfigure_alerts(CAN_COMMON_ALERTS, NULL);
   // Alerts armed — version broadcast TX failure is caught by the state machine.
    can_common_version_broadcast();

    typedef enum { TX_ACTIVE, TX_PROBING } tx_state_t;
    bool bus_off = false;
    tx_state_t tx_state = TX_ACTIVE;
    int tx_fail_count = 0;
    const int TX_FAIL_THRESHOLD = 3;
    int64_t last_tx_us = 0;
    const int64_t tx_period_us = CAN_STATUS_PERIOD_MS * 1000LL;
    const int64_t tx_probe_period_us = TX_PROBE_INTERVAL_MS * 1000LL;

    while (1) {
        uint32_t triggered;
        twai_read_alerts(&triggered, pdMS_TO_TICKS(CAN_STATUS_PERIOD_MS));
        // Bus-off: stop transmitting, initiate recovery
        if (triggered & TWAI_ALERT_BUS_OFF) {
            ESP_LOGE(TAG, "TWAI bus-off, initiating recovery");
            bus_off = true;
            twai_initiate_recovery();
            // No continue — fall through so RX_DATA in the same poll is still processed.
        }
        if (triggered & TWAI_ALERT_BUS_RECOVERED) {
            ESP_LOGI(TAG, "TWAI bus recovered, restarting");
            twai_start();
            bus_off = false;
            tx_fail_count = 0;
            tx_state = TX_PROBING;
        }
        if (triggered & TWAI_ALERT_ERR_PASS) {
            ESP_LOGW(TAG, "TWAI error passive");
        }
        if (triggered & TWAI_ALERT_TX_FAILED) {
            if (tx_state == TX_ACTIVE) {
                tx_fail_count++;
                if (tx_fail_count >= TX_FAIL_THRESHOLD) {
                    tx_state = TX_PROBING;
                    ESP_LOGW(TAG, "TWAI no peers, entering slow probe");
                }
            }
        }
        if (triggered & TWAI_ALERT_TX_SUCCESS) {
            if (tx_state == TX_PROBING) {
                tx_state = TX_ACTIVE;
                tx_fail_count = 0;
                can_common_version_broadcast();
                ESP_LOGI(TAG, "TWAI peer detected, resuming normal TX");
            }
            tx_fail_count = 0;
        }

        // Drain received messages and dispatch
        if (triggered & TWAI_ALERT_RX_DATA) {
            if (tx_state == TX_PROBING) {
                tx_state = TX_ACTIVE;
                tx_fail_count = 0;
                can_common_version_broadcast();
                ESP_LOGI(TAG, "TWAI peer detected via RX, resuming normal TX");
            }
            twai_message_t msg;
            while (twai_receive(&msg, 0) == ESP_OK) {
                if (msg.rtr) continue;
                switch (msg.identifier) {
                case CAN_ID_OTA:
                    if (msg.data_length_code >= 3)
                        ota_handle_trigger(msg.data, msg.data_length_code);
                    break;
                case CAN_ID_WIFI_CONFIG:
                    if (msg.data_length_code >= 1)
                        wifi_config_handle_can(msg.data, msg.data_length_code);
                    break;
                case CAN_ID_DISCOVERY_TRIGGER:
                    discovery_handle_trigger();
                    break;
                default:
                    break;
                }
            }
        }

        wifi_config_check_timeout();

        // Periodic TX — send pending sensor data frame
        int64_t now_us = esp_timer_get_time();
        int64_t effective_period = (tx_state == TX_PROBING) ? tx_probe_period_us : tx_period_us;
        if (!bus_off && (now_us - last_tx_us >= effective_period)) {
            last_tx_us = now_us;

            // Transmit latest sensor data if pending
            if (s_can_env.pending) {
                can_env_frame_t snap;
                // Snapshot the volatile struct — single-byte and aligned 16-bit
                // reads are atomic on Xtensa, but snapshot for clarity.
                snap.sensor_index    = s_can_env.sensor_index;
                snap.temp_c          = s_can_env.temp_c;
                snap.temp_f          = s_can_env.temp_f;
                snap.humidity_scaled = s_can_env.humidity_scaled;
                snap.co2_ppm        = s_can_env.co2_ppm;
                snap.sensor_count   = s_can_env.sensor_count;

                twai_message_t m = {
                    .identifier = CAN_ID_SENSOR_DATA,
                    .data_length_code = 8,
                    .data = {
                        snap.sensor_index,
                        (uint8_t)snap.temp_c,
                        (uint8_t)snap.temp_f,
                        (snap.humidity_scaled >> 8) & 0xFF,
                        snap.humidity_scaled & 0xFF,
                        (snap.co2_ppm >> 8) & 0xFF,
                        snap.co2_ppm & 0xFF,
                        snap.sensor_count,
                    }
                };
                twai_transmit(&m, 0);
                s_can_env.pending = false;
            }
        }

        // Alarm frames are sent immediately (edge-triggered, not periodic)
        if (!bus_off && s_can_alarm.pending) {
            can_alarm_frame_t asnap;
            asnap.sensor_index = s_can_alarm.sensor_index;
            asnap.alarm_flags  = s_can_alarm.alarm_flags;
            asnap.co2_ppm      = s_can_alarm.co2_ppm;

            twai_message_t m = {
                .identifier = CAN_ID_ALARM,
                .data_length_code = 4,
                .data = {
                    asnap.sensor_index,
                    asnap.alarm_flags,
                    (asnap.co2_ppm >> 8) & 0xFF,
                    asnap.co2_ppm & 0xFF,
                }
            };
            twai_transmit(&m, 0);
            s_can_alarm.pending = false;
        }
    }
}

// ---------------------------------------------------------------------------
// Alarm edge detection
// ---------------------------------------------------------------------------

static void check_alarm(uint8_t sensor_index, uint16_t co2_ppm)
{
    sensor_state_t *s = &s_sensors[sensor_index];
    bool now_alarm = (co2_ppm >= CO2_ALARM_PPM);

    if (now_alarm != s->alarm_active) {
        s->alarm_active = now_alarm;

        // Wait for any previous alarm frame to be consumed
        // (in practice this is near-instant since the CAN task runs at high priority)
        while (s_can_alarm.pending) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        s_can_alarm.sensor_index = sensor_index;
        s_can_alarm.alarm_flags  = now_alarm ? 0x01 : 0x00;
        s_can_alarm.co2_ppm     = co2_ppm;
        s_can_alarm.pending     = true;

        ESP_LOGW(TAG, "SCD41[%d] CO2 alarm %s: %u ppm",
                 sensor_index,
                 now_alarm ? "ACTIVE" : "CLEARED",
                 co2_ppm);
    }
}

// ---------------------------------------------------------------------------
// Main application — staggered single-shot pipeline
// ---------------------------------------------------------------------------

void app_main(void)
{
    ESP_LOGI(TAG, "=== TrailCurrent Australis ===");

    // NVS, hostname, WiFi credentials
    ESP_ERROR_CHECK(wifi_config_init());
    char ssid[33] = {0}, password[64] = {0};
    if (wifi_config_load(ssid, sizeof(ssid), password, sizeof(password))) {
        ESP_LOGI(TAG, "WiFi credentials loaded from NVS");
    } else {
        ESP_LOGI(TAG, "No WiFi credentials — OTA disabled until provisioned via CAN");
    }

    discovery_init();
    ota_init();
    ESP_LOGI(TAG, "Hostname: %s", wifi_config_get_hostname());

    // CAN bus — own task on core 1
    ESP_ERROR_CHECK(can_common_init(CAN_TX_PIN, CAN_RX_PIN));
    xTaskCreatePinnedToCore(twai_task, "twai", 4096, NULL, 5, NULL, 1);

    // Sensors — init I²C + probe mux channels
    if (sensors_init(I2C_SDA_PIN, I2C_SCL_PIN) != ESP_OK) {
        ESP_LOGE(TAG, "Sensor init failed — halting sensor loop");
        while (1) vTaskDelay(pdMS_TO_TICKS(10000));
    }

    s_sensor_count = sensors_get_count();
    if (s_sensor_count == 0) {
        ESP_LOGE(TAG, "No SCD41 sensors found — halting sensor loop");
        while (1) vTaskDelay(pdMS_TO_TICKS(10000));
    }

    // Build dense index map
    uint8_t slot = 0;
    for (uint8_t ch = 0; ch < MAX_SCD41_SENSORS; ch++) {
        // We check by trying to read — sensors_get_count already validated
        // Use the internal present array via a trigger attempt
        if (scd41_trigger_single_shot(ch) == ESP_OK) {
            s_index_map[slot++] = ch;
            // Cancel the trigger — we'll re-trigger in the stagger sequence
            // (The SCD41 measurement is already in progress but that's fine,
            //  we just won't read it and will re-trigger properly below)
        }
    }
    // Correct: slot should equal s_sensor_count. If not, adjust.
    s_sensor_count = slot;

    ESP_LOGI(TAG, "%d SCD41 sensor(s) active, stagger interval = %d ms",
             s_sensor_count, SCD41_MEASUREMENT_TIME_MS / s_sensor_count);

    // Initialize per-sensor state
    memset(s_sensors, 0, sizeof(s_sensors));

    // -----------------------------------------------------------------------
    // Staggered single-shot pipeline
    //
    // With N sensors and a 5000ms measurement time:
    //   stagger_ms = 5000 / N
    //   Each sensor is triggered every 5000ms
    //   One reading becomes available every stagger_ms
    //
    // Example (N=4, stagger=1250ms):
    //   t=0       trigger S0
    //   t=1250    trigger S1
    //   t=2500    trigger S2
    //   t=3750    trigger S3
    //   t=5000    read S0 → TX, re-trigger S0
    //   t=6250    read S1 → TX, re-trigger S1
    //   ...
    // -----------------------------------------------------------------------

    const uint32_t stagger_ms = SCD41_MEASUREMENT_TIME_MS / s_sensor_count;

    // Initial staggered triggers
    for (uint8_t i = 0; i < s_sensor_count; i++) {
        uint8_t ch = s_index_map[i];
        scd41_trigger_single_shot(ch);
        s_sensors[ch].trigger_time_us = esp_timer_get_time();
        s_sensors[ch].triggered = true;
        ESP_LOGI(TAG, "Initial trigger: SCD41[%d] (slot %d/%d)", ch, i, s_sensor_count);

        if (i < s_sensor_count - 1) {
            vTaskDelay(pdMS_TO_TICKS(stagger_ms));
        }
    }

    ESP_LOGI(TAG, "=== Setup complete — entering sensor loop ===");

    // Steady-state loop: cycle through sensors round-robin
    uint8_t current_slot = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(stagger_ms));

        uint8_t ch = s_index_map[current_slot];
        sensor_state_t *ss = &s_sensors[ch];

        // Check if enough time has elapsed since trigger
        int64_t elapsed_ms = (esp_timer_get_time() - ss->trigger_time_us) / 1000;

        if (ss->triggered && elapsed_ms >= SCD41_MEASUREMENT_TIME_MS) {
            // Read the result
            scd41_data_t data = scd41_read(ch);

            if (data.valid) {
                ss->last_reading = data;

                float tF = data.temperature_c * 9.0f / 5.0f + 32.0f;
                ESP_LOGI(TAG, "SCD41[%d]: %.1f°C / %.1f°F  RH %.1f%%  CO2 %u ppm",
                         ch, data.temperature_c, tF, data.humidity, data.co2_ppm);

                // Queue for CAN TX
                // Wait for previous frame to be consumed (should be instant)
                while (s_can_env.pending) {
                    vTaskDelay(pdMS_TO_TICKS(1));
                }

                s_can_env.sensor_index    = ch;
                s_can_env.temp_c          = (int8_t)(data.temperature_c + 0.5f);
                s_can_env.temp_f          = (int8_t)(tF + 0.5f);
                s_can_env.humidity_scaled  = (uint16_t)(data.humidity * 100.0f);
                s_can_env.co2_ppm         = data.co2_ppm;
                s_can_env.sensor_count    = s_sensor_count;
                s_can_env.pending         = true;

                // Alarm edge detection
                check_alarm(ch, data.co2_ppm);
            } else {
                ESP_LOGW(TAG, "SCD41[%d]: read returned invalid (not ready?)", ch);
            }

            // Re-trigger for next cycle
            scd41_trigger_single_shot(ch);
            ss->trigger_time_us = esp_timer_get_time();
            ss->triggered = true;
        }

        // Advance to next sensor
        current_slot = (current_slot + 1) % s_sensor_count;
    }
}
