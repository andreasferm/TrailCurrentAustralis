#include "sensors.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "sensors";

// ---------------------------------------------------------------------------
// I²C bus and device handles
// ---------------------------------------------------------------------------

static i2c_master_bus_handle_t s_i2c_bus = NULL;
static i2c_master_dev_handle_t s_mux_dev = NULL;
static i2c_master_dev_handle_t s_scd41_dev = NULL;  // single handle, mux selects the physical sensor

static bool     s_sensor_present[MAX_SCD41_SENSORS];
static uint8_t  s_sensor_count = 0;
static int8_t   s_current_channel = -1;  // currently selected mux channel

// ---------------------------------------------------------------------------
// CRC-8 (polynomial 0x31, init 0xFF) — Sensirion standard
// ---------------------------------------------------------------------------

static uint8_t sensirion_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
        }
    }
    return crc;
}

static esp_err_t sensirion_send_cmd(uint16_t cmd)
{
    uint8_t buf[2] = { (cmd >> 8) & 0xFF, cmd & 0xFF };
    return i2c_master_transmit(s_scd41_dev, buf, 2, 1000);
}

static esp_err_t sensirion_send_cmd_with_arg(uint16_t cmd, uint16_t arg)
{
    uint8_t buf[5];
    buf[0] = (cmd >> 8) & 0xFF;
    buf[1] = cmd & 0xFF;
    buf[2] = (arg >> 8) & 0xFF;
    buf[3] = arg & 0xFF;
    buf[4] = sensirion_crc8(&buf[2], 2);
    return i2c_master_transmit(s_scd41_dev, buf, sizeof(buf), 1000);
}

// ---------------------------------------------------------------------------
// TCA9548A multiplexer control
// ---------------------------------------------------------------------------

static esp_err_t mux_select_channel(uint8_t channel)
{
    if (channel >= MAX_SCD41_SENSORS) return ESP_ERR_INVALID_ARG;
    if (s_current_channel == (int8_t)channel) return ESP_OK;  // already selected

    uint8_t mask = 1 << channel;
    esp_err_t ret = i2c_master_transmit(s_mux_dev, &mask, 1, 1000);
    if (ret == ESP_OK) {
        s_current_channel = (int8_t)channel;
    } else {
        s_current_channel = -1;  // unknown state
        ESP_LOGE(TAG, "TCA9548A channel %d select failed: %s", channel, esp_err_to_name(ret));
    }
    return ret;
}

static esp_err_t mux_disable_all(void)
{
    uint8_t mask = 0x00;
    s_current_channel = -1;
    return i2c_master_transmit(s_mux_dev, &mask, 1, 1000);
}

// ---------------------------------------------------------------------------
// SCD41 commands
// ---------------------------------------------------------------------------

#define SCD41_CMD_STOP_PERIODIC      0x3F86
#define SCD41_CMD_SINGLE_SHOT        0x219D
#define SCD41_CMD_READ_MEAS          0xEC05
#define SCD41_CMD_GET_DATA_READY     0xE4B8
#define SCD41_CMD_SET_PRESSURE       0xE000
#define SCD41_CMD_GET_SERIAL         0x3682
#define SCD41_CMD_REINIT             0x3646

static bool scd41_data_ready(void)
{
    if (sensirion_send_cmd(SCD41_CMD_GET_DATA_READY) != ESP_OK) return false;
    vTaskDelay(pdMS_TO_TICKS(2));
    uint8_t resp[3];
    if (i2c_master_receive(s_scd41_dev, resp, 3, 1000) != ESP_OK) return false;
    if (sensirion_crc8(resp, 2) != resp[2]) return false;
    uint16_t status = ((uint16_t)resp[0] << 8) | resp[1];
    return (status & 0x07FF) != 0;
}

/**
 * Probe for an SCD41 on the currently selected mux channel.
 * Sends stop_periodic (safe even if not running), waits, then reads serial number.
 */
static bool scd41_probe(void)
{
    // Stop any in-progress periodic measurement (benign if not running)
    sensirion_send_cmd(SCD41_CMD_STOP_PERIODIC);
    vTaskDelay(pdMS_TO_TICKS(500));  // stop command needs 500ms

    // Try to read serial number — 3 words × (2 data + 1 CRC) = 9 bytes
    if (sensirion_send_cmd(SCD41_CMD_GET_SERIAL) != ESP_OK) return false;
    vTaskDelay(pdMS_TO_TICKS(1));

    uint8_t buf[9];
    if (i2c_master_receive(s_scd41_dev, buf, sizeof(buf), 1000) != ESP_OK) return false;

    // Validate all three CRC bytes
    for (int i = 0; i < 3; i++) {
        if (sensirion_crc8(&buf[i * 3], 2) != buf[i * 3 + 2]) return false;
    }

    uint16_t w0 = ((uint16_t)buf[0] << 8) | buf[1];
    uint16_t w1 = ((uint16_t)buf[3] << 8) | buf[4];
    uint16_t w2 = ((uint16_t)buf[6] << 8) | buf[7];
    ESP_LOGI(TAG, "  SCD41 serial: %04X-%04X-%04X", w0, w1, w2);
    return true;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

esp_err_t sensors_init(int sda_pin, int scl_pin)
{
    // Initialize I²C bus
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I²C bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Add TCA9548A multiplexer device
    i2c_device_config_t mux_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TCA9548A_ADDR,
        .scl_speed_hz = 100000,
    };
    ret = i2c_master_bus_add_device(s_i2c_bus, &mux_cfg, &s_mux_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TCA9548A attach failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "TCA9548A attached (0x%02X)", TCA9548A_ADDR);

    // Add SCD41 device — single handle used for all channels (mux switches the physical sensor)
    i2c_device_config_t scd41_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SCD41_ADDR,
        .scl_speed_hz = 100000,
    };
    ret = i2c_master_bus_add_device(s_i2c_bus, &scd41_cfg, &s_scd41_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SCD41 device handle creation failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Probe each mux channel for an SCD41
    ESP_LOGI(TAG, "Probing TCA9548A channels for SCD41 sensors...");
    s_sensor_count = 0;
    memset(s_sensor_present, false, sizeof(s_sensor_present));

    for (uint8_t ch = 0; ch < MAX_SCD41_SENSORS; ch++) {
        if (mux_select_channel(ch) != ESP_OK) continue;

        ESP_LOGI(TAG, "Channel %d:", ch);
        if (scd41_probe()) {
            s_sensor_present[ch] = true;
            s_sensor_count++;
            ESP_LOGI(TAG, "  → SCD41 detected");
        } else {
            ESP_LOGI(TAG, "  → no SCD41");
        }
    }

    mux_disable_all();
    ESP_LOGI(TAG, "Sensor discovery complete: %d SCD41(s) found", s_sensor_count);

    if (s_sensor_count == 0) {
        ESP_LOGW(TAG, "No SCD41 sensors detected — check wiring and TCA9548A");
    }

    return ESP_OK;
}

uint8_t sensors_get_count(void)
{
    return s_sensor_count;
}

esp_err_t scd41_trigger_single_shot(uint8_t sensor_index)
{
    if (sensor_index >= MAX_SCD41_SENSORS || !s_sensor_present[sensor_index]) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = mux_select_channel(sensor_index);
    if (ret != ESP_OK) return ret;

    ret = sensirion_send_cmd(SCD41_CMD_SINGLE_SHOT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SCD41[%d] single-shot trigger failed: %s",
                 sensor_index, esp_err_to_name(ret));
    }
    return ret;
}

scd41_data_t scd41_read(uint8_t sensor_index)
{
    scd41_data_t out = { .valid = false };
    if (sensor_index >= MAX_SCD41_SENSORS || !s_sensor_present[sensor_index]) {
        return out;
    }

    if (mux_select_channel(sensor_index) != ESP_OK) return out;
    if (!scd41_data_ready()) return out;

    if (sensirion_send_cmd(SCD41_CMD_READ_MEAS) != ESP_OK) return out;
    vTaskDelay(pdMS_TO_TICKS(2));

    uint8_t buf[9];
    if (i2c_master_receive(s_scd41_dev, buf, sizeof(buf), 1000) != ESP_OK) return out;

    if (sensirion_crc8(&buf[0], 2) != buf[2] ||
        sensirion_crc8(&buf[3], 2) != buf[5] ||
        sensirion_crc8(&buf[6], 2) != buf[8]) {
        ESP_LOGW(TAG, "SCD41[%d] CRC error", sensor_index);
        return out;
    }

    uint16_t raw_co2 = ((uint16_t)buf[0] << 8) | buf[1];
    uint16_t raw_t   = ((uint16_t)buf[3] << 8) | buf[4];
    uint16_t raw_rh  = ((uint16_t)buf[6] << 8) | buf[7];

    out.co2_ppm       = raw_co2;
    out.temperature_c = -45.0f + 175.0f * ((float)raw_t / 65535.0f);
    out.humidity      = 100.0f * ((float)raw_rh / 65535.0f);
    out.valid         = true;
    return out;
}

esp_err_t scd41_set_ambient_pressure(uint8_t sensor_index, uint16_t pressure_hpa)
{
    if (sensor_index >= MAX_SCD41_SENSORS || !s_sensor_present[sensor_index]) {
        return ESP_ERR_INVALID_ARG;
    }
    if (mux_select_channel(sensor_index) != ESP_OK) return ESP_FAIL;
    return sensirion_send_cmd_with_arg(SCD41_CMD_SET_PRESSURE, pressure_hpa);
}