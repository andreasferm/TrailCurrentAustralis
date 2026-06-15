# TrailCurrentAustralis

ESP32-S3 environmental sensor (1-8 sensors). Using SCD41 to measure CO2, temp and humidity. As I²C address is fixed on these devices all sensors will need to be attached to an I²C multiplexer and each sensor will be read sequentially, it also unfortunately implies that the sensors cannot all be connected to the same cabling, but need its own, one per sensor. Australis will broadcast these measurements over CAN bus. Will hopefully support over-the-air updates, WiFi credential provisioning via CAN and network discovery for integration with TrailCurrent Headwaters.

## Hardware

- **MCU board:** [Waveshare ESP32-S3-RS485-CAN](https://www.waveshare.com/esp32-s3-rs485-can.htm)
  - ESP32-S3R8 (8MB PSRAM, external 16MB flash)
  - Built-in isolated CAN transceiver (TJA1051) on GPIO15/16
  - Built-in isolated RS485 transceiver (unused by Australis)
  - External PCF85063AT RTC
  - 7-36V DC input or 5V USB-C
- **CO2 / temp / humidity:** DFRobot SEN0536 (Sensirion SCD41, photoacoustic NDIR)
- **TCA9548A I²C Multiplexer
- **Will probably use GX16 4 pin connecters to hook up the sensors (as opposed to the DT connector used for power and CAN)

### Pin Assignments

| GPIO | Function |
|------|----------|
| 5 | I2C SDA (TCA9548A) |
| 6 | I2C SCL (TCA9548A) |
| 15 | CAN TX (internal transceiver) |
| 16 | CAN RX (internal transceiver) |

## Building

This project uses [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/get-started/index.html) (v5.x).

```bash
# Configure (first time only)
idf.py set-target esp32s3

# Build
idf.py build

# Flash via USB
idf.py flash

# Monitor serial output
idf.py monitor

# Build, flash, and monitor in one step
idf.py build flash monitor
```

### OTA Upload

```bash
curl -X POST http://<hostname>.local/ota --data-binary @build/australis.bin
```

## Architecture

| File | Purpose |
|------|---------|
| `main/main.c` | Sensor reading loop, TWAI task, CAN frame transmission |
| `main/sensors.c/h` | I²C driver for SCD41, and TCA9548A |
| `main/wifi_config.c/h` | NVS-backed WiFi credentials and STA connect/disconnect |
| `main/ota.c/h` | HTTP POST `/ota` server, runs in dedicated task |
| `main/discovery.c/h` | mDNS-based network discovery for Headwaters registration |

### CAN Bus Task

The TWAI (CAN) driver runs in a dedicated FreeRTOS task with alert-based message handling. It uses a dual-state transmission model:

- **TX_ACTIVE** (1000 ms period): Normal operation when peers are detected on the bus
- **TX_PROBING** (2000 ms period): Slow probe when no peers are ACKing, reduces bus noise

The task automatically transitions between states based on TX success/failure and incoming messages. Bus-off recovery is handled via TWAI alerts. This pattern is shared with all other TrailCurrent ESP-IDF modules — see `TrailCurrentReservoir`, `TrailCurrentSwitchback`, etc.

## CAN Bus Protocol

All communication uses a 500 kbps CAN bus. The device transmits sensor data and receives OTA / WiFi / discovery commands.

### Environmental Data (TX)

**CAN ID:** `0x30` | **DLC:** 8 | **Period:** 1000 ms

| Byte | Content | Type | Description |
|------|---------|------|-------------
| 0 | SensorIndex | uint8 | 0-7, which SCD41 this reading is for (°C, signed int8, rounded) |
| 1 | TemperatureCelsius | int8 |  whole °C |
| 2 | TemperatureFahrenheit | int8 |  whole °F |
| 3-4 | HumidityScaled | uint16 BE | RH*100, eg 5523 = 55.23% |
| 5-6 | Real CO2 in ppm | uint16 BE | NDIR measurement from SCD41, 400-5000ppm |
| 7 | SensorCount | uint8 | total sensors detected at boot (1-8) |

### Alarm data (TX)

**CAN ID:** `0x31` | **DLC:** 4 

| Byte | Content | Type | Description |
|------|---------|------|-------------
| 0 | SensorIndex | uint8 | Which sensor triggered/cleared |
| 1 | AlarmFlags | uint8 | CO2 > 2000ppm 0x01 = active, 0x00 = cleared |
| 2-3 | CO2Ppm | uint16 BE | The CO2 value at the transition |

### OTA Trigger (RX)

**CAN ID:** `0x00` | **DLC:** 3+

| Byte | Content |
|------|---------|
| 0-2 | Target MAC suffix (e.g., `F2 7E 6C` for hostname `esp32-F27E6C`) |

When the MAC matches, the device connects to its configured WiFi network, starts an HTTP server at `/ota`, and waits for a firmware upload for 3 minutes before returning to normal operation.

### WiFi Configuration (RX)

**CAN ID:** `0x01` | **DLC:** varies

WiFi credentials are provisioned over CAN using a chunked protocol. Credentials are stored in NVS (non-volatile storage) and persist across reboots.

| Byte 0 | Message Type |
|--------|-------------|
| `0x01` | Start: contains SSID length, password length, chunk counts |
| `0x02` | SSID chunk: 6-byte payload with chunk index |
| `0x03` | Password chunk: 6-byte payload with chunk index |
| `0x04` | End: contains XOR checksum for validation |

The protocol includes a 5-second timeout — if chunks stop arriving, the state resets.

### Discovery Trigger (RX)

**CAN ID:** `0x02` | **DLC:** any (broadcast)

When received, the device connects to WiFi and advertises itself via mDNS (`_trailcurrent._tcp`) with TXT records:

| Key | Value |
|-----|-------|
| `type` | `australis` |
| `canid` | `0x30` |
| `fw` | firmware version |

Headwaters confirms registration by calling `GET /discovery/confirm`. The discovery window is 3 minutes.

### Firmware Version Report (TX)

**CAN ID:** `0x04` | **DLC:** 6

Sent once on boot after CAN initialization, then again whenever a peer is first detected. Reports the running firmware version so Headwaters can track what each device is running.

| Byte | Content |
|------|---------|
| 0-2 | Last 3 bytes of device WiFi MAC (matches hostname `esp32-XXYYZZ`) |
| 3 | Version major |
| 4 | Version minor |
| 5 | Version patch |

## Air Quality and Safety Thresholds

### Real CO2 (SCD41, NDIR)
| ppm | Level |
|-----|-------|
| < 800 | Fresh / well-ventilated |
| 800-1499 | Normal indoor |
| 1500-2499 | Stuffy / drowsiness possible |
| ≥ 2000 | Alarm |


## OTA Updates

1. Provision WiFi credentials via CAN (one-time setup, stored in NVS)
2. Send an OTA trigger message on CAN ID `0x00` with the target device's MAC suffix
3. The device connects to WiFi
4. Upload firmware via HTTP: `curl -X POST http://<hostname>.local/ota --data-binary @build/australis.bin`
5. On success, the device reboots with new firmware
6. On timeout (3 minutes), the device disconnects WiFi and resumes normal operation

The device hostname is printed to serial at boot (format: `esp32-XXYYZZ`).

## License

[MIT](LICENSE)
