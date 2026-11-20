# Laundry Notify — ESP32-S3 TinyML Firmware

Edge AI firmware for an IoT washing machine vibration monitor built on a custom **ESP32-S3** PCB, running an **Edge Impulse** TinyML classifier with **MPU6050 Wake-on-Motion (WoM)** deep sleep and **ThingsBoard Cloud** MQTT telemetry.

## Related Links

- **Web Application (PWA) Deployment:** `https://laundry-notify-iot.netlify.app` *(replace with your Netlify URL)*
- **Web Application Repository:** `https://github.com/balm4869/laundry-notify-app` *(replace with your App Repo URL)*

---

## Key Features

- **On-Device TinyML Classification:** Runs Edge Impulse 3-axis accelerometer inference (50 Hz, 2-second window) with a 5-window majority voting filter to classify `idle`, `washing`, and `spinning`.
- **Ultra-Low-Power Wake-on-Motion (WoM):** Configures the MPU6050 in 5 Hz Low-Power Accelerometer-Only Cycle Mode and sleeps the ESP32-S3 until physical washer vibration (`EXT1`) or user button press (`EXT0`) occurs.
- **Soak & Refill Pause Filtering:** Prevents false completion alerts during water-refill or soaking pauses with configurable timeout presets (`4m / 6m / 2m / 1m TEST`) persisted in RTC memory across Deep Sleep.
- **3-Page OLED UI & 4 Hardware Buttons:** Displays live wash state, RMS vibration (`m/s²`), AI confidence, Wi-Fi RSSI, and quick controls directly on a 1.3" SH1106 I2C OLED.
- **Real-Time ThingsBoard Cloud Sync:** Publishes structured JSON telemetry via MQTT (`v1/devices/me/telemetry`) for live monitoring and 7-day cloud wash history.

---

## Hardware Pinout (ESP32-S3)

| Component | Pin / Interface | GPIO | Notes |
| :--- | :--- | :--- | :--- |
| **MPU6050 & SH1106 OLED** | I2C SDA | `GPIO 16` | Shared I2C bus (100 kHz) |
| **MPU6050 & SH1106 OLED** | I2C SCL | `GPIO 15` | OLED Address: `0x3C`, MPU6050: `0x68/0x69` |
| **MPU6050 Interrupt** | INT (WoM) | `GPIO 17` | `EXT1` Active HIGH wakeup from Deep Sleep |
| **BTN3 (Page / Wake)** | Digital Input | `GPIO 4` | `EXT0` Active LOW wakeup / Switch 3 OLED pages |
| **BTN4 (Action)** | Digital Input | `GPIO 5` | Cycle soak-wait preset or confirm unload on `DONE` |
| **BTN5 (Sync / Reset)** | Digital Input | `GPIO 6` | Short: Force Cloud Sync / Hold 2s: Reset to `IDLE` |
| **BTN2 (Mode / Sleep)** | Digital Input | `GPIO 0` | Short: Toggle `[HOLD]` awake / Hold 2s: Sleep now |

---

## Telemetry Payload (`v1/devices/me/telemetry`)

| Key | Type | Description |
| :--- | :--- | :--- |
| `machine_state` | String | Normalized state: `IDLE`, `WASHING`, `SOAK_WAIT`, `SPINNING`, `DONE` |
| `vibration_rms` | Float | Dynamic 3-axis acceleration RMS (`m/s²`) |
| `confidence` | Float | Edge Impulse top-class confidence (`0.0` – `100.0%`) |
| `wash_elapsed_sec` | Integer | Elapsed cycle duration in seconds |
| `soak_remaining_sec` | Integer | Countdown remaining during `SOAK_WAIT` in seconds |
| `wifi_rssi` | Integer | Wi-Fi signal strength (`dBm`) |

---

## Getting Started (PlatformIO)

1. Clone this repository and open it in **VS Code** with the **PlatformIO IDE** extension.
2. Duplicate `include/secrets.example.h` and rename the copy to `include/secrets.h`.
3. Fill in your Wi-Fi credentials and ThingsBoard Device Access Token inside `include/secrets.h`.
4. Build and upload to your ESP32-S3 board:
   ```bash
   pio run --target upload
   pio device monitor -b 115200