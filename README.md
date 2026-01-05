# ESP32 Fall Detection

ESP32-based fall detection system that reads IMU data, detects impact/inactivity events, and triggers HTTP webhook alerts. Built for portfolio-ready embedded work with Wi-Fi captive portal onboarding and secure configuration.

## Features
- Captive portal (soft AP) for provisioning Wi-Fi credentials
- EEPROM storage for SSID/password after setup
- IMU-based fall detection with free-fall, impact, and orientation change analysis
- Webhook alerts for fall, reset, and telemetry collection events (IFTTT-compatible)
- Secrets managed via `include/config.h` (ignored by Git) with debug masking of credentials

## Hardware
- ESP32 development board (ESP32-DevKitC or equivalent)
- MPU6050 IMU (accelerometer + gyroscope)
- RGB LED + button for status and user interactions

## How It Works
1. Samples MPU6050 accelerometer/gyroscope data continuously.
2. Detects free-fall, impact, and angular change over a configurable window.
3. Uses button for clearing fall state, deep sleep, or entering Wi-Fi portal.
4. Publishes fall/reset/collection events via HTTP webhook after a confirmed fall.

## Setup & Flash
### Arduino IDE
1. Copy `include/config.example.h` to `include/config.h` and populate fields.
2. Open `ESP32_Fall_Detection.ino` in Arduino IDE.
3. Select your ESP32 board (e.g., ESP32 Dev Module) and the correct port.
4. Install required libraries (MPU6050, WiFi, HTTPClient, WebServer, EEPROM).
5. Upload sketch to the board.

### PlatformIO
PlatformIO configuration is not yet provided. You can adapt `src/main.ino` under a PlatformIO project (`platformio.ini`) targeting `esp32dev` when the build system is added.

## Configuration
1. Copy `include/config.example.h` → `include/config.h`.
2. Set `ALERT_WEBHOOK_KEY`, webhook events, and other options.
3. `include/config.h` is ignored by Git and should never be committed.
4. Debug logs mask the webhook key when printing URLs.

## Calibration Tips
Adjust the constants in `src/main.ino`:
- `freeFallThreshold`
- `impactThreshold`
- `gyroThreshold`
- `fallTime`

Lower these values for more sensitivity or raise them to reduce false positives. Consider adding additional debounce or cooldown in `checkForFall()` if necessary.

## Example Serial Output
```
MPU6050 connection successful
Missing include/config.h - HTTP alerts disabled. Copy include/config.example.h to include/config.h.
Webhook URL: http://maker.ifttt.com/trigger/Fall_detect/with/key/<KEY>
Fall detected, sending notification...
Reset notification triggered
```

## Project Structure
- `ESP32_Fall_Detection.ino` – wrapper for Arduino IDE
- `src/main.ino` – primary implementation
- `include/config.example.h` – template for private configuration
- `include/` – reserved for headers (config template)
- `docs/` – documentation assets (placeholder)
- `assets/` – photos/screenshots (placeholder)
