# HydroGuard - ESP32 Pump Controller and sensor hub with MQTT Integration
![PlatformIO](https://img.shields.io/badge/PlatformIO-FFD700?style=flat&logo=platformio&logoColor=white)
![License](https://img.shields.io/badge/License-GPLv3-blue)
![ESP32](https://img.shields.io/badge/ESP32-ESP32-blue)
![MQTT](https://img.shields.io/badge/MQTT-3.1.1-brightgreen)

Advanced pump control system with multi-sensor integration and IoT capabilities using ESP32 and MQTT.

**Key Features**:
- Real-time sensor monitoring (Flow, Pressure, Water Level, Flood Detection)
- MQTT-based remote control and configuration
- Safety-focused state machine with watchdog timer
- FreeRTOS-based task management

## Table of Contents
- [Hardware Setup](#hardware-setup)
- [PlatformIO Setup](#platformio-setup)
- [Configuration](#configuration)
- [MQTT Integration](#mqtt-integration)
- [State Machine](#state-machine)
- [Troubleshooting](#troubleshooting)
- [License](#license)

## Hardware Setup
| Component                        | ESP32 Pin | Parameters                             |
|--------------------------------------|-----------|----------------------------------------|
| Relay                                | GPIO16    |                                        |
| Flow Meter (YF-G1 Flow sensor)       | GPIO14    | k*pulse/L (where k can be set via mqtt)|
| Pressure Sensor (YD6080)             | ADS1115   | {p_{pump_on}} and {p_{pump_off}}       |
| Ultrasonic Sensor (AJ-SR04M module)  | GPIO27/26 | height of the sensor                   |
| Flood Detector (water sensor)        | GPIO23/36 | Analog threshold: 800                  |

## PlatformIO Setup
1. Clone repository:
```bash
git clone https://github.com/your-username/esp32-pump-controller
cd esp32-pump-controller
```

2. Rename configuration file template ```localWiFiConfig_template.h``` to ```localWiFiConfig.h``` :
```cpp
// include/localWiFiConfig.h (rename include/localWiFiConfig_template.h)
#define MY_SSID "YOUR_WIFI_SSID"
#define MY_PASSWORD "YOUR_WIFI_PASSWORD"
#define MQTT_BROKER_IP "YOUR_MQTT_BROKER_IP"
#define MQTT_USER "Your_MQTT_User"
#define MQTT_PASSWORD "Your_MQTT_Password"
#define MQTT_ID "Your_MQTT_Client_ID"
```

3. Install dependencies (`platformio.ini`):
```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps = 
	knolleary/PubSubClient@^2.8
	adafruit/Adafruit ADS1X15@^2.5.0
monitor_speed = 115200
```

4. Build & Upload:
```bash
pio run --target upload && pio device monitor
```

## Configuration
### Debug Levels (src/main.cpp)
```cpp
#define DEBUG 4 // 0 (Silent) to 4 (Verbose)
```

### Sensor Parameters
| Parameter               | Default | Range       |
|-------------------------|---------|-------------|
| Max Pump Runtime        | 10 min  | 1-inf min   |
| Switch-On Pressure      | 1.5 Bar | 0-inf Bar   |
| Switch-Off Pressure     | 3.0 Bar | 0-inf Bar   |
| Flood Detection Threshold | 800   | 0-4095      |

## MQTT Integration
### Published Topics
| Topic                                      | Format | Description              |
|-------------------------------------------|--------|--------------------------|
| PumpController/flow_meter/Q               | float  | Flow rate (L/min)        |
| PumpController/pressure_sensor/pressure   | float  | Pressure (Bar)           |
| Cisterna/livello_cisterna/distance        | float  | Water level (cm)         |
| PumpController/pump_state_machine/state   | int    | Current pump state       |

### Subscribed Topics
| Topic                                      | Format | Description              |
|-------------------------------------------|--------|--------------------------|
| PumpController/pump_state_machine/pump_on | 0/1    | Manual pump control      |
| PumpController/+/mqtt_input               | varies | Parameter configuration  |

## State Machine
**States**:
- `STATE_0`: Pump Off (Idle)
- `STATE_1`: Pump Active
- `STATE_2`: Safety Lockout

**Transitions**:
1. Normal Operation:
   - OFF → ON when pressure < switch_on_pressure
   - ON → OFF when pressure > switch_off_pressure
2. Safety Conditions:
   - ANY → STATE_2 on flood detection
   - STATE 1 → STATE_2 after reaching max_run_time
   - STATE_2 → STATE_0 when user resets the safety block via mqtt

## Troubleshooting
**Common Issues**:
1. **MQTT Connection Failures**
   - Verify broker IP in `localWiFiConfig.h`
   - Monitor serial output (DEBUG >= 2)

2. **Sensor Reading Errors**
   ```bash
   # Enable verbose logging
   #define DEBUG 4
   pio device monitor
   ```

3. **Pump Not Responding**
   - Check safety_block status via MQTT
   - Verify relay wiring (GPIO16)
   - Monitor pressure sensor readings

## License
This project is licensed under **GNU GPLv3** - see [LICENSE](LICENSE) for details.

**Copyright Notice** (Required in all source files):
```cpp
/*
 * ESP32 Pump Controller
 * Copyright (C) 2025 Matteo Formigli
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation...
 */
```

**Legal Disclaimer**: This software comes with NO WARRANTY. Use at your own risk with proper safety measures.
