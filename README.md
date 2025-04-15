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

### Debug Levels (src/nodoCantina.ino)

The system includes a global debug level and component-specific debug levels to control the verbosity of debug messages. This allows for precise control over which types of debug messages are displayed.

#### Global Debug Level:
```cpp
#define DEBUG 1 // 0 (Silent) to 4 (Verbose)
```

#### Component-Specific Debug Levels:
```cpp
#define DEBUGLevel_Mqtt_Generic 3
#define DEBUGLevel_Mqtt_Error 0
#define DEBUGLevel_Mqtt_FlowMeter 3
#define DEBUGLevel_Mqtt_PressureSensor 3
#define DEBUGLevel_Mqtt_WaterLevel 3
#define DEBUGLevel_Mqtt_FloodSensor 3
#define DEBUGLevel_Mqtt_StateMachine 3
```

#### Explanation:
- The `DEBUG` constant sets the overall verbosity level for the system.
- Component-specific constants (e.g., `DEBUGLevel_Mqtt_Generic`) determine the minimum priority of messages for each grouping (e.g., MQTT topics, flow meter, pressure sensor).
- Messages with a priority below the component's debug level will be displayed.

#### Example Usage:
- To log messages for a specific component:
  ```cpp
  dsPrintln("MQTT connection error!", DEBUGLevel_Mqtt_Error);
  dsPrint("Flow meter data published", DEBUGLevel_Mqtt_FlowMeter);
  ```

By adjusting these constants, you can focus on the debug information most relevant to your needs while silencing less critical messages.


### Sensor Parameters
| Parameter               | Default | Range       |
|-------------------------|---------|-------------|
| Max Pump Runtime        | 10 min  | 1-inf min   |
| Switch-On Pressure      | 1.5 Bar | 0-inf Bar   |
| Switch-Off Pressure     | 3.0 Bar | 0-inf Bar   |
| Flood Detection Threshold | 800   | 0-4095      |

### MQTT Integration

The system uses MQTT for communication, allowing real-time data publishing and parameter configuration. The topics are organized to manage different sensors, the pump state machine, and general MQTT status.

#### MQTT Topics

**Published Topics**
| Topic                                      | Format  | Description                          |
|-------------------------------------------|---------|--------------------------------------|
| `PumpController/flow_meter/Q`             | `float` | Flow rate (L/min)                   |
| `PumpController/flow_meter/V`             | `float` | Total volume flowed (L)             |
| `PumpController/flow_meter/conversionCoefficient` | `float` | Conversion coefficient for flow sensor |
| `PumpController/flow_meter/read_period`   | `int`   | Sensor read interval (ms)           |
| `PumpController/pressure_sensor/pressure` | `float` | Pressure (Bar)                      |
| `PumpController/pressure_sensor/conversionCoefficient` | `float` | Conversion coefficient for pressure sensor |
| `PumpController/pressure_sensor/read_period` | `int`  | Sensor read interval (ms)           |
| `Cisterna/livello_cisterna/distance`      | `float` | Water level distance (cm)           |
| `Cisterna/livello_cisterna/read_period`   | `int`   | Sensor read interval (ms)           |
| `Cisterna/livello_cisterna/read_repetitions` | `int` | Number of repetitions for averaging |
| `PumpController/flood_detector/analog_read` | `int`  | Analog reading from flood detector  |
| `PumpController/flood_detector/treshold`  | `int`   | Flood detection threshold           |
| `PumpController/flood_detector/read_period` | `int`  | Sensor read interval (ms)           |
| `PumpController/pump_state_machine/current_state` | `int` | Current pump state                  |
| `PumpController/pump_state_machine/pump_on` | `int`   | Pump activation (1 = ON, 0 = OFF)   |
| `PumpController/pump_state_machine/safety_block` | `int` | Safety block (1 = Active, 0 = Inactive) |
| `PumpController/pump_state_machine/max_run_time` | `float` | Maximum runtime (ms)                |
| `PumpController/pump_state_machine/switch_on_pressure` | `float` | Pressure to switch pump ON (Bar)    |
| `PumpController/pump_state_machine/switch_off_pressure` | `float` | Pressure to switch pump OFF (Bar)   |
| `PumpController/mqttState`                | `int`   | MQTT watchdog state                 |

**Subscribed Topics**
| Topic                                      | Format  | Description                          |
|-------------------------------------------|---------|--------------------------------------|
| `PumpController/flow_meter/read_period/mqtt_input` | `int` | Update flow meter read period       |
| `PumpController/flow_meter/V/mqtt_input`  | `float` | Update total flowed volume          |
| `PumpController/flow_meter/conversionCoefficient/mqtt_input` | `float` | Update flow meter conversion coefficient |
| `PumpController/pressure_sensor/read_period/mqtt_input` | `int` | Update pressure sensor read period  |
| `PumpController/pressure_sensor/conversionCoefficient/mqtt_input` | `float` | Update pressure sensor conversion coefficient |
| `Cisterna/livello_cisterna/read_period/mqtt_input` | `int` | Update water level read period      |
| `Cisterna/livello_cisterna/read_repetitions/mqtt_input` | `int` | Update number of averaging repetitions |
| `PumpController/flood_detector/read_period/mqtt_input` | `int` | Update flood detector read period   |
| `PumpController/flood_detector/treshold/mqtt_input` | `int` | Update flood detection threshold    |
| `PumpController/pump_state_machine/pump_on/mqtt_input` | `int` | Manually turn the pump ON or OFF    |
| `PumpController/pump_state_machine/safety_block/mqtt_input` | `int` | Activate or deactivate safety block |
| `PumpController/pump_state_machine/max_run_time/mqtt_input` | `int` | Update maximum runtime              |
| `PumpController/pump_state_machine/switch_on_pressure/mqtt_input` | `float` | Update ON pressure threshold        |
| `PumpController/pump_state_machine/switch_off_pressure/mqtt_input` | `float` | Update OFF pressure threshold       |
| `PumpController/mqttState/mqtt_input`     | `int`   | Update MQTT watchdog state          |

#### Key Highlights:
- **Dynamic Configuration:** The subscribed topics allow runtime updates for sensor parameters, pump settings, and thresholds.
- **Debug Levels:** Each topic grouping is associated with specific debug levels, enabling fine-grained monitoring and troubleshooting.
- **Safety Features:** Topics such as `safety_block` and `max_run_time` ensure system safety by preventing overuse or failure scenarios.

#### Example Usage:
To update the flow meter's read period via MQTT:
1. Publish the desired value to `PumpController/flow_meter/read_period/mqtt_input`.
   ```bash
   mosquitto_pub -h <broker_ip> -t "PumpController/flow_meter/read_period/mqtt_input" -m "5000"
   ```
2. The system will verify and apply the new value if valid.

### State Machine

The pump controller uses a state machine to manage its operation based on sensor data, safety conditions, and MQTT commands. The state machine ensures safe and efficient control of the pump while allowing for remote configuration and monitoring.

#### States
- **`STATE_0`**: Pump Off (Idle)
- **`STATE_1`**: Pump Active
- **`STATE_2`**: Safety Lockout

#### Transitions
1. **Normal Operation**:
   - `STATE_0` → `STATE_1`: When the pump is turned on (`pump_on = 1`) and pressure drops below `switch_on_pressure`.
   - `STATE_1` → `STATE_0`: When pressure exceeds `switch_off_pressure` or the pump is turned off (`pump_on = 0`).
2. **Safety Conditions**:
   - Any State → `STATE_2`: Triggered when safety conditions are met (e.g., flood detection or exceeding `max_run_time`).
   - `STATE_2` → `STATE_0`: Safety block is cleared via MQTT (`safety_block = 0`).

#### Parameters
The following parameters are used to control the state machine:
| Parameter             | Description                          | Default Value |
|-----------------------|--------------------------------------|---------------|
| `pump_on`            | Manual control to turn the pump ON/OFF (1/0). | `0`           |
| `safety_block`       | Indicates if the safety block is active (1/0). | `0`           |
| `max_run_time`       | Maximum allowed runtime for the pump (ms). | `600,000` ms (10 min) |
| `switch_on_pressure` | Pressure threshold to switch the pump ON (Bar). | `1.5` Bar     |
| `switch_off_pressure`| Pressure threshold to switch the pump OFF (Bar). | `3.0` Bar     |

#### MQTT Commands
The state machine can be controlled and monitored via MQTT:
- **Subscribed Topics**:
  - `PumpController/pump_state_machine/pump_on/mqtt_input`: Set `pump_on` (1 = ON, 0 = OFF).
  - `PumpController/pump_state_machine/safety_block/mqtt_input`: Reset safety block (1 = Active, 0 = Cleared).
  - `PumpController/pump_state_machine/max_run_time/mqtt_input`: Update maximum runtime.
  - `PumpController/pump_state_machine/switch_on_pressure/mqtt_input`: Update the ON pressure threshold.
  - `PumpController/pump_state_machine/switch_off_pressure/mqtt_input`: Update the OFF pressure threshold.
- **Published Topics**:
  - `PumpController/pump_state_machine/current_state`: Current state of the pump.
  - `PumpController/pump_state_machine/pump_on`: Current `pump_on` state.
  - `PumpController/pump_state_machine/safety_block`: Current safety block status.
  - `PumpController/pump_state_machine/max_run_time`: Current maximum runtime configuration.
  - `PumpController/pump_state_machine/switch_on_pressure`: Current ON pressure threshold.
  - `PumpController/pump_state_machine/switch_off_pressure`: Current OFF pressure threshold.

### Troubleshooting

This section provides solutions to common issues encountered while using the HydroGuard ESP32 Pump Controller.

#### 1. **MQTT Connection Failures**
- **Problem**: The controller fails to connect to the MQTT broker.
  - **Solution**:
    1. Verify the MQTT broker IP, username, and password in `localWiFiConfig.h`.
    2. Check the network connection and ensure the MQTT broker is reachable from the ESP32.
    3. Use the debug logging system to identify issues:
       - Set the global debug level to at least `2` in `src/main.cpp`:
         ```cpp
         #define DEBUG 2
         ```
       - Monitor the serial output to identify errors during the connection process.

#### 2. **Sensor Reading Errors**
- **Problem**: Sensor data is incorrect or not being updated.
  - **Solution**:
    1. Check the wiring and connections for all sensors.
    2. Validate sensor parameters via MQTT topics (e.g., read period, conversion coefficients).
    3. Use verbose debug logging for detailed insights:
       ```cpp
       #define DEBUG 4
       ```
       Run the following command to monitor the serial output:
       ```bash
       pio device monitor
       ```

#### 3. **Pump Not Responding**
- **Problem**: The pump does not turn on or off as expected.
  - **Solution**:
    1. Check the state of the `safety_block` via the MQTT topic `PumpController/pump_state_machine/safety_block`.
    2. Verify the relay wiring to the ESP32's `GPIO16` pin.
    3. Ensure the pressure sensor's readings are within the configured thresholds (`switch_on_pressure` and `switch_off_pressure`).

#### 4. **Flood Detection Issues**
- **Problem**: Flood detection is triggering incorrectly or not triggering at all.
  - **Solution**:
    1. Verify the flood detector's wiring and ensure the power pin (`GPIO23`) is correctly connected.
    2. Adjust the flood detection threshold via the MQTT topic `PumpController/flood_detector/treshold/mqtt_input`.

#### 5. **Debugging Tips**
- Use the component-specific debug levels to isolate issues (e.g., `DEBUGLevel_Mqtt_Generic`, `DEBUGLevel_Mqtt_Error`).
- Example: To debug MQTT issues, increase the `DEBUGLevel_Mqtt_Generic` to `3` or higher in `src/main.cpp`:
  ```cpp
  #define DEBUGLevel_Mqtt_Generic 3
  ```

#### 6. **Watchdog Timer Resets**
- **Problem**: The ESP32 appears to reset unexpectedly.
  - **Solution**:
    1. Check the MQTT watchdog state (`PumpController/mqttState`).
    2. Ensure the MQTT task is running without delays exceeding the configured watchdog timeout (`WDT_TIMEOUT`).

## License
This project is licensed under **GNU GPLv3** - see [LICENSE](LICENSE) for details.

**Copyright Notice** (Required in all source files):
```cpp
/*
 * HydroGuard ESP32 Pump Controller
 * Copyright (C) 2025 Matteo Formigli
 * 
 * This program is free software: you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3. This program is distributed 
 * in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR 
 * A PARTICULAR PURPOSE. See the GNU General Public License for more 
 * details. You should have received a copy of the GNU General Public 
 * License along with this program. If not, see 
 * <https://www.gnu.org/licenses/gpl-3.0.html>.
 */
```

**Legal Disclaimer**: This software comes with NO WARRANTY. Use at your own risk with proper safety measures.
