# Home Assistant Integration Guide for HydroGuard ESP32 Pump Controller

This guide will walk you through setting up your HydroGuard ESP32 Pump Controller with Home Assistant using MQTT.

---

## Prerequisites

Before starting, ensure you have:

- **Home Assistant** installed and running
- **MQTT Broker** configured in Home Assistant (Mosquitto add-on recommended)
- **HydroGuard ESP32 Pump Controller** flashed and connected to your Wi-Fi
- **MQTT credentials** for your broker (configured in `localWiFiConfig.h`)

---

## Step 1: Verify MQTT Communication

First, confirm that your HydroGuard is publishing data to your MQTT broker. You can use the **MQTT Explorer** add-on in Home Assistant or the **mosquitto_sub** command:

```bash
mosquitto_sub -h YOUR_BROKER_IP -u YOUR_USER -P YOUR_PASSWORD -t "PumpController/#" -v
```

You should see data flowing from topics like:
- `PumpController/flow_meter/Q`
- `PumpController/pressure_sensor/pressure`
- `Cisterna/livello_cisterna/distance`
- `PumpController/pump_state_machine/current_state`

---

## Step 2: Configure MQTT in Home Assistant

If you haven't already, add the MQTT integration in Home Assistant:

1. Go to **Settings → Devices & Services**
2. Click **Add Integration** → Search for **MQTT**
3. Enter your broker details (host, port, username, password)
4. Complete the setup

---

## Step 3: Add Sensors to Home Assistant

Add the following to your `configuration.yaml` file (or create a separate `hydroguard_sensors.yaml` and include it).

### Flow Meter Sensors

```yaml
sensor:
  - platform: mqtt
    name: "HydroGuard Flow Rate"
    state_topic: "PumpController/flow_meter/Q"
    unit_of_measurement: "L/min"
    device_class: "volume_flow_rate"
    state_class: "measurement"
    value_template: "{{ value | float }}"

  - platform: mqtt
    name: "HydroGuard Total Volume"
    state_topic: "PumpController/flow_meter/V"
    unit_of_measurement: "L"
    device_class: "volume"
    state_class: "total_increasing"
    value_template: "{{ value | float }}"

  - platform: mqtt
    name: "HydroGuard Flow Conversion Coefficient"
    state_topic: "PumpController/flow_meter/conversionCoefficient"
    entity_category: "diagnostic"
    value_template: "{{ value | float }}"

  - platform: mqtt
    name: "HydroGuard Flow Read Period"
    state_topic: "PumpController/flow_meter/read_period"
    unit_of_measurement: "ms"
    entity_category: "diagnostic"
    value_template: "{{ value | int }}"
```

### Pressure Sensor Sensors

```yaml
  - platform: mqtt
    name: "HydroGuard Pressure"
    state_topic: "PumpController/pressure_sensor/pressure"
    unit_of_measurement: "Bar"
    device_class: "pressure"
    state_class: "measurement"
    value_template: "{{ value | float }}"

  - platform: mqtt
    name: "HydroGuard Pressure Conversion Coefficient"
    state_topic: "PumpController/pressure_sensor/conversionCoefficient"
    entity_category: "diagnostic"
    value_template: "{{ value | float }}"

  - platform: mqtt
    name: "HydroGuard Pressure Read Period"
    state_topic: "PumpController/pressure_sensor/read_period"
    unit_of_measurement: "ms"
    entity_category: "diagnostic"
    value_template: "{{ value | int }}"
```

### Water Level Sensor

```yaml
  - platform: mqtt
    name: "HydroGuard Water Level Distance"
    state_topic: "Cisterna/livello_cisterna/distance"
    unit_of_measurement: "cm"
    device_class: "distance"
    state_class: "measurement"
    value_template: "{{ value | float }}"

  - platform: mqtt
    name: "HydroGuard Water Level Read Period"
    state_topic: "Cisterna/livello_cisterna/read_period"
    unit_of_measurement: "ms"
    entity_category: "diagnostic"
    value_template: "{{ value | int }}"

  - platform: mqtt
    name: "HydroGuard Water Level Read Repetitions"
    state_topic: "Cisterna/livello_cisterna/read_repetitions"
    entity_category: "diagnostic"
    value_template: "{{ value | int }}"
```

### Flood Detector

```yaml
  - platform: mqtt
    name: "HydroGuard Flood Detector"
    state_topic: "PumpController/flood_detector/analog_read"
    unit_of_measurement: "raw"
    device_class: "measurement"
    state_class: "measurement"
    value_template: "{{ value | int }}"

  - platform: mqtt
    name: "HydroGuard Flood Threshold"
    state_topic: "PumpController/flood_detector/treshold"
    entity_category: "diagnostic"
    value_template: "{{ value | int }}"

  - platform: mqtt
    name: "HydroGuard Flood Read Period"
    state_topic: "PumpController/flood_detector/read_period"
    unit_of_measurement: "ms"
    entity_category: "diagnostic"
    value_template: "{{ value | int }}"
```

### Pump State Machine Sensors

```yaml
  - platform: mqtt
    name: "HydroGuard Pump State"
    state_topic: "PumpController/pump_state_machine/current_state"
    value_template: "{{ value | int }}"
    device_class: "enum"
    options:
      - "0"   # STATE_0: Pump Off (Idle)
      - "1"   # STATE_1: Pump Active
      - "2"   # STATE_2: Safety Lockout

  - platform: mqtt
    name: "HydroGuard Pump Status"
    state_topic: "PumpController/pump_state_machine/pump_on"
    value_template: "{{ value | int }}"
    device_class: "running"
    entity_category: "diagnostic"

  - platform: mqtt
    name: "HydroGuard Safety Block"
    state_topic: "PumpController/pump_state_machine/safety_block"
    value_template: "{{ value | int }}"
    device_class: "problem"
    entity_category: "diagnostic"

  - platform: mqtt
    name: "HydroGuard Max Run Time"
    state_topic: "PumpController/pump_state_machine/max_run_time"
    unit_of_measurement: "ms"
    entity_category: "diagnostic"
    value_template: "{{ value | float }}"

  - platform: mqtt
    name: "HydroGuard Switch On Pressure"
    state_topic: "PumpController/pump_state_machine/switch_on_pressure"
    unit_of_measurement: "Bar"
    entity_category: "diagnostic"
    value_template: "{{ value | float }}"

  - platform: mqtt
    name: "HydroGuard Switch Off Pressure"
    state_topic: "PumpController/pump_state_machine/switch_off_pressure"
    unit_of_measurement: "Bar"
    entity_category: "diagnostic"
    value_template: "{{ value | float }}"
```

### MQTT Watchdog

```yaml
  - platform: mqtt
    name: "HydroGuard MQTT State"
    state_topic: "PumpController/mqttState"
    value_template: "{{ value | int }}"
    entity_category: "diagnostic"
```

---

## Step 4: Add Switches to Control the Pump

Add these switches to control the pump and reset the safety block:

```yaml
switch:
  - platform: mqtt
    name: "HydroGuard Pump"
    command_topic: "PumpController/pump_state_machine/pump_on/mqtt_input"
    payload_on: "1"
    payload_off: "0"
    state_topic: "PumpController/pump_state_machine/pump_on"
    state_on: "1"
    state_off: "0"
    icon: "mdi:water-pump"
    device_class: "switch"

  - platform: mqtt
    name: "HydroGuard Reset Safety Block"
    command_topic: "PumpController/pump_state_machine/safety_block/mqtt_input"
    payload_on: "0"
    payload_off: "1"
    state_topic: "PumpController/pump_state_machine/safety_block"
    state_on: "0"
    state_off: "1"
    icon: "mdi:shield-check"
    entity_category: "diagnostic"
```

> **Note:** The safety block reset switch is designed as a momentary "reset" button. When activated, it sends `0` to clear the safety block. The `state_on: "0"` and `state_off: "1"` configuration means the switch will appear "on" when the safety block is cleared (0) and "off" when it's active (1).

---

## Step 5: Add Number Entities for Configuration

Add these number entities to remotely adjust parameters:

```yaml
number:
  - platform: mqtt
    name: "HydroGuard Flow Read Period"
    command_topic: "PumpController/flow_meter/read_period/mqtt_input"
    state_topic: "PumpController/flow_meter/read_period"
    unit_of_measurement: "ms"
    min: 100
    max: 60000
    step: 100
    mode: "box"

  - platform: mqtt
    name: "HydroGuard Flow Conversion Coefficient"
    command_topic: "PumpController/flow_meter/conversionCoefficient/mqtt_input"
    state_topic: "PumpController/flow_meter/conversionCoefficient"
    min: 0.1
    max: 100.0
    step: 0.1
    mode: "box"

  - platform: mqtt
    name: "HydroGuard Pressure Read Period"
    command_topic: "PumpController/pressure_sensor/read_period/mqtt_input"
    state_topic: "PumpController/pressure_sensor/read_period"
    unit_of_measurement: "ms"
    min: 100
    max: 60000
    step: 100
    mode: "box"

  - platform: mqtt
    name: "HydroGuard Pressure Conversion Coefficient"
    command_topic: "PumpController/pressure_sensor/conversionCoefficient/mqtt_input"
    state_topic: "PumpController/pressure_sensor/conversionCoefficient"
    min: 0.1
    max: 100.0
    step: 0.1
    mode: "box"

  - platform: mqtt
    name: "HydroGuard Water Level Read Period"
    command_topic: "Cisterna/livello_cisterna/read_period/mqtt_input"
    state_topic: "Cisterna/livello_cisterna/read_period"
    unit_of_measurement: "ms"
    min: 100
    max: 60000
    step: 100
    mode: "box"

  - platform: mqtt
    name: "HydroGuard Water Level Read Repetitions"
    command_topic: "Cisterna/livello_cisterna/read_repetitions/mqtt_input"
    state_topic: "Cisterna/livello_cisterna/read_repetitions"
    min: 1
    max: 20
    step: 1
    mode: "box"

  - platform: mqtt
    name: "HydroGuard Flood Read Period"
    command_topic: "PumpController/flood_detector/read_period/mqtt_input"
    state_topic: "PumpController/flood_detector/read_period"
    unit_of_measurement: "ms"
    min: 100
    max: 60000
    step: 100
    mode: "box"

  - platform: mqtt
    name: "HydroGuard Flood Threshold"
    command_topic: "PumpController/flood_detector/treshold/mqtt_input"
    state_topic: "PumpController/flood_detector/treshold"
    min: 0
    max: 4095
    step: 1
    mode: "box"

  - platform: mqtt
    name: "HydroGuard Max Run Time"
    command_topic: "PumpController/pump_state_machine/max_run_time/mqtt_input"
    state_topic: "PumpController/pump_state_machine/max_run_time"
    unit_of_measurement: "ms"
    min: 60000
    max: 3600000
    step: 10000
    mode: "box"

  - platform: mqtt
    name: "HydroGuard Switch On Pressure"
    command_topic: "PumpController/pump_state_machine/switch_on_pressure/mqtt_input"
    state_topic: "PumpController/pump_state_machine/switch_on_pressure"
    unit_of_measurement: "Bar"
    min: 0.0
    max: 10.0
    step: 0.1
    mode: "box"

  - platform: mqtt
    name: "HydroGuard Switch Off Pressure"
    command_topic: "PumpController/pump_state_machine/switch_off_pressure/mqtt_input"
    state_topic: "PumpController/pump_state_machine/switch_off_pressure"
    unit_of_measurement: "Bar"
    min: 0.0
    max: 10.0
    step: 0.1
    mode: "box"
```

---

## Step 6: Add Useful Automations

Here are some example automations to get the most out of your HydroGuard:

### Flood Alert Automation

```yaml
automation:
  - alias: "HydroGuard Flood Detected"
    trigger:
      - platform: numeric_state
        entity_id: sensor.hydroguard_flood_detector
        above: 800
    condition:
      - condition: state
        entity_id: sensor.hydroguard_flood_detector
        state_not: "unavailable"
    action:
      - service: persistent_notification.create
        data:
          title: "⚠️ Flood Detected!"
          message: "HydroGuard flood sensor has detected water. Value: {{ states('sensor.hydroguard_flood_detector') }}"
      - service: notify.mobile_app_your_device
        data:
          message: "Flood detected by HydroGuard! Check immediately."
```

### Pump Runtime Tracking

```yaml
  - alias: "HydroGuard Pump Runtime Alert"
    trigger:
      - platform: state
        entity_id: switch.hydroguard_pump
        to: "on"
        for:
          minutes: 5
    action:
      - service: persistent_notification.create
        data:
          title: "⏱️ Pump Running"
          message: "The pump has been running for 5 minutes. Current pressure: {{ states('sensor.hydroguard_pressure') }} Bar"
```

### Safety Block Notification

```yaml
  - alias: "HydroGuard Safety Block Activated"
    trigger:
      - platform: state
        entity_id: sensor.hydroguard_safety_block
        to: "1"
    action:
      - service: persistent_notification.create
        data:
          title: "🔒 Safety Block Activated"
          message: "HydroGuard safety block has been activated. Check for issues and reset manually."
```

---

## Step 7: Create a Dashboard Card

Add this to your Lovelace dashboard for a quick overview:

```yaml
type: entities
title: HydroGuard Pump Controller
entities:
  - entity: switch.hydroguard_pump
    name: Pump
  - entity: sensor.hydroguard_pump_state
    name: State
  - entity: sensor.hydroguard_pressure
    name: Pressure
  - entity: sensor.hydroguard_flow_rate
    name: Flow Rate
  - entity: sensor.hydroguard_water_level_distance
    name: Water Level
  - entity: sensor.hydroguard_safety_block
    name: Safety Block
  - entity: switch.hydroguard_reset_safety_block
    name: Reset Safety Block
  - entity: sensor.hydroguard_mqtt_state
    name: MQTT State
```

---

## Step 8: Restart Home Assistant

After adding all configurations:

1. Go to **Settings → System**
2. Click **Restart** to apply the changes

---

## Troubleshooting

### No data appearing?

- Verify your MQTT broker credentials in the HydroGuard firmware (`localWiFiConfig.h`)
- Check that the HydroGuard is connected to Wi-Fi
- Use MQTT Explorer to verify topics are being published

### Entities show "unavailable"

- Ensure the MQTT integration is properly configured
- Check that the topic names match exactly (case-sensitive)
- Verify the device is publishing to the correct topics

### Pump switch doesn't control the pump

- Confirm the command topic is correct: `PumpController/pump_state_machine/pump_on/mqtt_input`
- Check that the payload values (`1` and `0`) match what the firmware expects

### Safety block won't reset

- Use the reset switch entity to send `0` to `PumpController/pump_state_machine/safety_block/mqtt_input`
- The safety block will automatically activate if safety conditions are met (flood detection or exceeding max run time)

---

## Complete Configuration Reference

Here's a summary of all MQTT topics used by the HydroGuard:

| Topic | Description |
|-------|-------------|
| **Published Topics** | |
| `PumpController/flow_meter/Q` | Flow rate (L/min) |
| `PumpController/flow_meter/V` | Total volume flowed (L) |
| `PumpController/pressure_sensor/pressure` | Pressure (Bar) |
| `Cisterna/livello_cisterna/distance` | Water level distance (cm) |
| `PumpController/flood_detector/analog_read` | Flood detector analog reading |
| `PumpController/pump_state_machine/current_state` | Current pump state |
| `PumpController/pump_state_machine/pump_on` | Pump ON/OFF status |
| `PumpController/pump_state_machine/safety_block` | Safety block status |
| `PumpController/mqttState` | MQTT watchdog state |
| **Subscribed Topics** | |
| `PumpController/.../mqtt_input` | All configuration update topics |

---

Your HydroGuard ESP32 Pump Controller is now fully integrated with Home Assistant! You can monitor all sensors, control the pump, adjust parameters remotely, and set up automations for a truly smart water management system.
