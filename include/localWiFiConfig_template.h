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

#ifndef LOCAL_WIFI_CONFIG_H
#define LOCAL_WIFI_CONFIG_H

// WiFi credentials
#define WIFI_SSID "Your_SSID"
#define WIFI_PASSWORD "Your_Password"

// Mqtt broker credentials
#define MQTT_BROKER_IP "Your_MQTT_Broker_IP"
#define MQTT_USER "Your_MQTT_User"
#define MQTT_PASSWORD "Your_MQTT_Password"
#define MQTT_ID "Your_MQTT_Client_ID"

// MQTT topics
#define TOPIC_FlowMeter_conversionCoefficient "PumpController/flow_meter/conversionCoefficient"
#define TOPIC_FlowMeter_readPeriod "PumpController/flow_meter/read_period"
#define TOPIC_FlowMeter_V "PumpController/flow_meter/V"
#define TOPIC_FlowMeter_Q "PumpController/flow_meter/Q"

#define TOPIC_PressureSensor_readPeriod "PumpController/pressure_sensor/read_period"
#define TOPIC_PressureSensor_conversionCoefficient "PumpController/pressure_sensor/conversionCoefficient"
#define TOPIC_PressureSensor_pressure "PumpController/pressure_sensor/pressure"

#define TOPIC_WaterLevel_readPeriod "Cisterna/livello_cisterna/read_period"
#define TOPIC_WaterLevel_readRepetitions "Cisterna/livello_cisterna/read_repetitions"
#define TOPIC_WaterLevel_distance "Cisterna/livello_cisterna/distance"

#define TOPIC_FloodSensor_readPeriod "PumpController/flood_detector/read_period"
#define TOPIC_FloodSensor_threshold "PumpController/flood_detector/threshold"
#define TOPIC_FloodSensor_analogRead "PumpController/flood_detector/analog_read"

#define TOPIC_StateMachine_currentState "PumpController/pump_state_machine/current_state"
#define TOPIC_StateMachine_pumpOn "PumpController/pump_state_machine/pump_on"
#define TOPIC_StateMachine_safetyBlock "PumpController/pump_state_machine/safety_block"
#define TOPIC_StateMachine_maxRunTime "PumpController/pump_state_machine/max_run_time"
#define TOPIC_StateMachine_switchOnPressure "PumpController/pump_state_machine/switch_on_pressure"
#define TOPIC_StateMachine_switchOffPressure "PumpController/pump_state_machine/switch_off_pressure"

#define TOPIC_mqttState "PumpController/mqttState"

#endif