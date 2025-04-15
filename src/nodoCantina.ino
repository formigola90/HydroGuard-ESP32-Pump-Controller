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

// IMPORTS
#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_task_wdt.h>
#include "localWiFiConfig.h"
#include <Adafruit_ADS1X15.h>


// DEFINES
// debug
#define DEBUG 1 // 0 (Silent) to 4 (Verbose)
#ifdef DEBUG
  #define dsPrint(s, level) if(level<DEBUG)Serial.print(s)
  #define dsPrintln(s, level) if(level<DEBUG)Serial.println(s)
#else
  #define dsPrint(s, level)
  #define dsPrintln(s, level)
#endif
#define DEBUGLevel_Mqtt_Generic 3
#define DEBUGLevel_Mqtt_Error 0
#define DEBUGLevel_Mqtt_FlowMeter 3
#define DEBUGLevel_Mqtt_PressureSensor 3
#define DEBUGLevel_Mqtt_WaterLevel 3
#define DEBUGLevel_Mqtt_FloodSensor 3
#define DEBUGLevel_Mqtt_StateMachine 3
// constants
#define SOUND_SPEED 0.034
// wtd
#define WDT_TIMEOUT 30
#define WDT_MQTT_CONTROLL_PERIOD 60000
// rele
#define RELAY_SIGNAL_PIN 16
// pump states
#define STATE_0 0 //pump off
#define STATE_1 1 //pump on
#define STATE_2 2 //pump safety block
// min sensor read period
#define MIN_SENSOR_READ_PERIOD 10 //[ms]

// SENSORS DEFINES
// flow meter
#define FLOW_METER_SIGNAL_PIN 14
// pressure sensor
#define PRESSURE_SENSOR_SIGNAL_CHANNEL 0
// distance sensor
#define AJSR04M_TRIGGER_PIN 27
#define AJSR04M_ECHO_PIN 26
// flood detector
#define FLOOD_DETECTOR_POWER_PIN 23
#define FLOOD_DETECTOR_SIGNAL_PIN 36

#ifndef MY_SSID
  #define MY_SSID "set_this_in_localWiFiConfig.h"
#endif
#ifndef MY_PASSWORD
  #define MY_PASSWORD "set_this_in_localWiFiConfig.h"
#endif
#ifndef MQTT_BROKER_IP
  #define MQTT_BROKER_IP "set_this_in_localWiFiConfig.h"
#endif


// WIFI
const char* ssid = MY_SSID;
const char* wifipassword = MY_PASSWORD;

WiFiClient wifi;

// MQTT
const char* mqtt_server = MQTT_BROKER_IP;
PubSubClient client(wifi);

// FREERTOS
SemaphoreHandle_t xMutex;
TaskHandle_t sensorsReadTaskHandle;
TaskHandle_t MqttTaskHandle;

//WTD
int mqttState = 0;
int previous_mqtt_check_time = 0;

// STRUCTS
struct PUMP_STATE_MACHINE {
  const uint8_t RELAY_PIN;
  uint8_t current_state;
  uint8_t new_state;
  bool new_data;
  bool pump_on;
  bool safety_block;
  long start_time;
  long max_run_time; //[ms]
  float switch_on_pressure; //[Bar]
  float switch_off_pressure; //[Bar]
};

// STRUCTS SENSORS
// flow meter 
struct YFG1 {
  int read_period; //[ms]
  int previous_read_time; //[ms]
  bool new_data;
	const uint8_t PIN;
	volatile uint32_t pulseNumber;
  float conversionCoefficient; // pulseNumber -> Q
	float Q; //[L/m]
	float V; //[L]
};
// pressure sensor
struct YD6080 {
  // this model measures from 0 to 0,5 MPa
  int read_period; //[ms]
  int previous_read_time; //[ms]
  bool new_data;
	const uint8_t ADC_CHANNEL;
	float analogRead;
	float conversionCoefficient; //[]
	float pressure; //[Bar]
};
// distance sensor
struct AJSR04M {
  int read_period; //[ms]
  int previous_read_time; //[ms]
  bool new_data;
	const uint8_t TRIG_PIN;
	const uint8_t ECHO_PIN;
  const float sound_speed;
  int read_repetitions;
  long duration;
  float distance; // [cm]
};
// flood detector
struct WATER_SENSOR{
  int read_period; //[ms]
  int previous_read_time; //[ms]
  bool new_data;
	const uint8_t POWER_PIN;
	const uint8_t SIGNAL_PIN;
  int analog_read;
  int treshold;
};
// MODULES INIT
PUMP_STATE_MACHINE pump_state_machine = {RELAY_SIGNAL_PIN, STATE_0, 0, 1, 0, 0, 0, 600000, 1.5, 3.0};
// SENSORS INIT
// flow meter
YFG1 flow_meter = {5000, 0, 0, FLOW_METER_SIGNAL_PIN, 0, 1, 0, 0};
// pressure sensor
YD6080 pressure_sensor = {5000, 0, 0, PRESSURE_SENSOR_SIGNAL_CHANNEL, 0, 1, 0};
// distance sensor
AJSR04M livello_cisterna = {5000, 0, 0, AJSR04M_TRIGGER_PIN, AJSR04M_ECHO_PIN, SOUND_SPEED, 1, 0, 0};
// flood detector
WATER_SENSOR flood_detector = {5000, 0, 0, FLOOD_DETECTOR_POWER_PIN, FLOOD_DETECTOR_SIGNAL_PIN, 800};

// CUSTOM FUNCTIONS
// flow meter pulse counter interrupt
void IRAM_ATTR isr() {
	flow_meter.pulseNumber++;
}
// AJSR04M ultrasound trigger
void triggerAJSR04M(uint8_t trigPin){
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);  
}
// AJSR04M ultrasound distance mesurment function
long measureAJSR04M(uint8_t trigPin, uint8_t echoPin){
  long duration_mean = 0;
  long duration = 0;
  long old_duration_mean = 0;
  int valid_measurements = livello_cisterna.read_repetitions;
  for(int i=0; i<(livello_cisterna.read_repetitions); i++){
    vTaskDelay(pdMS_TO_TICKS(1000)); //wait at least 1/2 sec
    triggerAJSR04M(trigPin);
    duration = pulseIn(echoPin, HIGH);
    dsPrint("AJSR04M measured echo delay ", 1);
    dsPrintln(duration, 1);
    duration_mean = duration_mean + duration;
    if(duration_mean == old_duration_mean && valid_measurements>1){
      valid_measurements = valid_measurements-1;
    }
    old_duration_mean = duration_mean;
  }
  duration_mean = duration_mean / valid_measurements; //mean of all valid measurments
  return duration_mean;
}

// TASK DEFINES
void sensorsReadTask(void * parameter) {
  
  // start with a delay to allow the MQTT task to start
  vTaskDelay(pdMS_TO_TICKS(1000));

  // Variables
  long read_time;

  // MODULES SETUP
  Adafruit_ADS1115 adc;
  if (!adc.begin()) {
    dsPrintln("Failed to initialize ADS.", 4);
    while (1);
  }
  // SENSORS SETUP
  // flow meter
  dsPrintln("Flow meter setup.", 1);
	pinMode(flow_meter.PIN, INPUT_PULLDOWN);
	attachInterrupt(flow_meter.PIN, isr, RISING);
  // pressure sensor;
  dsPrintln("Pressure sensor setup.", 1);
  // water level sensor
  dsPrintln("Water level sensor setup.", 1);
  pinMode(livello_cisterna.TRIG_PIN, OUTPUT); // Sets the trigPin as an Output
  pinMode(livello_cisterna.ECHO_PIN, INPUT_PULLDOWN); // Sets the echoPin as an Input
  // flood detector
  dsPrintln("Flood detector setup.", 1);
  pinMode(flood_detector.POWER_PIN, OUTPUT);
  digitalWrite(flood_detector.POWER_PIN, LOW);
	pinMode(flood_detector.SIGNAL_PIN, INPUT_PULLDOWN);

  for(;;){
    // flow mesurments
    read_time = millis();
    if ((read_time - flow_meter.previous_read_time)>flow_meter.read_period){
      flow_meter.new_data = 1;
      dsPrintln("Measuring flowrate.", 1);
      flow_meter.Q = flow_meter.pulseNumber * flow_meter.conversionCoefficient; // [L/min]
      flow_meter.pulseNumber = 0; // esetting flowmwter pulse counter
      flow_meter.V = flow_meter.V + flow_meter.Q/60/1000 * (read_time - flow_meter.previous_read_time); // volume calculation
      flow_meter.previous_read_time = read_time;
    }
    // pressure mesurments
    read_time = millis();
    if ((read_time - pressure_sensor.previous_read_time)>pressure_sensor.read_period){
      pressure_sensor.new_data = 1;
      pressure_sensor.previous_read_time = read_time;
      dsPrintln("Measuring pressure.", 1);
      pressure_sensor.analogRead = adc.computeVolts(adc.readADC_SingleEnded(pressure_sensor.ADC_CHANNEL));
      dsPrint("Analog read: ", 1);
      dsPrintln(pressure_sensor.analogRead, 1);
      pressure_sensor.pressure = pressure_sensor.analogRead * pressure_sensor.conversionCoefficient;
      dsPrint("Pressure: ", 1);
      dsPrintln(pressure_sensor.pressure, 1);
    }

    // Debug Serial
    //Serial.printf("Q =  %f L/min, V = %f L, P = %f mV\n", flow_meter.Q, flow_meter.V, pressure_sensor.pressure);
    // Serial.print("portata:");
    // Serial.print(flow_meter.Q);
    // Serial.print(",");
    // Serial.print("pressione:");
    // Serial.println(pressure_sensor.pressure);

    // water level mesurment
    read_time = millis();
    if ((read_time - livello_cisterna.previous_read_time)>livello_cisterna.read_period){
      livello_cisterna.new_data = 1;
      livello_cisterna.previous_read_time = read_time;
      dsPrintln("Measuring water level.", 1);
      xSemaphoreTake( xMutex, portMAX_DELAY );
      livello_cisterna.duration = measureAJSR04M( livello_cisterna.TRIG_PIN, livello_cisterna.ECHO_PIN);
      dsPrint("Duration:", 1);
      dsPrintln(livello_cisterna.duration, 1);
      livello_cisterna.distance = livello_cisterna.duration*livello_cisterna.sound_speed/2;
      dsPrint("Distance:", 1);
      dsPrintln(livello_cisterna.distance, 1);
      xSemaphoreGive(xMutex);
    }

    // flood detector
    read_time = millis();
    if ((read_time - flood_detector.previous_read_time)>flood_detector.read_period){
      flood_detector.new_data = 1;
      flood_detector.previous_read_time = read_time;
      dsPrintln("Flood checking.", 1);
      digitalWrite(flood_detector.POWER_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(10));
      flood_detector.analog_read = analogRead(flood_detector.SIGNAL_PIN);
      dsPrint("Flood detector analog read: ", 1);
      dsPrintln(flood_detector.analog_read, 1);
      digitalWrite(flood_detector.POWER_PIN, LOW);
      if(flood_detector.analog_read>flood_detector.treshold){
        dsPrintln("Flood detected!", 3);
        pump_state_machine.safety_block = 1;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(MIN_SENSOR_READ_PERIOD)); // allow Mqtt task to run
  }
}

void callback(char* topic, byte* message, unsigned int length) {
  dsPrint("Message arrived on topic: ", DEBUGLevel_Mqtt_Generic);
  dsPrint(topic, DEBUGLevel_Mqtt_Generic);
  dsPrint(". Message: ", DEBUGLevel_Mqtt_Generic);
  String messageTemp;
  char char_messageTemp[length];
  int int_data;
  float float_data;
  
  for (int i = 0; i < length; i++) {
    dsPrint((char)message[i], DEBUGLevel_Mqtt_Generic);
    char_messageTemp[i] = (char)message[i];
    messageTemp += (char)message[i];
  }
  dsPrintln("", DEBUGLevel_Mqtt_Generic);


  // MQTT MESSAGE DECODING
// flow_meter
  // read_period
  if (String(topic) == TOPIC_FlowMeter_readPeriod "/mqtt_input") {
    dsPrint("MQTT input received on " TOPIC_FlowMeter_readPeriod "/mqtt_input: ", DEBUGLevel_Mqtt_FlowMeter);
    int_data = messageTemp.toInt();
    if(int_data>MIN_SENSOR_READ_PERIOD){
      dsPrint(int_data, DEBUGLevel_Mqtt_FlowMeter);
      dsPrintln(" [ms]", DEBUGLevel_Mqtt_FlowMeter);
      flow_meter.read_period = int_data;
      flow_meter.new_data = 1;
    }
    else {
      dsPrint(int_data, DEBUGLevel_Mqtt_Error);
      dsPrintln("INVALID DATA ERROR on topic:" TOPIC_FlowMeter_readPeriod "/mqtt_input", DEBUGLevel_Mqtt_Error);
    }
  }
      // flow rate
      // V
  else if (String(topic) == TOPIC_FlowMeter_V "/mqtt_input") {
    dsPrint("MQTT input received on " TOPIC_FlowMeter_V "/mqtt_input: ", DEBUGLevel_Mqtt_FlowMeter);
    float_data = atof(char_messageTemp);
    if(float_data>=0){
      dsPrint(float_data, DEBUGLevel_Mqtt_FlowMeter);
      dsPrintln(" [Bar/1]", DEBUGLevel_Mqtt_FlowMeter);
      flow_meter.V = float_data;
      flow_meter.new_data = 1;
    }
    else {
      dsPrintln("INVALID DATA ERROR on topic:" TOPIC_FlowMeter_V "/mqtt_input", DEBUGLevel_Mqtt_Error);
    }
  }
      // conversionCoefficient
  else if (String(topic) == TOPIC_FlowMeter_conversionCoefficient "/mqtt_input") {
    dsPrint("MQTT input received on " TOPIC_FlowMeter_conversionCoefficient "/mqtt_input: ", DEBUGLevel_Mqtt_FlowMeter);
    float_data = atof(char_messageTemp);
    if(float_data>0){
      dsPrint(float_data, DEBUGLevel_Mqtt_FlowMeter);
      dsPrintln(" [Bar/1]", DEBUGLevel_Mqtt_FlowMeter);
      flow_meter.conversionCoefficient = float_data;
      flow_meter.new_data = 1;
    }
    else {
      dsPrintln("INVALID DATA ERROR on topic:" TOPIC_FlowMeter_conversionCoefficient "/mqtt_input", DEBUGLevel_Mqtt_Error);
    }
  }
// pressure_sensor
  // read_period
  else if (String(topic) == TOPIC_PressureSensor_readPeriod "/mqtt_input") {
    dsPrint("MQTT input received on " TOPIC_PressureSensor_readPeriod "/mqtt_input: ", DEBUGLevel_Mqtt_PressureSensor);
    int_data = messageTemp.toInt();
    if(int_data>MIN_SENSOR_READ_PERIOD){
      dsPrint(int_data, DEBUGLevel_Mqtt_PressureSensor);
      dsPrintln(" [ms]", DEBUGLevel_Mqtt_PressureSensor);
      pressure_sensor.read_period = int_data;
      pressure_sensor.new_data = 1;
    }
    else {
      dsPrintln("INVALID DATA ERROR on topic:" TOPIC_PressureSensor_readPeriod "/mqtt_input", DEBUGLevel_Mqtt_Error);
    }
  }
      // pressure
  // conversionCoefficient
  else if (String(topic) == TOPIC_PressureSensor_conversionCoefficient "/mqtt_input") {
    dsPrint("MQTT input received on " TOPIC_PressureSensor_conversionCoefficient "/mqtt_input: ", DEBUGLevel_Mqtt_PressureSensor);
    float_data = atof(char_messageTemp);
    if(float_data>0){
      dsPrint(float_data, DEBUGLevel_Mqtt_PressureSensor);
      dsPrintln(" [Bar/1]", DEBUGLevel_Mqtt_PressureSensor);
      pressure_sensor.conversionCoefficient = float_data;
      pressure_sensor.new_data = 1;
    }
    else {
      dsPrintln("INVALID DATA ERROR on topic:" TOPIC_PressureSensor_conversionCoefficient "/mqtt_input", DEBUGLevel_Mqtt_Error);
    }
  }
// livello_cisterna
  // read_period
  else if (String(topic) == TOPIC_WaterLevel_readPeriod "/mqtt_input") {
    dsPrint("MQTT input received on " TOPIC_WaterLevel_readPeriod "/mqtt_input: ", DEBUGLevel_Mqtt_WaterLevel);
    int_data = messageTemp.toInt();
    if(int_data>MIN_SENSOR_READ_PERIOD){
      dsPrint(int_data, DEBUGLevel_Mqtt_WaterLevel);
      dsPrintln(" [ms]", DEBUGLevel_Mqtt_WaterLevel);
      livello_cisterna.read_period = int_data;
      livello_cisterna.new_data = 1;
    }
    else {
      dsPrintln("INVALID DATA ERROR on topic:" TOPIC_WaterLevel_readPeriod "/mqtt_input", DEBUGLevel_Mqtt_Error);
    }
  }
  // read_repetitions
  else if (String(topic) == TOPIC_WaterLevel_readRepetitions "/mqtt_input") {
    dsPrint("MQTT input received on " TOPIC_WaterLevel_readRepetitions "/mqtt_input: ", DEBUGLevel_Mqtt_WaterLevel);
    int_data = messageTemp.toInt();
    if(int_data>0){
      dsPrint(int_data, DEBUGLevel_Mqtt_WaterLevel);
      dsPrintln(" measurments.", DEBUGLevel_Mqtt_WaterLevel);
      livello_cisterna.read_repetitions = int_data;
    }
    else {
      dsPrintln("INVALID DATA ERROR on topic:" TOPIC_WaterLevel_readRepetitions "/mqtt_input", DEBUGLevel_Mqtt_Error);
    }
  }
  // distance
// flood_detector
  // read_period
  else if (String(topic) == TOPIC_FloodSensor_readPeriod "/mqtt_input") {
    dsPrint("MQTT input received on " TOPIC_FloodSensor_readPeriod "/mqtt_input: ", DEBUGLevel_Mqtt_FloodSensor);
    int_data = messageTemp.toInt();
    if(int_data>MIN_SENSOR_READ_PERIOD){
      dsPrint(int_data, DEBUGLevel_Mqtt_FloodSensor);
      dsPrintln(" [ms]", DEBUGLevel_Mqtt_FloodSensor);
      flood_detector.read_period = int_data;
      flood_detector.new_data = 1;
    }
    else {
      dsPrintln("INVALID DATA ERROR on topic:" TOPIC_FloodSensor_readPeriod "/mqtt_input", DEBUGLevel_Mqtt_Error);
    }
  }
      // analog read
  // treshold
  else if (String(topic) == TOPIC_FloodSensor_threshold "/mqtt_input") {
    dsPrint("MQTT input received on " TOPIC_FloodSensor_threshold "/mqtt_input: ", DEBUGLevel_Mqtt_FloodSensor);
    int_data = messageTemp.toInt();
    if(int_data>MIN_SENSOR_READ_PERIOD){
      dsPrint(int_data, DEBUGLevel_Mqtt_FloodSensor);
      flood_detector.treshold = int_data;
      flood_detector.new_data = 1;
    }
    else {
      dsPrintln("INVALID DATA ERROR on topic:" TOPIC_FloodSensor_threshold "/mqtt_input", DEBUGLevel_Mqtt_Error);
    }
  }
    // pump_state_machine
      // current_state;
      // new_state;
      // pump_on;
  else if (String(topic) == TOPIC_StateMachine_pumpOn "/mqtt_input") {
    dsPrint("MQTT input received on " TOPIC_StateMachine_pumpOn "/mqtt_input: ", DEBUGLevel_Mqtt_StateMachine);
    int_data = messageTemp.toInt();
    if(int_data==0 || int_data==1){
      dsPrint(int_data, DEBUGLevel_Mqtt_StateMachine);
      dsPrintln(" [ms]", DEBUGLevel_Mqtt_StateMachine);
      pump_state_machine.pump_on = int_data;
      pump_state_machine.new_data = 1;
    }
    else {
      dsPrintln("INVALID DATA ERROR on topic:" TOPIC_StateMachine_pumpOn "/mqtt_input", DEBUGLevel_Mqtt_Error);
    }
  }
      // safety_block;
  else if (String(topic) == TOPIC_StateMachine_safetyBlock "/mqtt_input") {
    dsPrint("MQTT input received on " TOPIC_StateMachine_safetyBlock "/mqtt_input: ", DEBUGLevel_Mqtt_StateMachine);
    int_data = messageTemp.toInt();
    if(int_data==0 || int_data==1){
      dsPrint(int_data, DEBUGLevel_Mqtt_StateMachine);
      dsPrintln(" [ms]", DEBUGLevel_Mqtt_StateMachine);
      pump_state_machine.safety_block = int_data;
      pump_state_machine.new_data = 1;
    }
    else {
      dsPrintln("INVALID DATA ERROR on topic:" TOPIC_StateMachine_safetyBlock "/mqtt_input", DEBUGLevel_Mqtt_Error);
    }
  }
      // start_time;
      // max_run_time; //[ms]
  else if (String(topic) == TOPIC_StateMachine_maxRunTime "/mqtt_input") {
    dsPrint("MQTT input received on " TOPIC_StateMachine_maxRunTime "/mqtt_input: ", DEBUGLevel_Mqtt_StateMachine);
    int_data = messageTemp.toInt();
    if(int_data>MIN_SENSOR_READ_PERIOD){
      dsPrint(int_data, DEBUGLevel_Mqtt_StateMachine);
      dsPrintln(" [ms]", DEBUGLevel_Mqtt_StateMachine);
      pump_state_machine.max_run_time = int_data;
      pump_state_machine.new_data = 1;
    }
    else {
      dsPrintln("INVALID DATA ERROR on topic:" TOPIC_StateMachine_maxRunTime "/mqtt_input", DEBUGLevel_Mqtt_Error);
    }
  }
      // switch_on_pressure; //[Bar]
  else if (String(topic) == TOPIC_StateMachine_switchOnPressure "/mqtt_input") {
    dsPrint("MQTT input received on " TOPIC_StateMachine_switchOnPressure "/mqtt_input: ", 3);
    float_data = atof(char_messageTemp);
    if(float_data>0 && float_data<pump_state_machine.switch_off_pressure){
      dsPrint(float_data, DEBUGLevel_Mqtt_StateMachine);
      dsPrintln(" [Bar/1]", DEBUGLevel_Mqtt_StateMachine);
      pump_state_machine.switch_on_pressure = float_data;
      pump_state_machine.new_data = 1;
    }
    else {
      dsPrintln("INVALID DATA ERROR on topic:" TOPIC_StateMachine_switchOnPressure "/mqtt_input", DEBUGLevel_Mqtt_Error);
    }
  }
      // switch_off_pressure; //[Bar]
  else if (String(topic) == TOPIC_StateMachine_switchOffPressure "/mqtt_input") {
    dsPrint("MQTT input received on " TOPIC_StateMachine_switchOffPressure "/mqtt_input: ", DEBUGLevel_Mqtt_StateMachine);
    float_data = atof(char_messageTemp);
    if(float_data>0 && float_data>pump_state_machine.switch_on_pressure){
      dsPrint(float_data, DEBUGLevel_Mqtt_StateMachine);
      dsPrintln(" [Bar/1]", DEBUGLevel_Mqtt_StateMachine);
      pump_state_machine.switch_off_pressure = float_data;
      pump_state_machine.new_data = 1;
    }
    else {
      dsPrintln("INVALID DATA ERROR on topic:" TOPIC_StateMachine_switchOffPressure "/mqtt_input", DEBUGLevel_Mqtt_Error);
    }
  }
  // wtd_mqtt_state
  else if (String(topic) == TOPIC_mqttState "/mqtt_input") {
    dsPrint("MQTT input received on " TOPIC_mqttState "/mqtt_input: ", DEBUGLevel_Mqtt_Generic);
    int_data = messageTemp.toInt();
    if(int_data==1){
      dsPrint(int_data, DEBUGLevel_Mqtt_Generic);
      mqttState = 1;
    }
    else {
      dsPrintln("INVALID DATA ERROR on topic:" TOPIC_mqttState "/mqtt_input", DEBUGLevel_Mqtt_Error);
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    dsPrint("Attempting MQTT connection...", 2);
    // Attempt to connect
    if (client.connect(MQTT_ID, MQTT_USER, MQTT_PASSWORD)) {
      dsPrint("connected", 2);
      // TOPIC SUBSCRIPTIONS
    // flow_meter
      // read_period
      client.subscribe(TOPIC_FlowMeter_readPeriod "/mqtt_input");
      // flow rate
      // flowed volume
      client.subscribe(TOPIC_FlowMeter_V "/mqtt_input");
      // conversionCoefficient
      client.subscribe(TOPIC_FlowMeter_conversionCoefficient "/mqtt_input");
    // pressure_sensor
      // read_period
      client.subscribe(TOPIC_PressureSensor_readPeriod "/mqtt_input");
      // pressure
      // conversionCoefficient
      client.subscribe(TOPIC_PressureSensor_conversionCoefficient "/mqtt_input");
    // water level mesurment
      // read_period
      client.subscribe(TOPIC_WaterLevel_readPeriod"/mqtt_input");
      // read_repetitions
      client.subscribe(TOPIC_WaterLevel_readRepetitions "/mqtt_input");
      // distance
    // flood_detector
      // read_period
      client.subscribe(TOPIC_FloodSensor_readPeriod "/mqtt_input");
      // analog read
      // treshold
      client.subscribe(TOPIC_FloodSensor_threshold "/mqtt_input");
    // pump state machine
      // current_state;
      // new_state;
      // pump_on;
      client.subscribe(TOPIC_StateMachine_pumpOn "/mqtt_input");
      // malfunction_detected;
      client.subscribe(TOPIC_StateMachine_safetyBlock "/mqtt_input");
      // start_time;
      // max_run_time; //[ms]
      client.subscribe(TOPIC_StateMachine_maxRunTime "/mqtt_input");
      //switch_on_pressure; //[Bar]
      client.subscribe(TOPIC_StateMachine_switchOnPressure "/mqtt_input");
      // witch_off_pressure; //[Bar]
      client.subscribe(TOPIC_StateMachine_switchOffPressure "/mqtt_input");
      // mqttState
      client.subscribe(TOPIC_mqttState "/mqtt_input");
    } else {
      dsPrint("failed, rc=", DEBUGLevel_Mqtt_Error);
      dsPrint(client.state(), DEBUGLevel_Mqtt_Error);
      dsPrintln(" try again in 5 seconds", DEBUGLevel_Mqtt_Error);
      // Wait 5 seconds before retrying
      vTaskDelay(pdMS_TO_TICKS(5000));
    }
  }
}

void MqttTask(void * parameter) {

  int ret = 0;
  char dataString[8];

  dsPrintln("Configuring WDT...", 2);
  esp_task_wdt_init(3000, true); // Timeout of 3000ms and enable panic
  esp_task_wdt_add(NULL); //add current thread to WDT watch

  // NETWORK SETUP
  dsPrint("Connecting to SSID: ", 2);
  dsPrintln(ssid, 2);
  
  WiFi.begin(ssid, wifipassword);
  while( WiFi.status() != WL_CONNECTED ) {
    dsPrint(".", 2);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  dsPrint("\nConnected to ", 2);
  dsPrintln(ssid, 2);
  dsPrintln("IP address: ", 2);
  dsPrintln(WiFi.localIP(), 2);

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  // STATE MACHINE SETUP
  pinMode(pump_state_machine.RELAY_PIN, OUTPUT);
  digitalWrite(pump_state_machine.RELAY_PIN, LOW);

  for(;;){

    if (!client.connected()) {
      reconnect();
    }
    client.loop();
    
    // MQTT SENSORS DATA PUBLISHING
    // flow mesurments
    if (flow_meter.new_data) {
      flow_meter.new_data = 0;
      // read_period
      dtostrf(flow_meter.read_period, 1, 2, dataString);
      dsPrint(TOPIC_FlowMeter_readPeriod ": ", DEBUGLevel_Mqtt_FlowMeter);
      dsPrintln(dataString, DEBUGLevel_Mqtt_FlowMeter);
      client.publish(TOPIC_FlowMeter_readPeriod, dataString);
      // flow rate
      dtostrf(flow_meter.Q, 1, 2, dataString);
      dsPrint(TOPIC_FlowMeter_Q ": ", DEBUGLevel_Mqtt_FlowMeter);
      dsPrintln(dataString, DEBUGLevel_Mqtt_FlowMeter);
      client.publish(TOPIC_FlowMeter_Q, dataString);
      // flowed volume
      dtostrf(flow_meter.V, 1, 2, dataString);
      dsPrint(TOPIC_FlowMeter_V ": ", DEBUGLevel_Mqtt_FlowMeter);
      dsPrintln(dataString, DEBUGLevel_Mqtt_FlowMeter);
      client.publish(TOPIC_FlowMeter_V, dataString);
      // conversionCoefficient
      dtostrf(flow_meter.conversionCoefficient, 1, 2, dataString);
      dsPrint(TOPIC_FlowMeter_conversionCoefficient ": ", DEBUGLevel_Mqtt_FlowMeter);
      dsPrintln(dataString, DEBUGLevel_Mqtt_FlowMeter);
      client.publish(TOPIC_FlowMeter_conversionCoefficient, dataString);
    }
    // pressure mesurments
    if(pressure_sensor.new_data){
      pressure_sensor.new_data = 0;
      // read_period
      dtostrf(pressure_sensor.read_period, 1, 2, dataString);
      dsPrint(TOPIC_PressureSensor_readPeriod ": ", DEBUGLevel_Mqtt_PressureSensor);
      dsPrintln(dataString, DEBUGLevel_Mqtt_PressureSensor);
      client.publish(TOPIC_PressureSensor_readPeriod, dataString);
      // pressure
      dtostrf(pressure_sensor.pressure, 1, 2, dataString);
      dsPrint(TOPIC_PressureSensor_pressure ": ", DEBUGLevel_Mqtt_PressureSensor);
      dsPrintln(dataString, DEBUGLevel_Mqtt_PressureSensor);
      client.publish(TOPIC_PressureSensor_pressure, dataString);
      // conversionCoefficient
      dtostrf(pressure_sensor.conversionCoefficient, 1, 2, dataString);
      dsPrint(TOPIC_PressureSensor_conversionCoefficient ": ", DEBUGLevel_Mqtt_PressureSensor);
      dsPrintln(dataString, DEBUGLevel_Mqtt_PressureSensor);
      client.publish(TOPIC_PressureSensor_conversionCoefficient, dataString);
    }
    // water level mesurment
    if(livello_cisterna.new_data){
      livello_cisterna.new_data=0;
      // read_period
      String(livello_cisterna.read_period).toCharArray(dataString, 8);
      dsPrint(TOPIC_WaterLevel_readPeriod ": ", DEBUGLevel_Mqtt_WaterLevel);
      dsPrintln(dataString, DEBUGLevel_Mqtt_WaterLevel);
      client.publish(TOPIC_WaterLevel_readPeriod, dataString);
      // read_repetitions
      String(livello_cisterna.read_repetitions).toCharArray(dataString, 8);
      dsPrint(TOPIC_WaterLevel_readRepetitions ": ", DEBUGLevel_Mqtt_WaterLevel);
      dsPrintln(dataString, DEBUGLevel_Mqtt_WaterLevel);
      client.publish(TOPIC_WaterLevel_readRepetitions, dataString);
      // distance
      xSemaphoreTake( xMutex, portMAX_DELAY );
      dtostrf(livello_cisterna.distance, 1, 2, dataString);
      xSemaphoreGive(xMutex);
      dsPrint(TOPIC_WaterLevel_distance ": ", DEBUGLevel_Mqtt_WaterLevel);
      dsPrintln(dataString, DEBUGLevel_Mqtt_WaterLevel);
      client.publish(TOPIC_WaterLevel_distance, dataString);
    }
    // flood detector
    if(flood_detector.new_data){
      flood_detector.new_data = 0;
      // read_period
      dtostrf(flood_detector.read_period, 1, 2, dataString);
      dsPrint(TOPIC_FloodSensor_readPeriod ": ", DEBUGLevel_Mqtt_FloodSensor);
      dsPrintln(dataString, DEBUGLevel_Mqtt_FloodSensor);
      client.publish(TOPIC_FloodSensor_readPeriod, dataString);
      // analog read
      dtostrf(flood_detector.analog_read, 1, 2, dataString);
      dsPrint(TOPIC_FloodSensor_analogRead ": ", DEBUGLevel_Mqtt_FloodSensor);
      dsPrintln(dataString, DEBUGLevel_Mqtt_FloodSensor);
      client.publish(TOPIC_FloodSensor_analogRead, dataString);
      // treshold
      dtostrf(flood_detector.treshold, 1, 2, dataString);
      dsPrint(TOPIC_FloodSensor_threshold ": ", DEBUGLevel_Mqtt_FloodSensor);
      dsPrintln(dataString, DEBUGLevel_Mqtt_FloodSensor);
      client.publish(TOPIC_FloodSensor_threshold, dataString);
    }
    // pump_state_machine
    if(pump_state_machine.new_data){
      pump_state_machine.new_data = 0;
      // current_state;
      dtostrf(pump_state_machine.current_state, 1, 2, dataString);
      dsPrint(TOPIC_StateMachine_currentState ": ", DEBUGLevel_Mqtt_StateMachine);
      dsPrintln(dataString, DEBUGLevel_Mqtt_StateMachine);
      client.publish(TOPIC_StateMachine_currentState, dataString);
      // new_state;
      // pump_on;
      dtostrf(pump_state_machine.pump_on, 1, 2, dataString);
      dsPrint(TOPIC_StateMachine_pumpOn ": ", DEBUGLevel_Mqtt_StateMachine);
      dsPrintln(dataString, DEBUGLevel_Mqtt_StateMachine);
      client.publish(TOPIC_StateMachine_pumpOn, dataString);
      // safety_block;
      dtostrf(pump_state_machine.safety_block, 1, 2, dataString);
      dsPrint(TOPIC_StateMachine_safetyBlock ": ", DEBUGLevel_Mqtt_StateMachine);
      dsPrintln(dataString, DEBUGLevel_Mqtt_StateMachine);
      client.publish(TOPIC_StateMachine_safetyBlock, dataString);
      // start_time;
      // max_run_time; //[ms]
      dtostrf(pump_state_machine.max_run_time, 1, 2, dataString);
      dsPrint(TOPIC_StateMachine_maxRunTime ": ", DEBUGLevel_Mqtt_StateMachine);
      dsPrintln(dataString, DEBUGLevel_Mqtt_StateMachine);
      client.publish(TOPIC_StateMachine_maxRunTime, dataString);
      //switch_on_pressure; //[Bar]
      dtostrf(pump_state_machine.switch_on_pressure, 1, 2, dataString);
      dsPrint(TOPIC_StateMachine_switchOnPressure ": ", DEBUGLevel_Mqtt_StateMachine);
      dsPrintln(dataString, DEBUGLevel_Mqtt_StateMachine);
      client.publish(TOPIC_StateMachine_switchOnPressure, dataString);
      // switch_off_pressure; //[Bar]
      dtostrf(pump_state_machine.switch_off_pressure, 1, 2, dataString);
      dsPrint(TOPIC_StateMachine_switchOffPressure ": ", DEBUGLevel_Mqtt_StateMachine);
      dsPrintln(dataString, DEBUGLevel_Mqtt_StateMachine);
      client.publish(TOPIC_StateMachine_switchOffPressure, dataString);
    }

    // STATE MACHINE
    if(pump_state_machine.new_state != pump_state_machine.current_state){
      if(pump_state_machine.new_state == STATE_0){
        pump_state_machine.current_state = pump_state_machine.new_state;
        vTaskDelay(pdMS_TO_TICKS(3000));
        digitalWrite(pump_state_machine.RELAY_PIN, LOW);
        pump_state_machine.new_data = 1;
      }
      else if(pump_state_machine.new_state == STATE_1){
        pump_state_machine.start_time = millis();
        pump_state_machine.current_state = pump_state_machine.new_state;
        digitalWrite(pump_state_machine.RELAY_PIN, HIGH);
        pump_state_machine.new_data = 1;
      }
      else if(pump_state_machine.new_state == STATE_2){
        pump_state_machine.current_state = pump_state_machine.new_state;
        digitalWrite(pump_state_machine.RELAY_PIN, LOW);
        pump_state_machine.new_data = 1;
      }
    }
    else{
      if(pump_state_machine.current_state==STATE_2){
        if(!pump_state_machine.safety_block){
          pump_state_machine.new_state = STATE_0;
        }
      }
      if(pump_state_machine.current_state==STATE_0){
        if(pump_state_machine.safety_block){
          pump_state_machine.new_state = STATE_2;
        }
        else if(pump_state_machine.pump_on && 
                pressure_sensor.pressure < pump_state_machine.switch_on_pressure){
          pump_state_machine.new_state = STATE_1;
        }
      }
      if(pump_state_machine.current_state==STATE_1){
        if(millis()-pump_state_machine.start_time > pump_state_machine.max_run_time){
          pump_state_machine.safety_block = 1;
        }
        if(pump_state_machine.safety_block){
          pump_state_machine.new_state = STATE_2;
        }
        else if(!pump_state_machine.pump_on ||
                pressure_sensor.pressure > pump_state_machine.switch_off_pressure){
          pump_state_machine.new_state = STATE_0;
        }
      }
    }

    

    // WATCHDOG RESET
    if (millis()-previous_mqtt_check_time>WDT_MQTT_CONTROLL_PERIOD){
      client.publish(TOPIC_mqttState, "OK");
      while (!mqttState){
        if (!client.connected()) {
          reconnect();
        }
        client.loop();
      }
      ret=esp_task_wdt_reset();
      if (ret==ESP_OK){
        dsPrintln("WDT reset: ESP_OK",3);
      }
      else if(ret==ESP_ERR_INVALID_ARG){
        dsPrintln("WDT reset: ESP_ERR_INVALID_ARG", 0);
      }
      else{
        dsPrintln("WDT reset: ESP_ERR_INVALID_STATE", 0);
      }
      mqttState = 0;
      previous_mqtt_check_time = millis();
    }
    else{
      ret=esp_task_wdt_reset();
      if (ret==ESP_OK){
        dsPrintln("WDT reset: ESP_OK",3);
      }
      else if(ret==ESP_ERR_INVALID_ARG){
        dsPrintln("WDT reset: ESP_ERR_INVALID_ARG", 0);
      }
      else{
        dsPrintln("WDT reset: ESP_ERR_INVALID_STATE", 0);
      }
    }
  }
}



void setup() {
	Serial.begin(115200);

  xMutex = xSemaphoreCreateMutex();
  
  vTaskDelay(pdMS_TO_TICKS(500));
  xTaskCreatePinnedToCore(sensorsReadTask,"Task1",10000,NULL,2,&sensorsReadTaskHandle,1);
  vTaskDelay(pdMS_TO_TICKS(500));
  xTaskCreatePinnedToCore(MqttTask,"Task2",10000,NULL,1,&MqttTaskHandle,1);
  vTaskDelay(pdMS_TO_TICKS(500));
}


void loop(){
  // function loop() is empty because we are using FreeRTOS
}
