// IMPORTS
#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_task_wdt.h>
#include "localWiFiConfig.h"
#include <Adafruit_ADS1X15.h>
#include "esp_task_wdt.h"


// DEFINES
// debug
#define DEBUG 4 // 0-highest, 1-high, 2-medium, 3-low
#ifdef DEBUG
  #define dsPrint(s, level) if(level>DEBUG)Serial.print(s)
  #define dsPrintln(s, level) if(level>DEBUG)Serial.println(s)
#else
  #define dsPrint(s, level)
  #define dsPrintln(s, level)
#endif
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
  #define MY_SSID "your_ssid"
#endif
#ifndef MY_PASSWORD
  #define MY_PASSWORD "your_password"
#endif


// WIFI
const char* ssid = MY_SSID;
const char* wifipassword = MY_PASSWORD;

WiFiClient wifi;

// MQTT
const char* mqtt_server = "192.168.1.159";
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
  float conversionCoeficient; // pulseNumber -> Q
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
	float conversionCoeficient; //[]
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
    vTaskDelay(pdMS_TO_TICKS(1000)); //wait 1/2 sec
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
      flow_meter.Q = flow_meter.pulseNumber * flow_meter.conversionCoeficient; // [L/min]
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
      pressure_sensor.pressure = pressure_sensor.analogRead * pressure_sensor.conversionCoeficient;
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
  dsPrint("Message arrived on topic: ", 4);
  dsPrint(topic, 4);
  dsPrint(". Message: ", 4);
  String messageTemp;
  char char_messageTemp[length];
  int int_data;
  float float_data;
  
  for (int i = 0; i < length; i++) {
    dsPrint((char)message[i], 4);
    char_messageTemp[i] = (char)message[i];
    messageTemp += (char)message[i];
  }
  dsPrintln("", 4);


  // MQTT MESSAGE DECODING
// flow_meter
  // read_period
  if (String(topic) == "PumpController/flow_meter/read_period/mqtt_input") {
    dsPrint("MQTT input received on PumpController/flow_meter/read_period/mqtt_input: ", 3);
    int_data = messageTemp.toInt();
    if(int_data>MIN_SENSOR_READ_PERIOD){
      dsPrint(int_data, 3);
      dsPrintln(" [ms]", 3);
      flow_meter.read_period = int_data;
      flow_meter.new_data = 1;
    }
    else {
      dsPrint(int_data, 3);
      dsPrintln("INVALID DATA ERROR", 3);
    }
  }
      // flow rate
      // V
  else if (String(topic) == "PumpController/flow_meter/V/mqtt_input") {
    dsPrint("MQTT input received on PumpController/flow_meter/V/mqtt_input: ", 3);
    float_data = atof(char_messageTemp);
    if(float_data>=0){
      dsPrint(float_data, 3);
      dsPrintln(" [Bar/1]", 3);
      flow_meter.V = float_data;
      flow_meter.new_data = 1;
    }
    else {
      dsPrintln("INVALID DATA ERROR", 3);
    }
  }
      // conversionCoeficient
  else if (String(topic) == "PumpController/flow_meter/conversionCoeficient/mqtt_input") {
    dsPrint("MQTT input received on PumpController/flow_meter/conversionCoeficient/mqtt_input: ", 3);
    float_data = atof(char_messageTemp);
    if(float_data>0){
      dsPrint(float_data, 3);
      dsPrintln(" [Bar/1]", 3);
      flow_meter.conversionCoeficient = float_data;
      flow_meter.new_data = 1;
    }
    else {
      dsPrintln("INVALID DATA ERROR", 3);
    }
  }
// pressure_sensor
  // read_period
  else if (String(topic) == "PumpController/pressure_sensor/read_period/mqtt_input") {
    dsPrint("MQTT input received on PumpController/pressure_sensor/read_period/mqtt_input: ", 3);
    int_data = messageTemp.toInt();
    if(int_data>MIN_SENSOR_READ_PERIOD){
      dsPrint(int_data, 3);
      dsPrintln(" [ms]", 3);
      pressure_sensor.read_period = int_data;
      pressure_sensor.new_data = 1;
    }
    else {
      dsPrintln("INVALID DATA ERROR", 3);
    }
  }
      // pressure
  // conversionCoeficient
  else if (String(topic) == "PumpController/pressure_sensor/conversionCoeficient/mqtt_input") {
    dsPrint("MQTT input received on PumpController/pressure_sensor/conversionCoeficient/mqtt_input: ", 3);
    float_data = atof(char_messageTemp);
    if(float_data>0){
      dsPrint(float_data, 3);
      dsPrintln(" [Bar/1]", 3);
      pressure_sensor.conversionCoeficient = float_data;
      pressure_sensor.new_data = 1;
    }
    else {
      dsPrintln("INVALID DATA ERROR", 3);
    }
  }
// livello_cisterna
  // read_period
  else if (String(topic) == "Cisterna/livello_cisterna/read_period/mqtt_input") {
    dsPrint("MQTT input received on Cisterna/livello_cisterna/read_period/mqtt_input: ", 3);
    int_data = messageTemp.toInt();
    if(int_data>MIN_SENSOR_READ_PERIOD){
      dsPrint(int_data, 3);
      dsPrintln(" [ms]", 3);
      livello_cisterna.read_period = int_data;
      livello_cisterna.new_data = 1;
    }
    else {
      dsPrintln("INVALID DATA ERROR", 3);
    }
  }
  // read_repetitions
  else if (String(topic) == "Cisterna/livello_cisterna/read_repetitions/mqtt_input") {
    dsPrint("MQTT input received on Cisterna/livello_cisterna/read_repetitions/mqtt_input: ", 3);
    int_data = messageTemp.toInt();
    if(int_data>0){
      dsPrint(int_data, 3);
      dsPrintln(" measurments.", 3);
      livello_cisterna.read_repetitions = int_data;
    }
    else {
      dsPrintln("INVALID DATA ERROR", 3);
    }
  }
  // distance
// flood_detector
  // read_period
  else if (String(topic) == "PumpController/flood_detector/read_period/mqtt_input") {
    dsPrint("MQTT input received on PumpController/flood_detector/read_period/mqtt_input: ", 3);
    int_data = messageTemp.toInt();
    if(int_data>MIN_SENSOR_READ_PERIOD){
      dsPrint(int_data, 3);
      dsPrintln(" [ms]", 3);
      flood_detector.read_period = int_data;
      flood_detector.new_data = 1;
    }
    else {
      dsPrintln("INVALID DATA ERROR", 3);
    }
  }
      // analog read
  // treshold
  else if (String(topic) == "PumpController/flood_detector/treshold/mqtt_input") {
    dsPrint("MQTT input received on PumpController/flood_detector/treshold/mqtt_input: ", 3);
    int_data = messageTemp.toInt();
    if(int_data>MIN_SENSOR_READ_PERIOD){
      dsPrint(int_data, 3);
      flood_detector.treshold = int_data;
      flood_detector.new_data = 1;
    }
    else {
      dsPrintln("INVALID DATA ERROR", 2);
    }
  }
    // pump_state_machine
      // current_state;
      // new_state;
      // pump_on;
  else if (String(topic) == "PumpController/pump_state_machine/pump_on/mqtt_input") {
    dsPrint("MQTT input received on PumpController/pump_state_machine/pump_on/mqtt_input: ", 3);
    int_data = messageTemp.toInt();
    if(int_data==0 || int_data==1){
      dsPrint(int_data, 3);
      dsPrintln(" [ms]", 3);
      pump_state_machine.pump_on = int_data;
      pump_state_machine.new_data = 1;
    }
    else {
      dsPrintln("INVALID DATA ERROR", 3);
    }
  }
      // safety_block;
  else if (String(topic) == "PumpController/pump_state_machine/safety_block/mqtt_input") {
    dsPrint("MQTT input received on PumpController/pump_state_machine/safety_block/mqtt_input: ", 3);
    int_data = messageTemp.toInt();
    if(int_data==0 || int_data==1){
      dsPrint(int_data, 3);
      dsPrintln(" [ms]", 3);
      pump_state_machine.safety_block = int_data;
      pump_state_machine.new_data = 1;
    }
    else {
      dsPrintln("INVALID DATA ERROR", 3);
    }
  }
      // start_time;
      // max_run_time; //[ms]
  else if (String(topic) == "PumpController/pump_state_machine/max_run_time/mqtt_input") {
    dsPrint("MQTT input received on PumpController/pump_state_machine/max_run_time/mqtt_input: ", 3);
    int_data = messageTemp.toInt();
    if(int_data>MIN_SENSOR_READ_PERIOD){
      dsPrint(int_data, 3);
      dsPrintln(" [ms]", 3);
      pump_state_machine.max_run_time = int_data;
      pump_state_machine.new_data = 1;
    }
    else {
      dsPrintln("INVALID DATA ERROR", 3);
    }
  }
      // switch_on_pressure; //[Bar]
  else if (String(topic) == "PumpController/pump_state_machine/switch_on_pressure/mqtt_input") {
    dsPrint("MQTT input received on PumpController/pump_state_machine/switch_on_pressure/mqtt_input: ", 3);
    float_data = atof(char_messageTemp);
    if(float_data>0 && float_data<pump_state_machine.switch_off_pressure){
      dsPrint(float_data, 3);
      dsPrintln(" [Bar/1]", 3);
      pump_state_machine.switch_on_pressure = float_data;
      pump_state_machine.new_data = 1;
    }
    else {
      dsPrintln("INVALID DATA ERROR", 3);
    }
  }
      // switch_off_pressure; //[Bar]
  else if (String(topic) == "PumpController/pump_state_machine/switch_off_pressure/mqtt_input") {
    dsPrint("MQTT input received on PumpController/pump_state_machine/switch_off_pressure/mqtt_input: ", 3);
    float_data = atof(char_messageTemp);
    if(float_data>0 && float_data>pump_state_machine.switch_on_pressure){
      dsPrint(float_data, 3);
      dsPrintln(" [Bar/1]", 3);
      pump_state_machine.switch_off_pressure = float_data;
      pump_state_machine.new_data = 1;
    }
    else {
      dsPrintln("INVALID DATA ERROR", 3);
    }
  }
  // wtd_mqtt_state
  else if (String(topic) == "PumpController/mqttState/mqtt_input") {
    dsPrint("MQTT input received on PumpController/mqttState/mqtt_input: ", 4);
    int_data = messageTemp.toInt();
    if(int_data==1){
      dsPrint(int_data, 4);
      mqttState = 1;
    }
    else {
      dsPrintln("INVALID DATA ERROR", 4);
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    dsPrint("Attempting MQTT connection...", 2);
    // Attempt to connect
    if (client.connect("PumpControllerClient", "fortis10", "bocko")) {
      dsPrint("connected", 2);
      // TOPIC SUBSCRIPTIONS
    // flow_meter
      // read_period
      client.subscribe("PumpController/flow_meter/read_period/mqtt_input");
      // flow rate
      // flowed volume
      client.subscribe("PumpController/flow_meter/V/mqtt_input");
      // conversionCoeficient
      client.subscribe("PumpController/flow_meter/conversionCoeficient/mqtt_input");
    // pressure_sensor
      // read_period
      client.subscribe("PumpController/pressure_sensor/read_period/mqtt_input");
      // pressure
      // conversionCoeficient
      client.subscribe("PumpController/pressure_sensor/conversionCoeficient/mqtt_input");
    // water level mesurment
      // read_period
      client.subscribe("Cisterna/livello_cisterna/read_period/mqtt_input");
      // read_repetitions
      client.subscribe("Cisterna/livello_cisterna/read_repetitions/mqtt_input");
      // distance
    // flood_detector
      // read_period
      client.subscribe("PumpController/flood_detector/read_period/mqtt_input");
      // analog read
      // treshold
      client.subscribe("PumpController/flood_detector/treshold/mqtt_input");
    // pump state machine
      // current_state;
      // new_state;
      // pump_on;
      client.subscribe("PumpController/pump_state_machine/pump_on/mqtt_input");
      // malfunction_detected;
      client.subscribe("PumpController/pump_state_machine/safety_block/mqtt_input");
      // start_time;
      // max_run_time; //[ms]
      client.subscribe("PumpController/pump_state_machine/max_run_time/mqtt_input");
      //switch_on_pressure; //[Bar]
      client.subscribe("PumpController/pump_state_machine/switch_on_pressure/mqtt_input");
      // witch_off_pressure; //[Bar]
      client.subscribe("PumpController/pump_state_machine/switch_off_pressure/mqtt_input");
      // mqttState
      client.subscribe("PumpController/mqttState/mqtt_input");
    } else {
      dsPrint("failed, rc=", 2);
      dsPrint(client.state(), 2);
      dsPrintln(" try again in 5 seconds", 2);
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
      dsPrint("PumpController/flow_meter/read_period: ", 2);
      dsPrintln(dataString, 2);
      client.publish("PumpController/flow_meter/read_period", dataString);
      // flow rate
      dtostrf(flow_meter.Q, 1, 2, dataString);
      dsPrint("PumpController/flow_meter/Q: ", 2);
      dsPrintln(dataString, 2);
      client.publish("PumpController/flow_meter/Q", dataString);
      // flowed volume
      dtostrf(flow_meter.V, 1, 2, dataString);
      dsPrint("PumpController/flow_meter/V: ", 2);
      dsPrintln(dataString, 2);
      client.publish("PumpController/flow_meter/V", dataString);
      // conversionCoeficient
      dtostrf(flow_meter.conversionCoeficient, 1, 2, dataString);
      dsPrint("PumpController/flow_meter/conversionCoeficient: ", 2);
      dsPrintln(dataString, 2);
      client.publish("PumpController/flow_meter/conversionCoeficient", dataString);
    }
    // pressure mesurments
    if(pressure_sensor.new_data){
      pressure_sensor.new_data = 0;
      // read_period
      dtostrf(pressure_sensor.read_period, 1, 2, dataString);
      dsPrint("PumpController/pressure_sensor/read_period: ", 2);
      dsPrintln(dataString, 2);
      client.publish("PumpController/pressure_sensor/read_period", dataString);
      // pressure
      dtostrf(pressure_sensor.pressure, 1, 2, dataString);
      dsPrint("PumpController/pressure_sensor/pressure: ", 2);
      dsPrintln(dataString, 2);
      client.publish("PumpController/pressure_sensor/pressure", dataString);
      // conversionCoeficient
      dtostrf(pressure_sensor.conversionCoeficient, 1, 2, dataString);
      dsPrint("PumpController/pressure_sensor/conversionCoeficient: ", 2);
      dsPrintln(dataString, 2);
      client.publish("PumpController/pressure_sensor/conversionCoeficient", dataString);
    }
    // water level mesurment
    if(livello_cisterna.new_data){
      livello_cisterna.new_data=0;
      // read_period
      String(livello_cisterna.read_period).toCharArray(dataString, 8);
      dsPrint("Cisterna/livello_cisterna/read_period: ", 2);
      dsPrintln(dataString, 2);
      client.publish("Cisterna/livello_cisterna/read_period", dataString);
      // read_repetitions
      String(livello_cisterna.read_repetitions).toCharArray(dataString, 8);
      dsPrint("Cisterna/livello_cisterna/read_repetitions: ", 2);
      dsPrintln(dataString, 2);
      client.publish("Cisterna/livello_cisterna/read_repetitions", dataString);
      // distance
      xSemaphoreTake( xMutex, portMAX_DELAY );
      dtostrf(livello_cisterna.distance, 1, 2, dataString);
      xSemaphoreGive(xMutex);
      dsPrint("Cisterna/livello_cisterna/distance: ", 2);
      dsPrintln(dataString, 2);
      client.publish("Cisterna/livello_cisterna/distance", dataString);
    }
    // flood detector
    if(flood_detector.new_data){
      flood_detector.new_data = 0;
      // read_period
      dtostrf(flood_detector.read_period, 1, 2, dataString);
      dsPrint("PumpController/flood_detector/read_period: ", 2);
      dsPrintln(dataString, 2);
      client.publish("PumpController/flood_detector/read_period", dataString);
      // analog read
      dtostrf(flood_detector.analog_read, 1, 2, dataString);
      dsPrint("PumpController/flood_detector/analog_read: ", 2);
      dsPrintln(dataString, 2);
      client.publish("PumpController/flood_detector/analog_read", dataString);
      // treshold
      dtostrf(flood_detector.treshold, 1, 2, dataString);
      dsPrint("PumpController/flood_detector/treshold: ", 2);
      dsPrintln(dataString, 2);
      client.publish("PumpController/flood_detector/treshold", dataString);
    }
    // pump_state_machine
    if(pump_state_machine.new_data){
      pump_state_machine.new_data = 0;
      // current_state;
      dtostrf(pump_state_machine.current_state, 1, 2, dataString);
      dsPrint("PumpController/pump_state_machine/current_state: ", 2);
      dsPrintln(dataString, 2);
      client.publish("PumpController/pump_state_machine/current_state", dataString);
      // new_state;
      // pump_on;
      dtostrf(pump_state_machine.pump_on, 1, 2, dataString);
      dsPrint("PumpController/pump_state_machine/pump_on: ", 2);
      dsPrintln(dataString, 2);
      client.publish("PumpController/pump_state_machine/pump_on", dataString);
      // safety_block;
      dtostrf(pump_state_machine.safety_block, 1, 2, dataString);
      dsPrint("PumpController/pump_state_machine/safety_block: ", 2);
      dsPrintln(dataString, 2);
      client.publish("PumpController/pump_state_machine/safety_block", dataString);
      // start_time;
      // max_run_time; //[ms]
      dtostrf(pump_state_machine.max_run_time, 1, 2, dataString);
      dsPrint("PumpController/pump_state_machine/max_run_time: ", 2);
      dsPrintln(dataString, 2);
      client.publish("PumpController/pump_state_machine/max_run_time", dataString);
      //switch_on_pressure; //[Bar]
      dtostrf(pump_state_machine.switch_on_pressure, 1, 2, dataString);
      dsPrint("PumpController/pump_state_machine/switch_on_pressure: ", 2);
      dsPrintln(dataString, 2);
      client.publish("PumpController/pump_state_machine/switch_on_pressure", dataString);
      // switch_off_pressure; //[Bar]
      dtostrf(pump_state_machine.switch_off_pressure, 1, 2, dataString);
      dsPrint("PumpController/pump_state_machine/switch_off_pressure: ", 2);
      dsPrintln(dataString, 2);
      client.publish("PumpController/pump_state_machine/switch_off_pressure", dataString);
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
      client.publish("PumpController/mqttState", "OK");
      while (!mqttState){
        if (!client.connected()) {
          reconnect();
        }
        client.loop();
      }
      ret=esp_task_wdt_reset();
      if (ret==ESP_OK){
        dsPrintln("WDT reset: ESP_OK",2);
      }
      else if(ret==ESP_ERR_INVALID_ARG){
        dsPrintln("WDT reset: ESP_ERR_INVALID_ARG", 2);
      }
      else{
        dsPrintln("WDT reset: ESP_ERR_INVALID_STATE", 2);
      }
      mqttState = 0;
      previous_mqtt_check_time = millis();
    }
    else{
      ret=esp_task_wdt_reset();
      if (ret==ESP_OK){
        dsPrintln("WDT reset: ESP_OK",0);
      }
      else if(ret==ESP_ERR_INVALID_ARG){
        dsPrintln("WDT reset: ESP_ERR_INVALID_ARG", 2);
      }
      else{
        dsPrintln("WDT reset: ESP_ERR_INVALID_STATE", 2);
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
