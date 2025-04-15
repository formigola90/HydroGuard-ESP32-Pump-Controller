# JSN-SR04T Waterproof Ultrasonic Sensor Integration with HydroGuard ESP32 Pump Controller

This document provides information about integrating the JSN-SR04T waterproof ultrasonic sensor with the HydroGuard ESP32 Pump Controller.

---

### Sensor Overview

The JSN-SR04T is a waterproof ultrasonic distance measurement sensor designed for various applications, including obstacle avoidance, water level monitoring, and traffic control. Below are its key specifications:

#### Electrical Parameters:
- **Operating Voltage**: DC 5V
- **Quiescent Current**: 5mA
- **Working Current**: 30mA
- **Acoustic Emission Frequency**: 40kHz
- **Maximum Distance**: 4.5m
- **Resolution**: ~0.5cm
- **Blind Zone**: 25cm
- **Wiring**:
  - `+5V`: Positive power supply
  - `Trig`: Control pin (RX)
  - `Echo`: Receiver pin (TX)
  - `GND`: Ground
- **Working Temperature**: -10°C to 70°C
- **Storage Temperature**: -20°C to 80°C

#### Features:
1. Small size, easy to integrate.
2. Waterproof and suitable for harsh environments.
3. High accuracy and low power consumption.
4. Strong anti-jamming capability.

---

### Code Integration in HydroGuard ESP32 Pump Controller

The JSN-SR04T is integrated into the HydroGuard ESP32 Pump Controller as the `livello_cisterna` sensor, which measures water levels. Below are the details:

#### Code Configuration:

##### Pin Definitions:
```cpp
#define AJSR04M_TRIGGER_PIN 27
#define AJSR04M_ECHO_PIN 26
```

##### AJSR04M Structure:
```cpp
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
```

##### Initialization:
The sensor is initialized with specific properties:
```cpp
AJSR04M livello_cisterna = {5000, 0, 0, AJSR04M_TRIGGER_PIN, AJSR04M_ECHO_PIN, SOUND_SPEED, 1, 0, 0};
```

##### Trigger and Measurement Functions:
- **Trigger Function:**
  ```cpp
  void triggerAJSR04M(uint8_t trigPin) {
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
  }
  ```
- **Measurement Function:**
  ```cpp
  long measureAJSR04M(uint8_t trigPin, uint8_t echoPin) {
      long duration_mean = 0;
      for (int i = 0; i < livello_cisterna.read_repetitions; i++) {
          vTaskDelay(pdMS_TO_TICKS(1000)); // Wait at least 1 second
          triggerAJSR04M(trigPin);
          long duration = pulseIn(echoPin, HIGH);
          duration_mean += duration;
      }
      return duration_mean / livello_cisterna.read_repetitions;
  }
  ```

##### Pin Mode Configuration:
```cpp
pinMode(livello_cisterna.TRIG_PIN, OUTPUT);
pinMode(livello_cisterna.ECHO_PIN, INPUT_PULLDOWN);
```

#### Potential Considerations:
- The `INPUT_PULLDOWN` setting for the `ECHO_PIN` might not be suitable, as the JSN-SR04T sensor typically does not require an internal pull resistor. Verify the sensor's datasheet for electrical requirements.
- Ensure proper power supply levels. The sensor operates at 5V, while the ESP32 logic typically uses 3.3V. A level shifter might be required.

---

### Hardware Setup

Follow the guide at [MakerGuides](https://www.makerguides.com/interfacing-esp32-and-jsn-sr04t-waterproof-ultrasonic-sensor/#Step_1_Complete_the_hardware_connections) for connecting the JSN-SR04T sensor to an ESP32 board.

---

### Applications in HydroGuard
1. **Water Level Monitoring**:
   - Measure the water level in storage tanks.
   - Prevent overflow or pump dry runs.

2. **Obstacle Detection**:
   - Monitor obstacles in specific areas for safety.

---

### Additional Resources
- [JSN-SR04T Datasheet (Vendor Documentation)](https://www.makerguides.com/interfacing-esp32-and-jsn-sr04t-waterproof-ultrasonic-sensor/#Step_1_Complete_the_hardware_connections)
- [HydroGuard ESP32 GitHub Repository](https://github.com/formigola90/HydroGuard-ESP32-Pump-Controller)

---

### Troubleshooting
If the sensor does not provide accurate readings:
1. Verify the wiring and ensure solid connections.
2. Check compatibility of voltage levels between the sensor and the ESP32.
3. Review the software implementation for timing or pin configuration mismatches.
4. Ensure the sensor is not placed within its blind zone (25 cm).
