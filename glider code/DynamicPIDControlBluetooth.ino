#include "IMU.h"
#include "Sonar.h"
#include "Arduino.h"
#include <PID_v1.h>
#include <ArduinoBLE.h>

// Pin definitions for actuators
#define RPWM1 9   // Right PWM for buoyancy control
#define LPWM1 10  // Left PWM for buoyancy control
#define PWM1 12   // PWM for buoyancy control

#define RPWM2 6   // Right PWM for pitch control
#define LPWM2 5   // Left PWM for pitch control
#define PWM2 3    // PWM for pitch control

// Bluetooth Definitions
#define SERVICE_UUID "180A"
#define CONTROL_CHARACTERISTIC_UUID "2A57"
BLEService gliderService(SERVICE_UUID);
BLEByteCharacteristic ControlCharacteristic(CONTROL_CHARACTERISTIC_UUID, BLERead | BLEWrite);

// IMU and Sonar objects
IMU imu;
Sonar sonar(Serial1);

// Control constants
const float proximityThreshold = 1500.0; // 1.5 meters in mm
const unsigned long maxBuoyancyDuration = 9000; // 9 seconds
const unsigned long maxPitchDuration = 18000; // 18 seconds
unsigned long buoyancyStartTime, pitchStartTime;

// Declare PID variables and initialize the PID objects
double setpointBuoyancy, inputBuoyancy, outputBuoyancy;
double setpointPitch, inputPitch, outputPitch;
double KpBuoyancy = 2.0, KiBuoyancy = 0.5, KdBuoyancy = 1.0;
double KpPitch = 1.5, KiPitch = 0.4, KdPitch = 0.9;

PID buoyancyPID(&inputBuoyancy, &outputBuoyancy, &setpointBuoyancy, KpBuoyancy, KiBuoyancy, KdBuoyancy, DIRECT);
PID pitchPID(&inputPitch, &outputPitch, &setpointPitch, KpPitch, KiPitch, KdPitch, DIRECT);

bool manualControl = false;

void setup() {
  Serial.begin(115200);
  imu.begin();

  // Initialize sonar
  int initAttempts = 0;
  while (!sonar.initialize() && initAttempts < 3) {
    Serial.println("Sonar failed to initialize!");
    delay(2000);
    initAttempts++;
  }

  pinMode(RPWM1, OUTPUT);
  pinMode(LPWM1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(RPWM2, OUTPUT);
  pinMode(LPWM2, OUTPUT);
  pinMode(PWM2, OUTPUT);

  // Initialize Bluetooth LE
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }
  BLE.setLocalName("Glider");
  BLE.setAdvertisedService(gliderService);
  gliderService.addCharacteristic(ControlCharacteristic);
  BLE.addService(gliderService);
  ControlCharacteristic.writeValue(0);
  BLE.advertise();

  setpointBuoyancy = 100; // Target distance from the bottom in cm
  setpointPitch = 0; // Target pitch level in degrees

  buoyancyPID.SetMode(AUTOMATIC);
  pitchPID.SetMode(AUTOMATIC);
}

void loop() {
  if (BLE.connected()) {
    if (ControlCharacteristic.written()) {
      manualControl = ControlCharacteristic.value() != 0; // Check for non-zero value for manual control
    }
  }

  if (!manualControl) {
    // Read IMU data
    int ax, ay, az, gx, gy, gz, mx, my, mz;
    bool imuAvailable = imu.readAccelerometer(ax, ay, az) && imu.readGyroscope(gx, gy, gz) && imu.readMagnetometer(mx, my, mz);

    // Read distance from sonar
    int distance = sonar.getDistance();

    // Actuator control for sonar detection
    if (distance > 0 && distance <= proximityThreshold) {
      actuatorRetract(LPWM1, RPWM1, PWM1); // Retract if within proximity threshold
    } else if (!imuAvailable) {
      actuatorRetract(LPWM1, RPWM1, PWM1); // Retract if IMU data not available
      // Consider resurfacing or emergency protocols
    }

    // Update input from sensors
    inputBuoyancy = distance / 10.0; // Distance in cm (sonar reading in mm converted to cm)
    inputPitch = ay; // Assuming 'ay' represents pitch

    // Compute PID output for buoyancy and pitch
    buoyancyPID.Compute();
    pitchPID.Compute();

    // Actuator control based on PID output for buoyancy
    if (outputBuoyancy > 0) {
      actuatorExtend(LPWM1, RPWM1, PWM1);
    } else {
      actuatorRetract(LPWM1, RPWM1, PWM1);
    }

    // Actuator control based on PID output for pitch
    if (outputPitch > 0) {
      actuatorExtend(LPWM2, RPWM2, PWM2);
    } else {
      actuatorRetract(LPWM2, RPWM2, PWM2);
    }

    // Failsafe timing checks
    if (millis() - buoyancyStartTime > maxBuoyancyDuration) {
      actuatorStop(LPWM1, RPWM1, PWM1);
    }
    if (millis() - pitchStartTime > maxPitchDuration) {
      actuatorStop(LPWM2, RPWM2, PWM2);
    }
  } else {
    byte command = ControlCharacteristic.value();
    switch (command) {
      case 1: // Extend buoyancy actuator
        actuatorExtend(LPWM1, RPWM1, PWM1); 
        break;
      case 2: // Retract buoyancy actuator
        actuatorRetract(LPWM1, RPWM1, PWM1); 
        break;
      case 3: // Extend pitch actuator
        actuatorExtend(LPWM2, RPWM2, PWM2); 
        break;
      case 4: // Retract pitch actuator
        actuatorRetract(LPWM2, RPWM2, PWM2); 
        break;
      case 0: // Stop all actuators
        actuatorStop(LPWM1, RPWM1, PWM1);
        actuatorStop(LPWM2, RPWM2, PWM2); 
        break;
    }
  }
}

void actuatorExtend(int LPWM, int RPWM, int PWM) {
  digitalWrite(LPWM, LOW);
  digitalWrite(RPWM, HIGH);
  analogWrite(PWM, 255); // Fully extend
  Serial.println("Actuator is Extending");
  buoyancyStartTime = millis(); // Restart timer when extending
}

void actuatorRetract(int LPWM, int RPWM, int PWM) {
  digitalWrite(LPWM, HIGH);
  digitalWrite(RPWM, LOW);
  analogWrite(PWM, 255); // Fully retract
  Serial.println("Actuator is Retracting");
  buoyancyStartTime = millis(); // Restart timer when retracting
}

void actuatorStop(int LPWM, int RPWM, int PWM) {
  digitalWrite(LPWM, LOW);
  digitalWrite(RPWM, LOW);
  analogWrite(PWM, 0); // Stop
  Serial.println("Actuator Stopped");
}