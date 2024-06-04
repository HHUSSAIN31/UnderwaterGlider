#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MS5803.h>

// Motor control pins
int motor1pin1 = 3;
int motor1pin2 = 4;
int pwmPin = 9;

// Pressure sensor
Adafruit_MS5803 pressureSensor = Adafruit_MS5803();

// IMU sensor
Adafruit_MPU6050 mpu;

// Threshold values
float pressureThreshold = 500.0; // Example threshold value for depth

void setup() {
  // Initialize motor control pins as outputs
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  
  // Initialize serial communication for debugging
  Serial.begin(9600);
  
  // Initialize pressure sensor
  if (!pressureSensor.begin()) {
    Serial.println("Failed to find MS5803 chip");
    while (1) { delay(10); }
  }

  // Initialize IMU
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }

  Serial.println("Setup complete");
}

void loop() {
  // Read the pressure value from the sensor
  pressureSensor.read();
  float pressureValue = pressureSensor.pressure();
  
  // Read IMU values
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Print the pressure and IMU values for debugging
  Serial.print("Pressure: "); Serial.println(pressureValue);
  Serial.print("Accel X: "); Serial.print(a.acceleration.x);
  Serial.print(" Y: "); Serial.print(a.acceleration.y);
  Serial.print(" Z: "); Serial.println(a.acceleration.z);
  Serial.print("Gyro X: "); Serial.print(g.gyro.x);
  Serial.print(" Y: "); Serial.print(g.gyro.y);
  Serial.print(" Z: "); Serial.println(g.gyro.z);

  // Check if pressure exceeds the threshold
  if (pressureValue > pressureThreshold) {
    // Rotate motor in one direction
    digitalWrite(motor1pin1, HIGH);
    digitalWrite(motor1pin2, LOW);
    analogWrite(pwmPin, 255); // Set motor speed
  } else {
    // Rotate motor in the opposite direction
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, HIGH);
    analogWrite(pwmPin, 255); // Set motor speed
  }

  delay(1000); // Small delay to prevent too rapid switching
}
