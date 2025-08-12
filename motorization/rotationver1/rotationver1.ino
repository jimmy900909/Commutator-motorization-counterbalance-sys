#include <Wire.h>
#include <QMC5883LCompass.h>

// Stepper motor pins
const int dirPin = 2;
const int stepPin = 3;

// Stepper motor settings
const float stepAngle = 1.8;              // Degrees per step (adjust if microstepping)
const int stepDelay = 1000;               // Microseconds between steps
const float yawThreshold = 15.0;          // Minimum yaw change to trigger movement
const float maxDeltaYaw = 90.0;           // Limit maximum yaw correction
const int minStepsToMove = 3;             // Ignore very small moves to prevent jitter

// Yaw update timing
const unsigned long checkYawInterval = 500;  // Check every 1 second
unsigned long lastCheckTime = 0;

QMC5883LCompass compass;
float previousYaw = 0;

void setup() {
  Serial.begin(9600);
  delay(1000);

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  compass.init();
  Serial.println("QMC5883L initialized.");

  // Get initial yaw
  compass.read();
  int x = compass.getX();
  int z = compass.getZ();
  previousYaw = atan2((float)x, (float)z) * 180.0 / PI;
  if (previousYaw < 0) previousYaw += 360;
}

void loop() {
  unsigned long now = millis();
  if (now - lastCheckTime < checkYawInterval) return;  // wait for 1 second
  lastCheckTime = now;

  // Read current yaw
  compass.read();
  int x = compass.getX();
  int z = compass.getZ();
  float currentYaw = atan2((float)x, (float)z) * 180.0 / PI;
  if (currentYaw < 0) currentYaw += 360;

  // Compute deltaYaw
  float deltaYaw = currentYaw - previousYaw;
  if (deltaYaw > 180) deltaYaw -= 360;
  if (deltaYaw < -180) deltaYaw += 360;

  // Clamp deltaYaw to avoid extreme steps
  deltaYaw = constrain(deltaYaw, -maxDeltaYaw, maxDeltaYaw);

  Serial.print("Current Yaw: ");
  Serial.print(currentYaw);
  Serial.print("°,  Delta Yaw: ");
  Serial.print(deltaYaw);
  Serial.println("°");

  // Ignore small changes
  if (abs(deltaYaw) < yawThreshold) return;

  // Compute number of steps
  int stepsToMove = abs(deltaYaw) / stepAngle;
  if (stepsToMove < minStepsToMove) return;  // skip very small moves

  // Set motor direction
  digitalWrite(dirPin, deltaYaw > 0 ? HIGH : LOW);

  // Move stepper motor
  for (int i = 0; i < stepsToMove; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);
  }

  // Update reference yaw
  previousYaw = currentYaw;
}
