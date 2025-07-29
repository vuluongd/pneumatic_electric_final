#include <Arduino.h>
#include <math.h>

// Geometry constants
const float R = 35.0;
const float r = 20.0;
const float h = 26.5;
const float Ln = sqrt((R - r) * (R - r) + h * h);

// Joystick pin
const int joystickX = A6;
const int joystickY = A7;
const int button = 2;

// Data structure of cylinder
struct Cylinder {
  int potPin;    
  int relayA;    
  int relayB;    
  float targetPos; 
  int analogMin;   // Calibration: Min analog value
  int analogMax;   // Calibration: Max analog value
};

Cylinder cylinders[3] = {
  {A0, 6, 4, 50, 0, 0},
  {A1, 8, 3, 50, 0, 0},
  {A2, 7, 5, 50, 0, 0}
};

// Joystick parameters
int minValue = 0;
int maxValue = 1023;
int centerValue = 512;

// System parameters
int mode = 0;
const int tolerance = 10;
bool isCalibrating = false; // Calibration status
unsigned long calibrationStartTime;

// Angle limits for each mode
const int yawLimits[4][2] = {{-30, 30}, {-60, 60}, {0, 0}, {0, 0}};
const int pitchLimits[4][2] = {{-30, 30}, {-60, 60}, {0, 0}, {0, 0}};

// Debounce variables for joystick button
bool lastState = false;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

// Calculate the rotation matrix for given angles
void calculateRotationMatrix(float alpha, float beta, float T[3][3]) {
  T[0][0] = cos(beta);
  T[0][1] = sin(alpha) * sin(beta);
  T[0][2] = -cos(alpha) * sin(beta);
  T[1][0] = 0;
  T[1][1] = cos(alpha);
  T[1][2] = sin(alpha);
  T[2][0] = sin(beta);
  T[2][1] = -sin(alpha) * cos(beta);
  T[2][2] = cos(alpha) * cos(beta);
}

// Calculate cylinder length using inverse kinematics
float calculateCylinderLength(int i, float alpha, float beta) {
  float Pb[3][3] = {
    {0, sqrt(3) * R / 2, -sqrt(3) * R / 2},
    {R, -R / 2, -R / 2},
    {0, 0, 0}
  };
  float Pt[3][3] = {
    {0, sqrt(3) * r / 2, -sqrt(3) * r / 2},
    {r, -r / 2, -r / 2},
    {0, 0, 0}
  };

  float T[3][3];
  calculateRotationMatrix(alpha, beta, T);

  float Bt[3];
  for (int j = 0; j < 3; j++) {
    Bt[j] = T[j][0] * Pt[0][i] + T[j][1] * Pt[1][i] + T[j][2] * Pt[2][i];
  }

  float Po[3] = {0, 0, h};
  float deltaX = (Bt[0] + Po[0]) - Pb[0][i];
  float deltaY = (Bt[1] + Po[1]) - Pb[1][i];
  float deltaZ = (Bt[2] + Po[2]) - Pb[2][i];

  float L = sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
  return round(L - Ln); // Round to nearest integer mm
}

// Calculate all cylinder positions
void calculateAllCylinders(float alpha, float beta, Cylinder cylinders[], int numCylinders) {
  for (int i = 0; i < numCylinders; i++) {
    cylinders[i].targetPos = calculateCylinderLength(i, alpha, beta);
  }
}

// Control cylinder motion
void controlCylinders(Cylinder cylinders[], int numCylinders) {
  for (int i = 0; i < numCylinders; i++) {
    int currentPosition = analogRead(cylinders[i].potPin);

    if (currentPosition < cylinders[i].targetPos - tolerance) {
      digitalWrite(cylinders[i].relayA, LOW);  // Forward
      digitalWrite(cylinders[i].relayB, HIGH);
    } else if (currentPosition > cylinders[i].targetPos + tolerance) {
      digitalWrite(cylinders[i].relayA, HIGH);  // Backward
      digitalWrite(cylinders[i].relayB, LOW);
    } else {
      digitalWrite(cylinders[i].relayA, HIGH);  // Stop
      digitalWrite(cylinders[i].relayB, HIGH);
    }
  }
}

// Calibrate cylinders
void calibrateCylinders(Cylinder cylinders[], int numCylinders) {
  if (!isCalibrating) {
    calibrationStartTime = millis();
    isCalibrating = true;

    // Read initial analog values
    for (int i = 0; i < numCylinders; i++) {
      int sum = 0;
      for (int j = 0; j < 10; j++) {
        sum += analogRead(cylinders[i].potPin);
        delay(5);
      }
      cylinders[i].analogMin = sum / 10;
    }

    // Start moving cylinders forward
    for (int i = 0; i < numCylinders; i++) {
      digitalWrite(cylinders[i].relayA, LOW);
      digitalWrite(cylinders[i].relayB, HIGH);
    }
  }

  unsigned long elapsedTime = millis() - calibrationStartTime;

  // Move forward for 4 seconds
  if (elapsedTime > 4000 && elapsedTime <= 8000) {
    // Switch to moving backward
    for (int i = 0; i < numCylinders; i++) {
      digitalWrite(cylinders[i].relayA, HIGH);
      digitalWrite(cylinders[i].relayB, LOW);
    }
  }

  // Stop after another 4 seconds and read max analog values
  if (elapsedTime > 8000) {
    for (int i = 0; i < numCylinders; i++) {
      digitalWrite(cylinders[i].relayA, HIGH);
      digitalWrite(cylinders[i].relayB, HIGH);

      int sum = 0;
      for (int j = 0; j < 10; j++) {
        sum += analogRead(cylinders[i].potPin);
        delay(5);
      }
      cylinders[i].analogMax = sum / 10;
    }

    // Calibration complete
    isCalibrating = false;
    mode = 0; // Return to default operation mode
  }
}

void setup() {
  pinMode(button, INPUT);
  for (int i = 0; i < 3; i++) {
    pinMode(cylinders[i].relayA, OUTPUT);
    pinMode(cylinders[i].relayB, OUTPUT);
    digitalWrite(cylinders[i].relayA, HIGH);
    digitalWrite(cylinders[i].relayB, HIGH);
  }
}

void loop() {
  // JOystick button debounce logic
  bool currentState = !digitalRead(button);
  if (currentState && !lastState && (millis() - lastDebounceTime > debounceDelay)) {
    mode = (mode + 1) % 5;
    lastDebounceTime = millis();
  }
  lastState = currentState;

  // modes handling
  if (mode == 4) {
    calibrateCylinders(cylinders, 3);
  } else {
    int rawX = analogRead(joystickX);
    int rawY = analogRead(joystickY);

    float yaw = ((float)(rawX - centerValue) / (maxValue - centerValue)) * 90;
    float pitch = ((float)(rawY - centerValue) / (maxValue - centerValue)) * 90;

    yaw = constrain(yaw, yawLimits[mode][0], yawLimits[mode][1]);
    pitch = constrain(pitch, pitchLimits[mode][0], pitchLimits[mode][1]);

    if (mode == 0) {
      calculateAllCylinders(pitch, yaw, cylinders, 3);
    }
    controlCylinders(cylinders, 3);
  }
}
