#include <Arduino.h>
#include <math.h>

const float R = 35.0;
const float r = 20.0;
const float h = 26.5;
const float Ln = sqrt((R - r) * (R - r) + h * h);

const int joystickX = A6;
const int joystickY = A7;
const int joystickButton = 2; 
const int centerValue = 512;
const int minValue = 0;
const int maxValue = 1023;
const int DEAD_ZONE = 100;      // Dead zone threshold
const int SAMPLES = 10;        // Number of samples for averaging
const int SAMPLE_DELAY = 1;    // Delay between samples in ms

unsigned long previousMillis = 0;
unsigned long analogReadInterval = 5;
unsigned long lastJoystickUpdate = 0;
const unsigned long JOYSTICK_UPDATE_INTERVAL = 50; // Update every 50ms

struct Cylinder {
  int potPin;    
  int relayA;    
  int relayB;    
  float targetPos; 
  float initialPos;  
  int analogMin;   
  int analogMax;
  unsigned long lastAnalogRead;
  int analogReadCount;         
  long analogSum;              
};

Cylinder cylinders[3] = {
  {A0, 6, 4, 0.00, 0.00, 0, 0, 0, 0, 0},
  {A1, 8, 3, 0.00, 0.00, 0, 0, 0, 0, 0},
  {A2, 7, 5, 0.00, 0.00, 0, 0, 0, 0, 0}
};

// System parameters
bool systemEnabled = false;
bool isCalibrating = false;
const float tolerance = 2.00;
unsigned long calibrationStartTime;
bool phase1Complete = false;
bool phase2Complete = false;

// Calibration timing constants
unsigned long calibrationPhase1End = 4000;
unsigned long calibrationPhase2End = 8000;

// Angle limits
const float YAW_LIMIT = 60.0;
const float PITCH_LIMIT = 60.0;

// Button debounce variables
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

// New function for filtered analog reading
int readAnalogFiltered(int pin) {
    long sum = 0;
    for(int i = 0; i < SAMPLES; i++) {
        sum += analogRead(pin);
        delay(SAMPLE_DELAY);
    }
    return sum / SAMPLES;
}

// New function for applying dead zone
int applyDeadZone(int value, int center, int threshold) {
    if(abs(value - center) < threshold) {
        return center;
    }
    return value;
}

float mapAnalogToLength(int analogValue, int analogMin, int analogMax) {
  analogValue = constrain(analogValue, analogMin, analogMax);
  float length = map(analogValue, analogMin, analogMax, 0.00, 100.00);
  return round(length * 100.0) / 100.0;
}
void calculateRotationMatrix(float alpha, float beta, float T[3][3]) {
    T[0][0] = cos(beta);
    T[0][1] = sin(alpha) * sin(beta);
    T[0][2] = cos(alpha) * sin(beta);
    T[1][0] = 0;
    T[1][1] = cos(alpha);
    T[1][2] = -sin(alpha);
    T[2][0] = -sin(beta);
    T[2][1] = sin(alpha) * cos(beta);
    T[2][2] = cos(alpha) * cos(beta);
}

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
    return round(((L - Ln) * 10)+50);
}


void calculateAllCylinders(float alpha, float beta, Cylinder cylinders[], int numCylinders) {
  for (int i = 0; i < numCylinders; i++) {
    cylinders[i].targetPos = calculateCylinderLength(i, alpha, beta);
  }
}
void handleButton() {
  // Read current button state
  bool currentButtonState = digitalRead(joystickButton);
  
  // Print button state for debugging
  Serial.print("Button Raw State: ");
  Serial.println(currentButtonState);
  
  // Check for button press (transition from HIGH to LOW)
  if (currentButtonState == LOW && lastButtonState == HIGH) {
    Serial.println("BUTTON PRESSED!");
    
    if (!isCalibrating && !systemEnabled) {
      Serial.println("Starting calibration...");
      isCalibrating = true;
      calibrationStartTime = millis();
    }
  }
  
  lastButtonState = currentButtonState;
}
void calibrateCylinders() {
  unsigned long elapsedTime = millis() - calibrationStartTime;
  
  static unsigned long lastProgressPrint = 0;
  if (millis() - lastProgressPrint >= 1000) {
    Serial.print("Calibration Progress: ");
    Serial.print(elapsedTime);
    Serial.println("ms");
    lastProgressPrint = millis();
  }

  if (!phase1Complete) {
    // Phase 1: Extend cylinders to read MAX value
    Serial.println("Phase 1: Extending cylinders");
    bool allCylindersComplete = true;
    
    for (int i = 0; i < 3; i++) {
      digitalWrite(cylinders[i].relayA, LOW);
      digitalWrite(cylinders[i].relayB, HIGH);
      
      // Start reading after 3.5s
      if (elapsedTime >= 3500) {
        if (millis() - cylinders[i].lastAnalogRead >= 20) {
          cylinders[i].analogSum += analogRead(cylinders[i].potPin);
          cylinders[i].analogReadCount++;
          cylinders[i].lastAnalogRead = millis();
          
          // After 10 readings, calculate average for analogMax
          if (cylinders[i].analogReadCount >= 10) {
            cylinders[i].analogMax = cylinders[i].analogSum / cylinders[i].analogReadCount;
            Serial.print("Cylinder ");
            Serial.print(i);
            Serial.print(" Max: ");
            Serial.println(cylinders[i].analogMax);
          } else {
            allCylindersComplete = false;
          }
        }
      } else {
        allCylindersComplete = false;
      }
    }
    
    if (allCylindersComplete) {
      phase1Complete = true;
      // Reset counters for phase 2
      for (int i = 0; i < 3; i++) {
        cylinders[i].analogSum = 0;
        cylinders[i].analogReadCount = 0;
      }
    }
    return;  // Exit function after phase 1
  }

  if (!phase2Complete) {
    // Phase 2: Retract cylinders to read MIN value
    Serial.println("Phase 2: Retracting cylinders");
    bool allCylindersComplete = true;
    
    for (int i = 0; i < 3; i++) {
      digitalWrite(cylinders[i].relayA, HIGH);
      digitalWrite(cylinders[i].relayB, LOW);
      
      // Start reading after 1.5s in phase 2
      if (elapsedTime >= (4000 + 1500)) {
        if (millis() - cylinders[i].lastAnalogRead >= 20) {
          cylinders[i].analogSum += analogRead(cylinders[i].potPin);
          cylinders[i].analogReadCount++;
          cylinders[i].lastAnalogRead = millis();
          
          // After 10 readings, calculate average for analogMin
          if (cylinders[i].analogReadCount >= 10) {
            cylinders[i].analogMin = cylinders[i].analogSum / cylinders[i].analogReadCount;
            Serial.print("Cylinder ");
            Serial.print(i);
            Serial.print(" Min: ");
            Serial.println(cylinders[i].analogMin);
          } else {
            allCylindersComplete = false;
          }
        }
      } else {
        allCylindersComplete = false;
      }
    }
    
    if (allCylindersComplete) {
      phase2Complete = true;
    }
    return;  // Exit function after phase 2
  }

  // Phase 3: Complete calibration only when both phases are done
  // Verify all values are valid
  bool validCalibration = true;
  for (int i = 0; i < 3; i++) {
    if (cylinders[i].analogMin == 0 || cylinders[i].analogMax == 0 || 
        cylinders[i].analogMin >= cylinders[i].analogMax) {
      validCalibration = false;
      Serial.print("Invalid calibration for cylinder ");
      Serial.println(i);
    }
  }
    
  if (validCalibration) {
    Serial.println("Calibration complete!");
    for (int i = 0; i < 3; i++) {
      digitalWrite(cylinders[i].relayA, HIGH);
      digitalWrite(cylinders[i].relayB, HIGH);
        
      float currentPos = mapAnalogToLength(
        analogRead(cylinders[i].potPin),
        cylinders[i].analogMin,
        cylinders[i].analogMax
      );
      cylinders[i].initialPos = currentPos;
      cylinders[i].targetPos = currentPos;
        
      Serial.print("Cylinder ");
      Serial.print(i);
      Serial.print(" calibrated - Min: ");
      Serial.print(cylinders[i].analogMin);
      Serial.print(", Max: ");
      Serial.print(cylinders[i].analogMax);
      Serial.print(", Initial Position: ");
      Serial.println(cylinders[i].initialPos);
    }
    isCalibrating = false;
    systemEnabled = true;
  } else {
    // Reset calibration if invalid
    Serial.println("Calibration failed! Resetting...");
    isCalibrating = false;
    systemEnabled = false;
  }

  // Reset phase flags for next calibration
  phase1Complete = false;
  phase2Complete = false;
}
void controlCylinders(Cylinder cylinders[], int numCylinders) {
  const float tolerance = 1.00;
  
  for (int i = 0; i < numCylinders; i++) {
    int currentAnalog = analogRead(cylinders[i].potPin);
    float currentLength = map(currentAnalog, cylinders[i].analogMin, 
                            cylinders[i].analogMax, 0, 100);
                            
    if (currentLength < cylinders[i].targetPos - tolerance) {
      digitalWrite(cylinders[i].relayA, LOW);
      digitalWrite(cylinders[i].relayB, HIGH);
    } 
    else if (currentLength > cylinders[i].targetPos + tolerance) {
      digitalWrite(cylinders[i].relayA, HIGH);
      digitalWrite(cylinders[i].relayB, LOW);
    } 
    else {
      digitalWrite(cylinders[i].relayA, HIGH);
      digitalWrite(cylinders[i].relayB, HIGH);
    }
  }
}

void handleJoystickControl() {
    // Check if it's time to update
    if (millis() - lastJoystickUpdate < JOYSTICK_UPDATE_INTERVAL) {
        return;
    }
    lastJoystickUpdate = millis();
    
    // Read and filter joystick values
    int xValue = readAnalogFiltered(joystickX);
    int yValue = readAnalogFiltered(joystickY);
    
    // Apply dead zone
    xValue = applyDeadZone(xValue, centerValue, DEAD_ZONE);
    yValue = applyDeadZone(yValue, centerValue, DEAD_ZONE);
    
    // Calculate normalized values
    float xNormalized = ((float)(xValue - centerValue) / (maxValue - centerValue));
    float yNormalized = ((float)(yValue - centerValue) / (maxValue - centerValue));
    
    // Calculate and constrain angles
    float yaw = constrain(xNormalized * YAW_LIMIT, -YAW_LIMIT, YAW_LIMIT);
    float pitch = constrain(yNormalized * PITCH_LIMIT, -PITCH_LIMIT, PITCH_LIMIT);
    
    // Debug output
    static unsigned long lastDebugPrint = 0;
    if (millis() - lastDebugPrint >= 500) {  // Print debug every 500ms
        Serial.print("Joystick - Raw X: ");
        Serial.print(xValue);
        Serial.print(", Raw Y: ");
        Serial.print(yValue);
        Serial.print(" | Filtered - Yaw: ");
        Serial.print(yaw);
        Serial.print(", Pitch: ");
        Serial.println(pitch);
        lastDebugPrint = millis();
    }
    
    // Calculate and control cylinders
    calculateAllCylinders(radians(pitch), radians(yaw), cylinders, 3);
    controlCylinders(cylinders, 3);
}

void setup() {
  Serial.begin(9600);
  
  // Joystick setup
  pinMode(joystickX, INPUT);
  pinMode(joystickY, INPUT);
  pinMode(joystickButton, INPUT_PULLUP);
  
  for (int i = 0; i < 3; i++) {
    pinMode(cylinders[i].relayA, OUTPUT);
    pinMode(cylinders[i].relayB, OUTPUT);
    digitalWrite(cylinders[i].relayA, HIGH);
    digitalWrite(cylinders[i].relayB, HIGH);
  }
  
  Serial.println("=== System Started ===");
  Serial.println("Waiting for button press to start calibration...");
}

void loop() {
  // Always handle button input
  handleButton();
  
  // Print system state every second
  static unsigned long lastDebugPrint = 0;
  if (millis() - lastDebugPrint >= 1000) {
    Serial.print("System State - Calibrating: ");
    Serial.print(isCalibrating);
    Serial.print(", Enabled: ");
    Serial.println(systemEnabled);
    lastDebugPrint = millis();
  }

  if (isCalibrating) {
    calibrateCylinders();
  } 
  else if (systemEnabled) {
    handleJoystickControl();
  }
  
  // Small delay to prevent serial flooding
  delay(50);
}
