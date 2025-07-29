#include <Arduino.h>

// Define joystick pins
const int pinX = A0; // X-axis connected to analog pin A0
const int pinY = A1; // Y-axis connected to analog pin A1
const int pinButton = 2; // Button connected to digital pin 2

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);
  
  // Set button pin as input with pull-up resistor
  pinMode(pinButton, INPUT_PULLUP);
}

void loop() {
  // Read joystick X and Y values
  int xValue = analogRead(pinX);
  int yValue = analogRead(pinY);
  
  // Read button state (LOW when pressed, HIGH otherwise)
  bool buttonState = digitalRead(pinButton);
  
  // Print joystick values to Serial Monitor
  Serial.print("X: ");
  Serial.print(xValue);
  Serial.print("\tY: ");
  Serial.print(yValue);
  Serial.print("\tButton: ");
  Serial.println(buttonState ? "Released" : "Pressed");
  
  // Small delay for stability
  delay(100);
}
/*
const int joystickX = A6;
const int joystickY = A7;
const int joystickButton = 2;

// Calibration values
const int centerValue = 512;
const int minValue = 0;
const int maxValue = 1023;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Set joystick pin modes
  pinMode(joystickX, INPUT);
  pinMode(joystickY, INPUT);
  pinMode(joystickButton, INPUT_PULLUP);

  // Print header
  Serial.println("Joystick Test Program");
  Serial.println("Format: X, Y, Button");
}

void loop() {
  // Read analog values
  int xValue = analogRead(joystickX);
  int yValue = analogRead(joystickY);
  
  // Read button state (inverted because of INPUT_PULLUP)
  bool buttonPressed = !digitalRead(joystickButton);

  // Calculate normalized values (-1 to 1)
  float xNormalized = ((float)(xValue - centerValue) / (maxValue - centerValue));
  float yNormalized = ((float)(yValue - centerValue) / (maxValue - centerValue));

  // Calculate angles
  float yaw = xNormalized * 90.0;     // X-axis determines yaw
  float pitch = yNormalized * 90.0;   // Y-axis determines pitch

  // Print values
  Serial.print("Raw X: ");
  Serial.print(xValue);
  Serial.print(" | Normalized X: ");
  Serial.print(xNormalized);
  Serial.print(" | Yaw: ");
  Serial.print(yaw);
  Serial.print(" | Raw Y: ");
  Serial.print(yValue);
  Serial.print(" | Normalized Y: ");
  Serial.print(yNormalized);
  Serial.print(" | Pitch: ");
  Serial.print(pitch);
  Serial.print(" | Button: ");
  Serial.print(buttonPressed ? "PRESSED" : "RELEASED");
  Serial.println();

  // Small delay to prevent serial monitor flooding
  delay(200);
}
*/