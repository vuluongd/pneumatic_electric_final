#include <Arduino.h>
const int potPins[3] = {A0, A1, A2};          
const int analogRest[3] = {365, 990, 240};   
const int analogFullStroke[3] = {240, 805, 107};
const float cylinderLength = 100.0;        
float previousLengths[3] = {0, 0, 0};       
const float changeThreshold = 0.5;           
float mapToLength(int analogValue, int analogRest, int analogFullStroke) {
  return (analogRest - analogValue) * (cylinderLength / (analogRest - analogFullStroke));
}
void setup() {
  Serial.begin(9600);                       
}

void loop() {
  for (int i = 0; i < 3; i++) {
    int rawValue = analogRead(potPins[i]);  
    float currentLength = mapToLength(rawValue, analogRest[i], analogFullStroke[i]);

    if (abs(currentLength - previousLengths[i]) > changeThreshold) {
      previousLengths[i] = currentLength;    
      Serial.print("Cylinder ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(currentLength);
      Serial.println(" mm");
    }
  }
}
