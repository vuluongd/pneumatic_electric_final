const int numPots = 1;             
int potPins[numPots] = {A2}; 
int potValues[numPots];             

void setup() {
  Serial.begin(9600);  
}

void loop() {
  for (int i = 0; i < numPots; i++) {
    potValues[i] = analogRead(potPins[i]); 

  for (int i = 0; i < numPots; i++) {
    Serial.print("Potentiometer ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(potValues[i]);
    if (i < numPots - 1) Serial.print(" | ");
  Serial.println();
  }
  delay(500); 
}
}