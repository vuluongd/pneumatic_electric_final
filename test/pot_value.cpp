const int NUM_POT = 3;

const int potPins[NUM_POT] = {A0, A1, A2};

int potRead[NUM_POT][10];

long potTotal[NUM_POT] = {0};

int potValue[NUM_POT] = {0};
void updatePot() {
  for (int potIndex = 0; potIndex < NUM_POT; potIndex++) {
    for (int readIndex = 9; readIndex > 0; readIndex--) {
      potRead[potIndex][readIndex] = potRead[potIndex][potIndex - 1];
    }
    
    potRead[potIndex][0] = analogRead(potPins[potIndex]);
    
    potTotal[potIndex] = 0;
    for (int readIndex = 0; readIndex < 10; readIndex++) {
      potTotal[potIndex] += potRead[potIndex][readIndex];
    }
    
    potValue[potIndex] = potTotal[potIndex] / 10;
  }
}

void printPot() {
  Serial.print("Pot: ");
  for (int i = 0; i < NUM_POT; i++) {
    Serial.print(potValue[i]);
    Serial.print(" ");
  }
  Serial.println();
}


void setup() {
  Serial.begin(9600);
  
  analogReference(INTERNAL);
  
  for (int i = 0; i < NUM_POT; i++) {
    pinMode(potPins[i], INPUT);
  }
}
