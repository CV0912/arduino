int outputA =2;
int outputB=3;
int counter = 0;
int aState,bState;
int ppr = 400*4;
unsigned long lastTime;
int aLastState, bLastState;
int lastcount;
double dcount;
void setup() {
  pinMode(outputA, INPUT_PULLUP);
  pinMode(outputB, INPUT_PULLUP);
  Serial.begin(115200);

  aLastState = digitalRead(outputA);
  bLastState = digitalRead(outputB);

  attachInterrupt(digitalPinToInterrupt(outputA), readEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(outputB), readEncoderB, CHANGE);

}
void loop() {
  unsigned long now = millis();
  double velocity;
  double timeChange = (double)(now-lastTime)/1000;
  if (timeChange > 0) { // Avoid division by zero
    dcount = (double)(counter - lastcount);
    velocity = (2*dcount*3.14)/(ppr*timeChange);
  }
  lastTime = now;
  lastcount = counter;
  Serial.print("Counter: ");
  Serial.print(counter);
  Serial.print("\tVelocity (counts/s): ");
  Serial.println(dcount);
    Serial.print("\tVelocity (rad/s): ");
  Serial.println(velocity);

}

void readEncoderA() {
  aState = digitalRead(outputA);
  bState = digitalRead(outputB);

  if (aState != aLastState) {
    if (aState == bState) {
      counter--;
    } else {
      counter++;
    }
    aLastState = aState;
  }
  
}

void readEncoderB() {
  aState = digitalRead(outputA);
  bState = digitalRead(outputB);

  if (bState != bLastState) {
    if (aState != bState) {
      counter--;
    } else {
      counter++;
    }
    bLastState = bState;
  }
}
