volatile int counter = 0;
int degree = 0;
int aState, bState;
int aLastState, bLastState;
const int encoderPPR = 400*4;

void readEncoderA() {
  aState = digitalRead(2);
  bState = digitalRead(3);
  

  if (aState != aLastState) {
    if (aState == bState) {
      counter++;
    } else {
      counter--;
    }
    aLastState = aState;
  }
  
}

void readEncoderB() {
  aState = digitalRead(2);
  bState = digitalRead(3);

  if (bState != bLastState) {
    if (aState != bState) {
      counter++;
    } else {
      counter--;
    }
    bLastState = bState;
  }
}

void setup() {
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  Serial.begin(9600);

  aLastState = digitalRead(2);
  bLastState = digitalRead(3);

  attachInterrupt(0, readEncoderA, CHANGE);
  attachInterrupt(1, readEncoderB, CHANGE);
}

void loop() {
  Serial.println(counter);
  delay(100);
}
