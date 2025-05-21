const int pinA = 20;
const int pinB = 21;

volatile int encoderPos = 0;
volatile int lastState = LOW;

void setup() {
    Serial.begin(9600);
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(pinA), updateEncoder, CHANGE);
}

void loop() {
    Serial.print("Encoder Position: ");
    Serial.println(encoderPos);
    delay(100);
}

void updateEncoder() {
    int stateA = digitalRead(pinA);
    int stateB = digitalRead(pinB);

    if(stateA == stateB){
      encoderPos++;
    }
    else if(stateA != stateB){
      encoderPos--;
    }
    // if (state != lastState) {
    //     if (stateB == LOW) {
    //         encoderPos++;
    //     } else {
    //         encoderPos--;
    //     }
    // }
    // lastState = state;
}
