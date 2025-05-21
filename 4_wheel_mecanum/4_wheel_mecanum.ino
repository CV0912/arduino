#include <Wire.h>
#include <math.h>
const int motorfrontdir1 = 31;
const int motorfrontspeed1 = 8;
const int motorfrontdir2 = 29;
const int motorfrontspeed2 = 5;
const int motorbackdir1 = 35;
const int motorbackspeed1 = 6;
const int motorbackdir2 = 33;
const int motorbackspeed2 = 7;
const int deadzone = 40;
int outputA =2;
int outputB=3;
int counter = 0;
int aState,bState;
int ppr = 400*4;
unsigned long lastTime;
int aLastState, bLastState;
int lastcount;
double dcount;
int8_t xL, xR, yL, yR, circle;
double rL, thetaL, rR, thetaR;

double wfl, wfr, wrl, wrr, wz;
double lx = 0.13, ly = 0.25;
const double r = 0.075;
double target_wfl = 0, target_wfr = 0, target_wrl = 0, target_wrr = 0;
double current_wfl = 0, current_wfr = 0, current_wrl = 0, current_wrr = 0;
const double decelerationRate = 5.0; // Adjust this for smoother or faster deceleration
volatile long count1 = 0;
volatile long count2 = 0;
volatile long count3 = 0;
volatile long count4 = 0;

void compute() {
  Serial.print(xL);
  Serial.print(" ");
  Serial.print(yL);
  Serial.println(" ");

  // Calculate joystick-based velocities
  double vx = yL;
  double vy = xL;
  wz = xR;
  double speedFactorL = (rL/88+rR/88)/2;
  
  // Compute target angular velocities for each wheel
  target_wfr = (1/r) * (vx - vy + (lx + ly) * wz) * speedFactorL;
  target_wfl = (1/r) * (vx + vy - (lx + ly) * wz) * speedFactorL;
  target_wrr = (1/r) * (vx + vy + (lx + ly) * wz) * speedFactorL;
  target_wrl = (1/r) * (vx - vy - (lx + ly) * wz) *
  speedFactorL;

  // Constrain target wheel speeds to PWM range
  target_wfr = constrain(target_wfr, -120, 120);
  target_wfl = constrain(target_wfl, -120, 120);
  target_wrr = constrain(target_wrr, -120, 120);
  target_wrl = constrain(target_wrl, -120, 120);
}

void applyDeceleration() {
  // Gradually approach target speeds for each wheel
  current_wfl += (target_wfl - current_wfl) / decelerationRate;
  current_wfr += (target_wfr - current_wfr) / decelerationRate;
  current_wrl += (target_wrl - current_wrl) / decelerationRate;
  current_wrr += (target_wrr - current_wrr) / decelerationRate;

  // Update motor directions and speeds based on decelerated values
  if (current_wfl >= 0) {
    digitalWrite(motorfrontdir1, HIGH);
    analogWrite(motorfrontspeed1, current_wfl);
  } else {
    digitalWrite(motorfrontdir1, LOW);
    analogWrite(motorfrontspeed1, -current_wfl);
  }

  if (current_wfr >= 0) {
    digitalWrite(motorfrontdir2, HIGH);
    analogWrite(motorfrontspeed2, current_wfr);
  } else {
    digitalWrite(motorfrontdir2, LOW);
    analogWrite(motorfrontspeed2, -current_wfr);
  }

  if (current_wrl >= 0) {
    digitalWrite(motorbackdir1, HIGH);
    analogWrite(motorbackspeed1, current_wrl);
  } else {
    digitalWrite(motorbackdir1, LOW);
    analogWrite(motorbackspeed1, -current_wrl);
  }

  if (current_wrr >= 0) {
    digitalWrite(motorbackdir2, HIGH);
    analogWrite(motorbackspeed2, current_wrr);
  } else {
    digitalWrite(motorbackdir2, LOW);
    analogWrite(motorbackspeed2, -current_wrr);
  }
}

void setup() {
  pinMode(motorfrontdir1, OUTPUT);
  pinMode(motorfrontspeed1, OUTPUT);
  pinMode(motorfrontdir2, OUTPUT);
  pinMode(motorfrontspeed2, OUTPUT);
  pinMode(motorbackdir1, OUTPUT);
  pinMode(motorbackspeed1, OUTPUT);
  pinMode(motorbackdir2, OUTPUT);
  pinMode(motorbackspeed2, OUTPUT);
  Wire.begin(8);
  Wire.onReceive(receiveEvent);
  pinMode(outputA, INPUT_PULLUP);
  pinMode(outputB, INPUT_PULLUP);
  Serial.begin(9600);

  aLastState = digitalRead(outputA);
  bLastState = digitalRead(outputB);

  attachInterrupt(digitalPinToInterrupt(outputA), readEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(outputB), readEncoderB, CHANGE);
  Serial.begin(115200);
}
void receiveEvent(int bytes) {
  while (Wire.available()) {
    xL = Wire.read();
    yL = Wire.read();
    xR = Wire.read();
    yR = Wire.read();
    wz = xR;
   if (abs(xL) < deadzone) xL = 0;
else xL = (xL > 0) ? xL - deadzone : xL + deadzone;

if (abs(yL) < deadzone) yL = 0;
else yL = (yL > 0) ? yL - deadzone : yL + deadzone;

if (abs(xR) < deadzone) xR = 0;
else xR = (xR > 0) ? xR - deadzone : xR + deadzone;

if (abs(yR) < deadzone) yR = 0;
else yR = (yR > 0) ? yR - deadzone : yR + deadzone;
    circle = Wire.read();
    rL = sqrt(xL * xL + yL * yL);
    thetaL = atan2(xL, yL);
    rR = sqrt(xR * xR + yR * yR);
    thetaR = atan2(yR, xR);
  }
}
void encoder()
{
  unsigned long now = millis();
  double timeChange = (double)(now-lastTime)/1000;
  if (timeChange > 0) { // Avoid division by zero
    dcount = (double)(counter - lastcount) / timeChange;
    double velocity = (2*dcount*3.14)/(ppr*timechange);
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
void loop() {
  compute();
  applyDeceleration();
  encoder();
  delay(100);
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
