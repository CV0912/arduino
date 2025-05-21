#include <Wire.h>
#include <math.h>
//motor pin
const int motorfrontdir1 = 31;
const int motorfrontspeed1 = 8;
const int motorfrontdir2 = 29;
const int motorfrontspeed2 = 5;
const int motorbackdir1 = 35;
const int motorbackspeed1 = 6;
const int motorbackdir2 = 33;
const int motorbackspeed2 = 7;
const int deadzone = 40;
//motor velocity  
double velocity,velocity1,velocity2,velocity3;
//encoder pins
int outputA =2;
int outputB=3;
int outputA1=18,outputB1=19;
int outputA2=0,outputB2=0;
int outputA3=0,outputB3=0;
//encoder outputs
int counter = 0,counter1=0,counter2=0,counter3=0;
//encoder variables
int aState,bState;
int aState1,bState1;
int aState2,bState2;
int aState3,bState3;
int ppr = 400*4;
int aLastState, bLastState;
int aLastState1,bLastState1;
int aLastState2,bLastState2;
int aLastState3,bLastState3;
int lastcount,lastcount1,lastcount2,lastcount3;
double dcount,dcount1,dcount2,dcount3;
//pid
double errsum,errsum1;
double error;
double kp,ki,kd;
//controller input
int8_t xL, xR, yL, yR, circle;
double rL, thetaL, rR, thetaR;
unsigned long lastTime;
double wfl, wfr, wrl, wrr, wz;
double lx = 0.13, ly = 0.25;
const double r = 0.075;
double target_wfl = 0, target_wfr = 0, target_wrl = 0, target_wrr = 0;
double current_wfl = 0, current_wfr = 0, current_wrl = 0, current_wrr = 0;
const double decelerationRate = 2.0; // Adjust this for smoother or faster deceleration
void compute() {
  unsigned long now = millis();
  double lasterr,lasterr1;
  //calculating encoder velocity
  double timeChange = (double)(now-lastTime)/1000;
  if (timeChange > 0)
  //reverify this part of the code
  { // Avoid division by zero
    dcount = (double)(counter - lastcount) / timeChange;
    velocity = (2*dcount*3.14*r)/(ppr*timeChange) ;
    dcount1 = (double)(counter1 - lastcount1) / timeChange;
    velocity1 = (2*dcount1*3.14*r)/(ppr*timeChange);
    dcount2 = (double)(counter2 - lastcount2) / timeChange;
    velocity2 = (2*dcount2*3.14*r)/(ppr*timeChange);
    dcount3 = (double)(counter3 - lastcount3) / timeChange;
    velocity3 = (2*dcount3*3.14*r)/(ppr*timeChange);
  }
  lastcount = counter;
  lastcount1 = counter1;
  lastcount2 = counter2;
  lastcount3 = counter3;
  lastTime = now;
  Serial.print("\tVelocity fl (rad/s): ");
  Serial.println(velocity);  
  Serial.print("\tVelocity rr (rad/s): ");
  Serial.println(velocity1);
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
  target_wrl = (1/r) * (vx - vy - (lx + ly) * wz) * speedFactorL;
  //propotional integral derivative or pid controller
  error = target_wfl - velocity;
  errsum+= error*timeChange;
  double derr = (error-lasterr)/timeChange;
  double output = kp * error + ki * errsum + kd*derr;
  double error1 = target_wrr - velocity1;
  errsum1+= error1*timeChange;
  double derr1 = (error1-lasterr1)/timeChange;
  double output1 = kp * error + ki * errsum + kd*derr;
  target_wfr = constrain(target_wfr, -120, 120);
  target_wfl = constrain(target_wfl, -120, 120);
  target_wrr = constrain(target_wrr, -120, 120);
  target_wrl = constrain(target_wrl, -120, 120);
  lasterr = error;
  lasterr1=error1;
  
}
void applyDeceleration() {
  // Gradually approach target speeds for each wheel
  current_wfl+= (target_wfl - current_wfl) / decelerationRate;
  current_wfr+= (target_wfr - current_wfr) / decelerationRate;
  current_wrl+= (target_wrl - current_wrl) / decelerationRate;
  current_wrr+= (target_wrr - current_wrr) / decelerationRate;
  
  // Update motor directions and speeds based on decelerated values
  if (current_wfl >= 0) {
    digitalWrite(motorfrontdir1, LOW);
    analogWrite(motorfrontspeed1, current_wfl);
  } else {
    digitalWrite(motorfrontdir1, HIGH);
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
    digitalWrite(motorbackdir2, LOW);
    analogWrite(motorbackspeed2, current_wrr);
  } else {
    digitalWrite(motorbackdir2, HIGH);
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
  aLastState = digitalRead(outputA);
  bLastState = digitalRead(outputB);
  pinMode(outputA1, INPUT_PULLUP);
  pinMode(outputB1, INPUT_PULLUP);
  aLastState1 = digitalRead(outputA1);
  bLastState1 = digitalRead(outputB1);
  pinMode(outputA2, INPUT_PULLUP);
  pinMode(outputB2, INPUT_PULLUP);
  aLastState2 = digitalRead(outputA2);
  bLastState2 = digitalRead(outputB2);
  pinMode(outputA3, INPUT_PULLUP);
  pinMode(outputB3, INPUT_PULLUP);
  aLastState3 = digitalRead(outputA3);
  bLastState3 = digitalRead(outputB3);
  attachInterrupt(digitalPinToInterrupt(outputA), readEncoderA, RISING);
  /*attachInterrupt(digitalPinToInterrupt(outputA1), readEncoderA1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(outputB1), readEncoderB1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(outputA2), readEncoderA2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(outputB2), readEncoderB2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(outputA3), readEncoderA3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(outputB3), readEncoderB3, CHANGE);*/
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
void SetTunings(double Kp,double Ki,double Kd)
{
  kp=Kp;
  ki=Ki;
  kd=Kd;
  }
void loop() {
  SetTunings(-0.75,-0.001,0);
  compute();
  applyDeceleration();
  delay(100);
}
void readEncoderA() {
  bState = digitalRead(outputB);

    if(bState != 0) {
      counter--;
    } 
    else {
      counter++;
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
void readEncoderA1() {
  aState1 = digitalRead(outputA1);
  bState1 = digitalRead(outputB1);

  if (aState1 != aLastState1) {
    if (aState1 == bState1) {
      counter1--;
    } else {
      counter1++;
    }
    aLastState1 = aState1;
  }
  
}

void readEncoderB1() {
  aState1 = digitalRead(outputA1);
  bState1 = digitalRead(outputB1);

  if (bState1 != bLastState1) {
    if (aState1 != bState1) {
      counter1--;
    } else {
      counter1++;
    }
    bLastState1 = bState1;
  }
}
void readEncoderA2() {
  aState2 = digitalRead(outputA2);
  bState2 = digitalRead(outputB2);

  if (aState2 != aLastState2) {
    if (aState2 == bState2) {
      counter2--;
    } else {
      counter2++;
    }
    aLastState2 = aState2;
  }
  
}

void readEncoderB2() {
  aState2 = digitalRead(outputA2);
  bState2 = digitalRead(outputB2);

  if (bState2 != bLastState2) {
    if (aState2 != bState2) {
      counter2--;
    } else {
      counter2++;
    }
    bLastState2 = bState2;
  }
}
void readEncoderA3() {
  aState3 = digitalRead(outputA3);
  bState3 = digitalRead(outputB3);

  if (aState3 != aLastState3) {
    if (aState3 == bState3) {
      counter3--;
    } else {
      counter3++;
    }
    aLastState3 = aState3;
  }
  
}

void readEncoderB3() {
  aState3 = digitalRead(outputA3);
  bState3 = digitalRead(outputB3);

  if (bState3 != bLastState3) {
    if (aState3 != bState3) {
      counter3--;
    } else {
      counter3++;
    }
    bLastState3 = bState3;
  }
}
