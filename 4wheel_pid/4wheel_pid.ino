#include <Wire.h>
#include <math.h>
//motor pins
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
double velocityrr,velocityrl,velocityfl,velocityfr;
//encoder pins
int outputA =2;//rr
int outputB=10;
int outputA1=19,outputB1=13;//fl
int outputA2=18,outputB2=11;//fr
int outputA3=3,outputB3=12;//rl
//encoder outputs
int counter = 0,counter1=0,counter2=0,counter3=0;
//encoder variables
int aState,bState;
int aState1,bState1;
int aState2,bState2;
int aState3,bState3;
int ppr = 400;
int aLastState, bLastState;
int aLastState1,bLastState1;
int aLastState2,bLastState2;
int aLastState3,bLastState3;
int lastcount,lastcount1,lastcount2,lastcount3;
double dcount,dcount1,dcount2,dcount3;
//pid
double errsum,errsum1,errsum2,errsum3;
double error,error1,error2,error3,lasterr1,lasterr2,lasterr3,lasterr;
double kp,ki,kd;
double fvrr,fvrl,fvfr,fvfl;
int output,output1,output2,output3;
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
double v1prev,v2prev,v3prev,v4prev;
void compute() {
  unsigned long now = millis();
  double timeChange = (double)(now - lastTime) / 1000;

  if (timeChange > 0) {
    // Calculate velocities from encoders
    dcount = (double)(counter - lastcount) / timeChange;
    velocityrr = (2 * dcount) / (ppr * 17) * 60;
    fvrr = 0.854 * fvrr + 0.0728 * velocityrr + 0.0728 * v1prev;
    v1prev = velocityrr;

    dcount1 = (double)(counter1 - lastcount1) / timeChange;
    velocityfl = (2 * dcount1) / (ppr * 17) * 60;
    fvfl = 0.854 * fvfl + 0.0728 * velocityfl + 0.0728 * v2prev;
    v2prev = velocityfl;

    dcount2 = (double)(counter2 - lastcount2) / timeChange;
    velocityfr = (2 * dcount2) / (ppr * 17) * 60;
    fvfr = 0.854 * fvfr + 0.0728 * velocityfr + 0.0728 * v3prev;
    v3prev = velocityfr;

    dcount3 = (double)(counter3 - lastcount3) / timeChange;
    velocityrl = (2 * dcount3) / (ppr*17) * 60;
    fvrl = 0.854 * fvrl + 0.0728 * velocityrl + 0.0728 * v4prev;
    v4prev = velocityrl;

    lastcount = counter;
    lastcount1 = counter1;
    lastcount2 = counter2;
    lastcount3 = counter3;
    lastTime = now;
  }

  // Print velocities for debugging
  Serial.print("Velocity rr: "); Serial.println(velocityrr);
  Serial.print("Velocity fl: "); Serial.println(velocityfl);
  Serial.print("Velocity rl: "); Serial.println(velocityrl);
  Serial.print("Velocity fr: "); Serial.println(velocityfr);

  // Joystick inputs and target velocities
  double vx = yL;
  double vy = xL;
  wz = xR;
  double speedFactorL = (rL / 88 + rR / 88) / 2;

  target_wfr = (1 / r) * (vx - vy + (lx + ly) * wz) * speedFactorL;
  target_wfl = (1 / r) * (vx + vy - (lx + ly) * wz) * speedFactorL;
  target_wrr = (1 / r) * (vx + vy + (lx + ly) * wz) * speedFactorL;
  target_wrl = (1 / r) * (vx - vy - (lx + ly) * wz) * speedFactorL;

  // PID Control
  error = target_wrr - fvrr;
  error1 = target_wfl - fvfl;
  error2 = target_wfr - fvfr;
  error3 = target_wrl - fvrl;
  double derr = (error-lasterr)/timeChange;
  double derr1 = (error1-lasterr1)/timeChange;
  double derr2 = (error2-lasterr2)/timeChange;
  double derr3 = (error3-lasterr3)/timeChange;
  errsum += error * timeChange;
  errsum1 += error1 * timeChange;
  errsum2 += error2 * timeChange;
  errsum3 += error3 * timeChange;

  output = kp * error + ki * errsum+kd*derr;
  output1 = kp * error1 + ki * errsum1+kd*derr1;
  output2 = kp * error2 + ki * errsum2+kd*derr2;
  output3 = kp * error3 + ki * errsum3+kd*derr3;

  // Constrain outputs
  target_wrr = constrain(output, -100, 100);
  target_wfl = constrain(output1, -100, 100);
  target_wfr = constrain(output2, -100, 100);
  target_wrl = constrain(output3, -100, 100);

  // Set motor directions and speeds
  setMotor((target_wrr > 0) ? 1 : -1, abs(target_wrr), motorbackspeed2, motorbackdir2, abs(target_wrr) < 1 && abs(fvrr) < 1);
  setMotor((target_wfl > 0) ? 1 : -1, abs(target_wfl), motorfrontspeed1, motorfrontdir1, abs(target_wfl) < 1 && abs(fvfl) < 1);
  setMotor((target_wfr > 0) ? 1 : -1, abs(target_wfr), motorfrontspeed2, motorfrontdir2, abs(target_wfr) < 1 && abs(fvfr) < 1);
  setMotor((target_wrl > 0) ? 1 : -1, abs(target_wrl), motorbackspeed1, motorbackdir1, abs(target_wrl) < 1 && abs(fvrl) < 1);
  lasterr=error;
  lasterr1=error1;
  lasterr2=error2;
  lasterr3=error3;
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
  attachInterrupt(digitalPinToInterrupt(outputA1), readEncoderA1, RISING);
  attachInterrupt(digitalPinToInterrupt(outputA2), readEncoderA2, RISING);
  attachInterrupt(digitalPinToInterrupt(outputA3), readEncoderA3, RISING);
  Serial.begin(115200);
}
void receiveEvent(int bytes) {
  while (Wire.available()) {
    xL = Wire.read();
    yL = Wire.read();
    xR = Wire.read();
    yR = Wire.read();
    Serial.print(xL);
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
  SetTunings(1,0.00069,0);
  compute();
  //applyDeceleration();
  Serial.print("counter = ");
  Serial.println(error);
    Serial.print("counter1 = ");
  Serial.println(error1);
    Serial.print("counter2 = ");
  Serial.println(error2);
    Serial.print("counter3 = ");
  Serial.println(error3);
  delay(100);
}
void readEncoderA() {
  
  if (digitalRead(outputA) == digitalRead(outputB))
  {
  counter++;
  }
  else
  {
  counter--;
  }
}

void readEncoderA1() {
  
  if (digitalRead(outputA1) == digitalRead(outputB1))
  {
  counter1++;
  }
  else
  {
  counter1--;
  }
}
void readEncoderA2() {
  
  if (digitalRead(outputA2) == digitalRead(outputB2))
  {
  counter2++;
  }
  else
  {
  counter2--;
  }
}

void readEncoderA3() {
  
  if (digitalRead(outputA3) == digitalRead(outputB3))
  {
  counter3++;
  }
  else
  {
  counter3--;
  }
}
void setMotor(int dir, int pwmVal, int pwm, int dirPin, bool isStopping) {
  if (isStopping) {
    analogWrite(pwm, pwmVal);
  } else {
    // Change direction based on `dir`
    if (dir == 1) {
      digitalWrite(dirPin, HIGH); // Forward
    } else if (dir == -1) {
      digitalWrite(dirPin, LOW); // Reverse
    }
    analogWrite(pwm, pwmVal); // Set speed
  }
}
