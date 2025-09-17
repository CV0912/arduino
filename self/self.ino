#include <Wire.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define pi 3.1415926535
// Left Motor
const int LEFT_PWM_CHANNEL = 26;   // PWM channel for left motor
const int LEFT_DIR_PIN = 25;  // Direction pin for left motor
#define BNO055_SAMPLERATE_DELAY_MS (10)
//variables
double rolltarget=-24;
double rollerror,lasterr;
// double ki=0,kp=15,kd=9.2;
double ki=0,kp=16,kd=2.918;
double roll;
float errsum;
float signalOutput;
Adafruit_BNO055 myIMU = Adafruit_BNO055();
float q0,q1,q2,q3,yaw=0;
// Right Motor
const int RIGHT_PWM_CHANNEL = 13;  // PWM channel for right motor
const int RIGHT_DIR_PIN = 9;  // Direction pin for right motor
const int b = 106;
const int r = 65;
int lx, ly, rx, ry, tri, cir, squ, cro, l2, r2,l1,r1,left,right,up,down;
int buff = 40;
double targetwl,targetwr;
int sensorValue;
float Volt;
float Distance;
int sensor_pin = 3;
unsigned long lasttime;
void setup() {
  Serial.begin(115200);
  pinMode(LEFT_PWM_CHANNEL, OUTPUT);
  pinMode(RIGHT_PWM_CHANNEL, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  myIMU.begin();
  int8_t temp=myIMU.getTemp();
  myIMU.setExtCrystalUse(true);
  lasttime = millis(); 
  // Initialize motors to stopped state
  // stopMotors();
}
void computepid()
{
  unsigned long now = millis();
  float timeDelta =(now - lasttime)/1000.0;
  rollerror=rolltarget-roll;
  errsum+=rollerror*timeDelta;
  double derr = (rollerror-lasterr)/timeDelta;
  signalOutput = kp*rollerror+ki*errsum+kd*derr;
  signalOutput = constrain(signalOutput, -255, 255);
  signalOutput=abs(signalOutput);
  lasterr = rollerror;
  lasttime = now;
}
void loop() {
  uint8_t system, gryo, accel, mg = 0;
  imu::Quaternion quat=myIMU.getQuat();
  imu::Vector<3> acc =myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  q0=quat.w();
  q1=quat.x();
  q2=quat.y();
  q3=quat.z();
  float rolla = atan2(acc.y(), acc.z()) * 180.0 / PI;
  roll = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
  roll = roll * (180.0 / pi);
  Serial.print("ROLL = ");
  Serial.print(rolltarget);
  computepid();
  Serial.print(" ");
  Serial.println(roll);  

  if(rollerror>1)
  {
    forward(signalOutput);
  }
  else if(rollerror<-1)
  {
    backward(signalOutput);
  }
  else
  {
    stopMotors();
  }
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
// Motor control functions
void forward(int speed) {
  // Set direction pins
  digitalWrite(LEFT_DIR_PIN, LOW);    // Left motor backward
  digitalWrite(RIGHT_DIR_PIN, LOW);  // Right motor forward

  // Set speed
  analogWrite(LEFT_PWM_CHANNEL, speed);
  analogWrite(RIGHT_PWM_CHANNEL, speed);
  Serial.print("A");
}
void backward(int speed) {
  // Set direction pins
  digitalWrite(LEFT_DIR_PIN, HIGH);    // Left motor backward
  digitalWrite(RIGHT_DIR_PIN, HIGH);  // Right motor forward

  // Set speed
  analogWrite(LEFT_PWM_CHANNEL, speed);
  analogWrite(RIGHT_PWM_CHANNEL, speed);
  Serial.print("B");
}
void stopMotors() {
  // Set speed to zero
  analogWrite(LEFT_PWM_CHANNEL, 0);
  analogWrite(RIGHT_PWM_CHANNEL, 0);
}