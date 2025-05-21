#include <Wire.h>
#include <math.h>
const int motorfrontdir1 = 6;
const int motorfrontspeed1 =35;
const int motorbackdir1 = 31;
const int motorbackspeed1 =33;
const int motorbackdir2 = 27;
const int motorbackspeed2 = 37;
const int deadzone = 20;
int8_t xL,xR,yL,yR,circle;
double rL,thetaL,rR,thetaR;
double w1,w2,w3;
int v = 50;
const double r = 153/2000;
void clockwise()
{
  digitalWrite(motorfrontdir1, LOW);    
  analogWrite(motorfrontspeed1,200);
  digitalWrite(motorbackdir1,LOW);
  analogWrite(motorbackspeed1,200);
  digitalWrite(motorbackdir2,HIGH);
  analogWrite(motorbackspeed2,200);
}
void anticlockwise()
{
  digitalWrite(motorfrontdir1, HIGH);    
  analogWrite(motorfrontspeed1,200);
  digitalWrite(motorbackdir1,HIGH);
  analogWrite(motorbackspeed1,200);
  digitalWrite(motorbackdir2,LOW);
  analogWrite(motorbackspeed2,200);
}
void compute()
{
  // Maximum possible speed (adjust as needed)
  int maxSpeed = 200;

  // Compute each wheel's speed based on the joystick input angle thetaL and magnitude rL
  w1 = 100*cos(240-0); // 240 degrees in radians
  w2 = 100*cos(120-0); // 120 degrees in radians
  w3 = 100*cos(0-0);   // 0 degrees in radians
  // Motor control with PWM and direction based on wheel speeds
  Serial.println(w1);
  Serial.println(w2);
  if (w1 >= 0) {
    digitalWrite(motorbackdir2, HIGH);
    analogWrite(motorbackspeed2, w1);
  } else {
    digitalWrite(motorbackdir2, LOW);
    analogWrite(motorbackspeed2, -w1);
  }

  if (w2 >= 0) {
    digitalWrite(motorbackdir1, HIGH);
    analogWrite(motorbackspeed1, w2);
  } else {
    digitalWrite(motorbackdir1, LOW);
    analogWrite(motorbackspeed1, -w2);
  }

  if (w3 >= 0) {
    digitalWrite(motorfrontdir1, HIGH);
    analogWrite(motorfrontspeed1, w3);
  } else {
    digitalWrite(motorfrontdir1, LOW);
    analogWrite(motorfrontspeed1, -w3);
  }
}


void setup() {
  pinMode(motorfrontdir1, OUTPUT);
  pinMode(motorfrontspeed1, OUTPUT);
  pinMode(motorbackdir1, OUTPUT);
  pinMode(motorbackspeed1, OUTPUT);
  pinMode(motorbackdir2, OUTPUT);
  pinMode(motorbackspeed2, OUTPUT);
  Wire.begin(8);
  Wire.onReceive(receiveEvent);
  Serial.begin(115200);  
  }
void receiveEvent(int bytes) {
  Serial.println("recieved");
  while (Wire.available()) {
    xL = Wire.read();
    yL = Wire.read();
    xR = Wire.read();
    yR = Wire.read();
    if (abs(xL) < deadzone) xL = 0;
    if (abs(yL) < deadzone) yL = 0;
    if (abs(xR) < deadzone) xR = 0;
    if (abs(yR) < deadzone) yR = 0;
    circle = Wire.read();   
    rL=sqrt(xL*xL+yL*yL);
    thetaL=atan2(xL,yL);
    rR=sqrt(xR*xR+yR*yR);
    thetaR=atan2(yR,xR);
    Serial.print(rL);
    Serial.print(" ");
    Serial.print(thetaL);
    Serial.println(" ");
    
    
  }
}
void loop() {
  delay(100);
  compute();
  if(xR<-90 && -15<yR && yR<15)
  {
    anticlockwise();
//    Serial.println("anticlockwise");
  }
  else if(xR>90 && -15<yR && yR<15)
  {
    clockwise();
//    Serial.println("clockwise");
  }
}
   
