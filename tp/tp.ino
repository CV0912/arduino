
#include <Wire.h>
const int motorleftdir = 4;
const int motorleftspeed = 5;
const int motorrightdir = 6;
const int motorrightspeed =7;
int8_t xL,xR,yL,yR,circle;
void front()
{
  digitalWrite(motorleftdir, LOW);    
  analogWrite(motorleftspeed, 100);
  digitalWrite(motorrightdir,LOW);
  analogWrite(motorrightspeed,100);
}
void back()
{
  
  digitalWrite(motorleftdir, HIGH);    
  analogWrite(motorleftspeed, 100);
  digitalWrite(motorrightdir,HIGH);
  analogWrite(motorrightspeed,100);
}
void left()
{

}
void right()
{

}
void setup() {
  pinMode(motorleftdir, OUTPUT);
  pinMode(motorleftspeed, OUTPUT);
  pinMode(motorrightdir, OUTPUT);
  pinMode(motorrightspeed, OUTPUT);
  
  Wire.begin(8);
  Wire.onReceive(receiveEvent);
  Serial.begin(115200);  
  }
void receiveEvent(int bytes) {
  while (Wire.available()) {
    xL = Wire.read();
    yL = Wire.read();
    xR = Wire.read();
    yR = Wire.read();
    circle = Wire.read();   
  }
}
void loop() {
  delay(100);
  if(yL>90 && -15<xL && xL<15)
  {
    front();
  }
  else if(yL<-90 && -15<xL && xL<15)
  {
    back();
  }
  else if(xL<-90 && -15<yL && yL<15)
  {
    left();
  }
  else if(xL>90 && -15<yL && yL<15)
  {
    right();
  }
  else
  {
    analogWrite(motorleftspeed,0);
    analogWrite(motorrightspeed,0);
  }
}
    
 
  
