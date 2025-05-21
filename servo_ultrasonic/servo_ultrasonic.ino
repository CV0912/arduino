#include <Servo.h>
const int triggerPin=12;
const int echoPin=8;
float duration;
float distance;
const int motorpin =4;
Servo servo1;
void setup()
{
pinMode(triggerPin, OUTPUT);
pinMode(echoPin,INPUT);
servo1.attach(motorpin);
servo1.write(0);

Serial.begin(9600);
}
float Ultrasonic()
{
digitalWrite(triggerPin,LOW);
delay(2);
digitalWrite(triggerPin,HIGH);
delay(10);
digitalWrite(triggerPin,LOW);
duration = pulseIn(echoPin,HIGH);
distance = (duration * 0.0343)/2;
Serial.print("Distance: ");
Serial.println(distance);
delay(100);
return distance;
}
void servo()
{
  if(!servo1.attached())
  {
    servo1.attach(motorpin);
    }
  servo1.write(360);
  }
void servoleft()
{
  if(!servo1.attached())
  {
    servo1.attach(motorpin);
    }
  servo1.write(0);
  delay(1000);
  }
void loop()
{
float d = Ultrasonic();
if(d<4)
{
  servo();
  delay(500);
  }
  else if(d>4 && d<8)
  {
    servoleft();
    delay(500);
    }
else
{
  servo1.detach();
  }
}

  
