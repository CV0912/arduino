#include <Servo.h>
const int triggerPin=12;
const int echoPin=8;
float duration;
float distance;
const int motorpin =5;
const int irpin =3;
Servo servo1;
void setup()
{
pinMode(triggerPin, OUTPUT);
pinMode(echoPin,INPUT);
pinMode(irpin,INPUT);
servo1.attach(motorpin);

Serial.begin(9600);
}
void Ultrasonic()
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
}
void Sensor()
{
    int sensorvalue = digitalRead(irpin);
    if(sensorvalue == LOW)
    {
      Serial.println("Object detected");
      Ultrasonic();  
    }
    else
    {
      Serial.println("No object");
    }
}
void servo()
{
  servo1.write(360);
  }
void loop()
{
servo();
}

  
