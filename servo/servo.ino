#include<Servo.h>

Servo serv;

void setup() {
  serv.attach(9);
}

void loop() {

  
  serv.write(0);
  delay(1000);
  serv.write(360);
  delay(1000);
}