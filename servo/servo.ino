#include<Servo.h>

Servo serv;

void setup() {
  serv.attach(9);
}

void loop() {
  serv.write(90);
  delay(500);
   serv.write(180);
  delay(500);
   serv.write(0);
  delay(500);
}