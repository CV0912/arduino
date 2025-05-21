

#include <Servo.h>

Servo myservo;
Servo myservo1;
Servo myservo2;



void setup() {
  myservo.attach(6); 
  myservo1.attach(7); 
  myservo2.attach(11); 
  
}

void loop() {
    myservo.write(90);
    delay(1000);
    myservo1.write(90);
    delay(1000);
    myservo2.write(90);
    delay(1000);
    myservo.write(180);
    delay(1000);
    myservo1.write(180);
    delay(1000);
    myservo2.write(180);
    delay(1000);
}
