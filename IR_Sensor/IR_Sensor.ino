const int irpin =3;
void setup() {
  pinMode(irpin,INPUT);
  Serial.begin(9600);
}
int IRsensor()
{
    int sensorvalue = digitalRead(irpin);
    if(sensorvalue == LOW)
    {
      Serial.println("Object detected");
      return 1;  
    }
    else
    {
      Serial.println("No object");
      return 0;
    }
}
void loop() {
  IRsensor();

}
