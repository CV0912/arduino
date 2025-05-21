const int triggerPin =12;
const int echoPin =8;
float duration;
float distance;
void setup() {
  pinMode(triggerPin,OUTPUT);
  pinMode(triggerPin,INPUT);
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
void loop() {
  Ultrasonic();

}
