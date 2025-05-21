void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int sensorValue = analogRead(A0);
  float Volt = sensorValue * (5.0 / 1023.0);
  float Distance = 12.08 *pow(Volt , -1.058);
  if(Distance<30)
  Serial.println(Distance);
}
