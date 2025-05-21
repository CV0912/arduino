const int motordir = 8;
const int motorspeed = 9;
int speedval = 0;
void setup() {
  // put your setup code here, to run once:
  pinMode(motordir, OUTPUT);
  pinMode(motorspeed, OUTPUT);
  Serial.begin(9600);
}
void clockwise()
{
  digitalWrite(motordir, LOW);
  analogWrite(motorspeed,100);
}
 void anticlockwise(float d)
{
  digitalWrite(motordir, HIGH);
  int s = constrain(255 / max(d,1),0,255);
  analogWrite(motorspeed,
  
  s);
}
int sensorLeft()
{
  int sensorValue = analogRead(A0);
  float Volt = sensorValue * (5.0 / 1023.0);
  float Distance = 12.08 *pow(Volt , -1.058);
  Serial.println(Distance);
  if(Distance<20)
  {
  anticlockwise(Distance);
  }
  else
  {
    digitalWrite(motorspeed,LOW);
    }
}
void loop() 
{
  clockwise();
}
