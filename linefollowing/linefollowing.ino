// Define the pin connections
const int irSensorPin = 7;  // IR sensor output pin connected to digital pin 7
const int irSensorPin1 = 8;    // LED connected to digital pin 13 (optional)
const int motorleftdir = 12;
const int motorleftpwm = 11;
const int motorrightdir = 13;
const int motorrightpwm = 10;

void setup() {
  pinMode(irSensorPin, INPUT);  // Set IR sensor pin as input
  pinMode(irSensorPin1, INPUT);
  pinMode(motorleftdir,OUTPUT);
  pinMode(motorleftpwm, OUTPUT);
  pinMode(motorrightdir,OUTPUT);
  pinMode(motorrightpwm, OUTPUT);
  Serial.begin(9600);// Begin serial communication for debugging
}
void forward()
{
  digitalWrite(motorleftdir,HIGH);
  digitalWrite(motorrightdir,LOW);
  analogWrite(motorleftpwm, 200);
  analogWrite(motorrightpwm, 200);

}
void turnLeft()
{
  digitalWrite(motorleftdir,HIGH);
  digitalWrite(motorrightdir,HIGH);
  analogWrite(motorleftpwm, 200);
  analogWrite(motorrightpwm, 200);

}
void turnRight()
{
  digitalWrite(motorleftdir,LOW);
  digitalWrite(motorrightdir,LOW);
  analogWrite(motorleftpwm, 200);
  analogWrite(motorrightpwm, 200);

}
void Stop()
{
  analogWrite(motorleftpwm, 0);
  analogWrite(motorrightpwm, 0);

}
void loop() {
  int L_S = digitalRead(irSensorPin);
  int R_S = digitalRead(irSensorPin1);  // Read the value from the IR sensor
  Serial.print(L_S);
  Serial.print(" ");
  Serial.println(R_S);
  // if((digitalRead(R_S) == 0) && (digitalRead(L_S) == 0))
  // {
  //   forward();
  // } 
  // if((digitalRead(R_S) == 1) && (digitalRead(L_S) == 0))
  // {
  //   turnRight();
  // } 
  // if((digitalRead(R_S) == 0) && (digitalRead(L_S) == 1))
  // {
  //     turnLeft();
  // }
  //  if((digitalRead(R_S) == 1) && (digitalRead(L_S) == 1))
  //  {
  //   Stop();
  //  }  
  delay(100);  // Small delay for stability
}