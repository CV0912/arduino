int dir = 8;
int pwm = 9;

const int speed = 1000;
void setup() {
  // put your setup code here, to run once:
  pinMode(dir,OUTPUT);
  pinMode(pwm,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  int pwmSpeed = map(speed,0,3500,0,255);
  digitalWrite(dir,LOW);
  analogWrite(pwm,pwmSpeed);
}
