const int a = 2;
const int b = 3;
const int motordir = 8;
const int motorspeed = 9;
volatile long temp, counter = 0;
long previousMillis = 0;
const long interval = 10000;
bool clockwise = true;
void motor()
{
  long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;  
    clockwise = !clockwise;          
  }
  if (clockwise) {
    digitalWrite(motordir, LOW);    
    analogWrite(motorspeed, 100);   
  } else {
    digitalWrite(motordir, HIGH);   
    analogWrite(motorspeed, 100);   
  }
}
void setup() {
  pinMode(a, INPUT); 
  pinMode(b, INPUT);
  pinMode(motordir, OUTPUT);
  pinMode(motorspeed, OUTPUT);
  attachInterrupt(0, ai0, RISING);
  attachInterrupt(1, ai1, RISING);
  Serial.begin(9600);
  
  }
   
  void loop() {
  motor();
  if(counter != temp )
  {
    Serial.println(counter);
    
    temp = counter;
  }
  }
   
  void ai0() {
  if (digitalRead(a) == digitalRead(b)) {
    counter++;
  } else {
    counter--;
  }
  }
   
  void ai1() {
  if (digitalRead(a) != digitalRead(b)) {
    counter++;
  } else {
    counter--;
  }
}
