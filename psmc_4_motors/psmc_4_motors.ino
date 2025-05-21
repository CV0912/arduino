#include <avr/wdt.h>
#define sign(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))
int outputA =2;
int outputB=3;
unsigned long lastTime;
double output,setpoint=5000;
double errsum,lasterr;
double kp,kd,c,kt,k;
double counter = 0;
int pwm =5;
int dir =4;
//int dir2=6;
//int pwm2=7;
int dir1=8;
int pwm1=9;
int degree=0;
const int encoderPPR = 550;
int aState,
bState;
int aLastState, bLastState;
double error;

void Compute()
{
  unsigned long now = millis();
  double timeChange = (double)(now-lastTime);
  error = setpoint - counter;
  double derr = (error-lasterr)/timeChange;
  float s = c * error+ derr;
  output = kp *error+ kd*derr + kt* (k * sign(s));
  output = constrain(output,-100,100);
  output = abs(output);
  lasterr=error;
  lastTime = now;
  Serial.print("Error :");
  Serial.println(error);
  }
void SetTunings(double Kp,double Kd,double Kt,double K)
{
  kp=Kp;
  kd=Kd;
  kt=Kt;
  k=K;
  }
void setup() {
  pinMode(outputA, INPUT_PULLUP);
  pinMode(outputB, INPUT_PULLUP);
  pinMode(pwm, OUTPUT);
  pinMode(dir, OUTPUT);
  pinMode(pwm1, OUTPUT);
  pinMode(dir1, OUTPUT);
  //pinMode(pwm2, OUTPUT);
  //pinMode(dir2, OUTPUT);
  Serial.begin(9600);

  aLastState = digitalRead(outputA);
  bLastState = digitalRead(outputB);

  attachInterrupt(digitalPinToInterrupt(outputA), readEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(outputB), readEncoderB, CHANGE);

}
void psmc(){
  if(counter<1000)
  {
    digitalWrite(dir,HIGH);
    analogWrite(pwm,output);
    digitalWrite(dir1,LOW);
    analogWrite(pwm1,output);
    //digitalWrite(dir2,HIGH);
    //analogWrite(pwm2,output);
    }
    
  else if(counter>1000 && counter<2000)
  {
    digitalWrite(dir,LOW);
    analogWrite(pwm,output);
    digitalWrite(dir1,LOW);
    analogWrite(pwm1,output);
    //digitalWrite(dir2,LOW);
    //analogWrite(pwm2,output);
  }
  else
  {
    analogWrite(pwm,0);
    analogWrite(pwm1,0);
    //analogWrite(pwm2,0);
  }
}
void loop() {
   if (Serial.available() > 0) {
        char command = Serial.read();
        if (command == 'r') { // Reset command
            ResetArduino();
        }
  }
  SetTunings(-0.69,0.4,15,-0.02);
  Compute();
  if(abs(error)>10)
  {
    psmc();
  }
  else
  {
    analogWrite(pwm,0);
    analogWrite(pwm1,0);
  }
  Serial.println(counter);
  delay(100);

}
void ResetArduino() {
    // Enable the watchdog timer for a 1-second timeout
    wdt_enable(WDTO_1S);
    while (1) { /* Infinite loop, the watchdog timer will reset the Arduino */ }
}

void readEncoderA() {
  aState = digitalRead(outputA);
  bState = digitalRead(outputB);

  if (aState != aLastState) {
    if (aState == bState) {
      counter--;
    } else {
      counter++;
    }
    aLastState = aState;
  }
  
}

void readEncoderB() {
  aState = digitalRead(outputA);
  bState = digitalRead(outputB);

  if (bState != bLastState) {
    if (aState != bState) {
      counter--;
    } else {
      counter++;
    }
    bLastState = bState;
  }
}
