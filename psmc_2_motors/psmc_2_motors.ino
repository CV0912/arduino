#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>

#include <avr/wdt.h>
#define sign(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))
int outputA =2;
int outputB =3;
int outputA1=4;
int outputB1=5;
unsigned long lastTime;
double output,output1,setpoint=5000;
double errsum,lasterr;
double kp,kd,c,kt,k;
double counter = 0,counter1=0;
int pwm = 9;
int dir =8;
int pwm1=11;
int dir1=10;
int degree=0;
const int encoderPPR = 550;
int aState, bState,a1State,b1State;
int aLastState, bLastState,a1LastState,b1LastState;
double error;
double error1;
void Compute()
{
  unsigned long now = millis();
  double timeChange = (double)(now-lastTime);
  error = setpoint - counter;
  double derr = (error-lasterr)/timeChange;
  float s = c * error+ derr;
  output = kp *error+ kd*derr + kt* (k * sign(s));
  output = constrain(output,-255,255);
  output = abs(output);
  lasterr=error;
  lastTime = now;
  Serial.print("Error :");
  Serial.println(error);
  }
void Compute1()
{
  unsigned long now = millis();
  double timeChange = (double)(now-lastTime);
  error = setpoint - counter1;
  double derr = (error-lasterr)/timeChange;
  float s = c * error+ derr;
  output = kp *error+ kd*derr + kt* (k * sign(s));
  output = constrain(output,-255,255);
  output = abs(output);
  lasterr=error;
  lastTime = now;
  Serial.print("Error1 : ");
  Serial.println(error1);
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
  pinMode(outputA1, INPUT_PULLUP);
  pinMode(outputB1, INPUT_PULLUP);
  pinMode(pwm1, OUTPUT);
  pinMode(dir1, OUTPUT);
  Serial.begin(9600);

  aLastState = digitalRead(outputA);
  bLastState = digitalRead(outputB);
  a1LastState = digitalRead(outputA1);
  b1LastState = digitalRead(outputB1);

  attachInterrupt(digitalPinToInterrupt(outputA), readEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(outputB), readEncoderB, CHANGE);
  attachPinChangeInterrupt(outputA1,readEncoderA1, CHANGE);
  attachPinChangeInterrupt(digitalPinToInterrupt(outputB1), readEncoderB1, CHANGE);

}

void loop() {
   if (Serial.available() > 0) {
        char command = Serial.read();
        if (command == 'r') { // Reset command
            ResetArduino();
        }
  }
  SetTunings(-0.6999,0.5,24,-0.01);
  Compute();
  Compute1();
  if(counter<setpoint)
  {
    digitalWrite(dir,HIGH);
    analogWrite(pwm,output);
    }
    
  else if(counter>setpoint)
  {
    digitalWrite(dir,LOW);
    analogWrite(pwm,output);
  }
  else
  {
    analogWrite(pwm,0);
  }
  if(counter1<setpoint)
  {
    digitalWrite(dir1,HIGH);
    analogWrite(pwm1,output);
    }
    
  else if(counter1>setpoint)
  {
    digitalWrite(dir1,LOW);
    analogWrite(pwm1,output);
  }
  else
  {
    analogWrite(pwm1,0);
  }
  Serial.print("counter1 :");
  Serial.println(counter);
  Serial.print("counter2 :");
  Serial.println(counter1);
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
void readEncoderA1()
{
  a1State = digitalRead(outputA1);
  b1State = digitalRead(outputB1);
  if(a1State != a1LastState)
  {
    if(a1State == b1State){
      counter1--;
      }
    else
    {
      counter1++;
    }
    a1LastState = a1State;
    }
}
void readEncoderB1() {
  a1State = digitalRead(outputA1);
  b1State = digitalRead(outputB1);

  if (b1State != b1LastState) {
    if (a1State != b1State) {
      counter--;
    } else {
      counter++;
    }
    b1LastState = b1State;
  }
}
