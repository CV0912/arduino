int outputA =2;
int outputB=3;
unsigned long lastTime;
double output,setpoint=90;
;
double errsum,lasterr;
double kp,ki,kd;
double counter = 0;
int pwm = 11;
int dir =10;
int degree=0;
const int encoderPPR = 550;
int aState, bState;
int aLastState, bLastState;
double error;
void Compute()
{
  degree = counter * 360 / encoderPPR;
  unsigned long now = millis();
  double timeChange = (double)(now-lastTime);
  error = setpoint - degree;
  errsum += error*timeChange;
  double derr = (error-lasterr)/timeChange;
  output = kp * error + ki * errsum + kd*derr;
  output = constrain(output,-255,255);
  output = abs(output);
  lasterr=error;
  lastTime = now;
  Serial.print("Error :");
  Serial.println(error);
  if(error>0)
  {
    
  }
  }
void SetTunings(double Kp,double Ki,double Kd)
{
  kp=Kp;
  ki=Ki;
  kd=Kd;
  }
void setup() {
  pinMode(outputA, INPUT_PULLUP);
  pinMode(outputB, INPUT_PULLUP);
  pinMode(pwm, OUTPUT);
  pinMode(dir, OUTPUT);
  Serial.begin(9600);

  aLastState = digitalRead(outputA);
  bLastState = digitalRead(outputB);

  attachInterrupt(digitalPinToInterrupt(outputA), readEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(outputB), readEncoderB, CHANGE);

}

void loop() {
  degree = (counter/encoderPPR)*360;
  SetTunings(-0.75,-0.001,0);
  Compute();
  if(degree<setpoint)
  {
    digitalWrite(dir,HIGH);
    analogWrite(pwm,output);
    }
    
  else if(degree>setpoint)
  {
    digitalWrite(dir,LOW);
    analogWrite(pwm,output);
  }
  else
  {
    analogWrite(pwm,0);
  }
  Serial.println(degree);
  delay(100);

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
