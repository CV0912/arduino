#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
int outputA =2;
int outputB=3;
unsigned long lastTime;
double output,setpoint=0;
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
bool tuned = false;
void tunePID_ZieglerNichols() {
  static double kp_trial = 0;
  static bool oscillating = false;
  static unsigned long startTime = 0;
  static double lastError = 0;
  static int oscillationCount = 0;
  static double oscillationStart = 0;
  
  // Set Ki and Kd to zero initially
  ki = 0;
  kd = 0;

  // Incrementally increase Kp
  kp_trial += 0.1;

  // Simulate the system and check for oscillation
  double error = setpoint - degree;

  if (error * lastError < 0) { // Error sign flipped (oscillation detected)
    if (!oscillating) {
      oscillating = true;
      startTime = millis();
      oscillationStart = startTime;
    } else {
      // Count oscillations
      oscillationCount++;
      if (oscillationCount >= 3) {
        double Tu = (millis() - oscillationStart) / oscillationCount; // Oscillation period
        double Ku = kp_trial;

        // Apply Ziegler-Nichols rules
        kp = 0.6 * Ku;
        ki = 1.2 * Ku / Tu;
        kd = 0.075 * Ku * Tu;

        Serial.println("Tuned Gains:");
        Serial.print("Kp: ");
        Serial.println(kp);
        Serial.print("Ki: ");
        Serial.println(ki);
        Serial.print("Kd: ");
        Serial.println(kd);

        oscillating = false;
        kp_trial = 0; // Reset trial
      }
    }
  }
  lastError = error;
}

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
    digitalWrite(dir,HIGH);
    analogWrite(pwm,output);
  }
  else if(error<0)
  {
    digitalWrite(dir,LOW);
    analogWrite(pwm,output);
  }
  else
  {
    analogWrite(pwm,0);
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
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
 delay(100);
 bno.setExtCrystalUse(true);
  aLastState = digitalRead(outputA);
  bLastState = digitalRead(outputB);

  attachInterrupt(digitalPinToInterrupt(outputA), readEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(outputB), readEncoderB, CHANGE);

}

void loop() {
  SetTunings(-0.75,-0.001,0);
  tunePID_ZieglerNichols();
  Compute();
    sensors_event_t event;
  bno.getEvent(&event);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  Serial.print("Yaw (X): ");
  Serial.print(euler.x(), 0); // Display roll in degrees

  displayCalStatus();
  Serial.println("");
  double alpha = 0.1; // Smoothing factor (0 < alpha < 1)
  setpoint = alpha * euler.x() + (1 - alpha) * setpoint;
 

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
void displayCalStatus(void)
{
 /* Get the four calibration values (0..3) */
 /* Any sensor data reporting 0 should be ignored, */
 /* 3 means 'fully calibrated" */
 uint8_t system, gyro, accel, mag;
 system = gyro = accel = mag = 0;
 bno.getCalibration(&system, &gyro, &accel, &mag);
 /* The data should be ignored until the system calibration is &gt; 0 */
 Serial.print("\t");
 if (!system)
 {
 Serial.print("! ");
 }
 /* Display the individual values */
 Serial.print("Sys:");
 Serial.print(system, DEC);
 Serial.print(" G:");
 Serial.print(gyro, DEC);
 Serial.print(" A:");
 Serial.print(accel, DEC);
 Serial.print(" M:");
 Serial.println(mag, DEC);
}
