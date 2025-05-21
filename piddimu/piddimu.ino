#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// Pin definitions
const int outputA = 2;
const int outputB = 3;
const int pwm = 11;
const int dir = 10;

// Encoder variables
volatile long counter = 0;
const int encoderPPR = 550;
int aState, bState;
int aLastState, bLastState;

// PID variables
double kp = 0, ki = 0, kd = 0;
double setpoint = 0, output = 0;
double errsum = 0, lasterr = 0;
unsigned long lastTime = 0;

// Tuning variables
bool tuned = false;
double kp_trial = 0;
bool oscillating = false;
unsigned long startTime = 0;
int oscillationCount = 0;
double oscillationStart = 0;

void tunePID_ZieglerNichols() {
  static double lastError = 0;

  // Static setpoint for tuning
  setpoint = 45; // Example static angle for tuning

  // Set Ki and Kd to zero for initial tuning
  ki = 0;
  kd = 0;

  // Incrementally increase Kp
  kp_trial += 0.1;

  // Simulate the system and check for oscillation
  double degree = counter * 360.0 / encoderPPR; // Calculate current degree
  double error = setpoint - degree;

  if (error * lastError < 0) { // Error sign flipped (oscillation detected)
    if (!oscillating) {
      oscillating = true;
      startTime = millis();
      oscillationStart = startTime;
    } else {
      oscillationCount++;
      if (oscillationCount >= 3) { // Detect 3 oscillations
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
        tuned = true; // Mark tuning as complete
        kp_trial = 0; // Reset trial variable for safety
      }
    }
  }

  lastError = error;
}

void Compute() {
  double degree = counter * 360.0 / encoderPPR;

  unsigned long now = millis();
  double timeChange = (double)(now - lastTime);

  double error = setpoint - degree;
  if (output > -255 && output < 255) {
    errsum += error * timeChange; // Accumulate integral term
  }
  double derr = (error - lasterr) / timeChange;

  // Calculate PID output
  output = kp * error + ki * errsum + kd * derr;
  output = constrain(output, -255, 255);

  // Set motor direction and PWM
  if (error > 0) {
    digitalWrite(dir, HIGH);
  } else {
    digitalWrite(dir, LOW);
    output = -output; // Make PWM value positive
  }
  analogWrite(pwm, abs(output));

  lasterr = error;
  lastTime = now;

  Serial.print("Error: ");
  Serial.println(error);
}

void setup() {
  pinMode(outputA, INPUT_PULLUP);
  pinMode(outputB, INPUT_PULLUP);
  pinMode(pwm, OUTPUT);
  pinMode(dir, OUTPUT);

  Serial.begin(115200);
  Serial.println("Orientation Sensor Test");

  if (!bno.begin()) {
    Serial.println("No BNO055 detected. Check wiring!");
    while (1);
  }
  bno.setExtCrystalUse(true);

  attachInterrupt(digitalPinToInterrupt(outputA), readEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(outputB), readEncoderB, CHANGE);

  lastTime = millis();
}

void loop() {
  if (!tuned) {
    tunePID_ZieglerNichols(); // Perform Ziegler-Nichols tuning
  } else {
    // Read IMU yaw angle
    sensors_event_t event;
    bno.getEvent(&event);
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    setpoint = euler.x(); // Use dynamic yaw angle as setpoint after tuning

    Compute(); // Perform PID control
  }

  delay(BNO055_SAMPLERATE_DELAY_MS);
}

void readEncoderA() {
  int aState = digitalRead(outputA);
  if (aState != aLastState) {
    if (digitalRead(outputB) == aState) {
      counter--;
    } else {
      counter++;
    }
  }
  aLastState = aState;
}

void readEncoderB() {
  int bState = digitalRead(outputB);
  if (bState != bLastState) {
    if (digitalRead(outputA) != bState) {
      counter--;
    } else {
      counter++;
    }
  }
  bLastState = bState;
}

void displayCalStatus() {
  uint8_t system, gyro, accel, mag;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  Serial.print("\tSys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.println(mag, DEC);
}
