#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

#define pi 3.1415926535
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Variables
float yawtarget = -90;
float output1, output2, output3;
float yawerror;
Adafruit_BNO055 myIMU = Adafruit_BNO055();
float q0, q1, q2, q3, yaw = 0;
float kp = 1.0, ki = 0.1, kd = 0.5; // Tunable PID gains
double errsum = 0, lasterr = 0;
unsigned long lastTime = 0;

void compute() {
  unsigned long now = millis();
  double timeChange = (double)(now - lastTime) / 1000.0; // Convert to seconds

  if (timeChange > 0) {
    yawerror = yawtarget - yaw; // Calculate error
    errsum += yawerror * timeChange; // Accumulate integral error
    errsum = constrain(errsum, -1000, 1000); // Prevent integral windup

    double derr = (yawerror - lasterr) / timeChange; // Derivative term

    // PID formula
    float pidOutput = kp * yawerror + ki * errsum + kd * derr;

    // Clamp output
    pidOutput = constrain(pidOutput, -255, 255);

    // Calculate individual motor outputs based on PID output
    output1 = pidOutput; // Example for motor 1
    output2 = -pidOutput; // Example for motor 2 (reversed)
    output3 = pidOutput / 2; // Example for motor 3 (scaled)

    lasterr = yawerror; // Update last error
    lastTime = now; // Update last computation time

    // Debugging information
    Serial.print("Yaw Error: ");
    Serial.println(yawerror);
    Serial.print("PID Output: ");
    Serial.println(pidOutput);
  }
}

void setup() {
  Serial.begin(115200);
  myIMU.begin();
  delay(1000);
  myIMU.setExtCrystalUse(true);

  // Initial time
  lastTime = millis();
}

void loop() {
  uint8_t system, gyro, accel, mg = 0;
  myIMU.getCalibration(&system, &gyro, &accel, &mg);

  imu::Quaternion quat = myIMU.getQuat();

  q0 = quat.w();
  q1 = quat.x();
  q2 = quat.y();
  q3 = quat.z();
  yaw = -atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)) - pi / 2;
  yaw = yaw * (180.0 / pi);

  // Debugging yaw and error
  Serial.print("YAW = ");
  Serial.print(yaw);
  Serial.print(" Error = ");
  Serial.println(yawtarget - yaw);

  compute(); // Run PID computation
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
