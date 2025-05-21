#include <math.h>
#include <Wire.h>
#include <HardwareSerial.h>
#define pi 3.1415926535
#include <Servo.h>
Servo myservo;
Servo myservo1; 
Servo myservo2; 
int motorfrontdir = 30;
int motorfrontpwm = 4;
int motorreardirl = 28;
int motorrearpwml = 10;
int motorreardirr = 26;
int motorrearpwmr = 3;

// stepper pinouts
int dirPin = 25;
int enaPin = 27;
int stepPin = 29;
#define l0 0.3
double wfl, wfr, wrl, wrr, wz;
const double r = 75;
double target_wf = 0, target_wrl = 0, target_wrr = 0;
double current_wf = 0, current_wrl = 0, current_wrr = 0;
const double decelerationRate = 30.0;
double w;
double rL, thetaL, rR, thetaR;
const int deadzone = 40;
// int lx, ly, rx, ry, up, down, left, right, tri, squ, circle, cross, l1, l2, r1, r2;
int8_t lx, ly, rx, ry, up, down, left, right, tri, squ, circle, cross, l1, l2, r1, r2;
int indexA, indexB, indexC, indexD, indexE, indexF, indexG, indexH, indexI, indexJ, indexK, indelx, indexM, indexN, indexO, indexP;
String data1, data2, data3, data4, data5, data6, data7, data8, data9, data10, data11, data12, data13, data14, data15, data16;
String datain;

//ebike
//const int ebike_dir_pin = 50;
//const int ebike_speed_pin = 7;
const int ebike_dir_pin = 24;
const int ebike_speed_pin = 12;
int shooting = 0;
int dribbling = 0;
int32_t drib_speed = 50;
int32_t shoot_speed = 92;

//pneumatic
const int pneumatic = 5;
// const int pneumatic = 40;
static int pneumatic_open = 0;

static int prev_circle = 0;
static int prev_tri = 0;
static int prev_cro = 0;
static int prev_squ = 0;
static int prev_l1 = 0;
static int prev_r1 = 0;
static int prev_l2 = 0;
static int prev_r2 = 0;
static int prev_up = 0;


// servo
int servoState = 0;

// stepper
// int angle = 30;

// int STEPS = angle / 0.3;


// static uint32_t last_step_time = 0;
// static uint32_t last_step_time1 = 0;
// static int step_state = 0;
// static int step_state1 = 0;
// static int direction = 1;
// int step_delay = 1;
// int step_delay1 = 1;
// static int stepper_running = 0;
// static int stepper_running1 = 0;
// uint32_t target_steps = 0;
// uint32_t current_steps = 0;

void setup() {
  Serial.begin(9600);
  // Serial2.begin(500000);
  Wire.begin(8);
  Wire.onReceive(receiveEvent);
  pinMode(motorfrontpwm, OUTPUT);
  pinMode(motorfrontdir, OUTPUT);
  pinMode(motorrearpwml, OUTPUT);
  pinMode(motorreardirl, OUTPUT);
  pinMode(motorrearpwmr, OUTPUT);
  pinMode(motorreardirr, OUTPUT);
  pinMode(pneumatic, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enaPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  myservo.attach(6);
  myservo1.attach(11);
  myservo2.attach(7);
  // myservo.write(90);
  // myservo1.write(90);
  // myservo2.write(90);
}
void receiveEvent(int bytes) {
  while (Wire.available()) {
    lx = Wire.read();
    ly = Wire.read();
    rx = Wire.read();
    ry = Wire.read();
    tri = Wire.read();
    circle = Wire.read();
    cross = Wire.read();
    squ = Wire.read();
    l1 = Wire.read();
    r1 = Wire.read();
    l2 = Wire.read();
    r2 = Wire.read();
    up = Wire.read();
    down = Wire.read();
    right = Wire.read();
    left = Wire.read();
    
    // Serial.println("Recevied!");
    wz = rx;
    if (abs(lx) < deadzone) lx = 0;
    // else lx = (lx > 0) ? lx - deadzone : lx + deadzone;

    if (abs(ly) < deadzone) ly = 0;
    // else ly = (ly > 0) ? ly - deadzone : ly + deadzone;

    if (abs(rx) < deadzone) rx = 0;
    // else rx = (rx > 0) ? rx - deadzone : rx + deadzone;

    if (abs(ry) < deadzone) ry = 0;
    // else ry = (ry > 0) ? ry - deadzone : ry + deadzone;
    rL = sqrt(lx * lx + ly * ly);
    thetaL = atan2(lx, ly);
    rR = sqrt(rx * rx + ry * ry);
    thetaR = atan2(ry, rx);
  }
}
void loop() {
  // fetch_data();
  // parse_data(datain);
  // ps_print();
  compute3wheel();
  applyDeceleration();
  Serial.print(lx);
  Serial.print(" ");
  Serial.print(ly);
  Serial.print(" ");
  Serial.print(rx);
  Serial.print(" ");
  Serial.print(ry);
  Serial.print(" ");
  Serial.print(drib_speed);
  Serial.print(" ");
  // Serial.print(ebike_running);
  // Serial.print(" ");
  Serial.print(pneumatic_open);
  Serial.print(" ");
  Serial.print(target_wf);
  Serial.print(" ");
  Serial.print(target_wrl);
  Serial.print(" ");
  Serial.println(target_wrr);
  
  
  // Servo

  if (up == 1 && !prev_up) {
    servoState = !servoState;
  }

  prev_up = up;

  if (servoState) {
    myservo.write(90);
    delay(1000);
    myservo1.write(90);
    delay(1000);
    myservo2.write(90);
  } else {
    myservo.write(180);
    delay(1000);
    myservo1.write(180);
    delay(1000);
    myservo2.write(180);
  }

  /* Stepper */
  // digitalWrite(dirPin,
  //              direction ? HIGH : LOW);

  // if (l1 == 1 && !prev_l1) {  // l1 button pressed
  //   direction = 1;            // Set to Anti-clockwise
  //   stepper_running = 1;
  //   target_steps = STEPS;
  //   digitalWrite(enaPin, LOW);
  // }
  // if (r1 == 1 && !prev_r1) {  // r1 button pressed
  //   direction = 0;            // Set to Clockwise
  //   stepper_running = 1;
  //   target_steps = STEPS;
  //   digitalWrite(enaPin, LOW);
  // }
  // if (circle == 1 && !prev_circle) {
  //   stepper_running = 0;
  //   stepper_running1 = 0;
  //   current_steps = 0;
  //   digitalWrite(enaPin, HIGH);
  // }
  // prev_l1 = l1;
  // prev_r1 = r1;
  // prev_circle = circle;

  // if (stepper_running && (millis() - last_step_time) >= step_delay
  //     && current_steps < target_steps) {
  //   last_step_time = millis();  // Update last step time

  //   if (step_state == 0) {
  //     digitalWrite(stepPin, HIGH);  // Step HIGH
  //     step_state = 1;
  //     current_steps++;
  //   } else {
  //     digitalWrite(stepPin, LOW);  // Step LOW
  //     step_state = 0;
  //   }
  // }

  // // Stop when target steps are reached
  // if (current_steps >= target_steps) {
  //   stepper_running = 0;
  //   current_steps = 0;
  //   digitalWrite(enaPin, HIGH);
  // }


  /* Dribbling */
  if (tri == 1 && !prev_tri) {
    dribbling = 1;
  }
  if (cross == 1 && !prev_cro) {  // cro button pressed
    dribbling = 0;
  }
  if (l2 == 1 && !prev_l2) {
    drib_speed -= 2;  //62 speed for passing 50 for dribbling
  }
  if (r2 == 1 && !prev_r2) {
    drib_speed += 2;
  }
  if (drib_speed < 0) {
    drib_speed = 0;
  }
  if (drib_speed > 100) {
    drib_speed = 100;
  }

  prev_tri = tri;
  prev_cro = cross;
  prev_l2 = l2;
  prev_r2 = r2;
  //for dribbling
  if (dribbling) {
    // Serial.println("Ebike running");
    // int speed = map(drib_speed, 0, 100, 0, 255);  //rpm 2000 test
    int speed = (drib_speed * 255) / 100;
    digitalWrite(ebike_dir_pin, LOW);
    analogWrite(ebike_speed_pin, speed);
  } else {
    analogWrite(ebike_speed_pin, 0);
  }
  // pneumatics
  if (squ == 1) {
    pneumatic_open = 1;
  } else {
    pneumatic_open = 0;
  }
  //		prev_squ = squ;
  if (pneumatic_open) {
    Serial.println("Pnematic open");
    digitalWrite(pneumatic, HIGH);
  } else {
    digitalWrite(pneumatic, LOW);
  }
}

// void compute3wheel() {
//   // Calculate joystick-based velocities
//   const double theta = -30 * pi / 180;
//   double lx_rotated = lx * cos(theta) - ly * sin(theta);
//   double ly_rotated = lx * sin(theta) + ly * cos(theta);
//   double vx = ly_rotated;
//   double vy = lx_rotated;
//   w = rx;
//   double speedFactorL = 3 * (rL / 88 + rR / 88) / 2;
//   // Modify wheel velocities to include yaw correction
//   target_wf = ((-0.5 * vx) + (sqrt(3) / 2) * vy + l0 * w) * speedFactorL;
//   target_wrr = ((-0.5 * vx) - (sqrt(3) / 2) * vy + l0 * w) * speedFactorL;
//   target_wrl = (vx + l0 * w) * speedFactorL;
//   // target_wf=(-0.866*vx + (1/2)*vy + l0*w)*speedFactorL;
//   // target_wrr = (0.866*vx + (1/2)*vy + l0*w)*speedFactorL;
//   // target_wrl = (-vx + l0*w)*speedFactorL;
//   // Constrain target wheel speeds
//   target_wf = constrain(target_wf, -170, 170);
//   target_wrr = constrain(target_wrr, -170, 170);
//   target_wrl = constrain(target_wrl, -170, 170);
// }

// void applyDeceleration() {
//   // Gradually approach target speeds for each wheel
//   current_wf += (target_wf - current_wf) / decelerationRate;
//   current_wrl += (target_wrl - current_wrl) / decelerationRate;
//   current_wrr += (target_wrr - current_wrr) / decelerationRate;

//   // Serial.print("wf =");
//   // Serial.print(current_wf);
//   // Serial.print("wrr =");
//   // Serial.print(current_wrr);
//   // Serial.print("wrl =");
//   // Serial.println(current_wrl);

//   // Update motor directions a000000.nd speeds based on decelerated values
//   if (current_wf >= 0) {
//     digitalWrite(motorfrontdir, LOW);
//     analogWrite(motorfrontpwm, current_wf);
//   } else {
//     digitalWrite(motorfrontdir, HIGH);
//     analogWrite(motorfrontpwm, -current_wf);
//   }

//   if (current_wrl >= 0) {
//     digitalWrite(motorreardirl, LOW);
//     analogWrite(motorrearpwml, current_wrl);
//   } else {
//     digitalWrite(motorreardirl, HIGH);
//     analogWrite(motorrearpwml, -current_wrl);
//   }

//   if (current_wrr >= 0) {
//     digitalWrite(motorreardirr, LOW);
//     analogWrite(motorrearpwmr, current_wrr);
//   } else {
//     digitalWrite(motorreardirr, HIGH);
//     analogWrite(motorrearpwmr, -current_wrr);
//   }
// }