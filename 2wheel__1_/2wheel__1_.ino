#include <Arduino.h>  // Add this for ESP32 functionality
#include <ps5Controller.h>

// Left Motor
const int LEFT_PWM_CHANNEL = 0;   // PWM channel for left motor
const int LEFT_DIR_PIN = 26;  // Direction pin for left motor

// Right Motor
const int RIGHT_PWM_CHANNEL = 2;  // PWM channel for right motor
const int RIGHT_DIR_PIN = 33;  // Direction pin for right motor
const int b = 106;
const int r = 65;
int lx, ly, rx, ry, tri, cir, squ, cro, l2, r2,l1,r1,left,right,up,down;
int buff = 40;
double targetwl,targetwr;
int sensorValue;
float Volt;
float Distance;
int sensor_pin = 3;
// void readPS5Controller();
void compute2Wheel()
{
  float V = ly;
  float rX = rx*2;
  targetwr=((ly*200+(rX*b/2))/r);

  targetwl=((ly*200-(rX*b/2))/r);
  targetwr = constrain(map(targetwr,0,494,0,220),-255,255);
  targetwl = constrain(map(targetwl,0,494,0,220),-255,255);
  Serial.print(targetwr);
  Serial.print(" ");
  Serial.println(targetwl);
}
void applySpeed()
{
  if (targetwl >= 0) 
  {
  digitalWrite(LEFT_DIR_PIN, LOW);
  analogWrite(LEFT_PWM_CHANNEL, targetwl);
  } 
  else
  {
  digitalWrite(LEFT_DIR_PIN, HIGH); 
  analogWrite(LEFT_PWM_CHANNEL, -targetwl);
  }
  if (targetwr >= 0) 
  {
  digitalWrite(RIGHT_DIR_PIN, LOW); 
  analogWrite(RIGHT_PWM_CHANNEL, targetwr);
  } else {
  digitalWrite(RIGHT_DIR_PIN, HIGH);
  analogWrite(RIGHT_PWM_CHANNEL, -targetwr);
  }

}
int sharpSensor()
{
  sensorValue = analogRead(sensor_pin);
  Volt = sensorValue * (5.0 / 1023.0);
  Distance = 12.08 *pow(Volt , -1.058);
  if(Distance<30)
  return 1;
  else
  return 0;
}
void setup() {
  Serial.begin(9600);
  // ps5.begin("e8:47:3a:5a:a3:66");
  ps5.begin("7c:66:ef:78:76:f0");

  // pwm
  pinMode(LEFT_PWM_CHANNEL, OUTPUT);
  pinMode(RIGHT_PWM_CHANNEL, OUTPUT);

  // Set direction pins as outputs
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);

  // Initialize motors to stopped state
  // stopMotors();
}

void loop() {
  if (ps5.isConnected()) {
    lx = ps5.LStickX();  // Added missing semicolons
    ly = ps5.LStickY();
    rx = -ps5.RStickX();
    ry = ps5.RStickY();
    tri = ps5.Triangle();
    cir = ps5.Circle();
    cro = ps5.Cross();
    squ = ps5.Square();
    l2 = ps5.L2();
    r2 = ps5.R2();
    l1 = ps5.L1();
    r1 = ps5.R1();
    up=ps5.Up();
    down=ps5.Down();
    left=ps5.Left();
    right=ps5.Right();


    
  }

  if (abs(ly) < buff) ly = 0;
    // else ly = (ly > 0) ? ly - deadzone : ly + deadzone;

  if (abs(rx) < buff) rx = 0;
    // else rx = (rx > 0) ? rx - deadzone : rx + deadzone;
  // processCommand();
  // compute2Wheel();
  // if(sharpSensor()!=1)
  // applySpeed();
  // else
  // turnLeft(200);
    Serial.print(lx);
    Serial.print(" ");
    Serial.print(ly);
    Serial.print(" ");
    Serial.print(rx);
    Serial.print(" ");
    Serial.print(ry);
    Serial.print(" ");
    Serial.print(tri);
    Serial.print(" ");
    Serial.print(cir);
    Serial.print(" ");
    Serial.print(cro);
    Serial.print(" ");
    Serial.print(squ);
    Serial.print(" ");
    Serial.print(up);
    Serial.print(" ");
    Serial.print(down);
    Serial.print(" ");
    Serial.print(left);
    Serial.print(" ");
    Serial.print(right);
    Serial.print(" ");
    Serial.print(l1);
    Serial.print(" ");
    Serial.print(r1);
    Serial.print(" ");
    Serial.print(l2);
    Serial.print(" ");
    Serial.println(r2);
    delay(100);
}
// Motor control functions
void turnLeft(int speed) {
  // Set direction pins
  digitalWrite(LEFT_DIR_PIN, LOW);    // Left motor backward
  digitalWrite(RIGHT_DIR_PIN, HIGH);  // Right motor forward

  // Set speed
  analogWrite(LEFT_PWM_CHANNEL, speed);
  analogWrite(RIGHT_PWM_CHANNEL, speed);
}

void stopMotors() {
  // Set speed to zero
  analogWrite(LEFT_PWM_CHANNEL, 0);
  analogWrite(RIGHT_PWM_CHANNEL, 0);
}