#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include<HardwareSerial.h>
#define pi 3.1415926535
int motorfrontdir=2;
int motorfrontpwm=11;
int motorreardirl=4;
int motorrearpwml=12;
int motorreardirr=9;
int motorrearpwmr=10;
#define l0 310
double wfl, wfr, wrl, wrr, wz;
const double r = 75;
double target_wf = 0, target_wrl = 0, target_wrr = 0;
double current_wf = 0, current_wrl = 0, current_wrr = 0;
const double decelerationRate = 8.0;
double w;
double rL, thetaL, rR, thetaR;
const int deadzone = 40;

#define BNO055_SAMPLERATE_DELAY_MS (10)
//variables
double yawtarget=-90;
double yawerror;
Adafruit_BNO055 myIMU = Adafruit_BNO055();
float q0,q1,q2,q3,yaw=0;
int kp,kd,ki;
float output;
unsigned long lastTime;
double errsum,lasterr;
int lx,ly,rx,ry,up,down,left,right,tri,squ,circle,cross,l1,l2,r1,r2;

int indexA,indexB,indexC,indexD,indexE,indexF,indexG,indexH,indexI,indexJ,indexK,indelx,indexM,indexN,indexO,indexP;

String data1,data2,data3,data4,data5,data6,data7,data8,data9,data10,data11,data12,data13,data14,data15,data16;

String datain;
void fetch_data()
{ 
 if(Serial2.available() > 0)
 {
  datain= Serial2.readStringUntil('!');
 }
}

void parse_data(String datain)
{
indexA=datain.indexOf("A");
indexB=datain.indexOf("B");
indexC=datain.indexOf("C");
indexD=datain.indexOf("D");
indexE=datain.indexOf("E");
indexF=datain.indexOf("F");
indexG=datain.indexOf("G");
indexH=datain.indexOf("H");
indexI=datain.indexOf("I");
indexJ=datain.indexOf("J");
indexK=datain.indexOf("K");
indelx=datain.indexOf("L");
indexM=datain.indexOf("M");
indexN=datain.indexOf("N");
indexO=datain.indexOf("O");
indexP=datain.indexOf("P");

data1=datain.substring(0,indexA);
data2=datain.substring(indexA+1,indexB);
data3=datain.substring(indexB+1,indexC);
data4=datain.substring(indexC+1,indexD);
data5=datain.substring(indexD+1,indexE);
data6=datain.substring(indexE+1,indexF);
data7=datain.substring(indexF+1,indexG);
data8=datain.substring(indexG+1,indexH);
data9=datain.substring(indexH+1,indexI);
data10=datain.substring(indexI+1,indexJ);
data11=datain.substring(indexJ+1,indexK);
data12=datain.substring(indexK+1,indelx);
data13=datain.substring(indelx+1,indexM);
data14=datain.substring(indexM+1,indexN);
data15=datain.substring(indexN+1,indexO);
data16=datain.substring(indexO+1,indexP);

lx=data1.toInt();
ly=data2.toInt();
rx=data3.toInt();
ry=data4.toInt();
up=data5.toInt();
down=data6.toInt();
left=data7.toInt();
right=data8.toInt();
tri=data9.toInt();
squ=data10.toInt();
circle=data11.toInt();
cross=data12.toInt();
l1=data13.toInt();
l2=data14.toInt();
r1=data15.toInt();
r2=data16.toInt();
if (abs(lx) < deadzone) lx = 0;
if (abs(ly) < deadzone) ly = 0;
if (abs(rx) < deadzone) rx = 0;
else rx = (rx>0)?rx-deadzone : rx+deadzone;
if (abs(ry) < deadzone) ry = 0;
rL = sqrt(lx * lx + ly * ly);
thetaL = atan2(lx, ly);
rR = sqrt(rx * rx + ry * ry);
thetaR = atan2(ry, rx);
}
void ps_print()
{

Serial.print("lx=");
  Serial.print(lx);
  Serial.print(", ");
Serial.print("ly=");
  Serial.print(ly);
  Serial.print(", ");
// //  Serial.print("rx=");
//   Serial.print(rx);
//   Serial.print(", ");
//  // Serial.print("ry=");
//   Serial.print(ry);
//   Serial.print(" ");

//  // Serial.print("up=");
//   Serial.print(up);
//   Serial.print(", ");
//  // Serial.print("dowm=");
//   Serial.print(down);
//   Serial.print(", ");
// //  Serial.print("left=");
//   Serial.print(left);
//   Serial.print(", ");
// //  Serial.print("right=");
//   Serial.print(right);
//   Serial.print(",");
  
// //  Serial.print("tri=");
//   Serial.print(tri);
//   Serial.print(", ");
//  // Serial.print("squ=");
//   Serial.print(squ);
//   Serial.print(", ");
//  // Serial.print("circle=");
//   Serial.print(circle);
//   Serial.print(", ");
// //  Serial.print("cross=");
//   Serial.print(cross);
//   Serial.print(", ");

//  // Serial.print("l1=");
//   Serial.print(l1);
//   Serial.print(", ");
//  // Serial.print("l2=");
//   Serial.print(l2);
//   Serial.print(", ");
// //  Serial.print("r1=");
//   Serial.print(r1);
//   Serial.print(", ");
//  // Serial.print("r2=");
//   Serial.println(r2);
}
void compute()
{
  yawerror = yawtarget-yaw;
  errsum += yawerror;
  double derr = (yawerror-lasterr);
  output = kp * yawerror + ki * errsum + kd*derr;
  output = constrain(output,-200,200);
  lasterr=yawerror;
}
void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
Serial2.begin(115200);
myIMU.begin();
pinMode(motorfrontpwm,OUTPUT);
pinMode(motorfrontdir,OUTPUT);
pinMode(motorrearpwml,OUTPUT);
pinMode(motorreardirl,OUTPUT);
pinMode(motorrearpwmr,OUTPUT);
pinMode(motorreardirr,OUTPUT);
analogWrite(motorfrontpwm, 0);
analogWrite(motorrearpwml, 0);
analogWrite(motorrearpwmr, 0);
output = 0;
errsum = 0;
lasterr = 0;
int8_t temp=myIMU.getTemp();
myIMU.setExtCrystalUse(true);
SetTunings(5,0.9,-0.8);
}
void SetTunings(double Kp,double Ki,double Kd)
{
  kp=Kp;
  ki=Ki;
  kd=Kd;
  }
void loop() {
  // put your main code here, to run repeatedly:
  fetch_data();
  parse_data(datain);
  ps_print();
  compute3wheel();
  applyDeceleration();

  double motorspeed;
  double speedFactorL = (rL/88+rR/88)/2;
  motorspeed = 255 * speedFactorL;
  motorspeed = constrain(motorspeed,0,120);

  uint8_t system, gryo, accel, mg = 0;
  myIMU.getCalibration(&system, &gryo, &accel, &mg);

  imu::Quaternion quat=myIMU.getQuat();
  
  q0=quat.w();
  q1=quat.x();
  q2=quat.y();
  q3=quat.z();
  yaw=-atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))-pi/2;
  yaw = yaw * (180.0 / pi);
  Serial.print("YAW = ");
  Serial.print(yaw);
  compute();
  // if(yawerror>0)
  // {
  //     digitalWrite(motorfrontdir,LOW);
  //     analogWrite(motorfrontpwm,output);
  //     digitalWrite(motorreardirr,HIGH);
  //     analogWrite(motorrearpwmr,output);
  //     digitalWrite(motorreardirl,LOW);

  //     analogWrite(motorrearpwml,output);
  //     Serial.println(output);
  // }
  // else if(yawerror<0)
  // {
  //     digitalWrite(motorfrontdir,HIGH);
  //     analogWrite(motorfrontpwm,-output);
  //     digitalWrite(motorreardirr,LOW);
  //     analogWrite(motorrearpwmr,-output);
  //     digitalWrite(motorreardirl,HIGH);
  //     analogWrite(motorrearpwml,-output);
  //     Serial.println(output);    
  // }
const double YAW_CORRECTION_THRESHOLD = 5.0; // degrees
  if(rx!=0)
  {
    yawtarget=yaw;

  }
  if (abs(yawerror) > YAW_CORRECTION_THRESHOLD) {
    // Prioritize yaw correction
    digitalWrite(motorfrontdir, yawerror > 0 ? LOW : HIGH);
    analogWrite(motorfrontpwm, abs(output));
    
    digitalWrite(motorreardirr, yawerror > 0 ? LOW : HIGH);
    analogWrite(motorrearpwmr, abs(output));
    
    digitalWrite(motorreardirl, yawerror > 0 ? LOW : HIGH);
    analogWrite(motorrearpwml, abs(output));
  } else {
    // Normal movement
    applyDeceleration();
  }

  ps_print();
  delay(BNO055_SAMPLERATE_DELAY_MS);
}