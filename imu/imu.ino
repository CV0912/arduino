#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
 delay(1000);
 bno.setExtCrystalUse(true);

}
void loop() {
  // put your main code here, to run repeatedly:
  uint8_t system,gyro,accel,mg=0;
  bno.getCalibration(&system,&gyro,&accel,&mg);
  sensors_event_t event;
  bno.getEvent(&event);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  Serial.print("Yaw (X): ");
  Serial.print(euler.x(), 2); // Display roll in degrees
  Serial.println("");  
  delay(100);

  
} 
