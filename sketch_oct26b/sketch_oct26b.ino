#include <Wire.h>
#include <ps5Controller.h>

void setup() {
  Wire.begin();
  Serial.begin(115200);
  ps5.begin("7c:66:ef:78:76:f0");
}

void loop() {
  if (ps5.isConnected()) {
    Wire.beginTransmission(8);
    Wire.write(ps5.LStickX());
    Wire.write(ps5.LStickY());
    Wire.write(ps5.RStickX());
    Wire.write(ps5.RStickY());
     Serial.println(ps5.LStickX());
     Serial.println(ps5.LStickY());
     Serial.println(ps5.RStickX());
     Serial.println(ps5.RStickY());
    Wire.endTransmission();
    delay(100);
  }
}
