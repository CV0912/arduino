  #include <HX711.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <HX711_ADC.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif
#define DOUT  16
#define CLK  4
HX711 scale(DOUT, CLK);
 
// Comment this out to disable prints and save space
#define BLYNK_PRINT Serial
char auth[] = BLYNK_AUTH_TOKEN;
 
Your WiFi credentials.
Set password to "" for open networks.
char ssid[] = "AndroidAP3DEC";
char pass[] = "electroniclinic";
 
float weight; 
float calibration_factor = 71.46/2; // for me this vlaue works just perfect 211000  
 
void setup() {
  // Set up serial monitor
  Serial.begin(115200);
  Serial.println("HX711 calibration sketch");
  Serial.println("Remove all weight from scale");
  Serial.println("After readings begin, place known weight on scale");
  Serial.println("Press + or a to increase calibration factor");
  Serial.println("Press - or z to decrease calibration factor");
  scale.set_scale();
  scale.tare(); //Reset the scale to 0
  long zero_factor = scale.read_average(); //Get a baseline reading
  Serial.print("Zero factor: "); //This can be used to remove the need to tare the scale. Useful in permanent scale projects.
  Serial.println(zero_factor);
  // Blynk.begin(auth, ssid, pass);
 
 
}
 void loop() {
  // Blynk.run();
  measureweight();
}
 
void measureweight(){
 scale.set_scale(calibration_factor); //Adjust to this calibration factor
  Serial.print("Reading: ");
  weight = scale.get_units(5); 
    if(weight<0)
  {
    weight=0.00;
    }
  //Serial.print(scale.get_units(), 2);
 // Serial.print(" lbs"); //Change this to kg and re-adjust the calibration factor if you follow SI units like a sane person
  Serial.print("gram:");
  Serial.print( weight); 
  Serial.print(" g");
  Serial.print(" calibration_factor: ");
  Serial.print(calibration_factor);
  Serial.println();
  //     Blynk.virtualWrite(V0,weight);
  // Blynk.virtualWrite(V1,weight);
  // Delay before repeating measurement
  delay(100);
}