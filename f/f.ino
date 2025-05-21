// Define LCD library
#include <LiquidCrystal.h>

// Define PIR sensor pin
const int pirPin = 9;

// Define potentiometer pin
const int potPin = A0;

// Define buzzer pin
const int buzzerPin = 8;

// Define LCD pins (RS, E, D4, D5, D6, D7)
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// Variables to store sensor readings
int pirValue=0;
int potValue;

void setup() {
  // Initialize LCD with 16 columns and 2 rows
  lcd.begin(16, 2);
  lcd.print("Motion Detector");

  // Set PIR pin as input
  pinMode(pirPin, INPUT);

  // Set buzzer pin as output
  pinMode(buzzerPin, OUTPUT);

  // Initialize serial communication for debugging
  Serial.begin(9600);
  Serial.println("PIR Motion Detector with Potentiometer and Buzzer");
}

void loop() {
  // Read the value from the PIR sensor
  pirValue = digitalRead(pirPin);

  // Read the value from the potentiometer
  potValue = analogRead(potPin);

  // Map the potentiometer value to a delay range (adjust as needed)
  int delayTime = map(potValue, 0, 1023, 100, 200); // Delay between 0.1 to 2 seconds

  // Check if motion is detected
  if (pirValue == HIGH) {
    // Motion detected!
    lcd.clear();
    lcd.print("Motion Detected!");
    digitalWrite(buzzerPin, HIGH); // Turn on the buzzer

    // You can add more actions here when motion is detected

    Serial.println("Motion Detected!");
    Serial.println("Buzzer ON"); // Added serial output for debugging
  } else {
    // No motion detected
    lcd.clear();
    lcd.print("No Motion");
    digitalWrite(buzzerPin, LOW); // Turn off the buzzer

    Serial.println("No Motion");
    Serial.println("Buzzer OFF"); // Added serial output for debugging
  }

  // Small delay controlled by the potentiometer
  Serial.println(pirValue);
  delay(delayTime);
}
