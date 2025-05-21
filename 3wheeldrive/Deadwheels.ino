// //all units are in mm , s
// #include <math.h>

// const int ENCODER1_A = 3;
// const int ENCODER1_B = 5;
// const int ENCODER2_A = 2;
// const int ENCODER2_B = 4;
// const int ENCODER3_A = 18;
// const int ENCODER3_B = 6;
// const float WHEEL_DIAMETER_MM = 100.0;
// const float WHEEL_RADIUS_MM = WHEEL_DIAMETER_MM / 2.0;
// const int ENCODER_RESOLUTION = 400;  // Ticks per revolution
// const float MM_PER_TICK = (PI * WHEEL_DIAMETER_MM) / ENCODER_RESOLUTION;
// unsigned long lastTime = 0;
// int lastTickX = 0;
// int lastTickY= 0;
// float relX,relY,lastrelX,lastrelY;
// float X=0,Y=0,lastx,lasty;
// float forward,strafe,lastforward,laststrafe;
// float dforward,dstrafe,dtheta;
// float theta,lasttheta;
// volatile long L = 0;//encoder 1
// volatile long R = 0;//encoder 2
// volatile long B = 0;//encoder 3
// volatile long lastL = 0;
// volatile long lastR = 0;
// volatile long lastB = 0;
// volatile bool encoder1Direction = true;
// volatile bool encoder2Direction = true;
// volatile bool encoder3Direction = true;
// float Bx=310,ly=310,ry=310;
// float calculateWheelDistance(long currentTicks, long lastTicks, float timeChange) {
//     float deltaTicks = currentTicks - lastTicks;
//     return (deltaTicks * MM_PER_TICK);
// }
// float delta(float current,float last,float timeChange)
// {
//     return (current - last)/timeChange;
// }
// void computeRobotMotion() {
//     unsigned long currentTime = millis();
//     float timeChange = (currentTime - lastTime) / 1000.0;  // Convert to seconds
    
//     // Only update if enough time has passed (avoid division by very small numbers)
//     if (timeChange >= 0.01) {  // 10ms minimum interval
//         // Calculate wheel velocities in mm/s
//         float L = calculateWheelDistance(L,lastL,timeChange);
//         float R = calculateWheelDistance(R,lastR,timeChange);
//         float B = calculateWheelDistance(B,lastB,timeChange);
//         forward = (L+R)/2;//(R*ly-L*ry)/(ly-ry);
//         theta = (R-L)/(ly-ry);
//         strafe = B - Bx*theta;
//         dforward = delta(forward,lastforward,timeChange);
//         dtheta = delta(theta,lasttheta,timeChange);
//         dstrafe = delta(strafe,laststrafe,timeChange);
//         X = lastx+dforward*cos(theta)-dstrafe*sin(theta);
//         Y = lasty+dstrafe*cos(theta)-dforward*sin(theta);
//         // Store current values for next iteration
//         lastx = X;
//         lasty = Y;
//         lastforward = forward;
//         lasttheta = theta;
//         laststrafe = strafe;
//         lastTime = currentTime;
//     }
// }
// void setup() {
//   // put your setup code here, to run once:
//     Serial.begin(115200);  // Higher baud rate for better data transmission
//     // Configure encoder pins
//     pinMode(ENCODER1_A, INPUT_PULLUP);
//     pinMode(ENCODER1_B, INPUT_PULLUP);
//     pinMode(ENCODER2_A, INPUT_PULLUP);
//     pinMode(ENCODER2_B, INPUT_PULLUP);
//     pinMode(ENCODER3_A, INPUT_PULLUP);
//     pinMode(ENCODER3_B, INPUT_PULLUP);
//     // Attach interrupts
//     attachInterrupt(digitalPinToInterrupt(ENCODER1_A), encoder1Interrupt, RISING);
//     attachInterrupt(digitalPinToInterrupt(ENCODER2_A), encoder2Interrupt, RISING);
//     attachInterrupt(digitalPinToInterrupt(ENCODER3_A), encoder3Interrupt, RISING);
//   // Initialize time

// }
// void loop() {
//     computeRobotMotion();
//     Serial.print("Encoder Ticks - 1: ");
//     Serial.print(L);
//     Serial.print(" 2: ");
//     Serial.print(R);
//     Serial.print(" 3: ");
//     Serial.print(B);    
//     Serial.print("(X,Y) = ");
//     Serial.print(X);
//     Serial.print(" ");
//     Serial.println(Y);
//     delay(100);  // Small delay for stability
// }

// // Interrupt handlers for encoders
// void encoder1Interrupt() {
//     encoder1Direction = (digitalRead(ENCODER1_A) == digitalRead(ENCODER1_B));
//     L += encoder1Direction ? 1 : -1;
// }

// void encoder2Interrupt() {
//     encoder2Direction = (digitalRead(ENCODER2_A) == digitalRead(ENCODER2_B));
//     R += encoder2Direction ? 1 : -1;
// }

// void encoder3Interrupt() {
//     encoder3Direction = (digitalRead(ENCODER3_A) == digitalRead(ENCODER3_B));
//     B += encoder3Direction ? 1 : -1;
// }