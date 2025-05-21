#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

// Robot physical parameters
const float WHEEL_DIAMETER_MM = 150.0;
const float WHEEL_RADIUS_MM = WHEEL_DIAMETER_MM / 2.0;
const int ENCODER_RESOLUTION = 200;
const float MM_PER_TICK = (PI * WHEEL_DIAMETER_MM) / ENCODER_RESOLUTION;
const float ROBOT_RADIUS_MM = 75.0;

// Encoder pins
const int ENCODER1_A = 3;
const int ENCODER1_B = 4;
const int ENCODER2_A = 19;
const int ENCODER2_B = 7;
const int ENCODER3_A = 2;
const int ENCODER3_B = 24;

// Global position and motion state
struct RobotState {
    // Position
    double x = 0.0;        // mm
    double y = 0.0;        // mm
    double heading = 0.0;  // radians
    
    // Velocities
    double vx = 0.0;       // mm/s
    double vy = 0.0;       // mm/s
    double omega = 0.0;    // rad/s
};

// Sensor fusion parameters
const float ALPHA = 0.95;  // Complementary filter coefficient (trust in IMU vs encoders)
const int UPDATE_INTERVAL_MS = 10;

class RobotLocalization {
private:
    Adafruit_BNO055 imu;
    RobotState state;
    unsigned long lastUpdate;
    
    // Encoder states
    volatile long encoderCounts[3] = {0, 0, 0};
    long lastEncoderCounts[3] = {0, 0, 0};
    
    // IMU calibration status
    bool imuCalibrated = false;
    
    float normalizeAngle(float angle) {
        while (angle > PI) angle -= 2 * PI;
        while (angle < -PI) angle += 2 * PI;
        return angle;
    }
    
    void updateFromEncoders(float dt) {
        // Calculate wheel velocities
        float wheelVelocities[3];
        for(int i = 0; i < 3; i++) {
            long deltaTicks = encoderCounts[i] - lastEncoderCounts[i];
            wheelVelocities[i] = (deltaTicks * MM_PER_TICK) / dt;
            lastEncoderCounts[i] = encoderCounts[i];
        }
        
        // Convert wheel velocities to robot velocities using inverse kinematics
        float encoder_vx = (2.0/3.0) * (wheelVelocities[0] - 0.5*wheelVelocities[1] - 0.5*wheelVelocities[2]);
        float encoder_vy = (2.0/3.0) * (0.0*wheelVelocities[0] + 0.866*wheelVelocities[1] - 0.866*wheelVelocities[2]);
        float encoder_omega = (1.0/(3.0 * ROBOT_RADIUS_MM)) * 
                            (wheelVelocities[0] + wheelVelocities[1] + wheelVelocities[2]);
        
        // Store encoder-based velocities
        state.vx = encoder_vx;
        state.vy = encoder_vy;
        state.omega = encoder_omega;
    }
    
    void updateFromIMU() {
        imu::Vector<3> euler = imu.getVector(Adafruit_BNO055::VECTOR_EULER);
        float imu_heading = euler.x() * PI / 180.0;  // Convert to radians
        
        // Get angular velocity from gyroscope
        imu::Vector<3> gyro = imu.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        float imu_omega = gyro.z();  // Angular velocity around Z axis
        
        // Fuse heading using complementary filter
        state.heading = normalizeAngle(ALPHA * imu_heading + (1-ALPHA) * 
                       (state.heading + state.omega * (UPDATE_INTERVAL_MS/1000.0)));
        
        // Update angular velocity with IMU data
        state.omega = ALPHA * imu_omega + (1-ALPHA) * state.omega;
    }

public:
    RobotLocalization() : imu(55) {}
    
    bool init() {
        if (!imu.begin()) {
            Serial.println("Failed to initialize IMU!");
            return false;
        }
        
        imu.setExtCrystalUse(true);
        lastUpdate = millis();
        return true;
    }
    
    void update() {
        unsigned long now = millis();
        float dt = (now - lastUpdate) / 1000.0;
        
        if (dt >= UPDATE_INTERVAL_MS/1000.0) {
            // Update velocities from encoders
            updateFromEncoders(dt);
            
            // Update heading and angular velocity from IMU
            updateFromIMU();
            
            // Update position using fused data
            // Transform velocities based on current heading
            float cos_heading = cos(state.heading);
            float sin_heading = sin(state.heading);
            
            // Update global position
            state.x += (state.vx * cos_heading - state.vy * sin_heading) * dt;
            state.y += (state.vx * sin_heading + state.vy * cos_heading) * dt;
            
            lastUpdate = now;
            
            // Print debug information
            printState();
        }
    }
    
    void updateEncoder(int index, long count) {
        encoderCounts[index] = count;
    }
    
    void printState() {
        Serial.println("Robot State:");
        Serial.print("Position (mm) - X: ");
        Serial.print(state.x, 2);
        Serial.print(" Y: ");
        Serial.print(state.y, 2);
        Serial.print(" Heading (deg): ");
        Serial.println(state.heading * 180.0/PI, 2);
        
        Serial.print("Velocity (mm/s) - X: ");
        Serial.print(state.vx, 2);
        Serial.print(" Y: ");
        Serial.print(state.vy, 2);
        Serial.print(" Angular (rad/s): ");
        Serial.println(state.omega, 2);
        
        // Print IMU calibration status
        uint8_t system, gyro, accel, mag;
        imu.getCalibration(&system, &gyro, &accel, &mag);
        Serial.print("IMU Calibration - Sys:");
        Serial.print(system);
        Serial.print(" Gyro:");
        Serial.print(gyro);
        Serial.print(" Accel:");
        Serial.print(accel);
        Serial.print(" Mag:");
        Serial.println(mag);
        Serial.println();
    }
    
    const RobotState& getState() const {
        return state;
    }
};

RobotLocalization localization;

void setup() {
    Serial.begin(115200);
    
    // Initialize encoders
    pinMode(ENCODER1_A, INPUT_PULLUP);
    pinMode(ENCODER1_B, INPUT_PULLUP);
    pinMode(ENCODER2_A, INPUT_PULLUP);
    pinMode(ENCODER2_B, INPUT_PULLUP);
    pinMode(ENCODER3_A, INPUT_PULLUP);
    pinMode(ENCODER3_B, INPUT_PULLUP);
    
    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(ENCODER1_A), encoder1ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER2_A), encoder2ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER3_A), encoder3ISR, RISING);
    
    if (!localization.init()) {
        Serial.println("Localization initialization failed!");
        while(1);
    }
    
    Serial.println("Robot Localization System Initialized");
}

void loop() {
    localization.update();
    delay(10);
}

// Interrupt handlers
volatile long encoder1Ticks = 0;
volatile long encoder2Ticks = 0;
volatile long encoder3Ticks = 0;

void encoder1ISR() {
    encoder1Ticks += (digitalRead(ENCODER1_A) == digitalRead(ENCODER1_B)) ? 1 : -1;
    localization.updateEncoder(0, encoder1Ticks);
}

void encoder2ISR() {
    encoder2Ticks += (digitalRead(ENCODER2_A) == digitalRead(ENCODER2_B)) ? 1 : -1;
    localization.updateEncoder(1, encoder2Ticks);
}

void encoder3ISR() {
    encoder3Ticks += (digitalRead(ENCODER3_A) == digitalRead(ENCODER3_B)) ? 1 : -1;
    localization.updateEncoder(2, encoder3Ticks);
}