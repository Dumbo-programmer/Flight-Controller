#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>

// MPU6050 object
MPU6050 mpu;

// PID control variables
double roll, pitch, yaw;
double roll_setpoint = 0.0;
double pitch_setpoint = 0.0;
double yaw_setpoint = 0.0;
double roll_input, pitch_input, yaw_input;
double roll_output, pitch_output, yaw_output;

// PID controllers
PID roll_pid(&roll_input, &roll_output, &roll_setpoint, 1.0, 0.0, 0.0, DIRECT);
PID pitch_pid(&pitch_input, &pitch_output, &pitch_setpoint, 1.0, 0.0, 0.0, DIRECT);
PID yaw_pid(&yaw_input, &yaw_output, &yaw_setpoint, 1.0, 0.0, 0.0, DIRECT);

// Motor control variables
const int motor1Pin = 9;
const int motor2Pin = 10;
const int motor3Pin = 11;
const int motor4Pin = 3;

// Define PWM limits
const int PWM_MIN = 1000;
const int PWM_MAX = 2000;
const int PWM_NEUTRAL = (PWM_MAX + PWM_MIN) / 2;

void setup() {
    Wire.begin();
    Serial.begin(115200);

    // Initialize MPU6050
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
        while (1);
    }

    // Initialize PID controllers
    roll_pid.SetMode(AUTOMATIC);
    pitch_pid.SetMode(AUTOMATIC);
    yaw_pid.SetMode(AUTOMATIC);

    // Set PWM pins as outputs
    pinMode(motor1Pin, OUTPUT);
    pinMode(motor2Pin, OUTPUT);
    pinMode(motor3Pin, OUTPUT);
    pinMode(motor4Pin, OUTPUT);
}

void loop() {
    // Read sensor data
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Convert gyro data to angular rates
    roll_input = gx / 65.5;
    pitch_input = gy / 65.5;
    yaw_input = gz / 65.5;

    // Compute PID outputs
    roll_pid.Compute();
    pitch_pid.Compute();
    yaw_pid.Compute();

    // Calculate motor PWM values
    int motor1_pwm = constrain_pwm(PWM_NEUTRAL + roll_output + pitch_output + yaw_output);
    int motor2_pwm = constrain_pwm(PWM_NEUTRAL - roll_output + pitch_output - yaw_output);
    int motor3_pwm = constrain_pwm(PWM_NEUTRAL - roll_output - pitch_output + yaw_output);
    int motor4_pwm = constrain_pwm(PWM_NEUTRAL + roll_output - pitch_output - yaw_output);

    // Update motor PWM
    analogWrite(motor1Pin, motor1_pwm);
    analogWrite(motor2Pin, motor2_pwm);
    analogWrite(motor3Pin, motor3_pwm);
    analogWrite(motor4Pin, motor4_pwm);

    // Print sensor data and PID output for debugging
    Serial.print("Roll: "); Serial.print(roll_input);
    Serial.print(" Pitch: "); Serial.print(pitch_input);
    Serial.print(" Yaw: "); Serial.print(yaw_input);
    Serial.print(" Roll Output: "); Serial.print(roll_output);
    Serial.print(" Pitch Output: "); Serial.print(pitch_output);
    Serial.print(" Yaw Output: "); Serial.println(yaw_output);

    delay(10);  // Delay for sensor update interval
}

int constrain_pwm(int pwm) {
    if (pwm < PWM_MIN) return PWM_MIN;
    if (pwm > PWM_MAX) return PWM_MAX;
    return pwm;
}
