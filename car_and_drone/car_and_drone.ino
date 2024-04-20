#define BLYNK_TEMPLATE_ID           "TMPL6SD_p2nw8"
#define BLYNK_TEMPLATE_NAME         "cardrone"

#define BLYNK_FIRMWARE_VERSION        "0.1.0"

#define BLYNK_PRINT Serial
//#define BLYNK_DEBUG

#define APP_DEBUG

// Uncomment your board, or configure a custom board in Settings.h
#define USE_ESP32_DEV_MODULE
//#define USE_ESP32C3_DEV_MODULE
//#define USE_ESP32S2_DEV_KIT
//#define USE_WROVER_BOARD
//#define USE_TTGO_T7
//#define USE_TTGO_T_OI

#include "BlynkEdgent.h"
#include <ESP32Servo.h>
#include <Wire.h>
#include <MPU6050_tockn.h>

MPU6050 mpu6050(Wire); 
Servo ESC;     // create servo object to control the ESC

// Define your MotorControl class as before
class MotorControl {
  private:
    int front_A_pin;
    int front_B_pin;
    int back_A_pin;
    int back_B_pin;
    unsigned long lastActionTime; // Track the time of the last motor action
    const unsigned long stopDelay = 500; // Delay in milliseconds before stopping the motor
    
  public:
    MotorControl(int frontA, int frontB, int backA, int backB) {
      front_A_pin = frontA;
      front_B_pin = frontB;
      back_A_pin = backA;
      back_B_pin = backB;
      lastActionTime = 0; // Initialize last action time to 0
      
      pinMode(front_A_pin, OUTPUT);
      pinMode(front_B_pin, OUTPUT);
      pinMode(back_A_pin, OUTPUT);
      pinMode(back_B_pin, OUTPUT);
    }

    void forward() {
      digitalWrite(front_A_pin, HIGH);
      digitalWrite(front_B_pin, LOW);
      digitalWrite(back_A_pin, HIGH);
      digitalWrite(back_B_pin, LOW);
      lastActionTime = millis(); // Update last action time
    }

    void backward() {
      digitalWrite(front_A_pin, LOW);
      digitalWrite(front_B_pin, HIGH);
      digitalWrite(back_A_pin, LOW);
      digitalWrite(back_B_pin, HIGH);
      lastActionTime = millis(); // Update last action time
    }

    void turnLeft() {
      digitalWrite(front_A_pin, LOW);
      digitalWrite(front_B_pin, HIGH);
      digitalWrite(back_A_pin, HIGH);
      digitalWrite(back_B_pin, LOW);
      lastActionTime = millis(); // Update last action time
    }

    void turnRight() {
      digitalWrite(front_A_pin, HIGH);
      digitalWrite(front_B_pin, LOW);
      digitalWrite(back_A_pin, LOW);
      digitalWrite(back_B_pin, HIGH);
      lastActionTime = millis(); // Update last action time
    }

    void stop() {
      digitalWrite(front_A_pin, LOW);
      digitalWrite(front_B_pin, LOW);
      digitalWrite(back_A_pin, LOW);
      digitalWrite(back_B_pin, LOW);
    }

};

class DroneControl{
  private:
    int bldc_a;
    int bldc_b;
    int bldc_c;
    int bldc_d;
    int throttle = 0; // 0 - 180 using ESP32Servo library

    // Define PID constants for each axis (you need to tune these)
    float Kp_roll = 1.0;
    float Ki_roll = 0.0;
    float Kd_roll = 0.0;

    float Kp_pitch = 1.0;
    float Ki_pitch = 0.0;
    float Kd_pitch = 0.0;

    float Kp_yaw = 1.0;
    float Ki_yaw = 0.0;
    float Kd_yaw = 0.0;

    float target_roll = 0.0;
    float target_pitch = 0.0;
    float target_yaw = 0.0;

    float current_roll = 0.0;
    float current_pitch = 0.0;
    float current_yaw = 0.0;

    float prev_error_roll = 0.0;
    float prev_error_pitch = 0.0;
    float prev_error_yaw = 0.0;

    float integral_roll = 0.0;
    float integral_pitch = 0.0;
    float integral_yaw = 0.0;

  public:
    DroneControl(int a, int b, int c, int d){
      bldc_a = a;
      bldc_b = b;
      bldc_c = c;
      bldc_d = d;

      ESC.attach(bldc_a,1000,2000);
      ESC.attach(bldc_b,1000,2000);
      ESC.attach(bldc_c,1000,2000);
      ESC.attach(bldc_d,1000,2000);
    }

    void test_throttle(){
      for(int i = 0 ; i < 150 ; i++){
        ESC.write(i);
        delay(20); 
      }
    }

    void throttleUp(){
      throttle += 1;
      ESC.write(throttle);
    }
    void throttleDown(){
      throttle -= 1;
      ESC.write(throttle);
    }
    void disarm(){
      throttle = 0;
      ESC.write(throttle);
    }
    void PIDbalance(){
      // Read accelerometer and gyro data
      mpu6050.update();
      current_roll = mpu6050.getAngleX();
      current_pitch = mpu6050.getAngleY();
      current_yaw = mpu6050.getAngleZ();

      // Calculate PID errors
      float error_roll = target_roll - current_roll;
      float error_pitch = target_pitch - current_pitch;
      float error_yaw = target_yaw - current_yaw;

      // Calculate PID components
      float P_roll = Kp_roll * error_roll;
      float I_roll = Ki_roll * integral_roll;
      float D_roll = Kd_roll * (error_roll - prev_error_roll);
      
      float P_pitch = Kp_pitch * error_pitch;
      float I_pitch = Ki_pitch * integral_pitch;
      float D_pitch = Kd_pitch * (error_pitch - prev_error_pitch);

      float P_yaw = Kp_yaw * error_yaw;
      float I_yaw = Ki_yaw * integral_yaw;
      float D_yaw = Kd_yaw * (error_yaw - prev_error_yaw);

      // Calculate PID output
      float pid_roll = P_roll + I_roll + D_roll;
      float pid_pitch = P_pitch + I_pitch + D_pitch;
      float pid_yaw = P_yaw + I_yaw + D_yaw;

      // Update previous error for next iteration
      prev_error_roll = error_roll;
      prev_error_pitch = error_pitch;
      prev_error_yaw = error_yaw;

      // Apply PID outputs to motor control
      // Adjust motor speeds based on pid_roll, pid_pitch, and pid_yaw
      // You need to implement this part based on your motor configuration
      // This could involve adjusting the throttle or pulse width of each motor

      // Example:
      // motorControl.adjustSpeed(pid_roll, pid_pitch, pid_yaw);

      delay(10); // Add a small delay to stabilize loop rate
    }
};

MotorControl motorControl(18, 19, 16, 4);
DroneControl droneControl(25, 33, 32, 5);


BLYNK_WRITE(V0) { // Forward button in Blynk app
  int state = param.asInt();
  if (state == HIGH) {
    motorControl.forward();
  }
  else{
    motorControl.stop();
  }
}

BLYNK_WRITE(V1) { // Backward button in Blynk app
  int state = param.asInt();
  if (state == HIGH) {
    motorControl.backward();
  }
    else{
    motorControl.stop();
  }
}

BLYNK_WRITE(V2) { // Left button in Blynk app
  int state = param.asInt();
  if (state == HIGH) {
    motorControl.turnLeft();
  }
  else{
    motorControl.stop();
  }
}

BLYNK_WRITE(V3) { // Right button in Blynk app
  int state = param.asInt();
  if (state == HIGH) {
    motorControl.turnRight();
  }
  else{
    motorControl.stop();
  }
}

BLYNK_WRITE(V4) { // test
  int state = param.asInt();
  if (state == HIGH) {
    droneControl.test_throttle();
  }
  else{

  }
}

BLYNK_WRITE(V5) { // throttle up
  int state = param.asInt();
  if (state == HIGH) {
    droneControl.throttleUp();
  }
  else{
    droneControl.PIDbalance();
  }
}

BLYNK_WRITE(V6) { // throttle down
  int state = param.asInt();
  if (state == HIGH) {
    droneControl.throttleDown();
  }
  else{
    droneControl.PIDbalance();
  }
}

BLYNK_WRITE(V7) { // disarm
  int state = param.asInt();
  if (state == HIGH) {
    droneControl.disarm();
  }
  else{
    
  }
}
void setup() {
  //Serial.begin(115200);
  delay(100);
  // Initialize MPU6050
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  BlynkEdgent.begin();
}

void loop() {

  BlynkEdgent.run();

}
