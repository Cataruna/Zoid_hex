#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Define servo min and max pulse lengths (tune based on your servos)
#define SERVOMIN 150  // Min pulse length
#define SERVOMAX 600  // Max pulse length

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing PCA9685...");
  
  pwm.begin();
  pwm.setPWMFreq(60); // Analog servos run at ~60 Hz
}

// Function to map angles (0-180) to PWM pulse width
int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

void loop() {
  // Example: Move servos independently
  pwm.setPWM(0, 0, angleToPulse(45));  // Servo on channel 0 to 45 degrees
  pwm.setPWM(1, 0, angleToPulse(90));  // Servo on channel 1 to 90 degrees
  pwm.setPWM(2, 0, angleToPulse(135)); // Servo on channel 2 to 135 degrees
  
  delay(1000); // Wait for 1 second
  
  // Move them to other positions
  pwm.setPWM(0, 0, angleToPulse(90));  // Servo on channel 0 to 90 degrees
  pwm.setPWM(1, 0, angleToPulse(45));  // Servo on channel 1 to 45 degrees
  pwm.setPWM(2, 0, angleToPulse(0));   // Servo on channel 2 to 0 degrees
  
  delay(1000); // Wait for 1 second
}
