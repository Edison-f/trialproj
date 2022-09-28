#include <Wire.h>
#include <Servo.h>
#include <LiquidCrystal.h>


Servo servo;

const int regSelect = 12, enable = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(regSelect, enable, d4, d5, d6, d7);

const int MPU = 0x68;  // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

float zeroed_roll;
float zeroed_pitch;
float zeroed_yaw;

float target_angle;
float user_angle = 0;
float angle_offset = 0;

const int SERVO_PIN = 6;
const float modifier = 9.0 / 32.0;

const int INTERRUPT_PIN = 7;

void setup() {
  Serial.begin(19200);
  Wire.begin();                 // Initialize comunication
  Wire.beginTransmission(MPU);  // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);             // Talk to the register 6B
  Wire.write(0x00);             // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);   //end the transmission
  delay(20);
  pinMode(SERVO_PIN, OUTPUT);
  servo.attach(SERVO_PIN);
  analogWrite(10, 225);
  // analogReference()
  lcd.begin(16, 2);
}

// B10K
// 320 pot ticks is approx. horizontal  - 90
// 100 and 550 are approx. vertical     - 0, 180

void loop() {  //0x68
  lcd.clear();
  // int pot_value = analogRead(A0);
  int clock_value = analogRead(A5);
  int data_value = analogRead(A4);
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);  // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;  // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;  // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;  // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58;       // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58;  // AccErrorY ~(-1.58)
  // === Read gyroscope data === //
  previousTime = currentTime;                         // Previous time is stored before the actual time read
  currentTime = millis();                             // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000;  // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43);  // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);                    // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;  // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  GyroX = GyroX + 0.56;  // GyroErrorX ~(-0.56)
  GyroY = GyroY - 2;     // GyroErrorY ~(2)
  GyroZ = GyroZ + 0.79;  // GyroErrorZ ~ (-0.8)
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime;  // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw = yaw + GyroZ * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;

  // Print the values on the serial monitor
  Serial.print(roll);
  Serial.print("/");
  Serial.print(pitch);
  Serial.print("/");
  Serial.println(yaw);

  if(Serial.read() == 'g') {
    if(yaw < 90) {
      angle_offset = 90.0 - yaw;
    } else {
      angle_offset = (-1.0 * yaw) + 90;
    }
  }

  
  int target_angle = yaw;
  target_angle += angle_offset;
  target_angle = (target_angle > 135) ? 135 : (target_angle < 45) ? 45 : target_angle;
  if(target_angle > 90) {
    target_angle -= (target_angle - 90.0) * 2;
  } else {
    target_angle += (90.0 - target_angle) * 2;
  }
  
  lcd.setCursor(0, 0);
  lcd.print(String(target_angle) + " " + angle_offset);
  lcd.setCursor(8, 0);
  lcd.print(user_angle);
  lcd.setCursor(0, 1);
  lcd.print("p:" + String(pitch) + " y:" + String(yaw));

  servo.write(target_angle);
  delay(10);
}