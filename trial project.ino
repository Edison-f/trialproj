#include <Wire.h>
#include <Servo.h>
#include <LiquidCrystal.h>


Servo servo;
const int SERVO_PIN = 6;
const int SERVO_OFFSET = 15;

//LCD Pins
const int REG_SELECT_PIN = 12, ENABLE = 11;
const int d4 = 5, d5 = 4, d6 = 3, d7 = 2;

LiquidCrystal lcd(REG_SELECT_PIN, ENABLE, d4, d5, d6, d7);

const byte num_chars = 32;
char recieved_chars[num_chars];   // an array to store the received data

boolean new_data = false;

int data_as_num = 0;

const int MPU = 0x68;  // MPU6050 I2C address
float accel_x, accel_y, accel_z;
float raw_angle_x, raw_angle_y, raw_angle_z;
float accel_angle_x, accel_angle_y, angle_x, angle_y, gyroAngleZ;
float roll, pitch, yaw;
float elapsed_time, curr_time, prev_time;

int alt_angle; // Accel Angle

int user_angle = 90;
float angle_offset = 0;
float angle_offset_alt = 0;

int target_angle = 0;

// Need to set Serial Monitor to New Line and 9600 baud for input and outout 

void setup() {
  Serial.begin(9600);

  Wire.begin();                 // Initialize comunication
  Wire.beginTransmission(MPU);  // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);             // Talk to the register 6B
  Wire.write(0x00);             // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);   //end the transmission

  // Setup servo
  pinMode(SERVO_PIN, OUTPUT);
  servo.attach(SERVO_PIN);

  ProcessGyro();
  CalculateAngle();
  ZeroSensors();

  // Setup LCD
  analogWrite(10, 225);
  lcd.begin(16, 2);
  
  Serial.println("Start");
}

void loop() {  //0x68
  lcd.clear();
  
  ProcessInput();
  ProcessGyro();
  
  // alt_angle = map((AccY * 100), -100, 100, 180, 0); // Map acceleration values to degrees

  CalculateAngle();

  
  servo.write(target_angle - SERVO_OFFSET);
}

void UpdateLCD() {
  lcd.setCursor(0, 0);
  lcd.print(String(target_angle) + " " + angle_offset);
  lcd.setCursor(12, 0);
  lcd.print(user_angle);
  lcd.setCursor(0, 1);
  lcd.print("p:" + String(pitch) + " y:" + String(yaw));
}

void ProcessGyro() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);  // Read 6 registers total, each axis value is stored in 2 registers
  
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  accel_x = (Wire.read() << 8 | Wire.read()) / 16384.0;  // X-axis value
  accel_y = (Wire.read() << 8 | Wire.read()) / 16384.0;  // Y-axis value
  accel_z = (Wire.read() << 8 | Wire.read()) / 16384.0;  // Z-axis value
  
  // Calculating Roll and Pitch from the accelerometer data
  accel_angle_x = (atan(accel_y / sqrt(pow(accel_x, 2) + pow(accel_z, 2))) * 180 / PI) - 0.58;
  accel_angle_y = (atan(-1 * accel_x / sqrt(pow(accel_y, 2) + pow(accel_z, 2))) * 180 / PI) + 1.58;
  
  // === Read gyroscope data === //
  prev_time = curr_time;                         // Previous time is stored before the actual time read
  curr_time = millis();                             // Current time actual time read
  elapsed_time = (curr_time - prev_time) / 1000;  // Divide by 1000 to get seconds

  Wire.beginTransmission(MPU);
  Wire.write(0x43);  // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);                    // Read 4 registers total, each axis value is stored in 2 registers
  raw_angle_x = (Wire.read() << 8 | Wire.read()) / 131.0;  // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  raw_angle_y = (Wire.read() << 8 | Wire.read()) / 131.0;
  raw_angle_z = (Wire.read() << 8 | Wire.read()) / 131.0;
  
  // Correct the outputs with error offset values
  raw_angle_x += 2.72;
  raw_angle_y -= 1.45;
  raw_angle_z -= 0.485;

  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  angle_x = angle_x + raw_angle_x * elapsed_time;  // deg/s * s = deg
  angle_y = angle_y + raw_angle_y * elapsed_time;
  yaw = yaw + raw_angle_z * elapsed_time;

  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.96 * angle_x + 0.04 * accel_angle_x;
  pitch = 0.96 * angle_y + 0.04 * accel_angle_y;

  Serial.println("x: " + String(roll) + " \ty: " + String(pitch) + " \tz: " + String(yaw));
}

void CalculateAngle() {
  target_angle = (yaw + angle_offset);
  
  CheckAngle();

  if(user_angle < 90) {
    target_angle += (90 - user_angle);
  } else {
    target_angle -= (user_angle - 90);
  }

  target_angle = 90 + (90 - target_angle);
  
  CheckAngle();
}

void CheckAngle() {
  target_angle = (target_angle > 185) ? 185 : (target_angle < 25) ? 25
                                                                  : target_angle;
}

void ProcessInput() {
  RecieveWithEndMarker();
  SetAngle();
  ZeroSensors();
}

void RecieveWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
    
    if (Serial.available() > 0) {
        rc = Serial.read();

        if (rc != endMarker) {
            recieved_chars[ndx] = rc;
            ndx++;
            if (ndx >= num_chars) {
                ndx = num_chars - 1;
            }
        }
        else {
            recieved_chars[ndx] = '\0'; // terminate the string
            ndx = 0;
            new_data = true;
        }
    }
}

void SetAngle() {
  if(new_data && recieved_chars[0] != 'z') {
    user_angle = 0;
    user_angle = atoi(recieved_chars);
    Serial.println("New angle: " + String(recieved_chars));
    new_data = false;
  }
}

void ZeroSensors() {
  if(new_data && recieved_chars[0] == 'z') {
    if (alt_angle < 90) {
      angle_offset_alt = 90.0 - alt_angle;
    } else {
      angle_offset_alt = (-1.0 * alt_angle) + 90;
    }
    if (yaw < 90) {
      angle_offset = 90.0 - yaw;
    } else {
      angle_offset = (-1.0 * yaw) + 90;
    }
    Serial.println(String(angle_offset) + ", " + String(angle_offset_alt) + " Zeroed");
    new_data = false;
  }
}
