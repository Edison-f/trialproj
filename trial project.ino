#include <Servo.h>
#include <LiquidCrystal.h>

Servo servo;

const int regSelect = 12, enable = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(regSelect, enable, d4, d5, d6, d7);

const int SERVO_PIN = 6;
const float modifier = 9.0 / 32.0;

void setup() {
  Serial.begin(9600);
  pinMode(SERVO_PIN, OUTPUT);
  servo.attach(SERVO_PIN);
  analogWrite(10, 20);

  lcd.begin(16, 2);
  // lcd.print("Hello World");
}

// B10K
// 320 pot ticks is approx. horizontal  - 90
// 100 and 550 are approx. vertical     - 0, 180

void loop() {
  lcd.clear();
  int sensor_value = analogRead(A0);
  // Serial.println(sensor_value); // Raw Pot Output
  int simulated_angle = map(sensor_value, 150, 850, 0, 180);
  // Limit Range of Simulated Value
  if(simulated_angle > 180) {
    simulated_angle = 180;
  } else if(simulated_angle < 0) {
    simulated_angle = 0;
  }
  // Serial.println((int) (sensor_value * modifier)); // Simulated Angle using modifer
  Serial.println(simulated_angle);
  servo.write(simulated_angle);
  lcd.setCursor(0, 0);
  lcd.print(sensor_value);
  lcd.setCursor(0, 1);
  lcd.print(simulated_angle);
  // delay(1);
}
