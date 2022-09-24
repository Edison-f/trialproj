#include <Servo.h>

Servo servo;

const int SERVO_PIN = 6;
const float modifier = 9.0 / 32.0;

void setup() {
  Serial.begin(9600);
  pinMode(SERVO_PIN, OUTPUT);
  servo.attach(SERVO_PIN);
}

// B10K
// 320 pot ticks is approx. horizontal  - 90
// 100 and 550 are approx. vertical     - 0, 180

void loop() {
  int sensor_value = analogRead(A0);
  // Serial.println(sensor_value); // Raw Pot Output
  int simulated_angle = map(sensor_value, 100, 550, 0, 180);
  // Limit Range of Simulated Value
  if(simulated_angle > 180) {
    simulated_angle = 180;
  } else if(simulated_angle < 0) {
    simulated_angle = 0;
  }
  // Serial.println((int) (sensor_value * modifier)); // Simulated Angle using modifer
  Serial.println(simulated_angle);
  servo.write(simulated_angle);
  delay(1);
}
