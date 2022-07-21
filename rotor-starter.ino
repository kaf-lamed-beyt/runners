#include<Servo.h>

Servo esc_forward_one;
Servo esc_forward_two;
Servo esc_rear;

void setup() {
  esc_forward.attach(8);
  esc_rear.attach(9);
  esc_forward.writeMicroseconds(1000);
  esc_rear.writeMicroseconds(1000);
  Serial.begin(9600);
}

void loop() {
  int val;
  val = analogRead(A0);
  val = map(val, 0, 1023, 1000, 6000);
  esc_forward.writeMicroseconds(val);
  esc_rear.writeMicroseconds(val);
}