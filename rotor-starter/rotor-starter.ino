#include<Servo.h>

Servo esc_forward_one;
Servo esc_forward_two;
Servo esc_rear;

void setup() {
  // connecting all Electronic Speed Controllers to receive signals from the pins  
  esc_forward_one.attach(8);
  esc_forward_two.attach(9);
  esc_rear.attach(10);
  
  // add the delays to all inputs
  esc_forward_one.writeMicroseconds(10000);
  esc_forward_two.writeMicroseconds(10000);
  esc_rear.writeMicroseconds(10000);
  
  // start
  Serial.begin(9600);
}

void loop() {
  int val;
  val = analogRead(A0);
  val = map(val, 0, 1023, 1000, 20000);
  esc_forward_one.writeMicroseconds(val);
  esc_forward_two.writeMicroseconds(val);
  esc_rear.writeMicroseconds(val);
}