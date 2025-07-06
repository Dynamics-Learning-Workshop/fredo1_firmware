#include <Servo.h>

Servo myServo;

// Adjust these to your servo's specs
const int minPulse = 500;    // microseconds for 0°
const int maxPulse = 2500;   // microseconds for 270°

void setup() {
  myServo.attach(9); // attach servo to pin 9
}

void loop() {
  int angle = 0; // example angle 0 to 270

  // // Map angle 0-270 to pulse width 500-2500 us
  
  
  unsigned long start = millis();

  int pulse = map(0, 0, 270, minPulse, maxPulse);
  myServo.writeMicroseconds(pulse);
  delay(15); // servo pulse refresh rate

  pulse = map(270, 0, 270, minPulse, maxPulse);
  myServo.writeMicroseconds(pulse);

  unsigned long end = millis();
  // Serial.println(end - start);
  Serial.println(1);

  delay(15); // servo pulse refresh rate
}
