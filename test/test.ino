// gen by ChatGPT
#include <Servo.h>

const int servoPin = 11;
const int servoAnalogOut = A0;  // Connect to the potentiometer's wiper

Servo servo1;
void setup() {
  Serial.begin(9600);
  servo1.attach(servoPin);
}

void loop() {
  // Sweep forward: 500 to 2500 µs
  for (int pulseWidth = 500; pulseWidth <= 2500; pulseWidth += 10) 
  {    
    servo1.writeMicroseconds(pulseWidth);
    int potValue = analogRead(servoAnalogOut);
    Serial.print("Pulse: ");
    Serial.print(pulseWidth);
    Serial.print(" µs | Pot Value: ");
    Serial.println(potValue);

    delay(100);
  }

  // Sweep backward: 2500 to 500 µs
  for (int pulseWidth = 2500; pulseWidth >= 500; pulseWidth -= 10) {
    servo1.writeMicroseconds(pulseWidth);
    int potValue = analogRead(servoAnalogOut);
    Serial.print("Pulse: ");
    Serial.print(pulseWidth);
    Serial.print(" µs | Pot Value: ");
    Serial.println(potValue);

    delay(100);
  }
}
