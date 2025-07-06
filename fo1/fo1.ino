#include <Servo.h>

const int servoPin = 11;
const int servoAnalogOut = A0;  // Connect to the potentiometer's wiper

// const int servoPin = 10;
// const int servoAnalogOut = A1;

// const int servoPin = 9;
// const int servoAnalogOut = A2;

float feedback_lastrequest = 0;
float theta0 = 0;

void setup() {
    feedback_lastrequest = millis();
    Serial.begin(9600);
}

void loop() {

    if ( (millis() - feedback_lastrequest) > 20)
    {
        theta0 = analogRead(servoAnalogOut);
        Serial.println(theta0);
    }

    if (Serial.available()) {
        String incoming = Serial.readStringUntil('\n');
        Serial.println("\n\nGot: " + incoming);
    }
}
