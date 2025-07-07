#include <Servo.h>

const int joint1_servo_pin = 9;
const int joint1_feedback_pin = A0;
float theta1 = 0;

const int joint2_servo_pin = 10;
const int joint2_feedback_pin = A1;
float theta2 = 0;

const int joint3_servo_pin = 11;
const int joint3_feedback_pin = A2;
float theta3 = 0;

float system_lastrequest = 0;
bool system_wake = false;

String input_from_serial_link;
String output_to_serial_link;

void setup() {
    system_lastrequest = millis();
    Serial.begin(9600);
}

void loop() {

    if (Serial.available())
        input_from_serial_link = Serial.readStringUntil('\n');
    
    if (input_from_serial_link.toInt() == 7777)
        system_wake = true;
    else if (input_from_serial_link.toInt() == 8888)
    {
        system_wake = false;
    }
    else
    {
        // do control shit here
    }                                        

    if ( (millis() - system_lastrequest) > 10)
    {
        output_to_serial_link = "";

        if (system_wake)
        {
            theta1 = analogRead(joint1_feedback_pin);
            theta2 = analogRead(joint1_feedback_pin);
            theta3 = analogRead(joint1_feedback_pin);

            output_to_serial_link = String(millis()) + "-" + theta1 + "-" + theta2 + "-" + theta3 + "\n";
        }            
        else
            output_to_serial_link = String(millis()) + "-0-0-0-0\n";

        Serial.println(output_to_serial_link);
        system_lastrequest = millis();

        return;
    }
}
