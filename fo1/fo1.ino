#include <Servo.h>

const int joint1_servo_pin = 9;
const int joint1_feedback_pin = A0;
float theta1 = 0;
int joint1_pulse = 0;
Servo servo1;

const int joint2_servo_pin = 10;
const int joint2_feedback_pin = A1;
float theta2 = 0;
int joint2_pulse = 0;
Servo servo2;

const int joint3_servo_pin = 11;
const int joint3_feedback_pin = A2;
float theta3 = 0;
int joint3_pulse = 0;
Servo servo3;

float system_lastrequest = 0;
bool system_wake = false;

float ctrl_lastrequest = 0;

String input_from_serial_link;
String output_to_serial_link;

void setup() {
    system_lastrequest = millis();
    ctrl_lastrequest = millis();
    Serial.begin(9600);

    servo1.attach(joint1_servo_pin);
    servo2.attach(joint2_servo_pin);
    servo3.attach(joint3_servo_pin);
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
        // do control shit here -> decode shit
        input_from_serial_link;
        int firstDash = input_from_serial_link.indexOf('-');
        int secondDash = input_from_serial_link.indexOf('-', firstDash + 1);

            if (firstDash != -1 && secondDash != -1) 
            {
                joint1_pulse = input_from_serial_link.substring(0, firstDash).toInt();
                joint2_pulse = input_from_serial_link.substring(firstDash + 1, secondDash).toInt();
                joint3_pulse = input_from_serial_link.substring(secondDash + 1).toInt();
            }
    }

    if(
        (millis() - ctrl_lastrequest) > 20
    )
    {
        servo1.writeMicroseconds(joint1_pulse);
        servo2.writeMicroseconds(joint2_pulse);
        servo3.writeMicroseconds(joint3_pulse);       
        ctrl_lastrequest = millis(); 
    }

    // this is for sending to upper floor
    if ( 
        (millis() - system_lastrequest) > 10
    )
    {
        output_to_serial_link = "";

        if (system_wake)
        {
            theta1 = analogRead(joint1_feedback_pin);
            theta2 = analogRead(joint1_feedback_pin);
            theta3 = analogRead(joint1_feedback_pin);

            output_to_serial_link = 
                String(millis()) + "-" + 
                1 + "-" + 
                theta1 + "-" + 
                theta2 + "-" + 
                theta3 + "-" + 
                joint1_pulse + "-" +
                joint2_pulse + "-" +
                joint3_pulse + "-" +
                system_wake +
                "\n";
        }            
        else
            output_to_serial_link = String(millis()) + "-0-0-0-0\n";

        Serial.println(output_to_serial_link);
        system_lastrequest = millis();

        return;
    }
}
