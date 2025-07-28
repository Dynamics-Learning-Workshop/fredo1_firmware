// gen by ChatGPT
const int servoPin = 11;
const int servoAnalogOut = A0;  // Connect to the potentiometer's wiper

// const int servoPin = 10;
// const int servoAnalogOut = A1;

// const int servoPin = 9;
// const int servoAnalogOut = A2;

void setup() {
  Serial.begin(9600);
  pinMode(servoPin, INPUT);  // disables driving the servo

  // pinMode(servoPin, OUTPUT);
}

void loop() {
    
  pinMode(servoPin, INPUT); 
  int potValue = analogRead(servoAnalogOut);
  Serial.println(potValue);
  delay(20);
  // alalal
  // // Sweep forward: 300 to 2600 µs
  for (int pulseWidth = 100; pulseWidth <= 2400; pulseWidth += 100) {
    for (int i = 0; i < 50; i++) {  // Send 50 frames (~1 second)
      digitalWrite(servoPin, HIGH);
      delayMicroseconds(pulseWidth);
      digitalWrite(servoPin, LOW);
      delay(20);  // 50 Hz = 20 ms
    }

    int potValue = analogRead(servoAnalogOut);
    Serial.print("Pulse: ");
    Serial.print(pulseWidth);
    Serial.print(" µs | Pot Value: ");
    Serial.println(potValue);
  }

  // Sweep backward
  for (int pulseWidth = 2400; pulseWidth >= 100; pulseWidth -= 100) {
    for (int i = 0; i < 50; i++) {
      digitalWrite(servoPin, HIGH);
      delayMicroseconds(pulseWidth);
      digitalWrite(servoPin, LOW);
      delay(20);
    }

    int potValue = analogRead(servoAnalogOut);
    Serial.print("Pulse: ");
    Serial.print(pulseWidth);
    Serial.print(" µs | Pot Value: ");
    Serial.println(potValue);
  }

  // while (1)
  // ;  // Stop loop
}