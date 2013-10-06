#include <Servo.h>

Servo servo;
const int button = 2;

void setup() {
  pinMode(button, INPUT);
  servo.attach(9);
  delay(2000);
  Serial.println("set up");
}

void loop() {
  int buttonState;
  int position;
  
  buttonState = digitalRead(button);
  Serial.println(buttonState);
  Serial.println(position);
  
  if (buttonState == LOW) {
    servo.write(180);
    delay(1000);
    servo.write(0);
    Serial.println("PRESSED ME!");
  }
  
  else {
    Serial.println("PUSH ME!");
  }
}
