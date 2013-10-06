// Left and Right Turn Signal Buttons
const int rightButton = 2;
const int leftButton = 3;

// Speed of turn signal blinks
int blinkTime = 100;

// Turn Signal LEDS
int pinCount = 10;
int rightSignal[] = {
  11, 12, 13, 14
};

int leftSignal[] = {
  15, 16, 17, 18
};

// Setting Up Compontents
void setup() {
  // Hello Right and Left buttons
  pinMode(rightButton, INPUT);
  pinMode(leftButton, INPUT);
  
  // Hello LED Turn Signals
  for (int thisPin = pinCount - 1; thisPin >= 0; thisPin--)  {
    pinMode(rightSignal[thisPin], OUTPUT);  
    pinMode(leftSignal[thisPin], OUTPUT); 
  }
}

void loop() {
  // Master Clock to time events
  int clock;
  ++clock;
  
  // Save Button States
  int rightButtonState;
  int leftButtonState;
  
  // Read Button States
  rightButtonState = digitalRead(rightButton);
  leftButtonState = digitalRead(leftButton);
  
  if ( (rightButtonState == LOW) && (clock % 3 == 0) ) {
    digitalWrite(rightSignal[1], HIGH);
    delay(blinkTime);
    digitalWrite(rightSignal[1], LOW);
  }
  
  else if ( (leftButtonState == LOW) && (clock % 3 == 0) ){
    digitalWrite(rightSignal[1], HIGH);
    delay(blinkTime);
    digitalWrite(rightSignal[1], LOW);
  }
}
