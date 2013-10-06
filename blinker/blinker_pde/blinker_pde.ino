/*
SparkFun Inventor's Kit
Example sketch 05

PUSH BUTTONS

  Use pushbuttons for digital input

  Previously we've used the analog pins for input, now we'll use
  the digital pins for input as well. Because digital pins only
  know about HIGH and LOW signals, they're perfect for interfacing
  to pushbuttons and switches that also only have "on" and "off"
  states.
  
  We'll connect one side of the pushbutton to GND, and the other
  side to a digital pin. When we press down on the pushbutton,
  the pin will be connected to GND, and therefore will be read
  as "LOW" by the Arduino.
  
  But wait - what happens when you're not pushing the button?
  In this state, the pin is disconnected from everything, which 
  we call "floating". What will the pin read as then, HIGH or LOW?
  It's hard to say, because there's no solid connection to either
  5 Volts or GND. The pin could read as either one.
  
  To deal with this issue, we'll connect a small (10K, or 10,000 Ohm)
  resistance between the pin and 5 Volts. This "pullup" resistor
  will ensure that when you're NOT pushing the button, the pin will
  still have a weak connection to 5 Volts, and therefore read as
  HIGH.
  
  (Advanced: when you get used to pullup resistors and know when
  they're required, you can activate internal pullup resistors
  on the ATmega processor in the Arduino. See
  http://arduino.cc/en/Tutorial/DigitalPins for information.)

Hardware connections:

  Pushbuttons:
  
    Pushbuttons have two contacts that are connected if you're
    pushing the button, and disconnected if you're not.
    
    The pushbuttons we're using have four pins, but two pairs
    of these are connected together. The easiest way to hook up
    the pushbutton is to connect two wires to any opposite corners.

    Connect any pin on pushbutton 1 to ground (GND).
    Connect the opposite diagonal pin of the pushbutton to
    digital pin 2.

    Connect any pin on pushbutton 2 to ground (GND).
    Connect the opposite diagonal pin of the pushbutton to
    digital pin 3.

    Also connect 10K resistors (brown/black/red) between
    digital pins 2 and 3 and GND. These are called "pullup"
    resistors. They ensure that the input pin will be either
    5V (unpushed) or GND (pushed), and not somewhere in between.
    (Remember that unlike analog inputs, digital inputs are only
    HIGH or LOW.)

  LED:
  
    Most Arduinos, including the Uno, already have an LED
    and resistor connected to pin 13, so you don't need any
    additional circuitry.

    But if you'd like to connect a second LED to pin 13,

    Connect the positive side of your LED to Arduino digital pin 13
    Connect the negative side of your LED to a 330 Ohm resistor
    Connect the other side of the resistor to ground

This sketch was written by SparkFun Electronics,
with lots of help from the Arduino community.
This code is completely free for any use.
Visit http://learn.sparkfun.com/products/2 for SIK information.
Visit http://www.arduino.cc to learn about the Arduino.

Version 2.0 6/2012 MDG
*/


// First we'll set up constants for the pin numbers.
// This will make it easier to follow the code below.

const int buttonPin = 2;  // pushbutton 1 pin

int timer = 200;             // The higher the number, the slower the timing.
int ledPins[] = { 
     11, 12, 13 };          // an array of pin numbers to which LEDs are attached
int pinCount = 3;           // the number of pins (i.e. the length of the array)

void setup()
{
  // Set up the pushbutton pins to be an input:
  pinMode(buttonPin, INPUT);

  for (int thisPin = pinCount - 1; thisPin >= 0; thisPin--)  {
    pinMode(ledPins[thisPin], OUTPUT);      
  }
}


void loop()
{
  int buttonState;  // variables to hold the pushbutton states

  // Since a pushbutton has only two states (pushed or not pushed),
  // we've run them into digital inputs. To read an input, we'll
  // use the digitalRead() function. This function takes one
  // parameter, the pin number, and returns either HIGH (5V)
  // or LOW (GND).

  // Here we'll read the current pushbutton states into
  // two variables:

  buttonState = digitalRead(buttonPin);

  // Remember that if the button is being pressed, it will be
  // connected to GND. If the button is not being pressed,
  // the pullup resistor will connect it to 5 Volts.

  // So the state will be LOW when it is being pressed,
  // and HIGH when it is not being pressed.
  
  // Now we'll use those states to control the LED.
  // Here's what we want to do:
  
  // "If either button is being pressed, light up the LED"
  // "But, if BOTH buttons are being pressed, DON'T light up the LED"
  
  // Let's translate that into computer code. The Arduino gives you
  // special logic functions to deal with true/false logic:
  
  // A == B means "EQUIVALENT". This is true if both sides are the same.
  // A && B means "AND". This is true if both sides are true.
  // A || B means "OR". This is true if either side is true.
  // !A means "NOT". This makes anything after it the opposite (true or false).
  
  // We can use these operators to translate the above sentences
  // into logic statements (Remember that LOW means the button is
  // being pressed)
  
  // "If either button is being pressed, light up the LED"
  // becomes:
  // if ((button1State == LOW) || (button2State == LOW)) // light the LED
  
  // "If BOTH buttons are being pressed, DON'T light up the LED"
  // becomes:
  // if ((button1State == LOW) && (button2State == LOW)) // don't light the LED

  // Now let's use the above functions to combine them into one statement:
  
  if (buttonState == LOW)
  {
      for (int thisPin = 0; thisPin < pinCount; thisPin++) { 
      // turn the pin on:
      digitalWrite(ledPins[thisPin], HIGH);   
      delay(timer);                  
      // turn the pin off:
      digitalWrite(ledPins[thisPin], LOW);
      }

  }
    
    
//    digitalWrite(ledPin1, HIGH);   // turn the LED on (HIGH is the voltage level)
//    delay(300);
//    digitalWrite(ledPin1, LOW);    // turn the LED off by making the voltage LOW
//    digitalWrite(ledPin2, HIGH);   // turn the LED on (HIGH is the voltage level)
//    delay(300);
//    digitalWrite(ledPin2, LOW);    // turn the LED off by making the voltage LOW
//    digitalWrite(ledPin3, HIGH);   // turn the LED on (HIGH is the voltage level)
//    delay(300);
//    digitalWrite(ledPin3, LOW);    // turn the LED off by making the voltage LOW
//  }
  else
  {
  }
     	
  // As you can see, logic operators can be combined to make
  // complex decisions!

  // Don't forget that we use = when we're assigning a value,
  // and use == when we're testing a value for equivalence.
}
