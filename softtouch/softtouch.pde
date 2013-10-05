nt softpotPin = A0; //analog pin 0
int led = 13;

void setup(){
  digitalWrite(softpotPin, HIGH); //enable pullup resistor
  Serial.begin(9600);
  pinMode(led, OUTPUT);
}

void loop(){
  int softpotReading = analogRead(softpotPin); 

  Serial.println(softpotReading);
  delay(250); //just here to slow down the output for easier reading
  digitalWrite(led,HIGH);
  
  softpotReading = analogRead(softpotPin); 
  Serial.println(softpotReading);
  
  delay(750); //just here to slow down the output for easier reading
  digitalWrite(led,LOW);
  
  
}
