/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
 
  This example code is in the public domain.
 */
 
// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int sigs[]         = {21, 20, 19, 18};
const int NUM_SIGS = 4;

// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  for(int i = 0; i < NUM_SIGS; i++)
  {
    pinMode(sigs[i], OUTPUT);
  }  
}

// the loop routine runs over and over again forever:
void loop() {
  digitalWrite(sigs[0], HIGH);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(sigs[1], HIGH);
  digitalWrite(sigs[2], HIGH);
  delay(1);
  digitalWrite(sigs[0], LOW);
  digitalWrite(sigs[1], LOW);
  digitalWrite(sigs[2], LOW);
  delay(1);
}
