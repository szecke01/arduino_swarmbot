#include <TimerOne.h>

const int RED_LED_PIN = 15;
const int BLUE_LED_PIN = 14;
const int SENSOR_PIN = 0;
boolean output = HIGH;
void setup() {
  Serial.begin(9600);
  
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  
  Timer1.initialize(500000);
  Timer1.attachInterrupt(flash, 500000);
}

void loop() {
  // put your main code here, to run repeatedly:

}

void flash() {
  

  digitalWrite(BLUE_LED_PIN, output);
  digitalWrite(RED_LED_PIN, !output);
  if(output){
    Serial.print("blue led: ");
    Serial.println(analogRead(SENSOR_PIN));
  }
  if(!output){
    Serial.print("red led: ");
    Serial.println(analogRead(SENSOR_PIN));
  }
  output = !output;
}
