#include <TimerOne.h>

const int RED_LED_PIN = 15;
const int BLUE_LED_PIN = 14;
const int SENSOR_PIN = 0;
const int LED_FLASH_PERIOD = 50000000;
const int MIN_DIFF_THRESHOLD = 20;

boolean output = LOW;
int last_red   = -1;
int last_blue  = -1;
int color_diff = 0;

void setup() {
  Serial.begin(9600);
  
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  
  Timer1.initialize(500000);
  Timer1.attachInterrupt(flash, LED_FLASH_PERIOD);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  int light_intensity = analogRead(SENSOR_PIN);
 

}

void flash() {
  
  output = !output;
  digitalWrite(BLUE_LED_PIN, output);
 // digitalWrite(RED_LED_PIN, !output);
  if(!output)
  last_blue = analogRead(SENSOR_PIN);
  
  //Serial.print("The red_value is: ");
  //Serial.println(last_red);
  Serial.print("The blue_value is: ");
  Serial.println(last_blue);
  if (last_blue > 0 && last_red > 0)
  {
    color_diff = last_blue - last_red;
  }
  
  
}

