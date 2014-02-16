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
  
  if(color_diff > MIN_DIFF_THRESHOLD)
    Serial.println("We are on red paper! :)");
  if(color_diff < -1*MIN_DIFF_THRESHOLD)
    Serial.println("We are on blue paper! :)");

}

void flash() {
  output = !output;
  digitalWrite(BLUE_LED_PIN, output);
  digitalWrite(RED_LED_PIN, !output);
  
  if(!output) last_red = analogRead(SENSOR_PIN);
  else last_blue = analogRead(SENSOR_PIN);
  
  if (last_blue > 0 && last_red > 0)
  {
    color_diff = last_blue - last_red;
  }
  
}

