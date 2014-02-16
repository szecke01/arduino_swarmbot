#include <TimerOne.h>

/************************
  *  Authors: Sam Z     *
  *           Cooper L  *
  *           Chris R   *
  *  Date: 2/4/2014     *
  *  Project:           *
  *    H-Bridge Arduino *
  *    Motor Control    *
  ***********************/

/* TODO:
  - Change color_diff to buffer for average
    * this prevents noise from triggering unwanted state-changes
  - Separate 9 and 6 V power supplies
  - Check the motor (why is it sticking??)
*/

// Motor Control Pins
const int MOTOR_LEFT_F  = 4;  
const int MOTOR_LEFT_R  = 2;
const int MOTOR_RIGHT_F = 5;
const int MOTOR_RIGHT_R = 3;

// State values
const int STATE_STOPPED    = 0;
const int STATE_FORWARD    = 1;
const int STATE_REVERSE    = 2;
const int STATE_PIVOT_CCW  = 3;
const int STATE_PIVOT_CW   = 4;
const int STATE_TURN_CCW   = 5;
const int STATE_TURN_CW    = 6;

// Motor Constants
const float TURN_SPEED_RATIO = .25;
const int STATE_CHANGE_DELAY = 2000;
const int PIVOT_TIME_90 = 630;
const int PIVOT_TIME_60 = 420;

// Color Sensor Constants
const int RED_LED_PIN = 15;
const int BLUE_LED_PIN = 14;
const int COLOR_SENSOR_PIN = 0;
const int LED_FLASH_PERIOD = 50000000;
const int MIN_DIFF_THRESHOLD = 20;

// Color sensor variables
boolean color_sensor_output = LOW;
int last_red   = -1;
int last_blue  = -1;
int color_diff = 0;

// Current arduino state
int current_state;
float motor_duty_cycle;

void setup() {
  
  // Establish color params
  Serial.begin(9600);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  Timer1.initialize(500000);
  Timer1.attachInterrupt(flash, LED_FLASH_PERIOD);
  
  // Establish H-Bridge inputs as outputs from arduino
  pinMode(MOTOR_LEFT_F,  OUTPUT);
  pinMode(MOTOR_LEFT_R,  OUTPUT);
  pinMode(MOTOR_RIGHT_F, OUTPUT);
  pinMode(MOTOR_RIGHT_R, OUTPUT);
  
  // Initialize state machine to desired initial state
  set_state(STATE_FORWARD);
  motor_duty_cycle = .5;  
  
  //pinMode(motor_enable, INPUT_PULLUP);
  //digitalWrite(motor_pin, LOW);
}

void loop() {
  
  // Use color info to set state
  if(color_diff > MIN_DIFF_THRESHOLD)
    Serial.println("We are on red paper! :)");
  if(color_diff < -1*MIN_DIFF_THRESHOLD)
    {
      set_state(STATE_STOPPED);
      Serial.println("We are on blue paper! :)");
    }
  
  // Iterate through FSM and perform current state
  // Not using else-if's in case interrupt causes 
  // state changes
  if (current_state == STATE_STOPPED)
  {
    analogWrite(MOTOR_LEFT_F,  duty_cycle_to_byte(0));
    analogWrite(MOTOR_LEFT_R,  duty_cycle_to_byte(0));
    analogWrite(MOTOR_RIGHT_F, duty_cycle_to_byte(0));
    analogWrite(MOTOR_RIGHT_R, duty_cycle_to_byte(0));
    delay(1000);
    set_state(STATE_STOPPED);
  }
  
  if (current_state == STATE_FORWARD)
  {
    analogWrite(MOTOR_LEFT_F,  duty_cycle_to_byte(motor_duty_cycle));
    analogWrite(MOTOR_LEFT_R,  duty_cycle_to_byte(0));
    analogWrite(MOTOR_RIGHT_F, duty_cycle_to_byte(motor_duty_cycle));
    analogWrite(MOTOR_RIGHT_R, duty_cycle_to_byte(0));
    //delay(5000);
    //set_state(STATE_REVERSE);
  }
  
  if (current_state == STATE_REVERSE)
  {
    analogWrite(MOTOR_LEFT_F,  duty_cycle_to_byte(0));
    analogWrite(MOTOR_LEFT_R,  duty_cycle_to_byte(motor_duty_cycle));
    analogWrite(MOTOR_RIGHT_F, duty_cycle_to_byte(0));
    analogWrite(MOTOR_RIGHT_R, duty_cycle_to_byte(motor_duty_cycle));
    delay(5000);
    set_state(STATE_STOPPED);
  }  
  
  if (current_state == STATE_PIVOT_CW)
  {
    pivot_cw(PIVOT_TIME_60);
    set_state(STATE_STOPPED); 
  }
  
  if (current_state == STATE_PIVOT_CCW)
  {
    pivot_ccw(PIVOT_TIME_90);
    set_state(STATE_PIVOT_CW);
  }
  if (current_state == STATE_TURN_CW)
  {
    analogWrite(MOTOR_LEFT_F,  duty_cycle_to_byte(motor_duty_cycle*TURN_SPEED_RATIO));
    analogWrite(MOTOR_LEFT_R,  duty_cycle_to_byte(0));
    analogWrite(MOTOR_RIGHT_F, duty_cycle_to_byte(motor_duty_cycle));
    analogWrite(MOTOR_RIGHT_R, duty_cycle_to_byte(0));    
    delay(1000);
    set_state(STATE_PIVOT_CCW);
  }
  if (current_state == STATE_TURN_CCW)
  {
    analogWrite(MOTOR_LEFT_F,  duty_cycle_to_byte(motor_duty_cycle));
    analogWrite(MOTOR_LEFT_R,  duty_cycle_to_byte(0));
    analogWrite(MOTOR_RIGHT_F, duty_cycle_to_byte(motor_duty_cycle*TURN_SPEED_RATIO));
    analogWrite(MOTOR_RIGHT_R, duty_cycle_to_byte(0));    
    delay(1000);
    set_state(STATE_TURN_CW);
  }
}

// Sets state and gives delay
void set_state(int new_state)
{
  stop_motor();
  delay(STATE_CHANGE_DELAY);
  current_state = new_state;
}

// Sets analog out for motor to zero
void stop_motor()
{
  analogWrite(MOTOR_LEFT_F,  duty_cycle_to_byte(0));
  analogWrite(MOTOR_LEFT_R,  duty_cycle_to_byte(0));
  analogWrite(MOTOR_RIGHT_F, duty_cycle_to_byte(0));
  analogWrite(MOTOR_RIGHT_R, duty_cycle_to_byte(0));  
}

void pivot_cw(int pivot_time)
{
    analogWrite(MOTOR_LEFT_F,  duty_cycle_to_byte(motor_duty_cycle));
    analogWrite(MOTOR_LEFT_R,  duty_cycle_to_byte(0));
    analogWrite(MOTOR_RIGHT_F, duty_cycle_to_byte(0));
    analogWrite(MOTOR_RIGHT_R, duty_cycle_to_byte(motor_duty_cycle));    
    delay(pivot_time);
}

void pivot_ccw(int pivot_time)
{
    analogWrite(MOTOR_LEFT_F,  duty_cycle_to_byte(0));
    analogWrite(MOTOR_LEFT_R,  duty_cycle_to_byte(motor_duty_cycle));
    analogWrite(MOTOR_RIGHT_F, duty_cycle_to_byte(motor_duty_cycle));
    analogWrite(MOTOR_RIGHT_R, duty_cycle_to_byte(0));    
    delay(pivot_time);
}


// Converts duty cycle in decimal (between 0 & 1)
// to byte for analog read
byte duty_cycle_to_byte(float duty)
{
  return (byte) (duty * 255);
}

void flash() {
  color_sensor_output = !color_sensor_output;
  digitalWrite(BLUE_LED_PIN, color_sensor_output);
  digitalWrite(RED_LED_PIN, !color_sensor_output);
  
  if(!color_sensor_output) last_red = analogRead(COLOR_SENSOR_PIN);
  else last_blue = analogRead(COLOR_SENSOR_PIN);
  
  if (last_blue > 0 && last_red > 0)
  {
    color_diff = last_blue - last_red;
  }
  
}

