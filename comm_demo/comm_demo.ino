#include <TimerOne.h>
#include <SoftwareSerial.h>
#include <ParallaxLCD.h>

#define ROWS 2
#define COLS 16

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
  - Define communication protocol
*/

// Motor Control Pins
const int MOTOR_LEFT_F  = 4;  
const int MOTOR_LEFT_R  = 2;
const int MOTOR_RIGHT_F = 3;
const int MOTOR_RIGHT_R = 6; 

// State values
const int STATE_STOPPED     = 0;
const int STATE_FOL_COLOR   = 1; // Following Color
const int STATE_SEARCHING   = 2; // Look for colored paper by 
const int STATE_REFIND_LINE = 3; // Went off the color line, find it again
const int STATE_RX          = 4;
const int STATE_TX          = 5; 
const int STATE_COLLIDED    = 6;

// Action values
const int ACTION_FORWARD   = 0;
const int ACTION_REVERSE   = 1;
const int ACTION_PIVOT_CCW = 2;
const int ACTION_PIVOT_CW  = 3;
const int ACTION_TURN_CCW  = 4;
const int ACTION_TURN_CW   = 5;
const int ACTION_STOPPED   = 6;

// Motor Constants
const float TURN_SPEED_RATIO = .25;
const int STATE_CHANGE_DELAY = 200;
const int PIVOT_TIME_90 = 630;
const int PIVOT_TIME_60 = 420;

// Color Sensor Constants
const int RED_LED_PIN = 15;
const int BLUE_LED_PIN = 14;
const int COLOR_SENSOR_PIN = 0;
const int LED_FLASH_PERIOD = 41000000;
const int MIN_DIFF_THRESHOLD = 20;
const int COLOR_SENSE_BUFFER_SIZE = 2;
const int RED_COLOR  = -1;
const int BLUE_COLOR = 1;
const int NEUTRAL_COLOR = 0;

// Tx/Rx Constants
const int CARRIER_PIN = 5;
const int TX_PIN = 7;
const int RX_PIN = 2;
const int NUM_LISTENS = 5;
const int FOUND_RED  = 0;
const int FOUND_BLUE = 1;
const int HEARD_YOU  = 2;

// Collision Detector Constants
const int COLLISION_INTERRUPT_PIN = 21;
const int COLLISION_INTERRUPT_NO  = 2;
const int COLLISION_SWITCH_PINS[] = {41,43,45,47,49,51};
const int NUM_COLLISION_PINS = 6;

// Color sensor variables
boolean color_sensor_output = LOW;
int last_red     = -1;
int last_blue    = -1;
int color_diff   = 0;
int calib_offset = 0; 
int color_sense_buffer[COLOR_SENSE_BUFFER_SIZE];
int color_sense_buffer_index = 0;

// Tx/Rx variables
// I FOUND RED:  {1,1,1,1,1,1,1,0,0,0}; 7 1's per 10 bits
// I FOUND BLUE: {1,1,1,1,0,0,0,0,0,0}; 4 1's per 10 bits
// I HEARD YOU:  {1,1,0,0,0,0,0,0,0,0}; 2 1's per 10 bits
// Protocol is to listen to NUM_LISTENS sets of 10 bits
// take average # of 1's and round it -- 
// if 7: FOUND_RED
// if 4: FOUND_BLUE
// if 2: HEARD_YOU
// 
boolean tx_msg[] = {0,0,0,0,0,0,0,0,0,0}; 
boolean rx_msg[] = {0,0,0,0,0,0,0,0,0,0};
String rx_str  = "";

// Searching algorithm paramters
long last_search_time = 0;

// Collision Detector parameters
long last_collide_time = 0;
int collision_states[] = {0,0,0,0,0,0};
boolean collision_triggers_found = 0;

// Current arduino state
int current_state;
int current_action;
int last_state;
int fol_color; // Color to follow (either found, or communication dictated)
float motor_duty_cycle;

void setup() {
  
  // Serial (for debugging)
  Serial.begin(9600);
  
  // Initialize Timer
  Timer1.initialize();
  
  // Initialize and calibrate color sensor
  init_color_sensor();
  
  // Establish H-Bridge inputs as outputs from arduino
  init_motor_control();
  
  // Begin sending carrier signal to pin 5 and set the data pin low. 
  init_tx_rx();
  
  // Initialize collision detector interrupts
  init_collision_detector();
  
  // Initialize state machine to desired initial state
  set_state(STATE_SEARCHING);
  
  // Currently not following red or blue (red = 1, blue = -1)
  fol_color = 0;
  
  // let all pins settle
  delay(1000);
  
}

void loop() {
  
  // Calculate the sensed color
  int c_color = calculate_color();
  
  
  // Performs state actions
  handle_state();
  handle_action();
 
}

// Sets state and gives delay
void set_state(int new_state)
{
  // prevent shoot-through, set state
  last_state = state;
  stop_motor();
  delay(STATE_CHANGE_DELAY);
  current_state = new_state;
  
  // state specific event_handling
  if(current_state == STATE_REFIND_LINE)
    last_search_time = millis();
  
}

void set_action(int new_action)
{
  stop_motor();
  delay(STATE_CHANGE_DELAY);
  current_action = new_action; 
}

void handle_state()
{
  // Iterate through FSM and perform current state
  // Not using else-if's in case interrupt causes 
  // state changes
  
  if (current_state == STATE_FOL_COLOR)
  {
    if(fol_color == -1) //blue
    {
      
    }
    else if(fol_color == 1) //red
    {
      
    }
  }
  if (current_state == STATE_SEARCHING)
  {
    set_action(ACTION_FORWARD);
  }
  if (current_state == STATE_STOPPED)
  {
    stop_motor();
  }
  if (current_state == STATE_REFIND_LINE)
  {
    
    if(millis() - last_search_time < 600)
    {
      // Turn clockwise
      set_action(ACTION_PIVOT_CW);
    }
    else if(millis() - last_search_time < 650)
      stop_motor();
    else if(millis() - last_search_time < 1850)
    {
      // Turn counter-clockwise
      set_action(ACTION_PIVOT_CCW);
    }
    else if(millis() - last_search_time < 2250)
    {
       // Turn clockwise again
      set_action(ACTION_PIVOT_CW);
    }
    else if(millis() - last_search_time < 2300)
      set_action(ACTION_STOPPED);
    else
      last_search_time = millis();
  }
  if (current_state == STATE_TX)
  {
    
  }
  if (current_state == STATE_RX)
  {
    
  }
  if (current_state == STATE_COLLIDED)
  {
    
    if(last_state == STATE_SEARCHING && collision_triggers_found)
    {
      if(millis() - last_collide_time > 1000)
      {
        set_action(ACTION_PIVOT_CCW);
      }
      if(millis() - last_collide_time > 2500) {
        set_state(STATE_SEARCHING);
      }
    }
    
    // Waits one second before accepting new collision interrupts
    if(millis() - last_collide_time > 1000 && !collision_triggers_found)
    {
      find_collision_triggers();
    }

  }
}

void handle_action()
{
  if (current_action == ACTION_STOPPED)
  {
    stop_motor();
  }
  if (current_action == ACTION_FORWARD)
  {
    analogWrite(MOTOR_LEFT_F,  duty_cycle_to_byte(motor_duty_cycle));
    analogWrite(MOTOR_LEFT_R,  duty_cycle_to_byte(0));
    analogWrite(MOTOR_RIGHT_F, duty_cycle_to_byte(motor_duty_cycle));
    analogWrite(MOTOR_RIGHT_R, duty_cycle_to_byte(0));
  }
  
  if (current_action == ACTION_REVERSE)
  {
    analogWrite(MOTOR_LEFT_F,  duty_cycle_to_byte(0));
    analogWrite(MOTOR_LEFT_R,  duty_cycle_to_byte(motor_duty_cycle));
    analogWrite(MOTOR_RIGHT_F, duty_cycle_to_byte(0));
    analogWrite(MOTOR_RIGHT_R, duty_cycle_to_byte(motor_duty_cycle));
  }  
  
  if (current_action == ACTION_PIVOT_CW)
  {
    analogWrite(MOTOR_LEFT_F,  duty_cycle_to_byte(motor_duty_cycle));
    analogWrite(MOTOR_LEFT_R,  duty_cycle_to_byte(0));
    analogWrite(MOTOR_RIGHT_F, duty_cycle_to_byte(0));
    analogWrite(MOTOR_RIGHT_R, duty_cycle_to_byte(motor_duty_cycle)); 
  }
  
  if (current_action == ACTION_PIVOT_CCW)
  {
    analogWrite(MOTOR_LEFT_F,  duty_cycle_to_byte(0));
    analogWrite(MOTOR_LEFT_R,  duty_cycle_to_byte(motor_duty_cycle));
    analogWrite(MOTOR_RIGHT_F, duty_cycle_to_byte(motor_duty_cycle));
    analogWrite(MOTOR_RIGHT_R, duty_cycle_to_byte(0)); 
  }
  if (current_action == ACTION_TURN_CW)
  {
    analogWrite(MOTOR_LEFT_F,  duty_cycle_to_byte(motor_duty_cycle*TURN_SPEED_RATIO));
    analogWrite(MOTOR_LEFT_R,  duty_cycle_to_byte(0));
    analogWrite(MOTOR_RIGHT_F, duty_cycle_to_byte(motor_duty_cycle));
    analogWrite(MOTOR_RIGHT_R, duty_cycle_to_byte(0));    
  }
  if (current_action == ACTION_TURN_CCW)
  {
    analogWrite(MOTOR_LEFT_F,  duty_cycle_to_byte(motor_duty_cycle));
    analogWrite(MOTOR_LEFT_R,  duty_cycle_to_byte(0));
    analogWrite(MOTOR_RIGHT_F, duty_cycle_to_byte(motor_duty_cycle*TURN_SPEED_RATIO));
    analogWrite(MOTOR_RIGHT_R, duty_cycle_to_byte(0));    
  }
}

// Sets analog out for motor to zero
void stop_motor()
{
  analogWrite(MOTOR_LEFT_F,  duty_cycle_to_byte(0));
  analogWrite(MOTOR_LEFT_R,  duty_cycle_to_byte(0));
  analogWrite(MOTOR_RIGHT_F, duty_cycle_to_byte(0));
  analogWrite(MOTOR_RIGHT_R, duty_cycle_to_byte(0));  
}

// Converts duty cycle in decimal (between 0 & 1)
// to byte for analog read
byte duty_cycle_to_byte(float duty)
{
  return (byte) (duty * 255);
}

void flash() {
  
   // Set most recent red/blue 
  if(!color_sensor_output) last_red = analogRead(COLOR_SENSOR_PIN);
  else last_blue = analogRead(COLOR_SENSOR_PIN);
  
  // Toggle between red/blue
  color_sensor_output = !color_sensor_output;
  digitalWrite(BLUE_LED_PIN, color_sensor_output);
  digitalWrite(RED_LED_PIN, !color_sensor_output);
  
  // Sets the color difference if we have values in the buffer for both
  if (last_blue > 0 && last_red > 0)
  {
    color_diff = (last_blue - last_red)- calib_offset;
    color_sense_buffer[color_sense_buffer_index] = color_diff;
    color_sense_buffer_index = 
      (color_sense_buffer_index + 1) % COLOR_SENSE_BUFFER_SIZE;
      
  }
}

int calculate_color()
{
  int i;
  int num_red = 0;
  int num_blue = 0;
  // Iterate through buffer, calculate number of red/blue readings
  for(i = 0; i < COLOR_SENSE_BUFFER_SIZE; i++)
  {
    if(color_sense_buffer[i] < -1*(MIN_DIFF_THRESHOLD))
    {
      ++num_red;
    } 
    else if(color_sense_buffer[i] > MIN_DIFF_THRESHOLD)
    {
      ++num_blue;
    }
  }
  // return 1 if blue
  if(num_blue > (int)(.8*(float)COLOR_SENSE_BUFFER_SIZE))
  {
    return 1;
  }
  // return -1 if red
  if(num_red > (int)(.8*(float)COLOR_SENSE_BUFFER_SIZE))
  {
    return -1;
  }
  // return 0 if neutral
  return 0;
}

void init_color_sensor()
{
   // Establish color params
  Serial.begin(9600);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  
  // Calibrate lights
  for(int i = 0; i < COLOR_SENSE_BUFFER_SIZE; ++i)
  {
     color_sense_buffer[i] = 0;
  }
  delay(500);
  
  // Flash both colors, normalize by measured value
  int calib_iter  = 2;
  int temp_offset = 0;
  for(int i = 0; i < calib_iter; i++)
  {
    flash();
    delay(250);
    flash();
    delay(250);
    temp_offset = color_diff;
  }
  calib_offset = temp_offset;
  Timer1.attachInterrupt(flash, 50000);
}

void handle_collision()
{
  
 detachInterrupt(COLLISION_INTERRUPT_NO);
 
 // timer begin
 // when timer ends, call collision refresh
 last_collide_time = millis();
 collision_triggers_found = false;
 set_state(STATE_COLLIDED);

}

// After waiting a second after collision, we determine what switches have been triggered
void find_collision_triggers()
{
  
    Serial.println("The bumpers detected are: {");
    for(int i = 0; i < NUM_COLLISION_PINS; i++)
    {
      collision_states[i] = digitalRead(COLLISION_SWITCH_PINS[i];
      if(collision_states[i])
      {
        Serial.print(i);
        Serial.print(" ");
      }
    }
    Serial.print("}");
    Serial.println("");
    collision_triggers_found = true;
    attachInterrupt(COLLISION_INTERRUPT_NO, handle_collision, RISING);
    
}

void init_motor_control()
{
  // Assign motor to proper pins
  pinMode(MOTOR_LEFT_F,  OUTPUT);
  pinMode(MOTOR_LEFT_R,  OUTPUT);
  pinMode(MOTOR_RIGHT_F, OUTPUT);
  pinMode(MOTOR_RIGHT_R, OUTPUT);
  
  // Set duty cycle to desired speed
  motor_duty_cycle = .18;  
}

void init_tx_rx()
{
  pinMode(CARRIER_PIN, OUTPUT); 
  pinMode(TX_PIN, OUTPUT);
  pinMode(RX_PIN, INPUT);
  digitalWrite(TX_PIN, LOW);

  TCCR3A = _BV(COM3A0) | _BV(COM3B0) | _BV(WGM30) | _BV(WGM31);
  // sets COM Output Mode to FastPWM with toggle of OC3A on compare match with OCR3A
  // also sets WGM to mode 15: FastPWM with top set by OCR3A
  
  TCCR3B = _BV(WGM32) | _BV(WGM33) | _BV(CS31);
  // sets WGM as stated above; sets clock scaling to "divide by 8"
  
  OCR3A = 51;
}

void init_collision_detector()
{
  
  for(int i = 0; i < NUM_COLLISION_PINS; i++)
    pinMode(COLLISION_SWITCH_PINS[i], INPUT_PULLUP);
  
  attachInterrupt(COLLISION_INTERRUPT_NO, handle_collision, RISING);
  
}

