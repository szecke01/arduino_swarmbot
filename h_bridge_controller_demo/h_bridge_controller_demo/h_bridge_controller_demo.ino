/************************
  *  Authors: Sam Z     *
  *           Cooper L  *
  *           Chris R   *
  *  Date: 2/4/2014     *
  *  Project:           *
  *    H-Bridge Arduino *
  *    Motor Control    *
  ***********************/

// Motor Control Pins
const int MOTOR_LEFT_F  = 4;  
const int MOTOR_LEFT_R  = 2;
const int MOTOR_RIGHT_F = 5;
const int MOTOR_RIGHT_R = 3;

// State values
const int STATE_STOPPED  = 0;
const int STATE_FORWARD = 1;
const int STATE_REVERSE  = 2;
const int STATE_ROT_CCW  = 3;
const int STATE_ROT_CW   = 4;

// Current arduino state
int current_state;
int motor_duty_cycle;

void setup() {
  
  // Establish H-Bridge inputs as outputs from arduino
  pinMode(MOTOR_LEFT_F,  OUTPUT);
  pinMode(MOTOR_LEFT_R,  OUTPUT);
  pinMode(MOTOR_RIGHT_F, OUTPUT);
  pinMode(MOTOR_RIGHT_R, OUTPUT);
  
  // Initialize state machine to desired initial state
  set_state(STATE_FORWARD);
  motor_duty_cycle = 1;  
  
  //pinMode(motor_enable, INPUT_PULLUP);
  //digitalWrite(motor_pin, LOW);
}

void loop() {
  
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
    delay(1000);
    set_state(STATE_REVERSE);
  }
  
  if (current_state == STATE_REVERSE)
  {
    analogWrite(MOTOR_LEFT_F,  duty_cycle_to_byte(0));
    analogWrite(MOTOR_LEFT_R,  duty_cycle_to_byte(motor_duty_cycle));
    analogWrite(MOTOR_RIGHT_F, duty_cycle_to_byte(0));
    analogWrite(MOTOR_RIGHT_R, duty_cycle_to_byte(motor_duty_cycle));
    delay(1000);
    set_state(STATE_ROT_CCW);
  }  
  
  if (current_state == STATE_ROT_CW)
  {
    analogWrite(MOTOR_LEFT_F,  duty_cycle_to_byte(motor_duty_cycle));
    analogWrite(MOTOR_LEFT_R,  duty_cycle_to_byte(0));
    analogWrite(MOTOR_RIGHT_F, duty_cycle_to_byte(0));
    analogWrite(MOTOR_RIGHT_R, duty_cycle_to_byte(motor_duty_cycle));    
    delay(1000);
    set_state(STATE_STOPPED); 
  }
  
  if (current_state == STATE_ROT_CCW)
  {
    analogWrite(MOTOR_LEFT_F,  duty_cycle_to_byte(0));
    analogWrite(MOTOR_LEFT_R,  duty_cycle_to_byte(motor_duty_cycle));
    analogWrite(MOTOR_RIGHT_F, duty_cycle_to_byte(motor_duty_cycle));
    analogWrite(MOTOR_RIGHT_R, duty_cycle_to_byte(0));    
    delay(1000);
    set_state(STATE_ROT_CW);
  }
}

// Sets state and gives delay
void set_state(int new_state)
{
  delay(100);
  analogWrite(MOTOR_LEFT_F,  duty_cycle_to_byte(0));
  analogWrite(MOTOR_LEFT_R,  duty_cycle_to_byte(0));
  analogWrite(MOTOR_RIGHT_F, duty_cycle_to_byte(0));
  analogWrite(MOTOR_RIGHT_R, duty_cycle_to_byte(0));    
  current_state = new_state;
}

// Converts duty cycle in decimal (between 0 & 1)
// to byte for analog read
byte duty_cycle_to_byte(float duty)
{

  return (byte) (duty * 255);
}
