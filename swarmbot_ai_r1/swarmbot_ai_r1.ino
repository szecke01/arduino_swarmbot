#include <TimerOne.h>
#include <SoftwareSerial.h>
#include <ParallaxLCD.h>

#define ROWS 2
#define COLS 16
#define LCD_PIN 31
#define NO_COLOR -10

#define DEBUG 0


/************************
  *  Authors: Sam Z     *
  *           Cooper L  *
  *           Chris R   *
  *  Date: 2/4/2014     *
  *  Project:           *
  *    Arduino swarmbot *
  *    AI               *
  ***********************/

/* TODO:
  - Define communication protocol
*/

ParallaxLCD lcd(LCD_PIN,2,16); // desired pin, rows, cols

// Motor Control Pins
const int MOTOR_LEFT_F  = 10;  
const int MOTOR_LEFT_R  = 6;
const int MOTOR_RIGHT_F = 8;
const int MOTOR_RIGHT_R = 4; 

// State values
const int STATE_STOPPED     = 0;
const int STATE_FOL_COLOR   = 1; // Following Color
const int STATE_SEARCHING   = 2; // Look for colored paper by 
const int STATE_REFIND_LINE = 3; // Went off the color line, find it again
const int STATE_MASTER      = 4;
const int STATE_SLAVE       = 5; 
const int STATE_COLLIDED    = 6;
const int STATE_TURN_ARND   = 7;
const int STATE_DONE        = 8;
String STATE_STRINGS[]   = {"stopped", "following ", "searching" ,"refinding", "master", "slave", "collided", "turning around", "finished", "slave"};

// Action values
const int ACTION_FORWARD     = 0;
const int ACTION_REVERSE     = 1;
const int ACTION_PIVOT_CCW   = 2;
const int ACTION_PIVOT_CW    = 3;
const int ACTION_TURN_CCW    = 4;
const int ACTION_TURN_CW     = 5;
const int ACTION_REVERSE_CCW = 6;
const int ACTION_REVERSE_CW  = 7;
const int ACTION_STOPPED     = 8;
String ACTION_STRINGS[]   = {"forward", "reverse ", "pivot ccw" ,"pivot cw", "turn cw", "turn ccw", "reverse ccw", "reverse cw", "stopped"};

// Motor Constants
const float TURN_SPEED_RATIO = .25;
const int STATE_CHANGE_DELAY = 30;
const int PIVOT_TIME_90 = 630;
const int PIVOT_TIME_60 = 420;

// Color Sensor Constants
const int RED_LED_PIN = 26;
const int BLUE_LED_PIN = 24;
const int COLOR_SENSOR_PIN = 7;
const int LED_FLASH_PERIOD = 50;
const int MIN_DIFF_THRESHOLD = 20;
const int COLOR_SENSE_BUFFER_SIZE = 2;
const int RED_COLOR  = -1;
const int BLUE_COLOR = 1;
const int NEUTRAL_COLOR = 0;

// Tx/Rx Constants
const int CARRIER_PIN = 5;
const int NUM_LISTENS = 5;
const int FOUND_RED  = 0;
const int FOUND_BLUE = 1;
const int HEARD_YOU  = 2;
const int MSG_DELAY = 800; //us
const int MSG_DURATION_MILLIS = 80; //ms
const int MSG_LEN = 50; //bits
const int TX_PIN = 18;
const int RX_MASK_PIN = 33;
const int RX_INTERRUPT_PIN = 19;
const int RX_INTERRUPT_NO = 4;
const int RX_EDGE_COUNT_MIN = 3; // edges
const int RX_EDGE_TIMEOUT = 3; // us 
const int TX_TIMEOUT_ATTEMPTS = 7;

// Slave states
const int SLAVE_STATE_LISTEN_MESSAGE           = 0;
const int SLAVE_STATE_RESPOND_HEARD            = 1;
const int SLAVE_STATE_RESPOND_FINISHED_COMM    = 2;

// Master states
const int MASTER_STATE_BEGIN_HANDSHAKE = 0;
const int MASTER_STATE_RECIEVE_HANDSHAKE = 1;
const int MASTER_STATE_TRANSMIT_COLOR = 2;
const int MASTER_STATE_CONFIRM_TRANSMIT = 3;


// Message IDs
const int MSG_HELLO_ID           = 0;
const int MSG_HEARD_ID           = 1;
const int MSG_FOUND_BLUE_ID      = 2;
const int MSG_FOUND_RED_ID       = 3;
const int MSG_DONE_ID            = 4;
const int MSG_INVALID_ID         = 5;

const int MSG_LIST[] = { MSG_INVALID_ID, MSG_HEARD_ID, MSG_INVALID_ID, MSG_FOUND_BLUE_ID, MSG_INVALID_ID, MSG_FOUND_RED_ID, MSG_INVALID_ID, MSG_DONE_ID, MSG_INVALID_ID, MSG_INVALID_ID, MSG_INVALID_ID};

// Messages
const boolean MSG_HELLO[]      = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                              1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                              1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                              1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                              1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

const boolean MSG_HEARD[]      = {1, 0, 0 ,0, 0, 0, 0, 0, 0, 0,
                              1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
                              1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
                              1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
                              1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//15 1's
const boolean MSG_FOUND_BLUE[] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                              1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 
                              0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 
                              0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 
                              1, 0, 0, 1, 0, 0, 1, 0, 0, 0};            
//25 1's
const boolean MSG_FOUND_RED[]  = {1, 0, 1, 0, 1, 0, 1, 0, 1, 0,
                              1, 0, 1, 0, 1, 0, 1, 0, 1, 0,
                              1, 0, 1, 0, 1, 0, 1, 0, 1, 0,
                              1, 0, 1, 0, 1, 0, 1, 0, 1, 0,
                              1, 0, 1, 0, 1, 0, 1, 0, 1, 0};       
//35 1's    
const boolean MSG_DONE[]       = {1, 0, 1, 0, 1, 0, 1, 0, 1, 0,
                              1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
                              1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  
                              1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 
                              1, 0, 1, 0, 1, 0, 1, 0, 1, 0};            

// if recieved message is this, it is invalid
const int MSG_RCVD_INVALID[]      = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
                                     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
                                     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
                                     0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
                                                                     

// Collision Detector Constants
const int COLLISION_INTERRUPT_PIN = 21;
const int COLLISION_INTERRUPT_NO  = 2;
const int COLLISION_SWITCH_PINS[] = {41,43,45,47,49,51};
const int NUM_COLLISION_PINS = 6;
const int FC_COL_INDEX = 5; // front center collision index
const int FL_COL_INDEX = 3; // front left collision index
const int FR_COL_INDEX = 1; // front right collision index
const int L_COL_INDEX  = 4; // left side collision index
const int R_COL_INDEX  = 2; // right side collision index
const int B_COL_INDEX  = 0; // bumper collision index
const int COLLISION_REFRESH_TIME = 100;

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
boolean tx_msg[] = {0,1,1,0,1,1,1,0,1,0}; 
boolean rx_msg[] = {0,1,1,0,1,1,1,0,1,0};
String rx_str  = "";
long last_rx_edge_time = 0;
int rx_edge_count = 0;
boolean tx_data   = true;
int last_rx_id = -1;

// Searching algorithm paramters
long last_search_time = 0;

// Collision Detector parameters
long last_collide_time = 0;
int collision_states[] = {0,0,0,0,0,0};
boolean collision_triggers_found = 0;

// Current arduino states
int current_state = -1;
int current_action = -1;
int last_state = -1;
int fol_color = NO_COLOR; // Color to follow (either found, or communication dictated), default to -10 until told otherwise by sensor or other team
float motor_duty_cycle;
boolean returning = false;
long turn_arnd_time = 0;
long finished_time  = 0;
boolean begun = false;
int tx_state = -1;
boolean begin_sample_rx = false;
int num_tx_timeouts = 0;

void setup() {
  
  // LCD go beyond initialization
  lcd.setup();
  delay(100);
  lcd.cursorOff();
  lcd.backLightOn();
  lcd.at(0,4,"Let's Go!\0");
  delay(1000);
  
  // Serial (for debugging)
  Serial.begin(9600);
  
  // Initialize and calibrate color sensor
  init_color_sensor();
  
  // Establish H-Bridge inputs as outputs from arduino
  init_motor_control();
  
  // Begin sending carrier signal to pin 5 and set the data pin low. 
  init_tx_rx();
  
  // Initialize state machine to desired initial state
  set_state(STATE_STOPPED);
  
  // let all pins settle
  delay(1000);
  
  // Initialize collision detector interrupts
  init_collision_detector();
  
}

void loop() {

  //Serial.println(fol_color);
  if(DEBUG)
    motor_duty_cycle = 0;
  
  // rx listen function
  if(current_state == STATE_SEARCHING && fol_color == NO_COLOR)
    sense_rx();
  
  
  //send_TX(MSG_HELLO);
  // Performs state actions
  handle_state();
  handle_action();
 
 
}

// Sets state and gives delay
void set_state(int new_state)
{
 
  // state specific event_handling
  
  //refresh the finding line timer
  if(new_state == STATE_REFIND_LINE)
   {
     last_search_time = millis();
     motor_duty_cycle = .20;
   }
   else{
     motor_duty_cycle = .32;
   }
   
   if(new_state == STATE_DONE)
   {
     detachInterrupt(COLLISION_INTERRUPT_NO); 
   }
    
  // Set the lcd string for following color
  if(new_state == STATE_FOL_COLOR)
  {
    String color;
    if(fol_color == RED_COLOR)
      color = "red";
    if(fol_color == BLUE_COLOR)
      color = "blue";
    STATE_STRINGS[STATE_FOL_COLOR] = "following " + color;
  }
  
  // detatch the collision handler if we are tx/rx
  else if(new_state == STATE_MASTER || new_state == STATE_SLAVE)
  {
    
    detachInterrupt(COLLISION_INTERRUPT_NO);
  }
  
  // reattach the collision handler if we are not tx/rx
  else if((current_state == STATE_MASTER || current_state == STATE_SLAVE) 
          && (new_state != STATE_MASTER && new_state != STATE_SLAVE))
  {
    attachInterrupt(COLLISION_INTERRUPT_NO, handle_collision, RISING);
  }
  
  
  // prevent collided from being last state, or from last state being the current state
  if(current_state != STATE_COLLIDED && current_state != new_state)
    last_state = current_state;
    
  stop_motor();
  current_state = new_state;
  set_action(ACTION_STOPPED);

}

void set_action(int new_action)
{
  if(current_action != new_action)
  {
    
    // prevent shoot-through, set action
    stop_motor();
    delay(STATE_CHANGE_DELAY);
    current_action = new_action; 
    update_lcd();
    
  }
}

void handle_state()
{
  // Iterate through FSM and perform current state
  // Not using else-if's in case interrupt causes 
  // state changes
  
  if (current_state == STATE_FOL_COLOR)
  {
    // calculate color below
    int c_color = calculate_color();
    if(c_color == fol_color)
    {
      
      set_action(ACTION_FORWARD);
    }
    else{
      set_state(STATE_REFIND_LINE);
    }
  }
  
  // Searching for colored line
  if (current_state == STATE_SEARCHING)
  {
    set_action(ACTION_FORWARD);
      
    // Calculate the sensed color if we havent gotten off NO_COLOR
  
    // set followed color to red or blue if sensed
    int c_color = calculate_color();
    //Serial.println(c_color);
    if(fol_color == NO_COLOR && c_color != NEUTRAL_COLOR)
    {
      set_fol_color(c_color);
    }
    if(c_color == fol_color && current_state != STATE_MASTER)
    {
      set_state(STATE_FOL_COLOR);
    }
      
    
  }
  if (current_state == STATE_STOPPED)
  {
    stop_motor();
  }
  
  // We are following a color, but went off of the line
  if (current_state == STATE_REFIND_LINE)
  {
    int c_color = calculate_color();
    if(c_color == fol_color)
    {
      set_state(STATE_FOL_COLOR);
    }
    
    else
    {
      if(millis() - last_search_time < 750)
      {
        // Turn clockwise
        set_action(ACTION_PIVOT_CW);
      }
      else if(millis() - last_search_time < 750 + 1550)
      {
        // Turn counter-clockwise
        set_action(ACTION_PIVOT_CCW);
      }
      else
      {
        if(!returning)
          last_search_time = millis();
        else{
          set_state(STATE_DONE);
          finished_time = millis();
        }
      }
    }
  }
  
  // We are slave: precondition, this state was not set erroneously. We will be stuck here forever, otherwise
  if (current_state == STATE_SLAVE)
  {
    // First thing we do is respond we have heard
    if(tx_state == SLAVE_STATE_RESPOND_HEARD)
    {
      Serial.println("Responding HEARD!");
      send_TX(MSG_HEARD);
      tx_state = SLAVE_STATE_LISTEN_MESSAGE;
      delay(100);
    }
    
    if(tx_state == SLAVE_STATE_LISTEN_MESSAGE)
    {
      Serial.println("Awaiting message");
      int response = process_RX();
      // Listen for 300 milliseconds, or until you've heard a real response
      int time_elapsed = millis();
      while(response == MSG_INVALID_ID && time_elapsed < 300)
      {
        response = process_RX();
      }
      last_rx_id = response;
      
      if(response != MSG_FOUND_BLUE_ID && response != MSG_FOUND_RED_ID)
      {
        tx_state = SLAVE_STATE_RESPOND_HEARD;
        return;
      }
      
      if(response == MSG_FOUND_BLUE_ID)
      {
        Serial.println("I'll find red");
        fol_color = RED_COLOR;
        set_state(STATE_SEARCHING);
        return;
      }
      else
      {
        Serial.println("I'll find blue");
        fol_color = BLUE_COLOR;
        set_state(STATE_SEARCHING);
        return;
      }
      
    }

    
  }
  
  // We are recieving
  if (current_state == STATE_MASTER)
  {
    
     // First thing we do is say we found something
    if(tx_state == MASTER_STATE_BEGIN_HANDSHAKE)
    {
      Serial.println("BEGIN HANDSHAKE");
      send_TX(MSG_HELLO);
      tx_state = MASTER_STATE_RECIEVE_HANDSHAKE;
    }
    
    if(tx_state == MASTER_STATE_RECIEVE_HANDSHAKE)
    {
      Serial.println("AWAITING HANDSHAKE");
      int response = process_RX();
      
      // Listen for 500 milliseconds, or until you've heard a real response
      long time_elapsed = millis();
      while(response != MSG_HEARD_ID && time_elapsed < 300)
      {
        response = process_RX();
      }
      last_rx_id = response;
  
      // go back!
      if(response != MSG_HEARD_ID)
      {
        tx_state = MASTER_STATE_BEGIN_HANDSHAKE;
      }
      
      // keep going
      if(response == MSG_HEARD_ID)
      {
        tx_state = MASTER_STATE_TRANSMIT_COLOR;
      }
      
    }
    
    if(tx_state == MASTER_STATE_TRANSMIT_COLOR)
    {
      Serial.println("TRANSMITTING COLOR");
      // if we found red, say red
      
      for(int i = 0; i < 5; i++)
      {
        if(fol_color == BLUE_COLOR)
        {
          send_TX(MSG_FOUND_BLUE);
        }
        
        // if we found blue, say blue
        if(fol_color == RED_COLOR)
        {
          send_TX(MSG_FOUND_RED);
        }
      }
      
      delay(100);
      set_state(STATE_REFIND_LINE);
      
    }
    
  }
  
  // Turn around after hitting end
  if (current_state == STATE_TURN_ARND)
  {
    if(millis() - turn_arnd_time < 600)
      set_action(ACTION_REVERSE);
    
    else if(millis() - turn_arnd_time < 1550)
      set_action(ACTION_PIVOT_CW);
    
    else{
      turn_arnd_time = 0;
      set_state(STATE_FOL_COLOR);
    } 
     
  }
  
  if (current_state == STATE_DONE)
  {
    if(millis() - finished_time < 400)
       set_action(ACTION_REVERSE);
    else if(millis() - finished_time < 400 + 600)
       set_action(ACTION_PIVOT_CW);
    else if(millis() - finished_time < 400 + 600 + 300)
       set_action(ACTION_REVERSE);
    else{
        set_action(ACTION_STOPPED);
      }  
   }
  
  if (current_state == STATE_COLLIDED)
  {
    
    // For starting off
    if(!begun)
    {
      begun = true;
      attachInterrupt(COLLISION_INTERRUPT_NO, handle_collision, RISING);
      set_state(STATE_SEARCHING); 
    }
    
    // Collision detection
    else{
    
      if(collision_triggers_found)
      {
        // see color
        int c_color = calculate_color();
        if(fol_color == NO_COLOR && c_color != NEUTRAL_COLOR)
          {
            set_state(STATE_REFIND_LINE);
            set_fol_color(c_color);
          }
        else if(c_color == fol_color)
        {
          returning = true;
          turn_arnd_time = millis();
          set_state(STATE_TURN_ARND);
        }
         
        
        // front 3
        if(collision_states[FL_COL_INDEX] && collision_states[FC_COL_INDEX] && collision_states[FR_COL_INDEX])
        {
          respond_collision_front(); 
        }
        
        // front left or left side collision
        else if(collision_states[L_COL_INDEX] || collision_states[FL_COL_INDEX] 
        && !collision_states[FC_COL_INDEX]  && !collision_states[FR_COL_INDEX]  
        && !collision_states[R_COL_INDEX]  && !collision_states[B_COL_INDEX])
        {
          respond_collision_left();
        }
        
        // front right or right side collision
        else if(collision_states[R_COL_INDEX] || collision_states[FR_COL_INDEX] 
        && !collision_states[FC_COL_INDEX]  && !collision_states[FL_COL_INDEX]  
        && !collision_states[L_COL_INDEX]  && !collision_states[B_COL_INDEX])
        {
          respond_collision_right();
        }
        
        // front and front left
        else if(collision_states[FL_COL_INDEX] && collision_states[FC_COL_INDEX])
        {
          respond_collision_left();
        }
         
        // front and front right
        else if(collision_states[FR_COL_INDEX] && collision_states[FC_COL_INDEX])
        {
          respond_collision_right();
        }
        
        // just front
        else if(collision_states[FC_COL_INDEX] && !collision_states[FR_COL_INDEX] 
        && !collision_states[R_COL_INDEX]  && !collision_states[FL_COL_INDEX]  
        && !collision_states[L_COL_INDEX]  && !collision_states[B_COL_INDEX])
        {
          respond_collision_front();
        }
        
        // back collision
        else if(collision_states[B_COL_INDEX])
        {
          respond_collision_rear();
        }
        
        // none still toggled, ignore
        else if(!collision_states[R_COL_INDEX] && !collision_states[FR_COL_INDEX] 
        && !collision_states[FC_COL_INDEX]  && !collision_states[FL_COL_INDEX]  
        && !collision_states[L_COL_INDEX]  && !collision_states[B_COL_INDEX])
        {
          set_state(last_state);
        }
        
        // otherwise, respond like we hit something on the front
        else{
          respond_collision_front();
        }
        
      }
      
      // Waits one second before accepting new collision interrupts
      if(millis() - last_collide_time > COLLISION_REFRESH_TIME && !collision_triggers_found)
      {
        //Serial.println("finding collision triggers");
        find_collision_triggers();
      }
  
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
 
  if (current_action == ACTION_REVERSE_CW)
  {
    analogWrite(MOTOR_LEFT_F,  duty_cycle_to_byte(0));
    analogWrite(MOTOR_LEFT_R,  duty_cycle_to_byte(motor_duty_cycle*TURN_SPEED_RATIO));
    analogWrite(MOTOR_RIGHT_F, duty_cycle_to_byte(0));
    analogWrite(MOTOR_RIGHT_R, duty_cycle_to_byte(motor_duty_cycle));    
  } 
  
  if (current_action == ACTION_REVERSE_CCW)
  {
    analogWrite(MOTOR_LEFT_F,  duty_cycle_to_byte(0));
    analogWrite(MOTOR_LEFT_R,  duty_cycle_to_byte(motor_duty_cycle));
    analogWrite(MOTOR_RIGHT_F, duty_cycle_to_byte(0));
    analogWrite(MOTOR_RIGHT_R, duty_cycle_to_byte(motor_duty_cycle*TURN_SPEED_RATIO));    
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
  
    int c_color = calculate_color();
   // Serial.println("The bumpers detected are: {");
    for(int i = 0; i < NUM_COLLISION_PINS; i++)
    {
      collision_states[i] = digitalRead(COLLISION_SWITCH_PINS[i]);
      if(collision_states[i])
      {
       // Serial.print(i);
        //Serial.print(" ");
      }
    }
    //Serial.print("}");
    //Serial.println("");
    collision_triggers_found = true;
    attachInterrupt(COLLISION_INTERRUPT_NO, handle_collision, RISING);
    
}

void respond_collision_front()
{
  // begin reversing
  if(millis() - last_collide_time < COLLISION_REFRESH_TIME + 1000)
  {
    if(current_action != ACTION_REVERSE)
      set_action(ACTION_REVERSE);
  }
  
  // after reversing for one second, pivot ccw
  else if(millis() - last_collide_time < COLLISION_REFRESH_TIME + 1000 + 1500)
  {
    if(current_action != ACTION_PIVOT_CCW)
      set_action(ACTION_PIVOT_CCW);
  }
  
  // after pivoting ccw for 1.5 seconds, return to previous state
  else {
    set_state(last_state);
  }
}

void respond_collision_left()
{
  // begin reversing
  if(millis() - last_collide_time < COLLISION_REFRESH_TIME + 1500)
  {
    if(current_action != ACTION_REVERSE_CW)
      set_action(ACTION_REVERSE_CW);
  }
  
  // after pivoting ccw for 1.5 seconds, return to previous state
  else {
    set_state(last_state);
  }
}

void respond_collision_right()
{
  // begin reversing
  if(millis() - last_collide_time < COLLISION_REFRESH_TIME + 1500)
  {
    if(current_action != ACTION_REVERSE_CCW)
      set_action(ACTION_REVERSE_CCW);
  }
  
  // after pivoting ccw for 1.5 seconds, return to previous state
  else {
    set_state(last_state);
  }
}

void respond_collision_rear()
{
 // begin forwards
  if(millis() - last_collide_time < COLLISION_REFRESH_TIME + 1000)
  {
    if(current_action != ACTION_FORWARD)
      set_action(ACTION_FORWARD);
  }
  
  // after forwards, go back ot previous state
  else
  {
    set_state(last_state);
  }
  
}

void set_fol_color(int c_color)
{
  fol_color = c_color;
  
  if(current_state != STATE_MASTER && current_state != STATE_SLAVE)
  {
   // set_state(STATE_MASTER);
   // tx_state = MASTER_STATE_BEGIN_HANDSHAKE;
  }
  
}

void init_motor_control()
{
  // Assign motor to proper pins
  pinMode(MOTOR_LEFT_F,  OUTPUT);
  pinMode(MOTOR_LEFT_R,  OUTPUT);
  pinMode(MOTOR_RIGHT_F, OUTPUT);
  pinMode(MOTOR_RIGHT_R, OUTPUT);
  
  // Set duty cycle to desired speed
  motor_duty_cycle = .32;  
}

void init_tx_rx()
{
  pinMode(CARRIER_PIN, OUTPUT); 
  pinMode(RX_MASK_PIN, OUTPUT);
  pinMode(RX_INTERRUPT_PIN, INPUT);
  
  // HIGH = reading enabled, LOW = reading disabled
  digitalWrite(RX_MASK_PIN, HIGH);
  attachInterrupt(RX_INTERRUPT_NO, rx_isr, RISING);
 
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


void init_color_sensor()
{
   // Establish color params
  Timer1.initialize(50000);
  
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
  Timer1.attachInterrupt(flash);
}

void rx_isr() {
  detachInterrupt(RX_INTERRUPT_NO);
  rx_edge_count++;
  last_rx_edge_time = millis();
  begin_sample_rx = true;
  attachInterrupt(RX_INTERRUPT_NO, rx_isr, RISING);
}

void sense_rx(){
    
    if(rx_edge_count > 0)
    {   
     
      // if we waited longer than one millisecond between each edge
      if(millis() - last_rx_edge_time > 3)
      {
        // reset edge counter
         rx_edge_count = 0;
      }
      
      // If the number of edges is sufficient to assume we have heard the "HELLO" signal
      else if (rx_edge_count >= RX_EDGE_COUNT_MIN)
      {
        rx_edge_count = 0;
        Serial.println("SETTING STATE TO SLAVE");
        set_state(STATE_SLAVE);
        set_action(ACTION_STOPPED);
        update_lcd();
        tx_state = SLAVE_STATE_RESPOND_HEARD;
      }
    }

}
 
int process_RX(){
  if(begin_sample_rx)
  {
    rx_edge_count = 0;
    pass_time(MSG_DURATION_MILLIS);
    begin_sample_rx = false;
    rx_edge_count++;
    //Serial.println(rx_edge_count);
    float percentage = (float)(rx_edge_count)/(MSG_LEN)*10;
    return MSG_LIST[round(percentage)];
  }
  // invalid
  return 10;
  
}

void send_TX(const boolean message[]) {
  
  for (int i=0; i<MSG_LEN; i++) {
    digitalWrite(TX_PIN, message[i]);
    delayMicroseconds(MSG_DELAY);
    digitalWrite(TX_PIN, LOW);
    delayMicroseconds(MSG_DELAY);
  }
    
}

void pass_time(long time_to_waste)
{
    long time_elapsed = millis();
    while(millis() - time_elapsed < time_to_waste)
    {
     // allow time to pass w/ interrupts actually working
    }
}

void update_lcd()
{
  lcd.empty();
  lcd.at(0,3,STATE_STRINGS[current_state]);
  lcd.at(1,3,ACTION_STRINGS[current_action]);
}
