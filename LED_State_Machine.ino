/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
 
  This example code is in the public domain.
 */
 
// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int led_r = 53;
int led_b = 49;
int switch_1 = 2;
int switch_2 = 3;

// 0 = off
// 1 = running
// 2 = sleep
// 3 = diagnostic
int state = 0; 


// the setup routine runs once when you press reset:
void setup() {  
  
  // initialize the digital pins as an output/inputs.
  pinMode(led_r, OUTPUT);     
  pinMode(led_b, OUTPUT);
  pinMode(switch_1, INPUT_PULLUP);
  pinMode(switch_2, INPUT_PULLUP);
  
  // set up interrupts
  attachInterrupt(0, change_state, FALLING);
  attachInterrupt(1, change_state, FALLING);
}

// the loop routine runs over and over again forever:
void loop() {
  
  // if off
  if(state == 0)
  {
    perform_off();
  }
  
  if(state == 1)
  {
    perform_running();
  }
  
  if(state == 2)
  {
    perform_sleep();
  }
  
  if(state == 3)
  {
    perform_diagnostic();
  }
  
}

void perform_off()
{
  digitalWrite(led_r, LOW);
  digitalWrite(led_b, LOW);
}

void perform_running()
{
  
}

void perform_sleep()
{
  delay(2000);
  digitalWrite(led_b, HIGH);
  delay(50);
  digitalWrite(led_b, LOW);
}

void perform_diagnostic()
{
  delay(500);
  digitalWrite(led_b, HIGH);
  digitalWrite(led_r, LOW);
  delay(500);
  digitalWrite(led_b, LOW);
  digitalWrite(led_r, HIGH);
}

void change_state()
{
  switch_val_1 = digitalRead(switch_1);
  switch_val_2 = digitalRead(switch_2);
  
  if(switch_val_1 == HIGH && switch_val_2 == HIGH)
    state = 3;
  if 
  
}
