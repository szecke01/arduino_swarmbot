const int motor_pin = 53;    // FET control
const int motor_enable = 3;  // button 
int button_state;

void setup() {
  pinMode(motor_pin, OUTPUT);
  pinMode(motor_enable, INPUT_PULLUP);
  
  digitalWrite(motor_pin, LOW);
}

void loop() {
  button_state = digitalRead(motor_enable);
  if(button_state == LOW) { digitalWrite(motor_pin, HIGH); }
  else { digitalWrite(motor_pin, LOW); } 
}
