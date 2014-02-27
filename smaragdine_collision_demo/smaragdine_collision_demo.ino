const int RED_LED_PIN = 15;
const int SWITCH_IN = 41;

void setup() {
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(SWITCH_IN, INPUT);
}

void loop() {
  if(digitalRead(SWITCH_IN)==HIGH) {
    digitalWrite(RED_LED_PIN, HIGH);
  }
  else {
    digitalWrite(RED_LED_PIN, LOW);
  }
}
