/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
*/

#define LED 1

const bool heartbeats[] = {1, 0, 1, 0, 0, 0, 0, 0};

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED, heartbeats[((millis() >> 7) & 7)]);
  delay(10);                       // wait for a second
} 
