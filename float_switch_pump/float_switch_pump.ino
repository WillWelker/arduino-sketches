/*
  Pump Float Switch

  Turns the pump OFF when the water level gets low enough
  to let the float down.
  Then a set time after float goes up, the pump goes on again.


  The circuit:
   Relay attached from pin 13 to ground
   Float switch attached to pin 2 from +5V
   10K pull-down resistor attached to pin 2 from ground
   When float is down, the circuit is closed giving us a HIGH input

  https://github.com/WillWelker/arduino-sketches
  Adapted from the Arduino Button example
*/


const int floatSwitch = 2;     // input pin number for float switch
const int relayPin =  12;      // output pin to relay
const int ledPin =  13;      // output pin to LED

// variables will change:
int counter = 0;
int floatState = 0;         // variable for reading the float status
bool highWater = true;      // boolean variable to store state of water level

void setup() {
  // initialize the relay pin as an output:
  pinMode(relayPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  // initialize the float switch pin as an input:
  pinMode(floatSwitch, INPUT);
}

void fill() {
  // When float switch goes down, wait a while then stop pump so well can fill
  delay(2000);  // wait 2 seconds
  digitalWrite(relayPin, LOW);  // turn pump off
}

void drain() {
  // When float switch goes up, wait a while then start pump to drain well
  delay(40000);	// wait 40 seconds
  digitalWrite(relayPin, HIGH);  // turn pump on
}

void runTest() {
  digitalWrite(relayPin, HIGH);
  delay(2000);
  digitalWrite(ledPin, HIGH);
  delay(2000);
  digitalWrite(relayPin, LOW);
  delay(2000);
  digitalWrite(ledPin, LOW);
  delay(2000);

}

void loop() {
  floatState = digitalRead(floatSwitch);

  // check if the float is up or down.
  // if the float is down
  if (floatState == LOW) {

    digitalWrite(ledPin, HIGH);
    digitalWrite(relayPin, HIGH);
  }

  if (floatState == HIGH) {
    
    digitalWrite(ledPin, LOW);
    delay(1000);
    digitalWrite(relayPin, LOW);
    delay(40000);
  }



}
