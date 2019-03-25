/*
  Pump Float Switch
 
 Turns the pump OFF when the water level gets low enough
 to let the float down.
 Then a set time after float goes up, the pump goes on again. 
 
 
 The circuit:
 * Relay attached from pin 13 to ground 
 * float switch attached to pin 2 from +5V
 * 10K resistor attached to pin 2 from ground
 
 
 https://github.com/WillWelker/arduino-sketches
 Adapted from the Arduino Button example
 */


const int floatSwitch = 2;     // input pin number for float switch
const int relayPin =  13;      // output pin to relay

// variables will change:
int switchState = 0;         // variable for reading the float status

void setup() {
  // initialize the relay pin as an output:
  pinMode(relayPin, OUTPUT);      
  // initialize the float switch pin as an input:
  pinMode(floatSwitch, INPUT);     
}

void fill() {
  // When float switch goes up, wait a while then pump out water
}

void loop(){
  // read the state of the float switch value:
  switchState = digitalRead(floatSwitch);

  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:
  if (switchState == HIGH) {     
    // turn relay and pump on:    
    digitalWrite(relayPin, HIGH);  
  } 
  else {
    // turn relay and pum off:
    digitalWrite(relayPin, LOW); 
  }
}
