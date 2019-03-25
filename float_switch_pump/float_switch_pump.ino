/*
  Pump Float Switch
 
 Turns the pump OFF when the water level gets low enough
 to let the float down.
 Then a set time after float goes up, the pump goes on again. 
 
 
 The circuit:
 * Relay attached from pin 13 to ground 
 * float switch attached to pin 2 from +5V
 * 10K resistor attached to pin 2 from ground
 
 * Note: on most Arduinos there is already an LED on the board
 attached to pin 13.
 
 
 created 2005
 by DojoDave <http://www.0j0.org>
 modified 30 Aug 2011
 by Tom Igoe
 
 This example code is in the public domain.
 
 http://www.arduino.cc/en/Tutorial/Button
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

void loop(){
  // read the state of the pushbutton value:
  switchState = digitalRead(floatSwitch);

  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:
  if (switchState == HIGH) {     
    // turn LED on:    
    digitalWrite(relayPin, HIGH);  
  } 
  else {
    // turn LED off:
    digitalWrite(relayPin, LOW); 
  }
}
