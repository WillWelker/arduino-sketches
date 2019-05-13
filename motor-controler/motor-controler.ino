// https://electronicshobbyists.com/controlling-dc-motors-arduino-arduino-l298n-tutorial/

//Joystick Pins
int pot1a = A0;
int pot1b = A1;
int pos_a1;
int pos_a2;
//Motor Pins
int EN_A = 11;      //Enable pin for first motor
int IN1 = 9;       //control pin for first motor
int IN2 = 8;       //control pin for first motor
int IN3 = 7;        //control pin for second motor
int IN4 = 6;        //control pin for second motor
int EN_B = 10;      //Enable pin for second motor
//Initializing variables to store data
int baseSpeed;
int shouldSpeed;
void setup ( ) {
  Serial.begin (9600); //Starting the serial communication at 9600 baud rate
  //Initializing the motor pins as output
  pinMode(EN_A, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(EN_B, OUTPUT);
  //Initializng the pot pins as input
  pinMode (pot1a, INPUT) ;
  pinMode (pot1b, INPUT) ;
}
void loop () {
  pos_a1 = analogRead (pot1a) ;  //Reading input value
  pos_a2 = analogRead (pot1b) ;  //Reading robot position value
  //Serial.println(pos_a2);
  // BASE MOTOR

  if (pos_a1 > pos_a2) { //CCW
    baseSpeed = pos_a1 - pos_a2;
    baseSpeed = map(baseSpeed, 0, 1024, 0, 3000);
    baseSpeed = constrain(baseSpeed, 0, 255);
    //Serial.println(baseSpeed);
    if (pos_a2 < 900) {
      Serial.println(pos_a2);
      digitalWrite(IN1, LOW); //CCW
      digitalWrite(IN2, HIGH);
      analogWrite(EN_A, baseSpeed);
    } else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
    }

  }
  if (pos_a1 < pos_a2) { //CW
    baseSpeed = pos_a2 - pos_a1;
    baseSpeed = map(baseSpeed, 0, 1024, 0, 3000);
    baseSpeed = constrain(baseSpeed, 0, 255);
    //Serial.println(baseSpeed);
    if (pos_a2 > 90) {
      //Serial.println(pos_a2);
      digitalWrite(IN1, HIGH); //CW
      digitalWrite(IN2, LOW);
      analogWrite(EN_A, baseSpeed);
    } else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
    }

  }
  if (baseSpeed < 10) {
    baseSpeed = 0;
    //Serial.println("stop");
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }



  // MOTOR 2


}
