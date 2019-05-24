// https://electronicshobbyists.com/controlling-dc-motors-arduino-arduino-l298n-tutorial/
#include <Servo.h>
// Servo pin
Servo clawServo;
int servoPos;
//Potentiometer analog input pins a=arm sensor b=robot arm sensor
int pot_0 = A0; //hand sensor to control servo claw
int pot_1 = A1;
int pot_2 = A2; //elbow arm sensor
int pot_3 = A3; //elbow robot sensor
int pot_4 = A4; //shoulder robot sensor
int pot_6 = A6; //base robot sensor
//variables for potentiometer positions
int hand_pos;
int elbo1_pos;
int elbo2_pos;
int shoulder_pos;
int base_pos;


//          Motor Controler Pins
// elbow motor
int elboEnable = 11;      //Enable pin for base motor
int elboPin1 = 9;
int elboPin2 = 8;
// shoulder motor
int shouldEnable = 10;      //Enable pin for shoulder motor
int shouldPin1 = 7;
int shouldPin2 = 6;
// base motor
int baseEnable = 12;      //Enable pin for elbow motor
int basePin1 = 14;
int basePin2 = 15;

//Initializing variables to store data
int baseSpeed;
int shouldSpeed;
int elbowSpeed;
void setup ( ) {
  Serial.begin (9600); //Starting the serial communication at 9600 baud rate
  //Initialize the servo pin
  clawServo.attach(28);
  //Initialize the motor control pins as output
  pinMode(elboEnable, OUTPUT);
  pinMode(elboPin1, OUTPUT);
  pinMode(elboPin2, OUTPUT);
  pinMode(shouldEnable, OUTPUT);
  pinMode(shouldPin1, OUTPUT);
  pinMode(shouldPin2, OUTPUT);
  pinMode(baseEnable, OUTPUT);
  pinMode(basePin1, OUTPUT);
  pinMode(basePin2, OUTPUT);





  //Initializng the pot pins as input
  pinMode (pot_0, INPUT) ;
  pinMode (pot_1, INPUT) ;
  pinMode (pot_2, INPUT) ;
  pinMode (pot_3, INPUT) ;
  pinMode (pot_4, INPUT) ;
  pinMode (pot_6, INPUT) ;
}
void loop () {

  hand_pos = analogRead (pot_0) ;  //Reading input value
  // Claw Servo
  servoPos = map(hand_pos, 0, 1024, 0, 170);
  clawServo.write(servoPos);
  // ELBOW MOTOR
  elbo1_pos = analogRead (pot_2) ;  //Read elbow arm position value
  elbo2_pos = analogRead (pot_3) ;  //Read elbow robot position value
  if (elbo1_pos > elbo2_pos) { //CCW
    elbowSpeed = elbo1_pos - elbo2_pos;
    elbowSpeed = map(elbowSpeed, 0, 1024, 0, 3000);
    elbowSpeed = constrain(elbowSpeed, 0, 255);
    //Serial.println(elbowSpeed);
    if (elbo1_pos < 900) { //set limit for UP rotation
      Serial.println(elbo1_pos);
      digitalWrite(elboPin1, LOW); //UP
      digitalWrite(elboPin2, HIGH);
      analogWrite(elboEnable, elbowSpeed);
    } else {
      digitalWrite(elboPin1, LOW);// stop motor
      digitalWrite(elboPin2, LOW);
    }

  }
  if (elbo1_pos < elbo2_pos) { //DOWN
    baseSpeed = elbo2_pos - elbo1_pos;
    baseSpeed = map(baseSpeed, 0, 1024, 0, 3000);
    baseSpeed = constrain(baseSpeed, 0, 255);
    //Serial.println(baseSpeed);
    if (elbo1_pos > 90) { //set limit for DOWN rotation
      //Serial.println(elbo1_pos);
      digitalWrite(elboPin1, HIGH); //CW
      digitalWrite(elboPin2, LOW);
      analogWrite(elboEnable, elbowSpeed);
    } else {
      digitalWrite(elboPin1, LOW); //stop motor
      digitalWrite(elboPin2, LOW);
    }

  }
  //if robot position is close enough, stop motor
  if (elbowSpeed < 10) {
    elbowSpeed = 0;
    //Serial.println("stop");
    digitalWrite(elboPin1, LOW);
    digitalWrite(elboPin2, LOW);
  }



  // MOTOR 2


}
