//I use this CS file to copy/paste bits of code into for more advanced Sublime Text abilties

int pot_0 = A0; //hand sensor to control servo claw
int pot_1 = A1; //not used yet
int pot_2 = A2; //elbow arm sensor
int pot_3 = A3; //elbow robot sensor
int pot_4 = A4; //shoulder robot sensor
int pot_6 = A6; //base robot sensor

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

int enablePin = 16; // for enable/disable switch
int disengagePin = 17; // for push button disengage (like lifting up your mouse to reset its position)