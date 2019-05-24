// Athena's Robot Arm Project
#include <Wire.h>
#include <Servo.h>

//Variables for Gyroscope
int gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
boolean set_gyro_angles;
long acc_x, acc_y, acc_z, acc_total_vector;
float angle_roll_acc, angle_pitch_acc, angle_yaw_acc;
float angle_pitch, angle_roll, angle_yaw;
int angle_pitch_buffer, angle_roll_buffer, angle_yaw_buffer;
float angle_pitch_output, angle_roll_output, angle_yaw_output;
int servo_x, servo_y, servo_z, servo_yz;
// Setup timers and temp variables
//long loop_timer;
int temp;

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
  Wire.begin();
  //Setup the registers of the MPU-6050
  setup_mpu_6050_registers();
  //Read the raw acc and gyro data from the MPU-6050 1000 times
  for (int cal_int = 0; cal_int < 1000 ; cal_int ++) {
    read_mpu_6050_data();
    //Add the gyro x offset to the gyro_x_cal variable
    gyro_x_cal += gyro_x;
    //Add the gyro y offset to the gyro_y_cal variable
    gyro_y_cal += gyro_y;
    //Add the gyro z offset to the gyro_z_cal variable
    gyro_z_cal += gyro_z;
    //Delay 3us to have 250Hz for-loop
    delay(3);
  }
  // Divide all results by 1000 to get average offset
  gyro_x_cal /= 1000;
  gyro_y_cal /= 1000;
  gyro_z_cal /= 1000;
  
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
  // Get data from MPU-6050
  read_mpu_6050_data();
  //Subtract the offset values from the raw gyro values
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;
  
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

// MCU setup functions
void setup_mpu_6050_registers() {

  //Activate the MPU-6050

  //Start communicating with the MPU-6050
  Wire.beginTransmission(0x68);
  //Send the requested starting register
  Wire.write(0x6B);
  //Set the requested starting register
  Wire.write(0x00);
  //End the transmission
  Wire.endTransmission();

  //Configure the accelerometer (+/-8g)

  //Start communicating with the MPU-6050
  Wire.beginTransmission(0x68);
  //Send the requested starting register
  Wire.write(0x1C);
  //Set the requested starting register
  Wire.write(0x10);
  //End the transmission
  Wire.endTransmission();

  //Configure the gyro (500dps full scale)

  //Start communicating with the MPU-6050
  Wire.beginTransmission(0x68);
  //Send the requested starting register
  Wire.write(0x1B);
  //Set the requested starting register
  Wire.write(0x08);
  //End the transmission
  Wire.endTransmission();

}


void read_mpu_6050_data() {

  //Read the raw gyro and accelerometer data

  //Start communicating with the MPU-6050
  Wire.beginTransmission(0x68);
  //Send the requested starting register
  Wire.write(0x3B);
  //End the transmission
  Wire.endTransmission();
  //Request 14 bytes from the MPU-6050
  Wire.requestFrom(0x68, 14);
  //Wait until all the bytes are received
  while (Wire.available() < 14);

  //Following statements left shift 8 bits, then bitwise OR.
  //Turns two 8-bit values into one 16-bit value
  acc_x = Wire.read() << 8 | Wire.read();
  acc_y = Wire.read() << 8 | Wire.read();
  acc_z = Wire.read() << 8 | Wire.read();
  temp = Wire.read() << 8 | Wire.read();
  gyro_x = Wire.read() << 8 | Wire.read();
  gyro_y = Wire.read() << 8 | Wire.read();
  gyro_z = Wire.read() << 8 | Wire.read();
}
