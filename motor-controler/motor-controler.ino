// Athena's Robot Arm Project
// Running on Arduino Mega
// Include Wire.h to communicate with MPU-6050 over I2C
#include <Wire.h>
// Include Servo.h to run the claw servo
#include <Servo.h>

//Varables for enable and reset position
bool enable;
bool disengage;
int enablePin = 16; // for enable/disable switch
int disengagePin = 17; // for push button disengage (like lifting up your mouse to reset its position)
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
long loop_timer;
int temp; // not currently usede but the MCU can give temp reading

// Servo pin
Servo clawServo;
int servoPos;
//Potentiometer analog input pins a=arm sensor b=robot arm sensor
int pot_0 = A0; //hand sensor to control servo claw
int pot_1 = A1; //not used yet
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
  // Init Timer
  loop_timer = micros();
  //Initialize the servo pin
  clawServo.attach(28);
  // enable switch input pin
  pinMode(enablePin, INPUT_PULLUP);
  pinMode(disengagePin, INPUT_PULLUP);
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
  enable = digitalRead(enablePin);
  if (enable = LOW) { // if enable pulled low, run everything
    // **Get data from MPU-6050** //
    read_mpu_6050_data();
    //Subtract the offset values from the raw gyro values
    gyro_x -= gyro_x_cal;
    gyro_y -= gyro_y_cal;
    gyro_z -= gyro_z_cal;
    //Gyro angle calculations . Note 0.0000611 = 1 / (250Hz x 65.5)

    //Calculate the traveled pitch angle and add this to the angle_pitch variable
    disengage = digitalRead(disengagePin);
    if (disengage = LOW) { // if disengage button is not pressed, calculate movement
      angle_pitch += gyro_x * 0.0000611;
      //Calculate the traveled roll angle and add this to the angle_roll variable
      //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
      angle_roll += gyro_y * 0.0000611;
      angle_yaw += gyro_z * 0.0000611;
      //If the IMU has yawed transfer the roll angle to the pitch angle
      //angle_pitch += angle_roll * sin(gyro_z * 0.000001066);
      //If the IMU has yawed transfer the pitch angle to the roll angle
      //angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);

      //Calculate the total accelerometer vector
      //acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));
    }
    //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
    //Calculate the pitch angle
    //angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;
    //Calculate the roll angle
    //angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;

    //Accelerometer calibration value
    angle_pitch_acc -= 0.0;
    angle_roll_acc -= 0.0;
    angle_yaw_acc -= 0.0;
    if (set_gyro_angles) {

      //If the IMU has been running
      //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
      //angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
      //Correct the drift of the gyro roll angle with the accelerometer roll angle
      //angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;
    }
    else {
      //IMU has just started
      //Set the gyro pitch/roll/yaw angle equal to the accelerometer pitch/roll/yaw angle
      angle_pitch = angle_pitch_acc;
      angle_roll = angle_roll_acc;
      angle_yaw = angle_yaw_acc;
      //Set the IMU started flag
      set_gyro_angles = true;
    }

    //To dampen the pitch and roll angles a complementary filter is used
    //Take 90% of the output value and add 10% of the raw value
    angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;
    angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;
    angle_yaw_output = angle_yaw_output * 0.9 + angle_yaw * 0.1;
    

    // Print to Serial Monitor

    servo_x = angle_pitch_output; //int(map(angle_pitch_output, -100, 100, 0, 180));
    servo_y = angle_roll_output; //int(map(angle_roll_output, 0, 200, 0, 180));
    servo_z = angle_yaw_output; //int(map(angle_yaw_output, 0, 200, 0, 180));
    if (servo_x > 0) {
      servo_yz = servo_y + servo_z;
    }
    if (servo_x < 0) {
      servo_yz = (servo_y * -1) + servo_z;
    }
    //Serial.print(" | x = "); Serial.print(servo_x);
    //Serial.print(" | YZ = "); Serial.println(servo_yz);
    //Serial.print(" | Z = "); Serial.println(servo_z);
    // ***END MPU calcs*** //

    // ***Claw Operation***  //
    hand_pos = analogRead (pot_0) ;  //Reading input value
    servoPos = map(hand_pos, 0, 1024, 0, 170);
    clawServo.write(servoPos);

    // ***Elbow***  //
    elbo1_pos = analogRead (pot_2);  //Read elbow arm position value
    elbo2_pos = analogRead (pot_3);  //Read elbow robot position value
    Serial.print("Elbo position: ");
    Serial.print(elbo2_pos);
    if (elbo1_pos > elbo2_pos) { //UP
      elbowSpeed = elbo1_pos - elbo2_pos;
      elbowSpeed = map(elbowSpeed, 0, 1024, 0, 3000);
      elbowSpeed = constrain(elbowSpeed, 0, 255);
      //Serial.println(elbowSpeed);
      digitalWrite(elboPin1, LOW); //UP
      if (elbo2_pos < 700) { //set limit for UP rotation
        digitalWrite(elboPin2, HIGH);
        //analogWrite(elboEnable, elbowSpeed);
      } else {
        digitalWrite(elboPin2, LOW); //stop motor
      }

    }
    if (elbo1_pos < elbo2_pos) { //DOWN
      baseSpeed = elbo2_pos - elbo1_pos;
      baseSpeed = map(baseSpeed, 0, 1024, 0, 3000);
      baseSpeed = constrain(baseSpeed, 0, 255);
      //Serial.println(baseSpeed);
      digitalWrite(elboPin2, LOW);
      if (elbo2_pos > 200) { //set limit for DOWN rotation
        //Serial.println(elbo1_pos);
        digitalWrite(elboPin1, HIGH); //DOWN
        //analogWrite(elboEnable, elbowSpeed);
      } else {
        digitalWrite(elboPin1, LOW); //stop motor
        
      }

    }
    //if robot position is close enough, stop motor
    if (elbowSpeed < 10) {
      elbowSpeed = 0;
      //Serial.println("stop");
      digitalWrite(elboPin1, LOW);
      digitalWrite(elboPin2, LOW);
    }

    //  ***Shoulder Operation***  //
    shoulder_pos = analogRead (pot_4);
    Serial.print("| Shoulder position: ");
    Serial.print(shoulder_pos);
    servo_x = map(servo_x, -20, 20, 90, 120);
    if (servo_x > shoulder_pos) { //UP
      shouldSpeed = servo_x - shoulder_pos;
      shouldSpeed = map(shouldSpeed, 0, 1024, 0, 3000);
      shouldSpeed = constrain(shouldSpeed, 0, 255);
      //Serial.println(shouldSpeed);
      digitalWrite(shouldPin1, LOW); //UP
      if (servo_x < 900) { //set limit for UP rotation
        digitalWrite(shouldPin2, HIGH);
        //analogWrite(shouldEnable, shouldSpeed);
      } else {
        digitalWrite(shouldPin2, LOW);
      }

    }
    if (servo_x < shoulder_pos) { //DOWN
      shouldSpeed = shoulder_pos - servo_x;
      shouldSpeed = map(shouldSpeed, 0, 1024, 0, 3000);
      shouldSpeed = constrain(shouldSpeed, 0, 255);
      //Serial.println(baseSpeed);
      digitalWrite(shouldPin2, LOW);
      if (servo_x > 90) { //set limit for DOWN rotation
        digitalWrite(shouldPin1, HIGH); //DOWN
        //analogWrite(shouldEnable, shouldSpeed);
      } else {
        digitalWrite(shouldPin1, LOW); //stop motor
      }

    }
    //if robot position is close enough, stop motor
    if (shouldSpeed < 10) {
      shouldSpeed = 0;
      //Serial.println("stop");
      digitalWrite(shouldPin1, LOW);
      digitalWrite(shouldPin2, LOW);
    }

    //  ***Base Operation***  //
    base_pos = analogRead (pot_6);
    Serial.print("| Base position: ");
    Serial.println(base_pos);
    servo_yz = map(servo_yz, -20, 20, 90, 120);
    if (servo_yz > base_pos) { //Left
      baseSpeed = servo_yz - base_pos;
      baseSpeed = map(baseSpeed, 0, 1024, 0, 3000);
      baseSpeed = constrain(baseSpeed, 0, 255);
      //Serial.println(baseSpeed);
      digitalWrite(basePin1, LOW); //Left
      if (servo_yz < 900) { //set limit for Left rotation
        digitalWrite(basePin2, HIGH);
        //analogWrite(baseEnable, baseSpeed);
      } else {
        digitalWrite(basePin2, LOW); //stop motor
      }

    }
    if (servo_yz < base_pos) { //Right
      baseSpeed = base_pos - servo_yz;
      baseSpeed = map(baseSpeed, 0, 1024, 0, 3000);
      baseSpeed = constrain(baseSpeed, 0, 255);
      //Serial.println(baseSpeed);
      digitalWrite(basePin2, LOW);
      if (servo_yz > 90) { //set limit for Right rotation
        digitalWrite(basePin1, HIGH); //Right
        //analogWrite(baseEnable, baseSpeed);
      } else {
        digitalWrite(basePin1, LOW); //stop motor
      }

    }
    //if robot position is close enough, stop motor
    if (baseSpeed < 10) {
      baseSpeed = 0;
      //Serial.println("stop");
      digitalWrite(basePin1, LOW);
      digitalWrite(basePin2, LOW);
    }


    //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
    // The MPU 6050 produces new data at 250HZ
    // So we pause until enough microseconds have passed
    while (micros() - loop_timer < 4000);
    //Reset the loop timer
    loop_timer = micros();
  }else{
    //reset to MPU angles
    set_gyro_angles = false;
  }
}
// ***End of Loop *** //

// ***MCU functions*** //
// This function is called during board setup
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

// This function is called in the Loop
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
