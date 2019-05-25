//I use this CS file to copy/paste bits of code into for more advanced Sublime Text abilties

//  ***Base Operation***  //
  base_pos = analogRead (pot_6);
  servo_yz = map(servo_yz, -20, 20, 90, 120);
  if (servo_yz > base_pos) { //UP
    baseSpeed = servo_yz - base_pos;
    baseSpeed = map(baseSpeed, 0, 1024, 0, 3000);
    baseSpeed = constrain(baseSpeed, 0, 255);
    //Serial.println(baseSpeed);
    if (servo_yz < 900) { //set limit for UP rotation
      Serial.println(servo_yz);
      digitalWrite(basePin1, LOW); //UP
      digitalWrite(basePin2, HIGH);
      analogWrite(baseEnable, baseSpeed);
    } else {
      digitalWrite(basePin1, LOW);// stop motor
      digitalWrite(basePin2, LOW);
    }

  }
  if (servo_yz < base_pos) { //DOWN
    baseSpeed = base_pos - servo_yz;
    baseSpeed = map(baseSpeed, 0, 1024, 0, 3000);
    baseSpeed = constrain(baseSpeed, 0, 255);
    //Serial.println(baseSpeed);
    if (servo_yz > 90) { //set limit for DOWN rotation
      //Serial.println(base_pos);
      digitalWrite(basePin1, HIGH); //CW
      digitalWrite(basePin2, LOW);
      analogWrite(baseEnable, baseSpeed);
    } else {
      digitalWrite(basePin1, LOW); //stop motor
      digitalWrite(basePin2, LOW);
    }

  }
  //if robot position is close enough, stop motor
  if (baseSpeed < 10) {
    baseSpeed = 0;
    //Serial.println("stop");
    digitalWrite(basePin1, LOW);
    digitalWrite(basePin2, LOW);
  }
