/*===========================
 | RAS Competition Code
 |   DISCLAIMER!!!
 |      ALL OF THIS WAS WRITTEN WITHOUT TESTING. SOMEONE SHOULD TEST IT BEFORE GETTING TOO ATTACHED
 |
 |  EVERYONE WHO CONTRIBUTED PUT NAMES HERE
 |  Segments of code by:
 |    Alexander Maricich, 
 ===========================*/

#include <AFMotor.h>

AF_DCMotor leftMotor(1); // Left Motor
AF_DCMotor rightMotor(2); // Right Motor
int leftHit = 1;
int rightHit = 2;

void setup() {
  // Initialize serial
  Serial.begin(9600);
  // Set up motors initial settings
  moveMotor(leftMotor, 0, FORWARD);
  moveMotor(rightMotor, 0, FORWARD);
  // LATER: ensure that the line is between the two sensors
}

void loop() {
  lineFollow();
  // angularChallenge();
  // boulderField();
}


//  Line Follow Code
// =================================

void lineFollow() {
  // Read in the line information
  int* lineData = retrieveLineSensorData();
  // Move the motors in the proper direction
  steer(lineData);
}

int* retrieveLineSensorData() {
  int data[3] = {};
  data[0] = analogRead(A0);
  data[1] = analogRead(A1);
  data[2] = analogRead(A2);
  return data;
}

void steer(int* dataArray) {
  // Find what direction the robot needs to turn
  int instructions = findContact(dataArray);
  // Set motors to proper value
  if(instructions == leftHit) {
    // Adjust to the right
    moveMotor(leftMotor, 255, FORWARD);
    moveMotor(rightMotor, 255, BACKWARD);
  }
  else if(instructions == rightHit) {
    // Adjust to the left
    moveMotor(leftMotor, 255, BACKWARD);
    moveMotor(rightMotor, 255, FORWARD);
  }
  else {
    // Full speed ahead! 
    moveMotor(leftMotor, 255, FORWARD);
    moveMotor(rightMotor, 255, FORWARD);
  }
}

void moveMotor(AF_DCMotor motor, int sp, int direct) {
  // Pass a motor, speed, and direction to move each motor
  motor.setSpeed(sp);
  motor.run(RELEASE);
  motor.run(direct);
}

int findContact(int* motorValues) {
  // Return what sensor was hit
  if(motorValues[0] > 300){
    // left sensor is hit, adjust right
    return leftHit;
  }
  else if(motorValues[2] > 300){
    // right sensor is hit, adjust left
    return rightHit;
  }
  else
    return 0;
}




////
// END Line Follow Code
////

//  Angular Challenge Code
// =================================

void angularChallenge() {
  // $$$ TODO
}

////
// END Angular Challenge Code
////

//  Angular Challenge Code
// =================================

void boulderField() {
  // $$$ TODO
}


////
// END Angular Challenge Code
////
