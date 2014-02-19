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

AF_DCMotor leftMotor(3); // Left Motor
AF_DCMotor rightMotor(4); // Right Motor
int leftHit = 1;
int rightHit = 2;

void setup() {
  // Initialize serial
  Serial.begin(9600);
  // Set up motors initial settings
  //moveMotor(leftMotor, FORWARD);
  //moveMotor(rightMotor, FORWARD);
  // LATER: ensure that the line is between the two sensors
}

void loop() {
  //lineFollow();
  // angularChallenge();
  // boulderField();
}


//  Line Follow Code
// =================================

void lineFollow() {
  int data[2] = {0, 0};

  // Read in the line information
  retrieveLineSensorData(data);
  // Move the motors in the proper direction
  steer(data);
}

void retrieveLineSensorData(int* data) {
  data[0] = analogRead(A0);
  data[1] = analogRead(A1);
}

void steer(int* dataArray) {
  // Find what direction the robot needs to turn
  int instructions = findContact(dataArray);
  // Set motors to proper value
  if(instructions == leftHit) {
    // Adjust to the right
    leftMotor.run(FORWARD);
    rightMotor.run(BACKWARD);
  }
  else if(instructions == rightHit) {
    // Adjust to the left
    leftMotor.run(BACKWARD);
    rightMotor.run(FORWARD);
  }
  else {
    // Full speed ahead! 
    leftMotor.run(FORWARD);
    rightMotor.run(FORWARD);
  }
}

int findContact(int* motorValues) {
  // Return what sensor was hit
  if(motorValues[0] > 300){
    // left sensor is hit, adjust right
    // Serial.println("turn left");
    return leftHit;
  }
  else if(motorValues[1] > 300){
    // right sensor is hit, adjust left
    // Serial.println("turn right");
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
