
/*===========================
 | RAS Competition Code
 |   DISCLAIMER!!!
 |      ALL OF THIS WAS WRITTEN WITHOUT TESTING. SOMEONE SHOULD TEST IT BEFORE GETTING TOO ATTACHED
 |
 |  EVERYONE WHO CONTRIBUTED PUT NAMES HERE
 |  Segments of code by:
 |    Alexander Maricich
 |    Daniel Steele
 |    Ethan Reesor
 |    Daniel Whiteside
 |    Kierra Harrison
 ===========================*/

#include <Servo.h>

Servo mL, mR;
int leftHit = 1;
int rightHit = 2;

void setup() {
  // Initialize serial
  Serial.begin(9600);
  
  // Attach Servos
  mL.attach(10);
  mR.attach(9);
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
    mL.writeMicroseconds(2000);
    mR.writeMicroseconds(1500);
  }
  else if(instructions == rightHit) {
    // Adjust to the left
    mL.writeMicroseconds(1500);
    mR.writeMicroseconds(2000);
  }
  else {
    // Full speed ahead! 
    mL.writeMicroseconds(2000);
    mR.writeMicroseconds(2000);
  }
}

int findContact(int* sensorValues) {
  // Return what sensor was hit
  if(sensorValues[0] > 300){
    // left sensor is hit, adjust right
    // Serial.println("turn left");
    return leftHit;
  }
  else if(sensorValues[1] > 300){
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
