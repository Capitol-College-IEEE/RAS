#include <AFMotor.h>

AF_DCMotor motor(1); // Left Motor
AF_DCMotor motor2(2); // Right Motor

uint8_t speedRight = 122; // Speed of Right Motor
uint8_t speedLeft = 122; // Speed of Left Motor

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
   motor.setSpeed(speedLeft);
   motor2.setSpeed(speedRight);
   motor.run(RELEASE);
   motor2.run(RELEASE);
   Serial.println("Start");
}
//Black is High White is Low
// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  Serial.println("test");
  int RightLine = analogRead(A0);
  int CenterLine = analogRead(A1);
  int LeftLine = analogRead(A2);
  
  if(CenterLine > 300){
    stayCenter();
  }
  else if(RightLine > 300){
    adjustLeft();
  }
  else if(LeftLine > 300){
    adjustRight();
  }
   motor.setSpeed(speedLeft);
   motor2.setSpeed(speedRight);
   motor.run(RELEASE);
   motor2.run(RELEASE);
   /*
  // print out the value you read:
  Serial.print(RightLine);// Right- Purple- AO
  Serial.print("\t");
  Serial.print(CenterLine);//Center-Yellow- A1
  Serial.print("\t");
  Serial.println(LeftLine); // Left - Green- A2
  */
 
  delay(1);        // delay in between reads for stability
}

void adjustLeft(){
  Serial.println("Adjusting Left");
  speedRight += 20; 
}

void adjustRight(){
  Serial.println("Adjusting Right");
  speedLeft += 20; 
}

void stayCenter(){
  Serial.println("Staying on Course");
  speedRight = 122;
  speedLeft = 122;
}
