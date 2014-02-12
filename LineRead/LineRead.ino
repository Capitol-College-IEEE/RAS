/*
  AnalogReadSerial
  Reads an analog input on pin 0, prints the result to the serial monitor.
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.
 
 This example code is in the public domain.
 */

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
 
  int RightLine = analogRead(A0);
  int CenterLine = analogRead(A1);
  int LeftLine = analogRead(A2);
   
  // print out the value you read:
  Serial.print(RightLine);// Right- Purple- AO
  Serial.print("\t");
  Serial.print(CenterLine);//Center-Yellow- A1
  Serial.print("\t");
  Serial.println(LeftLine); // Left - Green- A2

 
  delay(1);        // delay in between reads for stability
}
