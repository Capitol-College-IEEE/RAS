
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

//#include <Servo.h>
#include <Wire.h> // Used for I2C

#define RIGHT_MOTOR 9
#define LEFT_MOTOR 10
#define RIGHT_SENSOR A1
#define LEFT_SENSOR A0
#define RIGHT_HIT (1 << 0)
#define LEFT_HIT (1 << 1)

#define PWM_FREQ 488.
#define PWM_PERIOD_MS (1000./PWM_FREQ)
#define PWM_MAX 255
#define PULSE(milliseconds) ((milliseconds) / PWM_PERIOD_MS * PWM_MAX)
#define MPULSE(power) PULSE(1.5 + power / 2.)


// The SparkFun breakout board defaults to 1, set to 0 if SA0 jumper on the bottom of the board is set
#define MMA8452_ADDRESS 0x1D  // 0x1D if SA0 is high, 0x1C if low
//Define a few of the registers that we will be accessing on the MMA8452
#define OUT_X_MSB 0x01
#define XYZ_DATA_CFG  0x0E
#define WHO_AM_I   0x0D
#define CTRL_REG1  0x2A
#define GSCALE 2 // Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.

void setup() {
  // Initialize serial
  Serial.begin(9600);
  
  // Attach Servos
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(LEFT_MOTOR, OUTPUT);
  
  // TODO ensure that the line is between the two sensors
  Serial.println("MMA8452 Basic Example");

  Wire.begin(); //Join the bus as a master

  initMMA8452(); //Test and intialize the MMA8452
}

void loop() {
  lineFollow();
  //angularChallenge();
  //boulderField();
}


//  Line Follow Code
// =================================

void lineFollow() {
  // Read in the line information
  // Move the motors in the proper direction
  steerLineFollow(lineSensorHits());
}

char lineSensorHits() {
  char hits = 0;
  
  if (analogRead(RIGHT_SENSOR) > 300)
    hits |= RIGHT_HIT;
  
  if (analogRead(LEFT_SENSOR) > 300)
    hits |= LEFT_HIT;
  
  return hits;
}

void steerLineFollow(char hits) {
  Serial.print("Hits: ");
  Serial.println((int)hits);
  
  if ((hits & RIGHT_HIT) && (hits & LEFT_HIT))
  // both sides triggering
  {
    Serial.print("STOP ");
    Serial.println((int)hits);
    analogWrite(RIGHT_MOTOR, MPULSE( 0.0));
    analogWrite(LEFT_MOTOR,  MPULSE( 0.0));
  }
  else if (hits & RIGHT_HIT)
  // right side triggering
  {
    Serial.print("LEFT ");
    Serial.println((int)hits);
    analogWrite(RIGHT_MOTOR, MPULSE(+0.3));
    analogWrite(LEFT_MOTOR,  MPULSE(+0.2));
  }
  else if (hits & LEFT_HIT)
  // left side triggering
  {
    Serial.print("RIGHT ");
    Serial.println((int)hits);
    analogWrite(RIGHT_MOTOR, MPULSE(-0.2));
    analogWrite(LEFT_MOTOR,  MPULSE(-0.3));
  }
  else
  // neither side triggering
  {
    Serial.print("GO ");
    Serial.println((int)hits);
    analogWrite(RIGHT_MOTOR, MPULSE(+0.3));
    analogWrite(LEFT_MOTOR,  MPULSE(-0.3));
  }
}

////
// END Line Follow Code
////

//  Angular Challenge Code
// =================================

void angularChallenge() {
  // $$$ TODO
}

void readAccelData(int *destination)
{
  byte rawData[6];  // x/y/z accel register data stored here

  readRegisters(OUT_X_MSB, 6, rawData);  // Read the six raw data registers into data array

  // Loop to calculate 12-bit ADC and g value for each axis
  for(int i = 0; i < 3 ; i++)
  {
    int gCount = (rawData[i*2] << 8) | rawData[(i*2)+1];  //Combine the two 8 bit registers into one 12-bit number
    gCount >>= 4; //The registers are left align, here we right align the 12-bit integer

    // If the number is negative, we have to make it so manually (no 12-bit data type)
    if (rawData[i*2] > 0x7F)
    {  
      gCount = ~gCount + 1;
      gCount *= -1;  // Transform into negative 2's complement #
    }

    destination[i] = gCount; //Record this gCount into the 3 int array
  }
}

// Initialize the MMA8452 registers 
// See the many application notes for more info on setting all of these registers:
// http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=MMA8452Q
void initMMA8452()
{
  byte c = readRegister(WHO_AM_I);  // Read WHO_AM_I register
  if (c == 0x2A) // WHO_AM_I should always be 0x2A
  {  
    Serial.println("MMA8452Q is online...");
  }
  else
  {
    Serial.print("Could not connect to MMA8452Q: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }

  MMA8452Standby();  // Must be in standby to change registers

  // Set up the full scale range to 2, 4, or 8g.
  byte fsr = GSCALE;
  if(fsr > 8) fsr = 8; //Easy error check
  fsr >>= 2; // Neat trick, see page 22. 00 = 2G, 01 = 4A, 10 = 8G
  writeRegister(XYZ_DATA_CFG, fsr);

  //The default data rate is 800Hz and we don't modify it in this example code

  MMA8452Active();  // Set to active to start reading
}

// Sets the MMA8452 to standby mode. It must be in standby to change most register settings
void MMA8452Standby()
{
  byte c = readRegister(CTRL_REG1);
  writeRegister(CTRL_REG1, c & ~(0x01)); //Clear the active bit to go into standby
}

// Sets the MMA8452 to active mode. Needs to be in this mode to output data
void MMA8452Active()
{
  byte c = readRegister(CTRL_REG1);
  writeRegister(CTRL_REG1, c | 0x01); //Set the active bit to begin detection
}

// Read bytesToRead sequentially, starting at addressToRead into the dest byte array
void readRegisters(byte addressToRead, int bytesToRead, byte * dest)
{
  Wire.beginTransmission(MMA8452_ADDRESS);
  Wire.write(addressToRead);
  Wire.endTransmission(false); //endTransmission but keep the connection active

  Wire.requestFrom(MMA8452_ADDRESS, bytesToRead); //Ask for bytes, once done, bus is released by default

  while(Wire.available() < bytesToRead); //Hang out until we get the # of bytes we expect

  for(int x = 0 ; x < bytesToRead ; x++)
    dest[x] = Wire.read();    
}

// Read a single byte from addressToRead and return it as a byte
byte readRegister(byte addressToRead)
{
  Wire.beginTransmission(MMA8452_ADDRESS);
  Wire.write(addressToRead);
  Wire.endTransmission(false); //endTransmission but keep the connection active

  Wire.requestFrom(MMA8452_ADDRESS, 1); //Ask for 1 byte, once done, bus is released by default

  while(!Wire.available()) ; //Wait for the data to come back
  return Wire.read(); //Return this one byte
}

// Writes a single byte (dataToWrite) into addressToWrite
void writeRegister(byte addressToWrite, byte dataToWrite)
{
  Wire.beginTransmission(MMA8452_ADDRESS);
  Wire.write(addressToWrite);
  Wire.write(dataToWrite);
  Wire.endTransmission(); //Stop transmitting
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
