
/*===========================
 | RAS Competition Code
 |
 |  Contributors:
 |    Alexander Maricich
 |    Daniel Steele
 |    Ethan Reesor
 |    Daniel Whiteside
 |    Kierra Harrison
 ===========================*/

#include <Wire.h> // Used for I2C
#include <XBee.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// Line Sensor
#define RIGHT_MOTOR 9
#define LEFT_MOTOR 10
#define RIGHT_SENSOR A1
#define LEFT_SENSOR A0
#define RIGHT_HIT (1 << 0)
#define LEFT_HIT (1 << 1)

// Accelerometer
MPU6050 mpu;
#define INT_in 2
#define X_OFFSET 220
#define Y_OFFSET 76
#define Z_OFFSET -85
#define Z_ACCEL_OFFSET 1788
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// Motors
#define PWM_FREQ 488.
#define PWM_PERIOD_MS (1000./PWM_FREQ)
#define PWM_MAX 255
#define PULSE(milliseconds) (PWM_MAX * (milliseconds) / PWM_PERIOD_MS)
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

  // Attach Servos
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(LEFT_MOTOR, OUTPUT);
  
  
  initComms();
  //Setup for jumper for control
  pinMode(12, OUTPUT);
  pinMode(11, INPUT);
  
  digitalWrite(12, HIGH);
  
  
  pinMode(13, OUTPUT);

  // 
  accelerometerSetup();

}

void loop() {
  static unsigned char led_pattern[] = {
    1,1,1,              0,0,0,
    1,                  0,0,0,
    1,0,1,0,1,          0,0,0,
    1,0,1,1,1,0,1,0,1,  0,0,0,
    1,0,1,1,1,          0,0,0,
    0,                  0,0,0
  };
  static unsigned int led_index = 0;
  static unsigned long time = 0;
  
  unsigned long next_time = millis();
  if (next_time - time > 100) {
    digitalWrite(13, led_pattern[led_index++]);
    if (led_index >= sizeof(led_pattern))
      led_index = 0;
    time = next_time;
  }
  
  if(digitalRead(11) == HIGH){
    remoteControl();
  }  
  else{
    lineFollow();
    //angularChallenge();
    //boulderField();
  }
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
  Serial.println((unsigned char) hits);
  return hits;
}

void steerLineFollow(char hits) {
  if ((hits & RIGHT_HIT) && (hits & LEFT_HIT))
  // both sides triggering
  {
    analogWrite(RIGHT_MOTOR, MPULSE( 0.0));
    analogWrite(LEFT_MOTOR,  MPULSE( 0.0));
  }
  else if (hits & RIGHT_HIT)
  // right side triggering
  {
    analogWrite(RIGHT_MOTOR, MPULSE(+0.6));
    analogWrite(LEFT_MOTOR,  MPULSE(+0.4));
  }
  else if (hits & LEFT_HIT)
  // left side triggering
  {
    analogWrite(RIGHT_MOTOR, MPULSE(-0.4));
    analogWrite(LEFT_MOTOR,  MPULSE(-0.6));
  }
  else
  // neither side triggering
  {
    analogWrite(RIGHT_MOTOR, MPULSE(+0.6));
    analogWrite(LEFT_MOTOR,  MPULSE(-0.6));
  }
}

////
// END Line Follow Code
////


// Commumications & Control
// ====================================

XBee xbee = XBee();
Rx16IoSampleResponse io16 = Rx16IoSampleResponse();
Rx64IoSampleResponse io64 = Rx64IoSampleResponse();


void remoteControl(){
  updateComms();
  
}  

void remoteDrive(int yL, int xL, int yR, int xR){
  //0 to 1024
  yL = 512 - yL;
  yR = 512 - yR;
  yL = MPULSE(((float)yL) / 512.);
  yR = MPULSE(((float)yR) / 512.);
  analogWrite(RIGHT_MOTOR, yR);
  analogWrite(LEFT_MOTOR,  yL);
}

void initComms() {
  Serial.begin(9600);
  
  xbee.setSerial(Serial);
}

void updateComms() {
  XBeeResponse response;
  
  xbee.readPacket();
  response = xbee.getResponse();
  
  if (!response.isAvailable())
    return;
  
  if (response.getApiId() == RX_16_IO_RESPONSE) {
    response.getRx16IoSampleResponse(io16);
    remoteDrive(io16.getAnalog(0,0),
                    io16.getAnalog(1,0),
                    io16.getAnalog(2,0),
                    io16.getAnalog(3,0));
  } else if (response.getApiId() == RX_64_IO_RESPONSE) {
    response.getRx64IoSampleResponse(io64);
    remoteDrive(io64.getAnalog(0,0),
                    io64.getAnalog(1,0),
                    io64.getAnalog(2,0),
                    io64.getAnalog(3,0));
  } else {
    //Serial.print("API Packet: ");
   // Serial.println(response.getApiId(), HEX);
  }
}

////
// END Coms
////



//  Angular Challenge Code
// =================================

void accelerometerSetup() {
  ////
  // Accelerometer Setup
  ////
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial);
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("Connection Successful");
  }
  else {
    Serial.println("Connection ERROR");
  }
  // Supply gyro offsets
  mpu.setXGyroOffset(X_OFFSET);
  mpu.setYGyroOffset(Y_OFFSET);
  mpu.setZGyroOffset(Z_OFFSET);
  mpu.setZAccelOffset(Z_ACCEL_OFFSET);
  // Enable DMP
  mpu.setDMPEnabled(true);


// END Accelerometer Setup
}

void angularChallenge() {
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180/M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180/M_PI);
      Serial.print("\t");
      Serial.println(ypr[2] * 180/M_PI);
  }
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
