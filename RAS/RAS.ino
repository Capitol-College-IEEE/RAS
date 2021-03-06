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

#include <LCD.h>
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

#define INT_in 2
#define X_OFFSET -1720
//#define Y_OFFSET 10
//#define Z_OFFSET 10
//#define Z_ACCEL_OFFSET 10
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

#define CTRL_MODE_LINE_FOLLOW       0
#define CTRL_MODE_ANGULAR_CHALLENGE 1
#define CTRL_MODE_BOULDER_FIELD     2
#define CTRL_MODE_MANUAL            3

uint8_t control_mode = CTRL_MODE_MANUAL;

// MPU control/status vars
MPU6050 mpu;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
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
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
LCD screen(7,6,5,4,3);

bool reseting = false;

void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {

  // Attach Servos
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(LEFT_MOTOR, OUTPUT);
  
  screen.initialize();
  screen.sendString(0,0,"CAPITOL IEEE");
  //zwhile(true);
  initComms();
  //Setup for jumper for control
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  
  
  pinMode(13, OUTPUT);

  // 
  accelerometerSetup();

}

void loop() {
  do {
    control_mode = digitalRead(12) << 1 | digitalRead(11);
    Serial.print("Control mode: ");
    Serial.println(control_mode);
    mainLoop();
  } while (!mpuInterrupt && fifoCount < packetSize && control_mode != CTRL_MODE_ANGULAR_CHALLENGE);
  
  Serial.println("Data");
  
  //Serial.println("ANGLUAR");
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      
      reseting = true;
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      reseting = false;
      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }
}

void mainLoop() {
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
  
  Serial.println("Loop");
  
  //screen.sendString(0,0, "IEEE");
  
  unsigned long next_time = millis();
  if (next_time - time > 100) {
    digitalWrite(13, led_pattern[led_index++]);
    if (led_index >= sizeof(led_pattern))
      led_index = 0;
    time = next_time;
  }
  
  switch (control_mode) {
    case CTRL_MODE_LINE_FOLLOW:
      Serial.println("Line Follow");
      screen.clearScreen();
      screen.sendString(0,0,"Line Follow");
      lineFollow();
      break;
    
    case CTRL_MODE_ANGULAR_CHALLENGE:
      Serial.println("Anglular");
      angularChallenge();
      break;
    
    case CTRL_MODE_BOULDER_FIELD:
      Serial.println("Boulder");
      screen.clearScreen();
      screen.sendString(0,0,"Boulder");
      boulderField();
      break;
      
    default:
      Serial.println("Remote");
      screen.clearScreen();
      screen.sendString(0,0,"Remote");
      remoteControl();
      break;
  }
}


//  Line Follow Code
// =================================

void lineFollow() {
  // Read in the line information
  // Move the motors in the proper direction
  if(digitalRead(8) == HIGH){
    analogWrite(RIGHT_MOTOR, MPULSE(0.0));
    analogWrite(LEFT_MOTOR,  MPULSE(0.0));
    return;
  }
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
    analogWrite(RIGHT_MOTOR, MPULSE(+0.3));
    analogWrite(LEFT_MOTOR,  MPULSE(+0.2));
  }
  else if (hits & LEFT_HIT)
  // left side triggering
  {
    analogWrite(RIGHT_MOTOR, MPULSE(-0.2));
    analogWrite(LEFT_MOTOR,  MPULSE(-0.3));
  }
  else
  // neither side triggering
  {
    analogWrite(RIGHT_MOTOR, MPULSE(+0.3));
    analogWrite(LEFT_MOTOR,  MPULSE(-0.3));
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
  if(digitalRead(8) == HIGH){
    analogWrite(RIGHT_MOTOR, MPULSE(0.0));
    analogWrite(LEFT_MOTOR,  MPULSE(0.0));
    return;
  }
  //0 to 1024
  yL = 512 - yL;
  yR = 512 - yR;
  yL = MPULSE(((float)yL) / 512. / 3. * 2.);
  yR = MPULSE(((float)yR) / 512. / 3. * 2.);
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
  //Serial.begin(115200);
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("Connection Successful");
  }
  else {
    Serial.println("Connection ERROR");
  }
  
  // END Accelerometer Setup
}

void angularChallenge() {
  static double total, offset;
  static int8_t state = -3;
  static uint16_t count = 0;
  static long start_time = 0;
  
  int16_t ax, ay, az, gx, gy, gz;
  char data[20];
  double angle, avg;
  long this_time;
  
  if(reseting)
    return;
  
  this_time = millis();
  
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  angle = atan2(ay, az) * 180/M_PI - offset;

  
  if(digitalRead(8) == HIGH || state < 0) {
     analogWrite(RIGHT_MOTOR, MPULSE(0.0));
     analogWrite(LEFT_MOTOR,  MPULSE(0.0));
  }
  else {
     analogWrite(RIGHT_MOTOR, MPULSE(+0.3));
     analogWrite(LEFT_MOTOR,  MPULSE(-0.3));
  }
  
  if(state == 4){
    avg = total / count;
    
    sprintf(data, "AVG : %d.%d", (int) avg, abs((int) ((avg - (int) avg) * 100)));
    Serial.println(data);
    screen.clearScreen();
    screen.sendString(0,0, data );
  }
  else{
    if (state == -3) {
      total = 0;
      count = 0;
      offset = 0;
      start_time = this_time;
      state++;
    }
    
    if (state == -2 && this_time - start_time > 250)
      state++;
    
    if (state == -1) {
      total += angle;
      count++;
      //offset += angle * 0.9;
      
      if (this_time - start_time < 250) {
        state++;
        offset = total / count;
        total = 0;
        count = 0;
        start_time = 0;
      }
    }
    
    if((state == 0 && abs(angle) > 10) || (state == 1 && abs(angle) < 10) || (state == 2 && abs(angle) > 10) || (state == 3 && abs(angle) < 10)){
      state++;
    }
    
    if (state == 1 || state == 3) {
      total += abs(angle);
      count++;
      angle = total / count;
    }
    
    if (this_time - start_time > 500) {
      start_time = this_time;
      sprintf(data, "%d.%d\x7f %d", (int) angle, abs((int) ((angle - (int) angle) * 100)), state);
      //Serial.println(data);
      screen.clearScreen();
      screen.sendString(0,0, data );
    }
    
    
    Serial.print("Mode: ");
    Serial.print(state);
    Serial.print("\tOffset: ");
    Serial.print(offset);
    Serial.print("\tAngle: ");
    Serial.print(angle);
  }
//}
}


////
// END Angular Challenge Code
////

//  Angular Challenge Code
// =================================

void boulderField() {
  if(digitalRead(8) == HIGH){
    analogWrite(RIGHT_MOTOR, MPULSE(0.0));
    analogWrite(LEFT_MOTOR,  MPULSE(0.0));
    return;
  }
  analogWrite(RIGHT_MOTOR, MPULSE(+0.6));
  analogWrite(LEFT_MOTOR,  MPULSE(-0.6));
}


////
// END Angular Challenge Code
////
