
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

void setup() {
  // Initialize serial
  Serial.begin(9600);
  
  // Attach Servos
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(LEFT_MOTOR, OUTPUT);
  
  // TODO ensure that the line is between the two sensors
}

void loop() {
  // lineFollow();
  angularChallenge();
  // boulderField();
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
    analogWrite(RIGHT_MOTOR, MPULSE(+0.6));
    analogWrite(LEFT_MOTOR,  MPULSE(+0.4));
  }
  else if (hits & LEFT_HIT)
  // left side triggering
  {
    Serial.print("RIGHT ");
    Serial.println((int)hits);
    analogWrite(RIGHT_MOTOR, MPULSE(-0.4));
    analogWrite(LEFT_MOTOR,  MPULSE(-0.6));
  }
  else
  // neither side triggering
  {
    Serial.print("GO ");
    Serial.println((int)hits);
    analogWrite(RIGHT_MOTOR, MPULSE(+0.8));
    analogWrite(LEFT_MOTOR,  MPULSE(-0.8));
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
