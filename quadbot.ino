#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Depending on your servo make, the pulse width min and max may vary, you
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// our servo # counter
uint8_t servonum = 0;

//define servos' ports
//        Coxa  Femur   Tibia
//  Leg0: 0     1       2
//  Leg1: 3     4       5
//  Leg2: 6     7       8
//  Leg3: 9     10      11
// (Shoulder, elbow, wrist)
// Neutral position should be out to the side and Tibia straight down.

const int servo_pin[4][3] = { {0, 1, 2}, {4, 5, 6}, {8, 9, 10}, {12, 13, 14} };
//                    leg #          1                   2                   3                   4
//                 position    f     c     b       f     c     b       f     c     b       f     c     b
const int coxa_pos[4][3]  = { {1000, 1500, 2000}, {2000, 1500, 1000}, {1000, 1500, 2000}, {2000, 1500, 1000} };
const int femur_pos[4][3] = { {2000, 1500,  900}, {4, 1500, 6}, {8, 1500, 10}, {12, 1500, 14} };
const int tibia_pos[4][3] = { {2000, 1500,  900}, {4, 1500, 6}, {8, 1500, 10}, {12, 1500, 14} };
/*
 * leg up = f, f, f 
 */

void setup() {
  // initialize serial communication:
  Serial.begin(9600);
  
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

}

void setAngle(uint8_t servo, uint8_t angle){
  // Angle to micro-sec
  pwm.writeMicroseconds(servo, map(angle, 0, 180, USMIN, USMAX));
}

void loop() {
  int usec = 1600;
  //int usec = 600;

  // read the sensor:
  if (Serial.available() > 0) {
    int inByte = Serial.read();
    // do something different depending on the character received.
    // The switch statement expects single number values for each case; in this
    // example, though, you're using single quotes to tell the controller to get
    // the ASCII value for the character. For example 'a' = 97, 'b' = 98,
    // and so forth:

    switch (inByte) {
      case '1':
        coxaBack(0);
        break;
      case 'a':
        coxaFwd(0);
        break;
      case 'q':
        legUp(1, 2);
        break;
      case 'z':
        legDown(1, 2);
        break;
      case '2':
        coxaBack(4);
        break;
      case 's':
        coxaFwd(4);
        break;
      case 'w':
        legUp(5, 6);
        break;
      case 'x':
        legDown(5, 6);
        break;
      case 'm':
        setAllCenter();
        break;
      default:
        break;
    }
 }

 delay(50);
}

void setAllCenter() {
  for (uint16_t id = 0; id <= 14; id++){
    pwm.writeMicroseconds(id, 1500);
delay(50);
  }
}

void coxaCenter(int id) {
  // leg shoulder center left/right
  pwm.writeMicroseconds(id, 1500); // shoulder
}

void coxaBack(int id) {
  // leg back
  pwm.writeMicroseconds(id, 2000); // shoulder
}

void coxaFwd(int id) {
  // leg fwd
  pwm.writeMicroseconds(id, 1000); // shoulder
}

void legDown(int femur, int tibia) {
  // leg down
  pwm.writeMicroseconds(femur, 900); // tip
  pwm.writeMicroseconds(tibia, 2000); // femur
}

void legUp(uint8_t femur, uint8_t tibia) {
  // leg up
  pwm.writeMicroseconds(femur, 2000); // tibia
  pwm.writeMicroseconds(tibia, 2000); // femur
}

void legNeutral(uint8_t femur, uint8_t tibia) {
  // leg neutral
  pwm.writeMicroseconds(femur, 1700);
  pwm.writeMicroseconds(tibia, 1600);
}
