#include <IBusBM.h>
#include <SoftwareSerial.h>
#include <RoboClaw.h>
#include <math.h>

#define ADDRESS 0x80 // pretty common serial address should work
#define VOLTAGE 25.9 // Battery voltage
#define MAXINPUT 128 // max roboclaw input for motor speed
#define TOLERANCE 5 // wiggle room in case joystick isn't purely zero
#define LIPO4S 14.8 // voltage of one 4s lipo battery
#define MOTORVOLT 12.0 // max operating voltage for redline motors
const long begin_speed = 38400; // For intializing serial communication

// Defin PWM input pins for reading remote control signals
const int PWM_PIN_LEFT = 3; // Left motor control
const int PWM_PIN_RIGHT = 2; // PWM signal pin for right motor control

// Create a RoboClaw object
SoftwareSerial serial(10, 11);
RoboClaw roboclaw(&serial, 10000); // Use Serial Communication

// Create iBus Object
IBusBM ibus;
 
// Read the number of a given channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  uint16_t ch = ibus.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}
 
// Read the channel and return a boolean value
bool readSwitch(byte channelInput, bool defaultValue) {
  int intDefaultValue = (defaultValue) ? 100 : 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}
// Convert a channel value to a usable speed value via linear transformation
double speedValue(int channel) {
  // Use a linear transformation from [-100, 0, 100]->[0, 64, 127]
  double speed = ((channel + 100) / 200.0) * MAXINPUT; // 127 is our maximum roboclaw input, need to cast to floating point to not get zero
  return speed;
}

int sign(int value) { // easy way to check signs of ints
  if (value < 0) {
    return -1;
  }
  else {
    return 1;
  }
}

double bounce(double angle) {
  // return an angle between 0 and 90 or 0 and pi
  angle = fmod(angle, M_PI);  // limit by 180
  if (abs(angle) > (M_PI / 2)) { // we have overshot and need to bounce back into our range
    angle = M_PI - abs(angle);
  }
  else {
    angle = abs(angle);
  }
  return abs(angle); // make sure we return a positive angle
}

double scaler(double angle) {
  // get a value from an angle of the joystick vector to scale the speed of the non primary motor
  double scale_factor = (angle - (M_PI / 4)) / (M_PI / 4); // convert to a value between [-1, 1] via linear transform
  return scale_factor;
}

void setup() {
  Serial.begin(begin_speed);
  // serial.begin(begin_speed);
  roboclaw.begin(begin_speed); // More initialization
  ibus.begin(Serial1); // for receiver
  // Set PWM input pins
  // pinMode(PWM_PIN_LEFT, INPUT);
  // pinMode(PWM_PIN_RIGHT, INPUT); // Set up the pins for input
}


void loop() {
  // Define duty cycle parameters
  //
  bool continue_check = 1; // basically lets us choose when to continue to the next loop
  double duty_cycle = MOTORVOLT / (4 * LIPO4S); // for scaling our inputs into the motors
  int killSpeed = 64; // for setting our motors to zero velocity
  int rightInput = 0;
  int leftInput = 0; // these inputs values are from [-100, 100] and need to get scaled and cycled
  double joystick_angle = 0.0;
  double overdrive = 1.0; // an overdrive value for temporary boost in speed and power
  // int max_speed = 100;
  // int cycle = (int)(max_speed * duty_cycle); // take the floor by casting to int, always > 0

  // these readChannel lines should constrain us between -100 and 100
  int leftRight = readChannel(3, -84, 84, 0); // one channel/stick controls turning, one controls forward and back. this one is turning
  int frontBack = readChannel(1, -84, 84, 0); // these are zero indexed compared to the flysky, so RC Ch1 is computer Ch0 etc
  int killSwitch = readChannel(4, -100, 100, 0); // for stopping instantly
  int drivetrain = readChannel(6, -100, 100, 0); // three gearings, slow, medium, fast

  // set up killswitch with a small delay that skips over all driving
  if (killSwitch > 0) {
    roboclaw.ForwardBackwardM1(ADDRESS, killSpeed);
    roboclaw.ForwardBackwardM2(ADDRESS, killSpeed);
    Serial.println(killSwitch);
    Serial.print("KILLED");
    delay(1000); // stops for a second
    continue_check = 0;
  }
  // set up gearing, default being low gear
  if (drivetrain < 0) {
    overdrive = 1.0; // keep defalut
    Serial.println(drivetrain);
  }
  if (drivetrain == 0) {
    overdrive = 2.0; // average driving speed, may need to find this experimentally
    Serial.println(drivetrain);
  }
  if (drivetrain > 0) {
    overdrive = 4.0; // max operating voltage
    Serial.println(drivetrain);
  }

  /* We have three conditions, pure straight,
  * a pure turn, and diagonal drive
  */
  // PURE STRAIGHT
  if (abs(leftRight) < TOLERANCE && continue_check) { // Give ourselves a small tolerance for driving straight
    rightInput = frontBack;
    leftInput = frontBack;
    continue_check = 0; 
  }
  // PURE TURN
  if (abs(frontBack) < TOLERANCE && continue_check) { 
    if (leftRight > TOLERANCE) { // turn right
      rightInput = -leftRight;
      leftInput = leftRight;
      continue_check = 0;
    }
    if (leftRight < -TOLERANCE && continue_check) { // turn left
      rightInput = -leftRight; //leftRight is now negative in this if statement
      leftInput = leftRight;
      continue_check = 0;
    }
  }
  // DIAGONAL
  if (leftRight < -TOLERANCE && continue_check) { // turn left
    rightInput = frontBack; // set our non-turning motor to be our max forward/backward value
    joystick_angle = atan2(frontBack, leftRight); // get an angle from the two channels, like a vector
    joystick_angle = bounce(joystick_angle); // put this angle between 0 and pi
    leftInput = scaler(joystick_angle) * rightInput; // our turning motor should be a fraction of the driving motor
    continue_check = 0;
  }
  if (leftRight > TOLERANCE && continue_check) { // turn right
    leftInput = frontBack;
    joystick_angle = atan2(frontBack, leftRight); // get an angle from the two channels, like a vector. I realize now I could've just put in absolute values here and not bounce the angle but eh
    joystick_angle = bounce(joystick_angle); // put this angle between 0 and pi
    rightInput = scaler(joystick_angle) * leftInput; // our turning motor should be a fraction of the primary motor
    continue_check = 0;
  }

  double rightMotor = speedValue(rightInput * duty_cycle * overdrive); // must scale the channel using duty cycle otherwise we can end up changing directions
  double leftMotor = speedValue(leftInput * duty_cycle * overdrive); // Convert our values from channel outputs to roboclaw inputs


  roboclaw.ForwardBackwardM1(ADDRESS, rightMotor);
  roboclaw.ForwardBackwardM2(ADDRESS, leftMotor); // Assume right motor is M1

  continue_check = 1; // reset our continue variable
  


  // Print values for debugging
  // Serial.print("leftRight: ");
  // Serial.println(leftRight);
  // Serial.print("frontBack: ");
  // Serial.println(frontBack);

  // Serial.print("rightMotor: ");
  // Serial.println(rightMotor);
  // Serial.print("leftMotor: ");
  // Serial.println(leftMotor);

  delay(100); // Wait 0.1 second
}
