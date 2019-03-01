#include <Sabertooth.h>
#include <SoftwareSerial.h>
#include <ros.h>
#include <manual/MovementCommand.h>
#include <manual/MovementFeedback.h>
/**
 * This is the code to run on the Arduinos.
 * Assume that all the motors in arrays appear in the following order:
 * Left Front, Left Middle, Left Back,
 * Right Front, Right Middle, Right Back.
 * 
 * This code is designed to run on two separate Arduinos,
 * the right side and left side controlled independentally.
 * 
 * Sabertooth documentation and libraries can be found here: https://www.dimensionengineering.com/software/SabertoothArduinoLibrary/html/index.html
 * We are using packetized serial which means we specify the address of a Sabertooth address and then we can run the motors all from one pin.
 * 
 * Stepper Motors run using frequency modulation. Higher frequency = faster steps = faster turning.
 */

//Use for testing and calibrating. For normal operation, make these false.
//Open Serial Monitor to see details.
const bool TEST_MOTORS = false;
const bool CALIBRATE_ENCODER = true;
const bool TEST_STEPPERS = false;
const int ENCODER_TO_CALIBRATE = 1;//0-2 from front to back.
const int STEPPER_TO_TEST = 0;//0 is allowed for both boxes. 1 and 2 are only allowed for left box (ARDUINO_NUM 0)

const int ARDUINO_NUM = 0;//0 is left arduino, 1 is right.

//In order to output human readable, useful data goes on the Serial Monitor
//Control for the Sabertooths goes on pin 14.
SoftwareSerial SWSerial(NOT_A_PIN, 14);

/**
 * split into two arrays of length 3 because each Arduino gets one array.
 * The array that the Arduino gets in decided by ARDUINO_NUM. This pattern continues for all constants
 */
Sabertooth ST[2][3] = {{Sabertooth(128, SWSerial), Sabertooth(133, SWSerial), Sabertooth(129, SWSerial)},
                       {Sabertooth(130, SWSerial), Sabertooth(131, SWSerial), Sabertooth(132, SWSerial)}};
//6 motor controllers (LF, LM, LB, RF, RM, RB)

//Array of booleans to tell if a particular wheel is supposed to run or not
boolean enabled[2][3] = {{true, true, true},
                         {true, true, true}};

//the encoders send values between 0 and 1024. This will convert it to degrees
const double ENCODER_SCALE = 360/1024;

//Angles for wheels to be in when in different states
const int PACKED_ANGLES[3] = {0, 0, 0};
const int TURN_ANGLES[3] = {135, 180, 135};
const int DRIVE_ANGLES[3] = {180, 180, 180};
const int SIDE_DRIVE_ANGLES[3] = {90, 90, 90};

//Left wheels from front to back
//turn CCW, CW, CW
//Right side is opposite
const bool TURN_CW[3] = {false, true, true};

//True zero of the encoders.
//Zero is taken to be the packed in state.
const int TRUE_ZERO[2][3] = {{0, 0, 0},
                             {450, 0, 80}};

//The analog pins where the encoders are plugged into
const int ENCODER_PINS[3] = {A0, A1, A0};

//the factor to adjust the drive motors' speeds by to try and keep them turning at the same speed.
const float SPEED_ADJUST[2][6] = {{1, 1, 1,  //Left Drive
                                   1, 1, 1}, //Left Art
                                  {1, 1, 1,  //Right Drive
                                   1, 1, 1}};//Right Art
//If the motors are wired backwards to how we expect,
//change the corresponding float to negative.

//Stepper motor controller pins.
const int STEPPER_PUL[3] = {11, 12, 13};//0 is conveyor, 1 is raise/lower bucket chain, 2 is power bucket chain
const int STEPPER_DIR[3] = {8, 9, 10};
const int STEPPER_ENA[3] = {5, 6, 7};

//Conveyor Belt and Camera Lift Motor Controller
Sabertooth ConveyorMotor = Sabertooth(135, SWSerial);

const int DRIVE = 1;//drive motors are on M1 on Sabertooth
const int ARTICULATION = 2;//articulation motors are on M2
const int DRIVE_SPEED = 50;//50/127 default speed for drive motors
const int TURN_SPEED = 50;//50/127 default speed for articulation motors
const int DIG_SPEED = 25;//25 microsecond pulse frequency (it runs through a 20:1 gear ratio
const int RAISE_SPEED = 500;//500 microsecond pulse frequency
const int MOVE_CONVEYOR_SPEED = 500;
const int CONVEYOR_SPEED = 50;//50/127
const int CAMERA_SPEED = 25;//25/127

/*ROS setup*/

ros::NodeHandle nh;
manual::MovementFeedback feedbackMessage;
ros::Publisher movementFeedback ("MovementFeedback", &feedbackMessage);

void messageCb( const manual::MovementCommand& msg){
  feedbackMessage.status = msg.driveDirection;
  feedbackMessage.message = "recieved a command";
  movementFeedback.publish(&feedbackMessage);
  //handle command
  nh.spinOnce();
}

ros::Subscriber<manual::MovementCommand> sub("MovementCommand", messageCb );

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(movementFeedback);
  SWSerial.begin(9600);
  Serial.begin(9600);//Used for human-readable feedback. Open Serial Monitor to view.
  Serial.println("I am a robot... Bleep Bloop.");
  pinMode(STEPPER_PUL[0], OUTPUT);//Initialize all the Stepper motor pins
  pinMode(STEPPER_DIR[0], OUTPUT);//right box only has 1 controller
  pinMode(STEPPER_ENA[0], OUTPUT);
  digitalWrite(STEPPER_ENA[0], HIGH);//Active-low enable, so we set it to high to prevent it from drawing current at rest
  if (ARDUINO_NUM == 0) { //left box has 3 controllers
    for (int i = 1; i < 3; i++) {
      pinMode(STEPPER_PUL[i], OUTPUT);
      pinMode(STEPPER_DIR[i], OUTPUT);
      pinMode(STEPPER_ENA[i], OUTPUT);
      digitalWrite(STEPPER_ENA[i], HIGH);
    }
  }
  for (int i = 0; i < 3; i++) {//make sure all the drive motors are off
    runWheelMotor(i, DRIVE, 0);
    runWheelMotor(i, ARTICULATION, 0);
  }
  delay(1000);//just a little break before we go to work
  if (TEST_MOTORS)//all the test functions. Booleans used to run them are found at the top of the file.
    testDrive(DRIVE_SPEED);
  if (CALIBRATE_ENCODER)
    calibrateEncoder(ENCODER_TO_CALIBRATE);
  if (TEST_STEPPERS)
    testStepper(STEPPER_TO_TEST);
}

void loop() {
  nh.spinOnce();
  delay(1000); 
  //need to have publisher in loop for consistent functionality
  movementFeedback.publish(&feedbackMessage);
}

/**
 * prints the values of all the encoders
 */
void printEncoders() {
  while (true) {
    int encoderVal = analogRead(ENCODER_PINS[0]);
    String str = "    ";
    Serial.println(str + encoderVal + str);
  }
}

/**
 * prints the value of the specified encoder while turning the corresponding wheel.
 */
void calibrateEncoder(int controller) {
  int encoderVal = analogRead(ENCODER_PINS[controller]);
  while (encoderVal != 0) {
    encoderVal = analogRead(ENCODER_PINS[controller]);
    runWheelMotor(controller, ARTICULATION, -30);
    String str = "Current Angle: ";
    Serial.println(str + encoderVal);
  }
}

/**
 * Tests all the drive motors with a delay between each one.
 */
void testDrive(int vel){
  String str = "Driving motor ";
  for(int i = 0; i < 3; i++){
    runWheelMotor(i, DRIVE, vel);
    Serial.println(str+i);
    delay(500);
    runWheelMotor(i, DRIVE, -vel);
    delay(500);
    runWheelMotor(i, DRIVE, 0);
  }
}

/**
 * Tests the specified stepper by running it forward for 1200 steps (usually 3 revs) and then backward for 5000 steps
 */
void testStepper(int controller) {
  int steps = 0;
  for (int i = 0; i < 1200; i++) {
    runStepperMotor(controller, true);
    steps++;
  }
  for (int i = 0; i < 1200; i++) {
    runStepperMotor(controller, false);
  }
}

/**
 * function to turn the robot in place
 * @param right whether we want to turn right or left
 */
void turnInPlace(bool right){
  alignWheels(TURN_ANGLES);
  int driveSpeed = DRIVE_SPEED;
  if(right && ARDUINO_NUM == 1 || !right && ARDUINO_NUM == 0)
    driveSpeed = -driveSpeed;
  for(int i = 0; i < 3; i++){
    runWheelMotor(i, DRIVE, driveSpeed);
  }
}

/**
   function to turn the robot while moving
*/
void turnDrive(bool right, bool forward) {

}

/**
 * function to drive the robot forward
 * @param forward whether we want to go forward or backward
 */
void driveStraight(bool forward){
  alignWheels(DRIVE_ANGLES);
  int driveSpeed = DRIVE_SPEED;
  if(!forward)
    driveSpeed = -driveSpeed;
  for(int i = 0; i < 3; i++){
    runWheelMotor(i, DRIVE, driveSpeed);
  }
}

/**
 * Align the wheels to the specified angles
 * @param angles the angles to align the wheels to
 */
void alignWheels(const int angles[3]){
  bool complete = false;
  bool aligned[3] = {false, false, false};
  for(int i = 0; i < 3; i++)//if a wheel is disabled, we do not want to try to align it
    if(!enabled[ARDUINO_NUM][i])
      aligned[i] = true;
  while(!complete){
    for(int i = 0; i < 3; i++){
      if(aligned[i])
        continue;
      if(abs((TRUE_ZERO[ARDUINO_NUM][i] + analogRead(ENCODER_PINS[i]) * ENCODER_SCALE) - angles[i]) < 3)//CHECK THIS
        aligned[i] = true;
      else{
      //CHECK THIS TOO (I think this actually works)
        bool cw = true;
        if((TRUE_ZERO[ARDUINO_NUM][i] + analogRead(ENCODER_PINS[i]) * ENCODER_SCALE) > angles[i])
          cw = !cw;
        if(!TURN_CW[i])
          cw = !cw;
        if(ARDUINO_NUM == 1)
          cw = !cw;
        if(cw){
          runWheelMotor(i, ARTICULATION, TURN_SPEED);
          runWheelMotor(i, DRIVE, DRIVE_SPEED);//check the drive directions
        }
        else{
          runWheelMotor(i, ARTICULATION, -TURN_SPEED);
          runWheelMotor(i, DRIVE, -DRIVE_SPEED);
        }
      }
    }
    if(aligned[0] && aligned[1] && aligned[2])
      complete = true;
  }
}

/**
 * runs a motor
 * @param controller The number of the controller to run (0-2)
 * @param motorNum DRIVE or ARTICULATION
 * @param vel Speed of the motor (-127 - 127)
 */
void runWheelMotor(int controller, int motorNum, int vel){
  if(!enabled[ARDUINO_NUM][controller])
    return;
  if(motorNum == ARTICULATION)
    ST[ARDUINO_NUM][controller].motor(ARTICULATION, SPEED_ADJUST[ARDUINO_NUM][controller+3] * vel);
  else
    ST[ARDUINO_NUM][controller].motor(DRIVE, SPEED_ADJUST[ARDUINO_NUM][controller] * vel);
}

/**
   method to run a stepper
   range is 0 to 200kHZ pulse frequency.
*/
void runStepperMotor(int stepper, bool dir) {
  int d;
  switch (stepper) {//sets the pulse frequency (speed) based on which stepper we are running
    case 0:
      d = MOVE_CONVEYOR_SPEED;
      break;
    case 1:
      d = RAISE_SPEED;
      break;
    case 2:
      d = DIG_SPEED;
      break;
  }
  digitalWrite(STEPPER_ENA[stepper], LOW);//enable the stepper (active-low)
  if (!dir) //control it to turn CW or CCW
    digitalWrite(STEPPER_DIR[stepper], HIGH);
  else
    digitalWrite(STEPPER_DIR[stepper], LOW);
  digitalWrite(STEPPER_PUL[stepper], HIGH);//pulse with a frequency of 1/RAISE_SPEED
  delayMicroseconds(d);
  digitalWrite(STEPPER_PUL[stepper], LOW);
  delayMicroseconds(d);
  digitalWrite(STEPPER_ENA[stepper], HIGH);//disable the stepper
}

/**
 * method to run the conveyor motor and the camera mount
 * runs on a sabertooth, just like the wheels, so -127 - 127 range
 */
void runConveyorMotor(int motorNum, bool dir) {
  if (motorNum == 0) {//conveyor
    if (dir)
      ConveyorMotor.motor(motorNum, CONVEYOR_SPEED);
    else
      ConveyorMotor.motor(motorNum, -CONVEYOR_SPEED);
  }
  else {//camera
    if (dir)
      ConveyorMotor.motor(motorNum, CAMERA_SPEED);
    else
      ConveyorMotor.motor(motorNum, -CAMERA_SPEED);
  }
}
