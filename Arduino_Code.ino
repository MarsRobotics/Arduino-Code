#include <Sabertooth.h>
#include <SoftwareSerial.h>
#include <ros.h>
#include <manual/SimpleCommand.h>
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
const bool DEBUG = false;
const bool TEST_MOTORS = false;
const bool CALIBRATE_ENCODER = false;
const bool TEST_STEPPERS = false;
const bool PRINT_ENCODERS = false;
const bool TEST_RAISE_LOWER_BUCKET_CHAIN = false;
const bool TEST_BUCKET_CHAIN = false;//ramp up from 500 delay to 250 delay (change DIG_SPEED to change the speed)
const bool TEST_BUCKET_CHAIN_SLOW = false;//safe route which stays at 500 delay
const bool TEST_CONVEYOR_RAISE = false;
const bool TEST_CONVEYOR_TURN = false;
const bool TEST_SIX_DRIVE = false;//only works if we have one serial line running to all six motor controllers.
const bool TEST_LOWER_DIG = false;
const bool TEST_RAISE_DUMP = false;
const int ENCODER_TO_CALIBRATE = 0;//0-2 from front to back.
const bool CALIBRATE_DIRECTION = false;//true for positive, false for negative
const int STEPPER_TO_TEST = 0;//0 is allowed for both boxes. 1 and 2 are only allowed for left box (ARDUINO_NUM 0)

const int ARDUINO_NUM = 0;//0 is left arduino, 1 is right.

//In order to output human readable, useful data goes on the Serial Monitor
//Control for the Sabertooths goes on pin 14.
SoftwareSerial SWSerial(NOT_A_PIN, 14);

/**
 * split into two arraUys of length 3 because each Arduino gets one array.
 * The array that the Arduino gets in decided by ARDUINO_NUM. This pattern continues for all constants
 */
Sabertooth ST[2][3] = {{Sabertooth(128, SWSerial), Sabertooth(133, SWSerial), Sabertooth(129, SWSerial)},
                       {Sabertooth(130, SWSerial), Sabertooth(131, SWSerial), Sabertooth(132, SWSerial)}};
//6 motor controllers (LF, LM, LB, RF, RM, RB)

//Array of booleans to tell if a particular wheel is supposed to run or not
boolean enabled[2][3] = {{true, true, true},
                         {true, true, true}};

//Angles for wheels to be in when in different states
const int PACKED_ANGLES[3] = {0, 0, 0};
const int TURN_ANGLES[3] = {384, 512, 640};
const int DRIVE_ANGLES[3] = {512, 512, 512};
const int SIDE_DRIVE_ANGLES[3] = {256, 256, 768};

//Left wheels from front to back
//turn CCW, CW, CW
//Right side is opposite
const bool TURN_CW[3] = {false, false, true};

//True zero of the encoders.
//Zero is taken to be the packed in state.
const int TRUE_ZERO[2][3] = {{0, 360, 640},
                             {770, 420, 90}};

//The analog pins where the encoders are plugged into
const int ENCODER_PINS[3] = {A0, A1, A2};

//the factor to adjust the drive motors' speeds by to try and keep them turning at the same speed. Order is front to back.
const float SPEED_ADJUST[2][6] = {{1, -0.8, 1,  //Left Drive
                                   -2, 0.8, -1}, //Left Art
                                  {-1, -1, -1,  //Right Drive
                                   1, -1, -1}};//Right Art
//If the motors are wired backwards to how we expect,
//change the corresponding float to negative.

//Stepper motor controller pins.
const int STEPPER_PUL[3] = {11, 12, 13};//0 is raise/lower bucket chain, 1 is conveyor, 2 is power bucket chain
const int STEPPER_DIR[3] = {8, 9, 10};
const int STEPPER_ENA[3] = {5, 6, 7};

//Conveyor Belt and Camera Lift Motor Controller
Sabertooth ConveyorMotor = Sabertooth(135, SWSerial);

//wheel motors are dual channel. The first motor is drive while the second is articulation
const int DRIVE = 1;//drive motors are on M1 on Sabertooth
const int ARTICULATION = 2;//articulation motors are on M2

//all the pre-set speeds which we run the motors at
const int DRIVE_SPEED = 50;//50/127 default speed for drive motors
const int TURN_SPEED = 50;//50/127 default speed for articulation motors
const int DIG_SPEED = 200;//25 microsecond pulse frequency (it runs through a 20:1 gear ratio
const int RAISE_SPEED = 500;//100 microsecond pulse frequency
const int MOVE_CONVEYOR_SPEED = 500;
const int CONVEYOR_SPEED = 50;//50/127
const int CAMERA_SPEED = 25;//25/127

//commands
const int PACK_IN = 1;
const int DRIVE_STRAIGHT = 2;//or pack_out
const int TURN = 3;
const int DRIVE_SIDE = 4;

bool motorDisable = false;

/*ROS setup*/

ros::NodeHandle nh;
manual::MovementFeedback feedbackMessage;
ros::Publisher movementFeedback ("MovementFeedback", &feedbackMessage);

void messageCb( const manual::SimpleCommand& msg){
  feedbackMessage.status = msg.data;
  feedbackMessage.message = "recieved a command";
  feedbackMessage.status = msg.data;
  movementFeedback.publish(&feedbackMessage);
  motorDisable = false;
  switch(msg.data){
    case 1:
      driveStraight(true); 
      feedbackMessage.message = "drive forward";
      break;
    case 2:
      driveStraight(false);
      feedbackMessage.message = "drive backward";
      break;
    case 3:
      //drive forward & turn articulation motors left
      turnDrive(false, true);
      //feedbackMessage.message = "drive & turn left";
      feedbackMessage.message = "not implemented";
      break;
    case 4:
      //drive forward & turn articulation motors right
      turnDrive(true, true);
      //feedbackMessage.message = "drive & turn right";
      feedbackMessage.message = "not implemented";
      break;
    case 5:
      //just turn articulation motors left
      turnInPlace(false);
      feedbackMessage.message = "articulate left";
      break;
    case 6:
      //just turn articulation motors right
      turnInPlace(true);
      feedbackMessage.message = "articulate right";
      break;
    case 7:
      //STOP ALL MOTORS
      feedbackMessage.message = "stop";
      fullStop();
      break;
    case 8:
      //pack in 
      packIn();
      feedbackMessage.message = "pack in";
      break;
    case 9:
      //pack out
      packOut();
      feedbackMessage.message = "pack out";
      break;
    case 10: 
      //turn bucket chain slow-- this is the 'safer' method
      testBucketChainSlow();
      feedbackMessage.message = "turn bucket chain";
      break;
    case 16: 
      //turn bucket chain faster with ramp up
      testBucketChain();
      feedbackMessage.message = "turn bucket chain fast";
      break;
    case 11: 
      //raise bucket chain
      testRaiseBucketChain();
      feedbackMessage.message = "raise bucket chain";
      break;
    case 12: 
      //lower bucket chain
      testLowerBucketChain(); 
      feedbackMessage.message = "lower bucket chain";
      break;
    case 13: 
      //raise conveyor
      raiseConveyor();
      feedbackMessage.message = "raise conveyor ";
      break;
    case 14: 
      //lower conveyor
      lowerConveyor();  
      //feedbackMessage.message = "lower conveyor ";
      break;
    case 15: 
      //turn conveyor 
      runConveyor();
      feedbackMessage.message = "turn conveyor ";
      break;
    case 17:
      //raise scissor lift
      //feedbackMessage.message = "raise scissor lift ";
      feedbackMessage.message = "not implemented";
      break;
    case 18:
      //lower scissor lift
      //feedbackMessage.message = "lower scissor lift ";
      feedbackMessage.message = "not implemented";
      break;
    case 999:
      //test
      runWheelMotor(0, DRIVE, 50);
      delay(500);
      runWheelMotor(0, DRIVE, 0);
      feedbackMessage.message = "test";
  }
  movementFeedback.publish(&feedbackMessage);
  nh.spinOnce();
}

ros::Subscriber<manual::SimpleCommand> sub("MovementCommand", messageCb );

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(movementFeedback);
  SWSerial.begin(9600);
  Serial.begin(9600);//Used for human-readable feedback. Open Serial Monitor to view.
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
    calibrateEncoder(ENCODER_TO_CALIBRATE, CALIBRATE_DIRECTION);
  if (TEST_STEPPERS)
    testStepper(STEPPER_TO_TEST);
  if (TEST_RAISE_LOWER_BUCKET_CHAIN)
      testBucketRaise();
  if (TEST_BUCKET_CHAIN)
    testBucketChain();
  if (TEST_BUCKET_CHAIN_SLOW)
    testBucketChainSlow();
  if(PRINT_ENCODERS)
    while(true)
      printEncoders();
  if (TEST_CONVEYOR_RAISE)
    testConveyorRaise();
  if (TEST_CONVEYOR_TURN)
    runConveyor();
  if (TEST_SIX_DRIVE)
    testSixDrive();
  if(TEST_LOWER_DIG)
    digLower();
  if(TEST_RAISE_DUMP)
    dumpRaise();
}

void loop() {
  nh.spinOnce();
}

/**
 * prints the values of all the encoders
 */
void printEncoders() {
  int encoderVal = 0;
  for(int i = 0; i < 3; i++){
    encoderVal = readEncoder(i);
    String str = "    ";
    Serial.print(str + encoderVal + str);
  }
  Serial.println();
}

/**
 * prints the value of the specified encoder while turning the corresponding wheel.
 */
void calibrateEncoder(int controller, bool dir) {
  int encoderVal = 0;
  while (true) {
    if(dir)
      runWheelMotor(controller, ARTICULATION, 30);
    else
      runWheelMotor(controller, ARTICULATION, -30);
    for(int i = 0; i < 3; i++){
      encoderVal = readEncoder(i);
      String str = "    ";
      Serial.print(str + encoderVal + str);
    }
    Serial.println();
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
 * Tests the specified stepper by running it forward for 3200 steps (usually 8 revs) and then backward for 5000 steps
 */
void testStepper(int controller) {
  //runStepperMotor(controller, 1000, true);
  runStepperMotor(controller, 1000, false);
}

void testBucketRaise(){
  //testLowerBucketChain();
  testRaiseBucketChain();
}

void testLowerBucketChain(){
  runStepperMotor(0, 3000, true);
}

void testRaiseBucketChain(){
  runStepperMotor(0, 3000, false);
}

void testBucketChain(){
  runStepperMotor(2, 5000, false);
}

void testBucketChainSlow(){
  runStepperSlow(2, 5000, true);
}

void testConveyorRaise(){
  raiseConveyor();
  lowerConveyor();
}

void raiseConveyor(){
  runStepperMotor(1, 3000, true);
}

void lowerConveyor(){
  runStepperMotor(1, 3000, false);
}

void runConveyor(){
  runConveyorMotor(0, CONVEYOR_SPEED);
}

void digLower(){
  digitalWrite(STEPPER_ENA[0], LOW);
  digitalWrite(STEPPER_ENA[2], LOW);
  digitalWrite(STEPPER_DIR[0], LOW);
  digitalWrite(STEPPER_DIR[2], HIGH);
  for(int i = 0; i < 100000; i++){
    if(i%8 == 0)
      digitalWrite(STEPPER_PUL[0], HIGH);
    digitalWrite(STEPPER_PUL[2], HIGH);
    delayMicroseconds(500);
    if(i%8 == 0)
      digitalWrite(STEPPER_PUL[0], LOW);
    digitalWrite(STEPPER_PUL[2], LOW);
    delayMicroseconds(500);
  }
  digitalWrite(STEPPER_ENA[0], HIGH);
  digitalWrite(STEPPER_ENA[2], HIGH);
}

void dumpRaise(){
  bool dir = false;//CHANGE FROM FALSE TO TRUE IF THE CONVEYOR IS MOVING THE WRONG DIRECTION
  runStepperSlow(1, 15000, dir);
  runConveyor();
  delay(60000);
  runConveyorMotor(0, 0);
  runStepperSlow(1, 15000, !dir);
}

void fullStop(){
  motorDisable = true;
  for(int i = 0; i < 3; i++){
    runWheelMotor(i, DRIVE, 0);//turn off the drive and articulation motors
    runWheelMotor(i, ARTICULATION, 0);
    if(ARDUINO_NUM == 0)
      digitalWrite(STEPPER_ENA[i], HIGH);//disable the steppers
  }
  if(ARDUINO_NUM == 1)
    digitalWrite(STEPPER_ENA[0], HIGH);
  runConveyorMotor(0, 0);//stop the conveyor and scissor lift
  runConveyorMotor(1, 0);
  
}

/**
 * function to turn the robot in place
 * @param right whether we want to turn right or left
 */
void turnInPlace(bool right){
  alignWheels(3);
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
  alignWheels(2);
  int driveSpeed = DRIVE_SPEED;
  if(!forward)
    driveSpeed = -driveSpeed;
  for(int i = 0; i < 3; i++){
    runWheelMotor(i, DRIVE, driveSpeed);
  }
}

void sideDrive(bool right){
  alignWheels(4);
  int driveSpeed = DRIVE_SPEED;
  if(right && ARDUINO_NUM == 1 || !right && ARDUINO_NUM == 0)
    driveSpeed = -driveSpeed;
  for(int i = 0; i < 2; i++){
    runWheelMotor(i, DRIVE, driveSpeed);
  }
  runWheelMotor(2, DRIVE, -driveSpeed);
}

void testSixDrive(){
  for(int i = 0; i < 3; i++){
    ST[0][i].motor(DRIVE, SPEED_ADJUST[0][i] * DRIVE_SPEED);
    ST[1][i].motor(DRIVE, SPEED_ADJUST[1][i] * DRIVE_SPEED);
  }
}

void packIn(){
  alignWheels(1);
}

void packOut(){
  alignWheels(2);
}

/**
 * Align the wheels to the specified angles
 * @param angles the angles to align the wheels to
 */
void alignWheels(int commandNum){
  bool complete = false;
  bool aligned[3] = {false, false, false};
  int angles[3];
  bool normalDir = true;
  switch(commandNum){//copy the proper array to angles;
    case 1:
      normalDir = false;
      for(int i = 0; i < 3; i++)
        angles[i] = PACKED_ANGLES[i];
      break;
    case 2:
      for(int i = 0; i < 3; i++)
        angles[i] = DRIVE_ANGLES[i];
      break;
    case 3:
      if(readEncoder(0) < 800 && readEncoder(0) > 360)//if we are basically packed out, we want to turn the opposite direction we would from being packed in.
        normalDir = false;
      if(ARDUINO_NUM == 0){
        for(int i = 0; i < 3; i++)
          angles[i] = TURN_ANGLES[i];
      }
      else{//angles are mirrored across 0-512 axis.
        for(int i = 2; i >= 0; i--)
          angles[2-i] = TURN_ANGLES[i];
      }
      break;
    case 4:
      if(readEncoder(0) > 256 && readEncoder(0) < 800)//same as before
        normalDir = false;
      for(int i = 0; i < 3; i++)
        angles[i] = SIDE_DRIVE_ANGLES[i];
      break;
  }
  for(int i = 0; i < 3; i++)//if a wheel is disabled, we do not want to try to align it
    if(!enabled[ARDUINO_NUM][i])
      aligned[i] = true;
  int lastDiff[3];
  bool lastDir[3];
  while(!complete && !motorDisable){
    nh.spinOnce();
    for(int i = 0; i < 3; i++){
      if(aligned[i])
        continue;
      int diff = readEncoder(i) - angles[i];
      if(DEBUG){
        Serial.print(readEncoder(i));
        Serial.print("  ");
        Serial.print(angles[i]);
        Serial.print("    ");
      }
      bool cw = true;
      if(abs(diff) < 7){//allow for 14 encoder counts of error (~5 degrees)
        runWheelMotor(i, ARTICULATION, 0);
        runWheelMotor(i, DRIVE, 0);
        aligned[i] = true;
      }  
      else{//we are far away from our target, so just make sure we are turning the correct direction to get there
        if(!normalDir)//packing in will have us go the opposite direction
          cw = !cw;
        if(!TURN_CW[i])//back wheels go the opposite direction
          cw = !cw;
        if(ARDUINO_NUM == 1)//the right side of the robot also goes in the opposite direction
          cw = !cw;
        if(cw){
          runWheelMotor(i, ARTICULATION, TURN_SPEED);
          if(ARDUINO_NUM == 0)
            runWheelMotor(i, DRIVE, DRIVE_SPEED);//check the drive directions
          else
            runWheelMotor(i, DRIVE, -DRIVE_SPEED);
        }
        else{
          runWheelMotor(i, ARTICULATION, -TURN_SPEED);
          if(ARDUINO_NUM == 0)
            runWheelMotor(i, DRIVE, -DRIVE_SPEED);
          else
            runWheelMotor(i, DRIVE, DRIVE_SPEED);
        }
      }
      lastDir[i] = cw;
      lastDiff[i] = diff;
    }
    if(DEBUG)
      Serial.println();
    if(aligned[0] && aligned[1] && aligned[2])
      complete = true;
  }
}

int readEncoder(int i){
  int encoderVal = analogRead(ENCODER_PINS[i]) - TRUE_ZERO[ARDUINO_NUM][i];
  if(encoderVal < 0)
    encoderVal += 1024;
  return encoderVal;
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
void runStepperMotor(int stepper, int steps, bool dir) {
  if(stepper > 0 && ARDUINO_NUM == 1)
    return;
  int finalD;
  int d = 500;
  switch (stepper) {//sets the pulse frequency (speed) based on which stepper we are running
    case 0:
      finalD = RAISE_SPEED;
      break;
    case 1:
      finalD = MOVE_CONVEYOR_SPEED;
      break;
    case 2:
      finalD = DIG_SPEED;
      break;
  }
  if(d < finalD)
    d = finalD;
  digitalWrite(STEPPER_ENA[stepper], LOW);//enable the stepper (active-low)
  if (!dir) //control it to turn CW or CCW
    digitalWrite(STEPPER_DIR[stepper], HIGH);
  else
    digitalWrite(STEPPER_DIR[stepper], LOW);
  int count = 0;
  //for(int i = 0; i < steps; i++){
  while(true){
    if(count % 50 == 0 && d > finalD){
      d -= 1;
    }
    nh.spinOnce();//make sure that messages can still be processed even while running the stepper
    if(motorDisable)
      break;
    stepperHelper(stepper, d);
    count++;
  }
  digitalWrite(STEPPER_ENA[stepper], HIGH);
}

void stepperHelper(int stepper, int d){
  digitalWrite(STEPPER_PUL[stepper], HIGH);//pulse with a frequency of 1/RAISE_SPEED
  delayMicroseconds(d);
  digitalWrite(STEPPER_PUL[stepper], LOW);
  delayMicroseconds(d);
}

void runStepperSlow(int stepper, int steps, bool dir){
  if(stepper > 0 && ARDUINO_NUM == 1)
    return;
  digitalWrite(STEPPER_ENA[stepper], LOW);//enable the stepper (active-low)
  if (!dir) //control it to turn CW or CCW
    digitalWrite(STEPPER_DIR[stepper], HIGH);
  else
    digitalWrite(STEPPER_DIR[stepper], LOW);
  for(int i = 0; i < steps; i++){
  //while(true){
    if(motorDisable)
      break;
    nh.spinOnce();
    stepperSlowHelper(stepper);
  }
  digitalWrite(STEPPER_ENA[stepper], HIGH);
}

void stepperSlowHelper(int stepper){
  digitalWrite(STEPPER_PUL[stepper], HIGH);//pulse with a frequency of 1/RAISE_SPEED
  delayMicroseconds(500);
  digitalWrite(STEPPER_PUL[stepper], LOW);
  delayMicroseconds(500);
}

/**
 * method to run the conveyor motor and the camera mount
 * runs on a sabertooth, just like the wheels, so -127 - 127 range
 */
void runConveyorMotor(int motorNum, int vel) {
  if(ARDUINO_NUM == 1)
    ConveyorMotor.motor(motorNum, vel);
}
