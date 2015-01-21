#include "Arduino.h"
#include "Drivetrain.h"

/**
 * Constructor. Sets the ports for all the motors and power for the motors.
 * center: The center of where the robot should go
 * deadZone: What should be included in the "center". (how big the center should be)
 * power: How much power to apply to the motors.
 * stepTimes[15]: How much time (as an element of the array)should be spent at each step of rotation.
 */
Drivetrain::Drivetrain(byte leftMotorForward, byte leftMotorBackward,
                       byte rightMotorForward, byte rightMotorBackward,
                       int center, int deadZone, int power,
                       int stepTimes[15])
{
  //Set the pinmode of the motor ports to be output.
  pinMode(leftMotorForward, OUTPUT);
  pinMode(leftMotorBackward, OUTPUT);
  pinMode(rightMotorForward, OUTPUT);
  pinMode(rightMotorBackward, OUTPUT);

  _leftMotorForward = leftMotorForward;
  _leftMotorBackward = leftMotorBackward;
  _rightMotorForward = rightMotorForward;
  _rightMotorBackward = rightMotorBackward;
  _center = center;
  _deadZone = deadZone;
  _power = power;
  _stepTimes = stepTimes;
}

/**
 * Destructor
 */
Drivetrain::~Drivetrain()
{
}

/******************
 * Public Methods *
 ******************/

/**
 * Tells the motor to keep the center of the block aligned with the requested center
 * set in the Drivetrain constructor
 */
void Drivetrain::goToFish(Block block)
{
  //If the center of the block is on the left side of the camera
  if (block.x < (_center - _deadZone))
  {
    Serial.print("turning left\n");
    turnLeft(_power);
  }
  //If the center of the block is on the right side of the camera
  else if (block.x > (_center + _deadZone))
  {
    Serial.print("turning right\n");
    turnRight(_power);
  }
  //The center of the block is in the center of the camera
  else //Dead zone
  {
    Serial.print("not turning: Dead Zone\n");
    goStraight(_power);
  }
}

/**
 * Rotates an amount based on what step we're on. StepNum > 0
 */
void Drivetrain::rotate(int stepNum)
{
  switch (stepNum)
  {
    case 1: turnRight(_power);
      delay(_stepTimes[0]);
      stopMotors();
      break;
    case 2: //Same as case 3
    case 3: turnLeft(_power);
      delay(_stepTimes[2]);
      stopMotors();
      break;
    //Turn to face the outer ring of fish
    case 4: turnRight(_power);
      delay(_stepTimes[3]);
      stopMotors();
      break;
    case 5: turnLeft(_power);
      delay(_stepTimes[4]);
      stopMotors();
      break;
    case 6: //Same as case 11
    case 7: //Same as case 11
    case 8: //Same as case 11
    case 9: //Same as case 11
    case 10: //Same as case 11
    case 11: turnLeft(_power);
      delay(_stepTimes[10]);
      stopMotors();
      break;
    //Turn right to face first bin
    case 12: turnRight(_power);
      delay(_stepTimes[11]);
      stopMotors();
      break;
    //TODO: more steps than I first thought, will need to finish the rest;
    // there are two steps for each bin
    case 13: turnLeft(_power);
      delay(_stepTimes[12]);
      stopMotors();
      break;
    case 14: turnRight(_power);
      delay(_stepTimes[13]);
      stopMotors();
      break;
    case 15: turnRight(_power);
      delay(_stepTimes[14]);
      stopMotors();
      break;
    case 16: turnRight(_power);
      delay(_stepTimes[15]);
      stopMotors();
      break;
  }
}

/**
 * Tells all motors to turn off.
 */
void Drivetrain::stopMotors()
{
  analogWrite(_rightMotorForward, 0);
  analogWrite(_rightMotorBackward, 0);
  analogWrite(_leftMotorForward, 0);
  analogWrite(_leftMotorBackward, 0);
}

/*******************
 * Private Methods *
 *******************/

/**
 * Tell the motors to drive forward at the same power. Since our motors move at different rates given the same power, it will inevitably turn. Need encoders or a reference
 * (like the pixy) to go in a straight line
 */
void Drivetrain::goStraight(int goSpeed)
{
  analogWrite(_rightMotorForward, goSpeed);
  analogWrite(_rightMotorBackward, 0);
  analogWrite(_leftMotorForward, goSpeed);
  analogWrite(_leftMotorBackward, 0);
}

/**
 * Tells the left motor to power forward.
 */
void Drivetrain::turnRight(int goSpeed)
{
  analogWrite(_rightMotorForward, 0);
  analogWrite(_rightMotorBackward, 0);
  analogWrite(_leftMotorForward, goSpeed);
  analogWrite(_leftMotorBackward, 0);
}

/**
 * Tells the right motor to power forward.
 */
void Drivetrain::turnLeft(int goSpeed)
{
  analogWrite(_rightMotorForward, goSpeed);
  analogWrite(_rightMotorBackward, 0);
  analogWrite(_leftMotorForward, 0);
  analogWrite(_leftMotorBackward, 0);
}




