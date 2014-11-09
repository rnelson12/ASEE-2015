/**
 * Tells the motors to go; 
 * if direction is straight it goes straight, 
 * if direction is right it turns right,
 * and if direction is left it turns left
 * Uses the Directions enum defined in Definitions.h
 */
void go(int goSpeed, Directions myDirection)
{
  if (myDirection == right) //turn Right
  {
    turnRight(goSpeed);
  }
  else if (myDirection == left) //Turn left
  {
    turnLeft(goSpeed);
  }
  else //myDirection == straight; go straight
  {
    goStraight(goSpeed);
  }
}

/**
 * Tell the motors to drive forward at the same power. Since our motors move at different rates given the same power, it will inevitably turn. Need encoders or a reference 
 * (like the pixy) to go in a straight line
 */
void goStraight(int goSpeed)
{
  analogWrite(rightMotorForward, goSpeed);
  analogWrite(rightMotorBackward, 0);
  analogWrite(leftMotorForward, goSpeed);
  analogWrite(leftMotorBackward, 0);
}

 /**
  * Tells the left motor to power forward.
  */
void turnRight(int goSpeed)
{
  analogWrite(rightMotorForward, 0);
  analogWrite(rightMotorBackward, 0);
  analogWrite(leftMotorForward, goSpeed);
  analogWrite(leftMotorBackward, 0);
}

/**
 * Tells the right motor to power forward.
 */
void turnLeft(int goSpeed)
{
  analogWrite(rightMotorForward, goSpeed);
  analogWrite(rightMotorBackward, 0);
  analogWrite(leftMotorForward, 0);
  analogWrite(leftMotorBackward, 0);
}

 /**
  * Tells all motors to turn off.
  */
void stopMotors()
{
  analogWrite(rightMotorForward, 0);
  analogWrite(rightMotorBackward, 0);
  analogWrite(leftMotorForward, 0);
  analogWrite(leftMotorBackward, 0);
}
