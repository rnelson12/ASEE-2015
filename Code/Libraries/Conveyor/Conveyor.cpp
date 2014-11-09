#include "Conveyor.h"

const byte conveyorMotorForwardPin = 2;
const byte conveyorMotorBackwardPin = 3;
const byte clawMotorPin = 4;

Conveyor::Conveyor()
{
  motorSpeed = 160;
  
  clawServo.attach(clawMotorPin);
  closedAngle = 0;
  openAngle = 90;

  pinMode(FISH, INPUT);
  pinMode(RED_BIN, INPUT);
  pinMode(YELLOW_BIN, INPUT);
  pinMode(BLUE_BIN, INPUT);
  pinMode(GREEN_BIN, INPUT);

  pinMode(conveyorMotorForwardPin, OUTPUT);
  pinMode(conveyorMotorBackwardPin, OUTPUT);
  pinMode(clawMotorPin, OUTPUT);
}

Conveyor::~Conveyor()
{
  
}

/**
 *  If claw position is fish, the motor should turn backward, otherwise it will turn forward until it hits the bin limit switch. Need to call this method twice to return the claw to
 *  fish position.
 */
void Conveyor::goToPosition(int clawPosition)
{
  if (clawPosition == FISH)
  {
    //Turn motor on backward
    analogWrite(conveyorMotorBackwardPin, motorSpeed);

    //If the limit switch of the requested bin is HIGH, stop motor
    if (digitalRead(clawPosition) == HIGH )
    {
      analogWrite(conveyorMotorBackwardPin, 0);
    }
  }
  else //Move claw to appropriate bin
  {
    //Turn motor on
    analogWrite(conveyorMotorForwardPin, motorSpeed);

    //If the limit switch of the requested bin is HIGH, stop motor
    if (digitalRead(clawPosition) == HIGH )
    {
      analogWrite(conveyorMotorForwardPin, 0);
    }
  }
}

/**
 * Close the claw
 */
void Conveyor::closeClaw()
{
  clawServo.write(closedAngle);
}

/**
 * Closes the claw
 */
void Conveyor::openClaw()
{
  clawServo.write(closedAngle);
}
