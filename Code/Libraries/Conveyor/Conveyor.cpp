#include "Conveyor.h"

const byte conveyorMotorForwardPin = 2;
const byte conveyorMotorBackwardPin = 3;
const byte clawMotorPin = 4;
const byte limitSwitchPin = 5 ;//?????
const byte binMotorForwardPin = 6;
const byte binMotorBackwardPin = 7;


Conveyor::Conveyor()
{
  motorSpeed = 160;
  binMotorSpeed = 200;
  binDumpingTime = 2000UL;
  clawServo.attach(clawMotorPin);
  closedAngle = 0;
  openAngle = 90;
  currentPosition = FISH;
  currentState = LOW;
  prevState = LOW;
  
  pinMode(conveyorMotorForwardPin, OUTPUT);
  pinMode(conveyorMotorBackwardPin, OUTPUT);
  pinMode(clawMotorPin, OUTPUT);
  pinMode(limitSwitchPin, INPUT);
  pinMode(binMotorForwardPin, OUTPUT);
  pinMode(binMotorBackwardPin, OUTPUT);

}

Conveyor::~Conveyor()
{
  
}

/**
 * This method counts using the limit switch and moves the claw backwards or forwards  
 * depending on where it mus go. This method also return a boolean  based on if the claw is
 * in the correct position. 
 */
bool Conveyor::goToBin(BinPosition binPosition)
{
bool correctPosition = false;
 // Checks current reading
 currentState = digitalRead(limitSwitchPin);
 //Counting part
 if(binPosition>currentPosition)  
 {
   if(currentState == HIGH && prevState == LOW)
   {
     currentPosition = (BinPosition)((int)(currentPosition) + 	1);
   } 
   analogWrite(conveyorMotorForwardPin, motorSpeed);
   analogWrite(conveyorMotorBackwardPin, 0);
 }
 else if(binPosition<currentPosition)  
 {
   if(currentState == HIGH && prevState == LOW)
   {
     currentPosition = (BinPosition)((int)(currentPosition) - 	1);

   }
   analogWrite(conveyorMotorBackwardPin, motorSpeed);
    analogWrite(conveyorMotorForwardPin, 0);
 }
 else //we're at correct position
 {
    analogWrite(conveyorMotorBackwardPin, 0);
    analogWrite(conveyorMotorForwardPin, 0);
  correctPosition = true;
 }

 prevState = currentState; 
 return correctPosition;
}

/**
 * Close the claw
 */
void Conveyor::closeClaw()
{
  clawServo.write(closedAngle);
}

/**
 * Opens the claw
 */
void Conveyor::openClaw()
{
  clawServo.write(closedAngle);
}
bool Conveyor::dumpBins(unsigned long startTime)
{
   unsigned long currentTime = millis();
   
   if((currentTime - startTime)>=binDumpingTime)
   {
    analogWrite(binMotorBackwardPin, 0);
    analogWrite(binMotorForwardPin, 0);
    return true;
   }
   else
   {
      analogWrite(binMotorBackwardPin, 0);
      analogWrite(binMotorForwardPin, binMotorSpeed);
	return false;
   }
}