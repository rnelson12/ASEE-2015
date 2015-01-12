#ifndef Drivetrain_h
#define Drivetrain_h

#include "Arduino.h"
#include <Pixy.h>
#include <SPI.h>

class Drivetrain
{
  public:
    Drivetrain(byte leftMotorForward, byte leftMotorBackward, byte rightMotorForward,
               byte rightMotorBackward, int center, int deadZone, int power, 
               int stepTimes[]);
    ~Drivetrain();
    void goToFish(Block block);
    void rotate(int stepNum);
    void stopMotors();
  private:
    byte _leftMotorForward;
    byte _rightMotorForward;
    byte _leftMotorBackward;
    byte _rightMotorBackward;
    int _center;
    int _deadZone;
    int _power;
    int *_stepTimes;
    void goStraight(int goSpeed);
    void turnLeft(int goSpeed);
    void turnRight(int goSpeed);
};

#endif
