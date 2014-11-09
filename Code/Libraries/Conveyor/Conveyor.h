#ifndef CONVEYOR_H
#define CONVEYOR_H

#include <Arduino.h>
#include <Servo.h>

enum ClawPosition : byte
{
  FISH = 22,
  RED_BIN = 23,
  YELLOW_BIN = 24,
  BLUE_BIN = 25,
  GREEN_BIN = 26
};

class Conveyor
{
  public:
    Conveyor();
    ~Conveyor();
    void goToPosition(int clawPosition);
    void openClaw();
    void closeClaw();
  private:
    int motorSpeed;
    int closedAngle;
    int openAngle;
    Servo clawServo;
};

#endif
    
