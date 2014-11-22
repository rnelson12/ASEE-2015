#ifndef VISUALSENSOR_H
#define VISUALSENSOR_H

#include <Arduino.h>
#include <SPI.h>
#include <Pixy.h>

class VisualSensor
{
  public:
    VisualSensor(char IRPort, float stopVoltage);
    ~VisualSensor();
    Block getBlock();
    boolean isClose();
  private:
    Pixy pixy;
    float _stopVoltage;
    char _IRPort;
};

#endif
    


