#include <Arduino.h>
#include "VisualSensor.h"

/**
 * Constructor
 */
VisualSensor::VisualSensor(char IRPort, float stopVoltage)
{
  //Initialize pixy
  pixy.init();

  //Set IR Port
  _IRPort = IRPort;
  _stopVoltage = stopVoltage;
}

/**
 * Destructor
 */
VisualSensor::~VisualSensor()
{
}

/**
 * Find the correct block to go to. Currently finds the lowest one in the view
 */
Block VisualSensor::getBlock()
{
  //Get all the blocks(detected objects) from the pixy
  int numBlocks = pixy.getBlocks();

  //Set the initial maximum to be the first block found
  int maxY = pixy.blocks[0].y;
  Block block;
  
  //Loop through each block
  for (int j = 0; j < numBlocks; j++)
  {
    
    //Find the lowest block in the frame (which should be the closest block)
    //higher y value means lower in the frame
    if (pixy.blocks[j].y > maxY)
    {
      maxY = pixy.blocks[j].y;
      block = pixy.blocks[j];
    }
  }
  return block;
}

/**
 * Reads the input from the IRSensor port. This number is from 0-1023,
 * so we convert this number into a float from 0.0 to 5.0 volts. Return true if it is 
 * in the voltage range we want.
 */
boolean VisualSensor::isClose()
{
  float voltage = analogRead(_IRPort) * (5.0 / 1023.0);
  if (voltage > _stopVoltage)
  {
   return true;
  }
  
  return false;
}

