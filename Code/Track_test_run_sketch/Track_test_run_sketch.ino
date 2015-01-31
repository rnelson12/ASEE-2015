#include <SPI.h>
#include <Pixy.h>
#include <VisualSensor.h>
#include <Drivetrain.h>

/***********************************************
 * Test Sketch for a test run around the track *
 ***********************************************
 * Tests what the robot should do when it is placed on the track. Should go around the track according to path#1 on the wiki.
 * Should do the following, in order (according to the state diagram on the wiki):
 * 1) Check if numFishCollected<12: If true the robot is in Pick Up Fish Route:
 *    A) Move towards the closest fish until it is close. (tests isClose() and getBlock() methods, as well as goToFish() method)
 *    B) If the robot is now close to the fish, it will stop for 4 seconds to simulate picking up a fish.
 *    C) The robot then rotates to face the next fish. (tests rotate() method)
 *    D) Go to step 1.
 * 2) numFishCollected = 12; The robot has collected all fish and is now in the Dump Fish Route:
 *    A) Move toward the bin closest to it. (tests getBlock() and goToFish() methods)
 *    B) If the robot is close to the bin, stop motors and rotate to position itself for dumping. (tests isClose() and rotate() methods)
 *    C) Robot stops for 4 seconds to simulate dumping a bin.
 *    D) Robot rotates to face the next bin. (tests rotate() method)
 *    E) Go to step 2)A).
 */

//Port Assignment
const byte leftMotorForward = 3;
const byte leftMotorBackward = 10;
const byte rightMotorForward = 5;
const byte rightMotorBackward = 6;
const char IRPort = A0;

//Adjustment values
float stopVoltage = 2.8; //How close the robot gets before it stops. Lower number means greater stopping distance. Maximum value is 3.2.
int center = 160; //Where the robot aims when it detects a block. Valid values are 0 - 319.
byte deadZone = 20; //How big the "center" of the robot is. Smaller values will cause robot to wiggle more.
byte power = 160; //How much power for wheel motors. Valid values are 0 - 255.

//Steptimes array; need to test robot around track to fill out these values
//It's an array where each element is how much time in milliseconds should be spent at each step of rotation.
int stepTimes[19] = {1000, //At fish 1, turn to fish 2
                     1000, //At fish 2, turn to fish 3
                     1000, //At fish 3, turn to fish 4

                     //Turn to face the outer ring of fish
                     1000, //At fish 4, turn to fish 5
                     1000, //At fish 5, turn to fish 6
                     1000, //At fish 6, turn to fish 7
                     1000, //At fish 7, turn to fish 8
                     1000, //At fish 8, turn to fish 9
                     1000, //At fish 9, turn to fish 10
                     1000, //At fish 10, turn to fish 11
                     1000, //At fish 11, turn to fish 12

                     //End of fish collection route
                     1000, //At fish 12, face bin 1
                     1000, //At bin 1, reposition for dumping
                     1000, //At bin 1, face bin 2
                     1000, //At bin 2, reposition for dumping
                     1000, //At bin 2, face bin 3
                     1000, //At bin 3, reposition for dumping
                     1000, //At bin 3, face bin 4
                     1000, //At bin 4, reposition for dumping
                    };

//Classes from our libraries
Drivetrain *wheels;
VisualSensor *eyes;

//Variables to keep track of
byte numFishCollected;
byte stepNum;

void setup()
{
  Serial.begin(9600);

  //Construct drivetrain and sensor objects
  wheels = new Drivetrain(leftMotorForward, leftMotorBackward, rightMotorForward, rightMotorBackward, center, deadZone, power, stepTimes);
  eyes = new VisualSensor(IRPort, stopVoltage);

  numFishCollected = 0;
  stepNum = 0;
}

void loop()
{
  //Test num fish collected; if less than 12 we are in fish collecting state.
  if (numFishCollected < 12)
  {
    //Check if we are close to a fish, if not:
    if (! (*eyes).isClose())
    {
      //Move toward the closest fish
      Block targetBlock = (*eyes).getBlock(); //Get closest fish
      
      //Get block returns a bad block if no blocks were found, check if the block is the bad block
      if(targetBlock.signature != (*eyes).badBlock.signature)
      {
        (*wheels).goToFish(targetBlock); //Block is good, Move toward it
      }
    }
    else //We are close to a fish:
    {
      //Insert conveyor code here
      //Conveyor code done; increment number of fish collected
      numFishCollected++;

      //4 second delay to simulate conveyor time
      (*wheels).stopMotors();
      delay(4000);

      //Rotate toward next fish
      stepNum++;
      (*wheels).rotate(stepNum);
    }
  }
  else //We are in dumping fish state
  {
    //Check if we are close to a bin, if not:
    if (! (*eyes).isClose())
    {
      //Move toward the closest bin
      Block targetBlock = (*eyes).getBlock(); //Get closest bin
      
      //Get block returns a bad block if no blocks were found, check if the block is the bad block
      if(targetBlock.signature != (*eyes).badBlock.signature)
      {
        (*wheels).goToFish(targetBlock); //Block is good, Move toward it
      }
    }
    else //We are close to a bin:
    {
      //Rotate to position for dumping
      stepNum++;
      (*wheels).rotate(stepNum);

      //Insert dumping code here
      //4 second delay to simulate dumping time
      (*wheels).stopMotors();
      delay(4000);

      //Rotate toward next bin
      stepNum++;
      (*wheels).rotate(stepNum);
    }
  }
}
