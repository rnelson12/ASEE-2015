#include <SPI.h>
#include <Pixy.h>
#include <Conveyor.h>

/************************************
 * Test Sketch for Conveyor Library *
 ************************************
* Tests the goToBin(int Position), openClaw(), and closeClaw()
*     methods
* 
* Should do the following things, in order:
* 1) Go to the FISH position, and open and close claws.
* 2) Go to the RED_BIN position, and open and close claws.
* 3) Go to the FISH position, and open and close claws.
* 4) Go to the YELLOW_BIN position, and open and close claws.
* 5) Go to the FISH position, and open and close claws.
* 6) Go to the BLUE_BIN position, and open and close claws.
* 7) Go to the FISH position, and open and close claws.
* 8) Go to the GREEN_BIN position, and open claw.
*/

Conveyor *claw;

void setup() {
  Serial.begin(9600);
  claw = new Conveyor();
}

void loop()
{
	//Go to fish position
	while(!claw->goToBin(FISH))
	{
	};
	//Close claw to pick up fish
	claw->closeClaw();
	delay(500);
	//Go to fish bin
	while(!claw->goToBin(RED_BIN))
	{
	};
	//Drop fish off
	claw->openClaw();

	//Go to fish position
	while(!claw->goToBin(FISH))
	{
	};
	//Close claw to pick up fish
	claw->closeClaw();
	delay(500);
	//Go to fish bin
	while(!claw->goToBin(YELLOW_BIN))
	{
	};
	//Drop fish off
	claw->openClaw();

	//Go to fish position
	while(!claw->goToBin(FISH))
	{
	};
	//Close claw to pick up fish
	claw->closeClaw();
	delay(500);
	//Go to fish bin
	while(!claw->goToBin(BLUE_BIN))
	{
	};
	//Drop fish off
	claw->openClaw();

	//Go to fish position
	while(!claw->goToBin(FISH))
	{
	};
	//Close claw to pick up fish
	claw->closeClaw();
	delay(500);
	//Go to fish bin
	while(!claw->goToBin(GREEN_BIN))
	{
	};
	//Drop fish off
	claw->openClaw();
}
