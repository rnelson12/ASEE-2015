#include <Arduino.h>
#include "Sensors.h"

/**
 * Constructor. Intitialize the following variables
 * badBlock: Initialize all values of the block to -1.
 * _pixy: Initialize the pixy camera.
 * _IRPort: Set the IRPort.
 * _stopVoltage: Set the stop voltage; The robot should stop whenever the 
 *               input voltage from the IR sensor is greater than this voltage.
 */
VisualSensor::VisualSensor(const char IRPort, const float stopVoltage)
{
  //Set all the badBlock's values to -1
  badBlock.signature = -1;
  badBlock.x = -1;
  badBlock.y = -1;
  badBlock.width = -1;
  badBlock.height = -1;

  //Initialize pixy
  _pixy.init();

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
 * Find the correct block to go to. 
 * Currently finds the lowest one in the Pixy's view (lower = closer).
 * Returns null if there are no blocks found.
 */
Block VisualSensor::getBlock()
{
  //Get the number of blocks(detected objects) from the pixy
  int numBlocks = _pixy.getBlocks();

  if (numBlocks == 0)
  {
	  return badBlock;
  }

  //Serial.println( "--------" );

  //Set the initial maximum to be the first block found
  int maxY = _pixy.blocks[0].y;
  Block block; //Declare block to be returned
  //Loop through each block
  for (int j = 0; j < numBlocks; j++)
  {
	// _pixy.blocks[ j ].print();
	
	//Find the lowest block in the frame (which should be the closest block)
	//higher y value means lower in the frame
	if (_pixy.blocks[j].y >= maxY)
	{
	  maxY = _pixy.blocks[j].y;
	  block = _pixy.blocks[j];
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

/**
* Constructor. Set the initial heading whenever the program starts.
* -declinationAngle: 'Error' of the magnetic field in your location. Find yours here: http://www.magnetic-declination.com/.
* -(for Norman, I calculated it to be: 0.069522276053)
*/
Compass::Compass(float declinationAngle)
{
	/* Initialize the sensor */
	if(!begin())
	{
		/* There was a problem detecting the HMC5883 ... check your connections */
		Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
	}

	_hmc5883_Gauss_LSB_XY = 1100.0F;  // Varies with gain
	_hmc5883_Gauss_LSB_Z = 980.0F;   // Varies with gain

	_declinationAngle = declinationAngle;
	_initMagVector = getMagVector();
}

/**
* Deconstructor
*/
Compass::~Compass()
{

}

/**
 * Returns the vector of magnetic values that the magnetometor is currently reading.
 */
hmc5883MagData Compass::getMagVector()
{
	// Read the magnetometer
	Wire.beginTransmission((byte) HMC5883_ADDRESS_MAG);
	Wire.write(HMC5883_REGISTER_MAG_OUT_X_H_M);
	Wire.endTransmission();
	Wire.requestFrom((byte) HMC5883_ADDRESS_MAG, (byte) 6);

	// Wait around until enough data is available
	while(Wire.available() < 6);

	// Note high before low (different than accel)  
	uint8_t xhi = Wire.read();
	uint8_t xlo = Wire.read();
	uint8_t zhi = Wire.read();
	uint8_t zlo = Wire.read();
	uint8_t yhi = Wire.read();
	uint8_t ylo = Wire.read();

	// Shift values to create properly formed integer (low byte first)
	hmc5883MagData magData;
	magData.x = (int16_t) (xlo | ((int16_t) xhi << 8));
	magData.y = (int16_t) (ylo | ((int16_t) yhi << 8));
	magData.z = (int16_t) (zlo | ((int16_t) zhi << 8));

	// Convert values to correct numbers
	hmc5883MagData magVector;
	magVector.x = magData.x / _hmc5883_Gauss_LSB_XY * 100; //* 100 to convert from gauss to microtesla
	magVector.y = magData.y / _hmc5883_Gauss_LSB_XY * 100;
	magVector.z = magData.z / _hmc5883_Gauss_LSB_Z * 100;

	return magData;
}


void Compass::calibrate()
{
	//Rotate the robot slowly in a circle (direction does not matter)
	//Calculate the center.x, center.y, and center.z by averaging all the values
	int numPoints = 1;
	hmc5883MagData startPoint = getMagVector();
	Serial.println("Start rotating the robot slowly when you see GO.");
	delay(1000);
	Serial.println("3...");
	delay(1000);
	Serial.println("2...");
	delay(1000);
	Serial.println("1...");
	delay(1000);
	Serial.println("GO");
	unsigned long timer = millis();
	while(_calibrate)
	{
		//Get the current point
		currentPoint = getMagVector();

		//When at least a second has passed...
		if(millis() - timer >= 1000UL)
		{
			//...and all the points are close to the starting point, end the calibration. Save the center point to the EEPROM
			if(abs(_centerPoint.x - currentPoint.x) < 10 && abs(_centerPoint.y - currentPoint.y) < 10 && abs(_centerPoint.z - currentPoint.z) < 10)
			{
				Serial.println("Calibrating done!");
				Serial.print("Center- X: ");
				Serial.print(_centerPoint.x);
				Serial.print(" Y: ");
				Serial.print(_centerPoint.y);
				Serial.print(" Z: ");
				Serial.println(_centerPoint.z);
			}
		}

		//Find the average x, y and z value of all the points
		_centerPoint.x = (_centerPoint.x * (numPoints - 1) + currentPoint.x) / numPoints;
		_centerPoint.y = (_centerPoint.y * (numPoints - 1) + currentPoint.y) / numPoints;
		_centerPoint.z = (_centerPoint.z * (numPoints - 1) + currentPoint.z) / numPoints;
	}
}


/**
* Returns the how many degrees the robot is rotated from the initial heading. Always positive, and always less than 180 degrees. (0 <= degrees < 180)
*/
float Compass::getDegrees()
{
	hmc5883MagData magVector = getMagVector(); //The magnetic values stored in a vector

	// Assume the point cloud of magnetic data has a centroid at the origin.
	//    If it doesn't, we need to make calibration code and run that, storing the values (center.x, center.y, center.z) in EEPROM to be used later.
	//    If we need to calibrate, there is the EEPROMAnything library we can use to store the center point, then subtract all magvectors from this point to determine the real vector.
	// Calculate the (always positive) angle in radians between the first heading and the new magVector.
	float heading = acos((magVector.x*_initMagVector.x + magVector.y*_initMagVector.y + magVector.z*_initMagVector.z) /
						 (sqrt(magVector.x*magVector.x + magVector.y*magVector.y + magVector.z*magVector.z)*sqrt(_initMagVector.x*_initMagVector.x + _initMagVector.y*_initMagVector.y + _initMagVector.z*_initMagVector.z)));

	// Once you have your heading, you must then add your 'Declination Angle',
	// which is the 'Error' of the magnetic field in your location.
	// Find yours here: http://www.magnetic-declination.com/
	//heading += _declinationAngle;

	// Correct for when signs are reversed.
	if(heading < 0)
		heading += 2 * PI;

	// Check for wrap due to addition of declination.
	if(heading > 2 * PI)
		heading -= 2 * PI;

	// Convert radians to degrees.
	float headingDegrees = heading * 180 / PI;
	return headingDegrees;
}

/**
 * Set the magnetometer's gain
 */
void Compass::setGain(hmc5883MagGain gain)
{
	write8(HMC5883_ADDRESS_MAG, HMC5883_REGISTER_MAG_CRB_REG_M, (byte) gain);

	switch(gain)
	{
		case HMC5883_MAGGAIN_1_3:
			_hmc5883_Gauss_LSB_XY = 1100;
			_hmc5883_Gauss_LSB_Z = 980;
			break;
		case HMC5883_MAGGAIN_1_9:
			_hmc5883_Gauss_LSB_XY = 855;
			_hmc5883_Gauss_LSB_Z = 760;
			break;
		case HMC5883_MAGGAIN_2_5:
			_hmc5883_Gauss_LSB_XY = 670;
			_hmc5883_Gauss_LSB_Z = 600;
			break;
		case HMC5883_MAGGAIN_4_0:
			_hmc5883_Gauss_LSB_XY = 450;
			_hmc5883_Gauss_LSB_Z = 400;
			break;
		case HMC5883_MAGGAIN_4_7:
			_hmc5883_Gauss_LSB_XY = 400;
			_hmc5883_Gauss_LSB_Z = 255;
			break;
		case HMC5883_MAGGAIN_5_6:
			_hmc5883_Gauss_LSB_XY = 330;
			_hmc5883_Gauss_LSB_Z = 295;
			break;
		case HMC5883_MAGGAIN_8_1:
			_hmc5883_Gauss_LSB_XY = 230;
			_hmc5883_Gauss_LSB_Z = 205;
			break;
	}
}

/**
* Set up the magnetometer
*/
bool Compass::begin()
{
	// Enable I2C
	Wire.begin();

	// Enable the magnetometer
	write8(HMC5883_ADDRESS_MAG, HMC5883_REGISTER_MAG_MR_REG_M, 0x00);

	// Set the gain to a known level
	setGain(HMC5883_MAGGAIN_1_3);

	return true;
}

/**
* Write data to the magnetometer
*/
void Compass::write8(byte address, byte reg, byte value)
{
	Wire.beginTransmission(address);
	Wire.write((uint8_t) reg);
	Wire.write((uint8_t) value);
	Wire.endTransmission();
}