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
*/
Compass::Compass(int sensorID)
{
	_sensorID = sensorID;
	_initDegrees = getDegrees();
}

Compass::~Compass()
{

}

/**
* Returns the current heading of the robot in degrees. (0 <= degrees < 360)
*/
int Compass::getDegrees()
{
	//TODO
	return -1;
}

/**
* Returns the initial heading when the program was first started in degrees. (0 <= degrees < 360)
*/
int Compass::getInitDegrees()
{
	return _initDegrees;
}



static float _hmc5883_Gauss_LSB_XY = 1100.0F;  // Varies with gain
static float _hmc5883_Gauss_LSB_Z = 980.0F;   // Varies with gain

/**************************************************************************/
/*!
@brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
void Compass::write8(byte address, byte reg, byte value)
{
	Wire.beginTransmission(address);
	Wire.write((uint8_t) reg);
	Wire.write((uint8_t) value);
	Wire.endTransmission();
}

/**************************************************************************/
/*!
@brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
byte Compass::read8(byte address, byte reg)
{
	byte value;

	Wire.beginTransmission(address);
	Wire.write((uint8_t) reg);
	Wire.endTransmission();

	Wire.requestFrom(address, (byte) 1);
	value = Wire.read();
	Wire.endTransmission();

	return value;
}

/**************************************************************************/
/*!
@brief  Reads the raw data from the sensor
*/
/**************************************************************************/
void Compass::read()
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
	_magData.x = (int16_t) (xlo | ((int16_t) xhi << 8));
	_magData.y = (int16_t) (ylo | ((int16_t) yhi << 8));
	_magData.z = (int16_t) (zlo | ((int16_t) zhi << 8));

	// Convert values to correct numbers
	_magnetic.x = _magData.x / _hmc5883_Gauss_LSB_XY * 100; //* 100 to convert from gauss to microtesla
	_magnetic.y = _magData.y / _hmc5883_Gauss_LSB_XY * 100;
	_magnetic.z = _magData.z / _hmc5883_Gauss_LSB_Z * 100;

	// ToDo: Calculate orientation
	_magData.orientation = 0.0;
}

/**************************************************************************/
/*!
@brief  Setups the HW
*/
/**************************************************************************/
bool Compass::begin()
{
	// Enable I2C
	Wire.begin();

	// Enable the magnetometer
	write8(HMC5883_ADDRESS_MAG, HMC5883_REGISTER_MAG_MR_REG_M, 0x00);

	// Set the gain to a known level
	setMagGain(HMC5883_MAGGAIN_1_3);

	return true;
}

/**************************************************************************/
/*!
@brief  Sets the magnetometer's gain
*/
/**************************************************************************/
void Compass::setMagGain(hmc5883MagGain gain)
{
	write8(HMC5883_ADDRESS_MAG, HMC5883_REGISTER_MAG_CRB_REG_M, (byte) gain);

	_magGain = gain;

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