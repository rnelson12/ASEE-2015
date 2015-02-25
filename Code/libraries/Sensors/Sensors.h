#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <SPI.h>
#include <Pixy.h>
#include <Wire.h>
#include <limits.h>

/* Taken from ADAFRUIT Library */
#define HMC5883_ADDRESS_MAG (0x3C >> 1) // 0011110x // I2C ADDRESS/BITS
#define HMC5883_ID (0b11010100) //CHIP ID

/* REGISTERS */
typedef enum
{
	HMC5883_REGISTER_MAG_CRA_REG_M = 0x00,
	HMC5883_REGISTER_MAG_CRB_REG_M = 0x01,
	HMC5883_REGISTER_MAG_MR_REG_M = 0x02,
	HMC5883_REGISTER_MAG_OUT_X_H_M = 0x03,
	HMC5883_REGISTER_MAG_OUT_X_L_M = 0x04,
	HMC5883_REGISTER_MAG_OUT_Z_H_M = 0x05,
	HMC5883_REGISTER_MAG_OUT_Z_L_M = 0x06,
	HMC5883_REGISTER_MAG_OUT_Y_H_M = 0x07,
	HMC5883_REGISTER_MAG_OUT_Y_L_M = 0x08,
	HMC5883_REGISTER_MAG_SR_REG_Mg = 0x09,
	HMC5883_REGISTER_MAG_IRA_REG_M = 0x0A,
	HMC5883_REGISTER_MAG_IRB_REG_M = 0x0B,
	HMC5883_REGISTER_MAG_IRC_REG_M = 0x0C,
	HMC5883_REGISTER_MAG_TEMP_OUT_H_M = 0x31,
	HMC5883_REGISTER_MAG_TEMP_OUT_L_M = 0x32
} hmc5883MagRegisters_t;
/*MAGNETOMETER GAIN SETTINGS*/
typedef enum
{
	HMC5883_MAGGAIN_1_3 = 0x20,  // +/- 1.3
	HMC5883_MAGGAIN_1_9 = 0x40,  // +/- 1.9
	HMC5883_MAGGAIN_2_5 = 0x60,  // +/- 2.5
	HMC5883_MAGGAIN_4_0 = 0x80,  // +/- 4.0
	HMC5883_MAGGAIN_4_7 = 0xA0,  // +/- 4.7
	HMC5883_MAGGAIN_5_6 = 0xC0,  // +/- 5.6
	HMC5883_MAGGAIN_8_1 = 0xE0   // +/- 8.1
} hmc5883MagGain;
/*INTERNAL MAGNETOMETER DATA TYPE*/
typedef struct hmc5883MagData_s
{
	float x;
	float y;
	float z;
	float orientation;
} hmc5883MagData;




/**
 * Class that contains sensors to be used to visually locate a block. This includes the Pixy and an IR sensor.
 */
class VisualSensor
{
  public:
	/**
	 * Constructor. Intitialize the following variables
	 * _pixy: Initialize the pixy camera.
	 * _IRPort: Set the IRPort.
	 * _stopVoltage: Set the stop voltage; The robot should stop whenever the
	 *               input voltage from the IR sensor is greater than this voltage.
	 */
    VisualSensor(const char IRPort, float stopVoltage);

	/**
	 * Destructor
	 */
    ~VisualSensor();

	/**
	 * Find the correct block to go to.
	 * Currently finds the lowest one in the Pixy's view (lower = closer).
	 */
    Block getBlock();

	/**
	 * Reads the input from the IRSensor port. This number is from 0-1023,
	 * so we convert this number into a float from 0.0 to 5.0 volts. Return true if it is
	 * in the voltage range we want.
	 */
    boolean isClose();

	Block badBlock; //Variable that is a "bad block", used when we find no good blocks
  private:
    Pixy _pixy; //Variable for pixy camera
    float _stopVoltage; //The robot should stop whenever the input voltage from the IR sensor is greater than this voltage.
    char _IRPort; //The port for the IR sensor
};

/**
 * The compass allows you to get the current heading in degrees and the initial heading of when the robot was first turned on.
 */
class Compass
{
public:

	Adafruit_HMC5883_Unified(int32_t sensorID = -1);

	bool begin(void);
	void setMagGain(hmc5883MagGain gain);
	void read(void);

	/**
	 * Constructor. Set the initial heading whenever the program starts.
	 */
	Compass();

	~Compass();

	/**
	 * Returns the current heading of the robot in degrees.
	 */
	int getDegrees();

	/**
	 * Returns the initial heading when the program was first started in degrees.
	 */
	int getInitDegrees();

private:
	int _initDegrees; //Initial degrees when the robot is started

	hmc5883MagGain   _magGain;
	hmc5883MagData   _magData;     // Last read magnetometer data will be available here
	int32_t         _sensorID;

	void write8(byte address, byte reg, byte value);
	byte read8(byte address, byte reg);
	
};

#endif