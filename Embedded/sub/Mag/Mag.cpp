/**
 * @file Mag.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "Mag.h"
#include <I2C.h>
#include <HMC5883L.h>

/**
 * Namespace Definitions
 */
namespace Mag
{
	HMC5883L mag(I2C::get_i2c());
	bool working_ = false;
}

/**
 * @brief Inits and tests mag
 */
void Mag::init()
{
	working_ = mag.init();
}

/**
 * @brief Returns mag working status
 */
bool Mag::working()
{
	return working_;
}

/**
 * @brief Updates mag readings
 */
void Mag::update()
{
	mag.update();
}

/**
 * @brief Gets x-axis field [uT]
 * 
 * Remaps axes to NED
 */
float Mag::get_mag_x()
{
	return +mag.get_x();
}

/**
 * @brief Gets y-axis field [uT]
 * 
 * Remaps axes to NED
 */
float Mag::get_mag_y()
{
	return -mag.get_y();
}

/**
 * @brief Gets z-axis field [uT]
 * 
 * Remaps axes to NED
 */
float Mag::get_mag_z()
{
	return -mag.get_z();
}
