/**
 * @file Imu.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "Imu.h"
#include <I2C.h>
#include <MPU6050.h>

/**
 * Namespace Definitions
 */
namespace Imu
{
	MPU6050 imu(I2C::get_i2c());
	bool working_ = false;
}

/**
 * @brief Inits and tests IMU
 */
void Imu::init()
{
	working_ = imu.init();
}

/**
 * @brief Returns IMU working status
 */
bool Imu::working()
{
	return working_;
}

/**
 * @brief Updates IMU readings
 */
void Imu::update()
{
	imu.update_gyr();
}

/**
 * @brief Gets x-axis angular velocity [rad/s]
 * 
 * Remaps axes to NED
 */
float Imu::get_gyr_x()
{
	return +imu.get_gyr_y();
}

/**
 * @brief Gets y-axis angular velocity [rad/s]
 * 
 * Remaps axes to NED
 */
float Imu::get_gyr_y()
{
	return +imu.get_gyr_x();
}

/**
 * @brief Gets z-axis angular velocity [rad/s]
 * 
 * Remaps axes to NED
 */
float Imu::get_gyr_z()
{
	return -imu.get_gyr_z();
}
