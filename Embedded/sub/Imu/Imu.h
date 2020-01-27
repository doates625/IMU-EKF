/**
 * @file Imu.h
 * @brief Subsystem for MPU6050 IMU
 * @author Dan Oates (WPI Class of 2020)
 */
#pragma once

/**
 * Namespace Declaration
 */
namespace Imu
{
	void init();
	bool working();
	void update();
	float get_gyr_x();
	float get_gyr_y();
	float get_gyr_z();
}
