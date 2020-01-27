/**
 * @file Mag.h
 * @brief Subsystem for HMC5883L magnetometer
 * @author Dan Oates (WPI Class of 2020)
 */
#pragma once

/**
 * Namespace Declaration
 */
namespace Mag
{
	void init();
	bool working();
	void update();
	float get_mag_x();
	float get_mag_y();
	float get_mag_z();
}
