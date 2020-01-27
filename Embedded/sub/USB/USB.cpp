/**
 * @file USB.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "USB.h"
#include <Platform.h>
#include <Imu.h>
#include <Mag.h>

/**
 * Namespace Definitions
 */
namespace USB
{
	const uint32_t baud = 115200;
}

/**
 * @brief Inits USB serial
 */
void USB::init()
{
	Serial.begin(baud);
}

/**
 * @brief Prints sensor readings to USB
 */
void USB::update()
{
	// Sensor readings
	Serial.printf("Sensor Readings:\n\n");

	// Gyro readings
	Serial.printf("Gyroscope [rad/s]:\n");
	Serial.printf("x: %+.2f\n", Imu::get_gyr_x());
	Serial.printf("y: %+.2f\n", Imu::get_gyr_y());
	Serial.printf("z: %+.2f\n", Imu::get_gyr_z());

	// Mag readings
	Serial.printf("Magnetometer [uT]:\n");
	Serial.printf("x: %+.2f\n", Mag::get_mag_x());
	Serial.printf("y: %+.2f\n", Mag::get_mag_y());
	Serial.printf("z: %+.2f\n", Mag::get_mag_z());

	// Trailing gap
	Serial.printf("\n");
}
