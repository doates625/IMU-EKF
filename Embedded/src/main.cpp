/**
 * @file main.cpp
 * @brief IMU Extended Kalman Filter
 * @author Dan Oates (WPI Class of 2020)
 */

/**
 * Includes
 */

// Libraries
#include <Platform.h>
#include <Timer.h>

// Subsystems
#include <I2C.h>
#include <Imu.h>
#include <Mag.h>
#include <State.h>
#include <BT.h>

// USB debug
#if defined(USB_DEBUG)
	#include <USB.h>
#endif

/**
 * Globals
 */

// Loop timing
const float fs = 2.0f;
const float ts = 1.0f / fs;
Timer timer;

/**
 * @brief Arduino setup
 */
void setup()
{
	// Init subsystems
	I2C::init();
	Imu::init();
	Mag::init();
	State::init();
	BT::init();

	// USB debug
	#if defined(USB_DEBUG)
		USB::init();
	#endif

	// Start timer
	timer.start();
}

/**
 * @brief Arduino loop
 */
void loop()
{
	// Update subsystems
	Imu::update();
	Mag::update();
	BT::update();
	State::update();

	// USB debug
	#if defined(USB_DEBUG)
		USB::update();
	#endif

	// Loop timing
	while (timer < ts);
	timer.reset();
}
