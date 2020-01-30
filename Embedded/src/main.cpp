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

/**
 * Globals
 */

// Loop timing
const float fs = 50.0f;
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

	// Loop timing
	while (timer < ts);
	timer.reset();
}
