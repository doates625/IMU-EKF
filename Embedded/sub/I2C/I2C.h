/**
 * @file I2C.h
 * @brief Subsystem for sensor I2C bus
 * @author Dan Oates (WPI Class of 2020)
 */
#pragma once
#include <Platform.h>

/**
 * Namespace Declaration
 */
namespace I2C
{
	void init();
	TwoWire* get_i2c();
}
