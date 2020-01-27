/**
 * @file I2C.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "I2C.h"

/**
 * Namespace Definitions
 */
namespace I2C
{
	TwoWire* const i2c = &Wire;
	const uint32_t i2c_clock = 100000;
	const uint8_t pin_scl = 19;
	const uint8_t pin_sda = 18;
}

/**
 * @brief Inits I2C bus
 */
void I2C::init()
{
	i2c->begin();
	i2c->setClock(i2c_clock);
	i2c->setSCL(pin_scl);
	i2c->setSDA(pin_sda);
}

/**
 * @brief Gets pointer to I2C bus
 */
TwoWire* I2C::get_i2c()
{
	return i2c;
}
