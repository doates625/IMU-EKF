/**
 * @file State.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "State.h"
#include <State.h>
#include <BT.h>
#include <Imu.h>
#include <Mag.h>
#include <Platform.h>
#include <DigitalOut.h>
using Platform::wait_ms;

/**
 * Namespace Definitions
 */
namespace State
{
	// State enum
	state_t state;

	// Error LED
	const uint8_t pin_led = 13;
	DigitalOut led(pin_led);
	void error(uint8_t blinks);
}

/**
 * @brief Inits state machine
 */
void State::init()
{
	state = idle;
	led = 0;
	if (!Imu::working()) error(1);
	if (!Mag::working()) error(2);
}

/**
 * @brief Updates state machine
 */
void State::update()
{
	switch (state)
	{
		case idle:
			// Wait for start
			if (BT::rx_start())
			{
				state = sampling;
				BT::tx_state();
			}
			break;
		case sampling:
			// Sample sensors
			if (BT::rx_stop())
			{
				state = idle;
				BT::tx_state();
			}
			BT::tx_data();
			break;
		default:
			break;
	}
}

/**
 * @brief Gets state as byte
 */
uint8_t State::get()
{
	return state;
}

/**
 * @brief Flashes LED to indicate error
 * @param blinks Number of LED blinks
 */
void State::error(uint8_t blinks)
{
	for (uint8_t b = 0; b < blinks; b++)
	{
		led = 1;
		wait_ms(100);
		led = 0;
		wait_ms(100);
	}
	wait_ms(1000);
}
