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
using State::state_t;
using Platform::wait_ms;

/**
 * Namespace Definitions
 */
namespace State
{
	// State enum
	state_t state;

	// Debug LED
	const uint8_t pin_led = LED_BUILTIN;
	DigitalOut led(pin_led);
	
	// Functions
	void led_error(uint8_t blinks);
	void set(state_t next_state);
}

/**
 * @brief Inits state machine
 */
void State::init()
{
	if (!Imu::working()) led_error(1);
	if (!Mag::working()) led_error(2);
	set(idle);
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
			if (BT::sampling()) set(sampling);
			break;
		case sampling:
			// Sample sensors
			if (!BT::sampling()) set(idle);
			else BT::send_data();
			break;
		default:
			break;
	}
}

/**
 * @brief Gets state as byte
 */
state_t State::get()
{
	return state;
}

/**
 * @brief Flashes LED to indicate error
 * @param blinks Number of LED blinks
 */
void State::led_error(uint8_t blinks)
{
	while (true)
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
}

/**
 * @brief Transitions to new state
 * @param next_state New state
 */
void State::set(state_t next_state)
{
	switch (next_state)
	{
		case idle:
			led = 0;
			break;
		case sampling:
			BT::reset();
			led = 1;
			break;
	}
	state = next_state;
}
