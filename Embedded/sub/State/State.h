/**
 * @file State.h
 * @brief Subsystem for embedded state machine
 * @author Dan Oates (WPI Class of 2020)
 */
#pragma once
#include <stdint.h>

/**
 * Namespace Declaration
 */
namespace State
{
	// State enum
	typedef enum
	{
		idle,
		sampling,
	}
	state_t;

	// Functions
	void init();
	void update();
	state_t get();
}
