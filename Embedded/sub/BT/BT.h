/**
 * @file BT.h
 * @brief Subsystem for Bluetooth communication
 * @author Dan Oates (WPI Class of 2020)
 */
#pragma once
#include <stdint.h>

/**
 * Namespace Declaration
 */
namespace BT
{
	void init();
	void reset();
	void update();
	bool sampling();
	void send_data();
}
