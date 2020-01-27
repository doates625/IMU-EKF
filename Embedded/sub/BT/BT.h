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
	void update();
	bool rx_start();
	bool rx_stop();
	void tx_state();
	void tx_data();
}
