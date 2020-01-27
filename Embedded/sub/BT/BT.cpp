/**
 * @file BT.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "BT.h"
#include <SerialServer.h>

/**
 * Namespace Definitions
 */
namespace BT
{
	// Serial port
	HardwareSerial* const serial = &Serial3;
	const uint32_t baud = 57600;

	// Message server
	const uint8_t start_byte = 0xFF;
	SerialServer server(serial, start_byte);
	// TODO message defs

	// Message data
	bool rx_start_ = false;
	bool rx_stop_ = false;
}

/**
 * @brief Inits Bluetooth serial port
 */
void BT::init()
{
	serial->begin(baud);
	// TODO config server
}

/**
 * @brief Processes all incoming messages
 */
void BT::update()
{
	rx_start_ = false;
	rx_stop_ = false;
	server.rx();
}

/**
 * @brief Returns true if start msg was received since last update
 */
bool BT::rx_start()
{
	return rx_start_;
}

/**
 * @brief Returns true if stop msg was received since last update
 */
bool BT::rx_stop()
{
	return rx_stop_;
}

/**
 * @brief Transmits state
 */
void BT::tx_state()
{
	// TODO
}

/**
 * @brief Transmits sensor data
 */
void BT::tx_data()
{
	// TODO
}
