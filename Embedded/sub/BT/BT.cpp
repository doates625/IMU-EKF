/**
 * @file BT.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "BT.h"
#include <State.h>
#include <Imu.h>
#include <Mag.h>
#include <SerialServer.h>
#include <Struct.h>

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
	const uint8_t id_start = 0xF0;
	const uint8_t id_stop = 0xF1;
	const uint8_t id_state = 0xF2;
	const uint8_t id_data = 0xF3;
	void cb_rx_start(uint8_t* data);
	void cb_rx_stop(uint8_t* data);
	void cb_tx_state(uint8_t* data);
	void cb_tx_data(uint8_t* data);
	SerialServer server(serial, start_byte);

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
	server.add_rx(id_start, 0, cb_rx_start);
	server.add_rx(id_stop, 0, cb_rx_stop);
	server.add_tx(id_state, 1, cb_tx_state);
	server.add_tx(id_data, 24, cb_tx_data);
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
	server.tx(id_state);
}

/**
 * @brief Transmits sensor data
 */
void BT::tx_data()
{
	server.tx(id_data);
}

/**
 * @brief Start RX callback
 * @param data Message data pointer
 * 
 * Data format:
 * - Empty
 */
void BT::cb_rx_start(uint8_t* data)
{
	rx_start_ = true;
}

/**
 * @brief Stop RX callback
 * @param data Message data pointer
 * 
 * Data format:
 * - Empty
 */
void BT::cb_rx_stop(uint8_t* data)
{
	rx_stop_ = true;
}

/**
 * @brief State TX callback
 * @param data Message data pointer
 * 
 * Data format:
 * - State byte [uint8_t]
 *     0x00 = Idle
 *     0x01 = Sampling
 */
void BT::cb_tx_state(uint8_t* data)
{
	Struct str(data);
	str << (uint8_t)State::get();
}

/**
 * @brief Sensor data TX callback
 * @param data Message data pointer
 * 
 * Data format:
 * - Angular velocity [float, [x; y; z]]
 * - Magnetic field [float, [x; y; z]]
 */
void BT::cb_tx_data(uint8_t* data)
{
	Struct str(data);
	str << Imu::get_gyr_x();
	str << Imu::get_gyr_y();
	str << Imu::get_gyr_z();
	str << Mag::get_mag_x();
	str << Mag::get_mag_y();
	str << Mag::get_mag_z();
}
