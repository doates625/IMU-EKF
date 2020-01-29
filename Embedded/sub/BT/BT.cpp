/**
 * @file BT.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "BT.h"
#include <State.h>
#include <Imu.h>
#include <Mag.h>
#include <SerialServer.h>
#include <Timer.h>
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
	const uint8_t id_samp = 0xF0;
	const uint8_t id_data = 0xF1;
	void cb_rx_samp(uint8_t* data);
	void cb_tx_data(uint8_t* data);
	SerialServer server(serial, start_byte);

	// Message data
	bool sampling_ = false;

	// Transmit timer
	Timer timer;
}

/**
 * @brief Inits Bluetooth serial port
 */
void BT::init()
{
	serial->begin(baud);
	server.add_rx(id_samp, 1, cb_rx_samp);
	server.add_tx(id_data, 28, cb_tx_data);
	timer.start();
}

/**
 * @brief Resets internal timer
 */
void BT::reset()
{
	timer.reset();
}

/**
 * @brief Processes all incoming messages
 */
void BT::update()
{
	server.rx();
}

/**
 * @brief Returns true if sampling is enabled
 */
bool BT::sampling()
{
	return sampling_;
}

/**
 * @brief Transmits sensor data
 */
void BT::send_data()
{
	server.tx(id_data);
}

/**
 * @brief Sampling enable RX callback
 * @param data Message data pointer
 * 
 * Data format:
 * - Sampling enable [uint8]
 *     0x00 = Disabled
 *     0x01 = Enabled
 */
void BT::cb_rx_samp(uint8_t* data)
{
	sampling_ = (bool)data[0];
}

/**
 * @brief Sensor data TX callback
 * @param data Message data pointer
 * 
 * Data format:
 * - Timestamp [float, s]
 * - Angular velocity [float, [x; y; z]]
 * - Magnetic field [float, [x; y; z]]
 */
void BT::cb_tx_data(uint8_t* data)
{
	Struct str(data);
	str << timer.read();
	str << Imu::get_gyr_x();
	str << Imu::get_gyr_y();
	str << Imu::get_gyr_z();
	str << Mag::get_mag_x();
	str << Mag::get_mag_y();
	str << Mag::get_mag_z();
}
