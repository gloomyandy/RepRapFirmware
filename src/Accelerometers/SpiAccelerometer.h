/*
 * SpiAccelerometer.h - base class for spi sensors
 *
 *  Created on: 5 Sep 2022
 *      Author: Andy
 */

#ifndef SRC_HARDWARE_SPIACCELEROMETER_H_
#define SRC_HARDWARE_SPIACCELEROMETER_H_

#include <RepRapFirmware.h>

#if SUPPORT_ACCELEROMETERS

#include <Hardware/Spi/SharedSpiClient.h>

class SpiAccelerometer : public SharedSpiClient
{
public:
	SpiAccelerometer(SharedSpiDevice& dev, uint32_t freq, SpiMode mode, Pin p_csPin, bool polarity) noexcept;
	SpiAccelerometer(const SpiAccelerometer&) = delete;
	virtual ~SpiAccelerometer() noexcept;

	// Do a quick test to check whether the accelerometer is present, returning true if it is
	virtual bool CheckPresent() noexcept = 0;

	// Return the type name of the accelerometer. Only valid after checkPresent returns true.
	virtual const char *GetTypeName() const noexcept = 0;

	// Configure the accelerometer to collect at or near the requested sampling rate and the requested resolution in bits.
	virtual bool Configure(uint16_t& samplingRate, uint8_t& resolution) noexcept = 0;

	// Start collecting data
	virtual bool StartCollecting(uint8_t axes) noexcept = 0;

	// Collect some data from the FIFO, suspending until the data is available
	virtual unsigned int CollectData(const uint16_t **collectedData, uint16_t &dataRate, bool &overflowed) noexcept = 0;

	// Stop collecting data
	virtual void StopCollecting() noexcept = 0;

	// Get a status byte
	virtual uint8_t ReadStatus() noexcept = 0;

	// Used by diagnostics
	virtual bool HasInterruptError() const noexcept = 0;

	// Factory method
	static SpiAccelerometer *Create(const char *typeName, SharedSpiDevice& dev, uint32_t freq, Pin p_csPin, Pin p_int1Pin, const StringRef& reply) noexcept;
};

#endif

#endif /* SRC_HARDWARE_SPIACCELEROMETER_H_ */
