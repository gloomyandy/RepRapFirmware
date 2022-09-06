/*
 * ADXL345.h
 *
 *  Created on: 5 Sep 2022
 *      Author: Andy
 */

#ifndef SRC_HARDWARE_ADXL345_H_
#define SRC_HARDWARE_ADXL345_H_

#include <RepRapFirmware.h>

#if SUPPORT_ACCELEROMETERS

#include "SpiAccelerometer.h"
#include <Hardware/Spi/SharedSpiClient.h>

class ADXL345 : public SpiAccelerometer
{
public:
	ADXL345(SharedSpiDevice& dev, uint32_t freq, Pin p_csPin, Pin p_int1Pin) noexcept;

	// Do a quick test to check whether the accelerometer is present, returning true if it is
	bool CheckPresent() noexcept override;

	// Return the type name of the accelerometer. Only valid after checkPresent returns true.
	const char *GetTypeName() const noexcept override;

	// Configure the accelerometer to collect at or near the requested sampling rate and the requested resolution in bits.
	bool Configure(uint16_t& samplingRate, uint8_t& resolution) noexcept override;

	// Start collecting data
	bool StartCollecting(uint8_t axes) noexcept override;

	// Collect some data from the FIFO, suspending until the data is available
	unsigned int CollectData(const uint16_t **collectedData, uint16_t &dataRate, bool &overflowed) noexcept override;

	// Stop collecting data
	void StopCollecting() noexcept override;

	// Get a status byte
	uint8_t ReadStatus() noexcept override;

	// Used by the ISR
	void Int1Isr() noexcept;

	// Used by diagnostics
	bool HasInterruptError() const noexcept override { return interruptError; }

	static constexpr const char *TypeName = "ADXL345";

private:
	enum class AdxlRegister : uint8_t
	{
		WhoAmI = 0x00,
		BWRate = 0x2c,
		PowerCtrl = 0x2d,
		FifoStatus = 0x39,
		FifoControl = 0x38,
		DataX0 = 0x32
	};

	bool ReadRegisters(AdxlRegister reg, size_t numToRead) noexcept;
	bool WriteRegisters(AdxlRegister reg, size_t numToWrite) noexcept;
	bool ReadRegister(AdxlRegister reg, uint8_t& val) noexcept;
	bool WriteRegister(AdxlRegister reg, uint8_t val) noexcept;

	volatile TaskHandle taskWaiting;
	uint32_t firstInterruptTime;
	uint32_t lastInterruptTime;
	uint32_t totalNumRead;
	bool interruptError;
	uint8_t currentAxis;
	Pin int1Pin;
	alignas(2) uint8_t transferBuffer[2 + 7];			// 1 dummy byte for alignment, one register address byte, 6 data bytes to read one FIFO entry + 1 extra reg
	uint8_t* const dataBuffer = transferBuffer + 2;
	uint16_t data[32*3];
};

#endif

#endif /* SRC_HARDWARE_LIS3DH_H_ */
