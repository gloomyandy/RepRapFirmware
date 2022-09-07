/*
 * ADXL345.cpp
 *
 *  Created on: 5 sep 2022
 *      Author: Andy
 */

#include "ADXL345.h"

#if SUPPORT_ACCELEROMETERS

#include <Hardware/IoPorts.h>
#include <Movement/StepTimer.h>

constexpr uint32_t AdxlSpiTimeout = 25;							// timeout while waiting for the SPI bus
constexpr uint32_t DataCollectionTimeout = (1000 * 32)/400 + 2;		// timeout whole collecting data, enough to fill the FIFO at 400Hz
constexpr uint8_t FifoInterruptLevel = 20;							// how full the FIFO must get before we want an interrupt
const SpiMode adxlMode = SpiMode::mode3;

static constexpr uint8_t WhoAmIValue_ADXL = 0xe5;

ADXL345::ADXL345(SharedSpiDevice& dev, uint32_t freq, Pin p_csPin, Pin p_int1Pin) noexcept
	: SpiAccelerometer(dev, freq, adxlMode, p_csPin, false), taskWaiting(nullptr), int1Pin(p_int1Pin)
{
}

// Do a quick test to check whether the accelerometer is present. If it is, return true and set the type.
bool ADXL345::CheckPresent() noexcept
{
	uint8_t val;
	if (ReadRegister(AdxlRegister::WhoAmI, val))
	{
		if (val == WhoAmIValue_ADXL)
		{
			return true;
		}
	}
	return false;
}

// Return the type name of the accelerometer. Only valid after checkPresent returns true.
const char *ADXL345::GetTypeName() const noexcept
{
	return "ADXL345";
}

uint8_t ADXL345::ReadStatus() noexcept
{
	uint8_t val;
	return (ReadRegister(AdxlRegister::FifoStatus, val)) ? val : 0xFF;
}

// Configure the accelerometer to collect for the requested axis at or near the requested sampling rate and the requested resolution in bits.
// Update the sampling rate and resolution to the actual values used.
bool ADXL345::Configure(uint16_t& samplingRate, uint8_t& resolution) noexcept
{
	bool ok;
	resolution = 10;
	// Set up sample rate
	uint8_t rate;
	if (samplingRate > 1600)
	{
		samplingRate = 3200;
		rate = 0xf;
	}
	else if (samplingRate == 0 || samplingRate >= 1200)
	{
		samplingRate = 1600;
		rate = 0xe;
	}
	else if (samplingRate >= 600)
	{
		samplingRate = 800;											// select 800Hz if we asked for 600 or higher
		rate = 0xd;
	}
	else
	{
		samplingRate = 400;											// set 400Hz, lower isn't useful
		rate = 0xc;
	}
	dataBuffer[0] = rate;											// set data rate
	dataBuffer[1] = 0; 												// default power control
	dataBuffer[2] = 0x3;											// int enable watermark and overrun
	dataBuffer[3] = 0;												// int mapping all to int 1
	dataBuffer[4] = 0;
	dataBuffer[5] = 0x4;
	ok = WriteRegisters(AdxlRegister::BWRate, 6);
	if (ok)
	{
		// Set the fifo mode
		ok = WriteRegister(AdxlRegister::FifoControl, (1u << 6) | (FifoInterruptLevel));
	}

	return ok;
}

void Int1Interrupt(CallbackParameter p) noexcept;						// forward declaration

// Start collecting data, returning true if successful
bool ADXL345:: StartCollecting(uint8_t axes) noexcept
{
	// Clear the fifo
	uint8_t val;
	int cnt = 0;
	while (ReadRegister(AdxlRegister::FifoStatus, val) && ((val & 0x3f) > 0) && (cnt < 128))	// while fifo not empty
	{
		if (!ReadRegisters(AdxlRegister::DataX0, 6))
		{
			return false;
		}
		cnt++;
	}
	totalNumRead = 0;

	// Before we enable data collection, check that the interrupt line is low
	pinMode(int1Pin, INPUT);			// make sure we can read the interrupt pin
	delayMicroseconds(5);
	interruptError = digitalRead(int1Pin);
	if (interruptError)
	{
		return false;
	}

	const bool ok = WriteRegister(AdxlRegister::PowerCtrl, 0x08);
	return ok && attachInterrupt(int1Pin, Int1Interrupt, InterruptMode::rising, CallbackParameter(this));
}

// Collect some data from the FIFO, suspending until the data is available
unsigned int ADXL345::CollectData(const uint16_t **collectedData, uint16_t &dataRate, bool &overflowed) noexcept
{
	// Wait until we have some data
	taskWaiting = TaskBase::GetCallerTaskHandle();
	while (!digitalRead(int1Pin))
	{
		if (!TaskBase::Take(DataCollectionTimeout))
		{
			return 0;
		}
	}
	taskWaiting = nullptr;
	// Read status to see how much data we can read and whether the fifo overflowed
	uint8_t fifoStatus;
	if (!ReadRegister(AdxlRegister::FifoStatus, fifoStatus))
	{
		return 0;
	}

	uint8_t numToRead = fifoStatus & 0x3F;
	if (numToRead != 0)
	{
		if (numToRead >= 31)
		{
			overflowed = true;
			if (numToRead > 32)
			{
				numToRead = 32;
			}
		}
		else
			overflowed = false;
		for(size_t cnt = 0; cnt < numToRead; cnt++)
		{
			// Read the data
			// Note we need a delay of at least 5uS between reads. The end of a read is marked by reading reg 0x38
			if (!ReadRegisters(AdxlRegister::DataX0, 7))
			{
				return 0;
			}
			delayMicroseconds(5);
			data[cnt*3] = *(reinterpret_cast<const uint16_t*>(dataBuffer));
			data[cnt*3 + 1] = *(reinterpret_cast<const uint16_t*>(dataBuffer + 2));
			data[cnt*3 + 2] = *(reinterpret_cast<const uint16_t*>(dataBuffer + 4));
		}
		*collectedData = data;
		const uint32_t interval = lastInterruptTime - firstInterruptTime;
		dataRate = (totalNumRead == 0 || interval == 0)
					? 0
					: (totalNumRead * (uint64_t)StepClockRate)/interval;
		totalNumRead += numToRead;
	}
	return numToRead;
}

// Stop collecting data
void ADXL345::StopCollecting() noexcept
{
	WriteRegister(AdxlRegister::PowerCtrl, 0);
}

// Read registers into dataBuffer
bool ADXL345::ReadRegisters(AdxlRegister reg, size_t numToRead) noexcept
{
	if (!Select(AdxlSpiTimeout))
	{
		return false;
	}
	delayMicroseconds(1);
	// On the ADXL345, bit 6 must be set to 7 to auto-increment the address when doing reading multiple registers
	transferBuffer[1] = (uint8_t)reg | (numToRead > 1 ? 0xC0 : 0x80);
	const bool ret = TransceivePacket(transferBuffer + 1, transferBuffer + 1, 1 + numToRead);
	Deselect();
	return ret;
}

// Write registers from dataBuffer
bool ADXL345::WriteRegisters(AdxlRegister reg, size_t numToWrite) noexcept
{
	if (!Select(AdxlSpiTimeout))
	{
		return false;
	}
	delayMicroseconds(1);
	transferBuffer[1] = (numToWrite < 2) ? (uint8_t)reg : (uint8_t)reg | 0x40;		// set auto increment bit if ADXL345
	const bool ret = TransceivePacket(transferBuffer + 1, transferBuffer + 1, 1 + numToWrite);
	Deselect();
	return ret;
}

bool ADXL345::ReadRegister(AdxlRegister reg, uint8_t& val) noexcept
{
	const bool ret = ReadRegisters(reg, 1);
	if (ret)
	{
		val = dataBuffer[0];
	}
	return ret;
}

bool ADXL345::WriteRegister(AdxlRegister reg, uint8_t val) noexcept
{
	dataBuffer[0] = val;
	return WriteRegisters(reg, 1);
}

void ADXL345::Int1Isr() noexcept
{
	const uint32_t now = StepTimer::GetTimerTicks();
	if (totalNumRead == 0)
	{
		firstInterruptTime = now;
	}
	lastInterruptTime = now;
	TaskBase::GiveFromISR(taskWaiting);
	taskWaiting = nullptr;
}

void Int1Interrupt(CallbackParameter p) noexcept
{
	static_cast<ADXL345*>(p.vp)->Int1Isr();
}

#endif

// End
