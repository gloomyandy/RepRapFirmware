/*
 * ADXL345.cpp
 *
 *  Created on: 5 sep 2022
 *      Author: Andy
 */

#include "ADXL345.h"

#if SUPPORT_ACCELEROMETERS

// Some notes on read timing
// The datasheet for the ADXL345 notes that a delay of at least 5us is needed between
// the end of reading one fifo entry and reading the first byte of the next. It goes on to
// note that at spi speeds below 1.6MHz the sending of the address byte is sufficient for
// this delay. but that at higher speeds a delay may be needed. The datasheet is a little
// ambiguous as it if the CS line needs to be toggled as well. See below for my findings.
//
// The current RRF code (as of 9/9/2022 contains 1us delays in each of the read, select and
// deselct calls. So using ReadRegisters will at the moment always add at least 3us of delays
// plus the time to send the address byte (8/<spi speed>). The current code takes
// these delays into account when calculating the per fifo read delay. 
//
// A number of tests have been conducted using an STM32F407 running at 168MHz and have shown 
// that the current SPI code has approximately 7us of overhead when using hardware spi
// and 4us of overhead when using software spi for a simple read request, with
// no bus speed change, select/deselect calls or the above mentioned delays. On faster mcus 
// (like the STM32H7 running at 480MHz), this overhead will be lower. The current delay
// calculation does not take the spi overhead into account as it will vary from mcu to mcu.
//
// Testing shows that if a sufficient delay is not present then the ADXL345 will return
// zero values rather than valid data. If the reads occur too fast then the expected 
// number of fifo elements will not be removed and this will result in overrun errors being
// reported. My tests indicate that if the CS pin is not toggled between reads then again
// zero values are returned and data overruns will be reported.
//
// Tests using a sample rate of 3200 and spi clock frequency of 0.5 to 6Mhz with the current
// settings show no problems with data overruns.

#include <Hardware/IoPorts.h>
#include <Movement/StepTimer.h>

constexpr uint32_t AdxlSpiTimeout = 25;							// timeout while waiting for the SPI bus
constexpr uint32_t DataCollectionTimeout = (1000 * 32)/400 + 2;	// timeout whole collecting data, enough to fill the FIFO at 400Hz
constexpr uint8_t FifoInterruptLevel = 20;						// how full the FIFO must get before we want an interrupt
const SpiMode adxlMode = SpiMode::mode3;
constexpr int32_t minimumFifoReadDelay = 5;                     // miniumum delay between fifo reads from the datasheet
constexpr int32_t spiReadDelay = 3;								// built in delays in current spi select/read/deselect code

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

	fifoReadDelay = minimumFifoReadDelay > (spiReadDelay + (8*1000000/GetFrequency())) ? minimumFifoReadDelay - (spiReadDelay + (8*1000000/GetFrequency())) : 0;
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
		delayMicroseconds(fifoReadDelay);
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
			delayMicroseconds(fifoReadDelay);
			// Read the data the auto increment from location 0x37 to 0x38 will mark the end of the fifo read
			if (!ReadRegisters(AdxlRegister::DataX0, 6))
			{
				return 0;
			}
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
	// On the ADXL345, bit 7 must be set to 1 to auto-increment the address when doing reading multiple registers
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
