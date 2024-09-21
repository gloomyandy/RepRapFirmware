/*
 * SharedSpiDevice.h
 *
 *  Created on: 16 Jun 2020
 *      Author: David
 */

#ifndef SRC_HARDWARE_SPI_SHAREDSPIDEVICE_H_
#define SRC_HARDWARE_SPI_SHAREDSPIDEVICE_H_

#include "RTOSIface/RTOSIface.h"
#include "SpiDevice.h"
#include <RTOSIface/RTOSIface.h>

class SharedSpiDevice : public SpiDevice
{
public:
#if STM32
	explicit SharedSpiDevice(SSPChannel chan) noexcept;
#else
	explicit SharedSpiDevice(uint8_t sercomNum) noexcept;
#endif

	// Get ownership of this SPI, return true if successful
	bool Take(uint32_t timeout) noexcept { return mutex.Take(timeout); }

	// Release ownership of this SPI
	void Release() noexcept { mutex.Release(); }

	static void Init() noexcept;
#if STM32
	static SharedSpiDevice& GetSharedSpiDevice(SSPChannel chan) noexcept { return (chan < NumSPIDevices) ? *Devices[chan] : *invalidDevice;  }
#else
	static SharedSpiDevice& GetMainSharedSpiDevice() noexcept { return *mainSharedSpiDevice; }
#endif

private:
	Mutex mutex;

#if STM32
	static SharedSpiDevice *Devices[];
	static SharedSpiDevice *invalidDevice;
#else
	static SharedSpiDevice *mainSharedSpiDevice;
#endif
};

#endif /* SRC_HARDWARE_SPI_SHAREDSPIDEVICE_H_ */
