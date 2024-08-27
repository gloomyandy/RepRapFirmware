/*
 * SharedSpiDevice.hpp
 *
 * LPC version of RRF SharedSpiDevice
 * Author: Andy
 */

#include <Hardware/IoPorts.h>

#include "Core.h"
#include "SoftwareSPI.h"
#include "HardwareSPI.h"
#include <Hardware/Spi/SharedSpiDevice.h>

constexpr uint32_t DefaultSharedSpiClockFrequency = 2000000;
constexpr uint32_t SpiTimeout = 10000;
static const char *names[] = { "SPI0", "SPI1", "SPI2", "SPI3", "SPI4", "SPI5" 
#if STM32H7
								"SPI6", "SPI7", "SPI8"
#endif
								};
// SharedSpiDevice members

SharedSpiDevice::SharedSpiDevice(SSPChannel chan) noexcept
    : SpiDevice(chan)
{
	mutex.Create(names[chan]);
}

// Static members

SharedSpiDevice *SharedSpiDevice::Devices[NumSPIDevices];
SharedSpiDevice *SharedSpiDevice::invalidDevice;

void SharedSpiDevice::Init() noexcept
{
	for(size_t i = 0; i < NumSPIDevices; i++)
		Devices[i] = new SharedSpiDevice((SSPChannel)i);
	invalidDevice = new SharedSpiDevice(SSPNONE);
}

// End
