/*******************************************************************************
SPI Driver:  
	Implementation

File Name:
    	CanSpi.cpp

Summary:
    	Implementation of MCU specific SPI functions.

Description:
	SPI Wrapper for MCP2517FD purposes.
 *******************************************************************************/
#include "Platform/RepRap.h"
#if SUPPORT_SPICAN
// Include files
#include "CanSpi.h"
#include "CoreImp.h"
#include "Hardware/Spi/SharedSpiDevice.h"
#include "Hardware/Spi/SharedSpiClient.h"
#include "RTOSIface/RTOSIface.h"
#include "StepTimer.h"

class CanSpiClient
{
public:
    CanSpiClient(SharedSpiDevice& dev, uint32_t freq, SpiMode mode) : spi(dev), freq(freq), mode(mode)
    {
        spi.SetClockFrequencyAndMode(freq, mode);
    }

    bool inline TransceivePacket(const uint8_t *_ecv_array null tx_data, uint8_t *_ecv_array null rx_data, size_t len) const noexcept
    {
        return spi.TransceivePacket(tx_data, rx_data, len, CanCsPin);
    }

    // Get ownership of this SPI, return true if successful
	bool inline Select() noexcept 
    {
        spi.Take(Mutex::TimeoutUnlimited);
        spi.SetClockFrequencyAndMode(freq, mode);
        return true;
    }

	// Release ownership of this SPI
	void inline Deselect() noexcept { spi.Release(); }

    SharedSpiDevice& spi;
    uint32_t freq;
    SpiMode mode;
};  

static CanSpiClient *spiDev;
extern "C" bool DRV_SPI_Initialize()
{
    if (CanCsPin == NoPin || CanSpiChannel == SSPNONE) return false;
	spiDev = new CanSpiClient(SharedSpiDevice::GetSharedSpiDevice(CanSpiChannel), CanSpiFrequency, SpiMode::mode0);
    pinMode(CanCsPin, OUTPUT_HIGH);
    return true;
}

extern "C" void DRV_SPI_Select()
{
    spiDev->Select();
}

extern "C" void DRV_SPI_Deselect()
{
    spiDev->Deselect();
}

extern "C" int8_t DRV_SPI_TransferData(uint32_t index, uint8_t *SpiTxData, uint8_t *SpiRxData, size_t spiTransferSize)
{
    return !spiDev->TransceivePacket((const uint8_t *) SpiTxData, (uint8_t *)SpiRxData, spiTransferSize);
}

extern "C" uint32_t DRV_SPI_GetStepTimerTicks()
{
    return StepTimer::GetTimerTicks();
}
#endif
