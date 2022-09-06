/*
 * SpiAccelerometer.h - base class for spi sensors
 *
 *  Created on: 5 Sep 2022
 *      Author: Andy
 */
#include "SpiAccelerometer.h"

#if SUPPORT_ACCELEROMETERS

#include "LIS3DH.h"
#include "ADXL345.h"


SpiAccelerometer::SpiAccelerometer(SharedSpiDevice& dev, uint32_t freq, SpiMode mode, Pin p_csPin, bool polarity) noexcept
	: SharedSpiClient(dev, freq, mode, p_csPin, polarity)
{
}

SpiAccelerometer::~SpiAccelerometer() noexcept
{
}

SpiAccelerometer *SpiAccelerometer::Create(const char *typeName, SharedSpiDevice& dev, uint32_t freq, Pin p_csPin, Pin p_int1Pin, const StringRef& reply) noexcept
{
	SpiAccelerometer *as;

	if (ReducedStringEquals(typeName, LIS3DH::TypeNameLIS3AutoDetect))
	{
		as = new LIS3DH(dev, freq, p_csPin, p_int1Pin, LIS3DH::LisType::automatic);
	}
	else if (ReducedStringEquals(typeName, LIS3DH::TypeNameLIS3DH))
	{
		as = new LIS3DH(dev, freq, p_csPin, p_int1Pin, LIS3DH::LisType::lis3dh);
	}
	else if (ReducedStringEquals(typeName, LIS3DH::TypeNameLIS3DSH))
	{
		as = new LIS3DH(dev, freq, p_csPin, p_int1Pin, LIS3DH::LisType::lis3dsh);
	}
	else if (ReducedStringEquals(typeName, ADXL345::TypeName))
	{
		as = new ADXL345(dev, freq, p_csPin, p_int1Pin);
	}
	else
	{
		as = nullptr;
		reply.printf("Unknown accleration sensor type name \"%s\"", typeName);
	}
	return as;
}

#endif
