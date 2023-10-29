/*
 * TmcDriverTemperatureSensor.cpp
 *
 *  Created on: 8 Jun 2017
 *      Author: David
 */

#include "TmcDriverTemperatureSensor.h"
#include <Platform/Platform.h>
#include <Platform/RepRap.h>

#if HAS_SMART_DRIVERS
#if STM32
#include <TMC22xx.h>
#endif

TmcDriverTemperatureSensor::TmcDriverTemperatureSensor(unsigned int sensorNum, unsigned int chan) noexcept
	: TemperatureSensor(sensorNum, "Stepper driver temperature warnings"), channel(chan)
{
}

const char *TmcDriverTemperatureSensor::GetShortSensorType() const noexcept
{
#ifdef DUET_NG
	return (channel == 1) ? DuexTypeShortName : PrimaryTypeName;
#else
	return PrimaryTypeName;
#endif
}

void TmcDriverTemperatureSensor::Poll() noexcept
{
#if STM32
	float maxTemp = 0.0f;
	for(size_t drive = 0; drive < totalSmartDrivers; drive++)
		maxTemp = max<float>(maxTemp, SmartDrivers::GetDriverTemperature(drive));
	SetResult(maxTemp, TemperatureError::ok);
#else	
	SetResult(reprap.GetPlatform().GetTmcDriversTemperature(channel), TemperatureError::ok);
#endif
}


#if STM32
// Class TmcDriverActualTemperatureSensor members
TmcDriverActualTemperatureSensor::TmcDriverActualTemperatureSensor(unsigned int sensorNum) noexcept
	: AdditionalOutputSensor(sensorNum, "Stepper driver temperature", false)
{
}

TmcDriverActualTemperatureSensor::~TmcDriverActualTemperatureSensor() noexcept
{
}

void TmcDriverActualTemperatureSensor::Poll() noexcept
{
	SetResult(SmartDrivers::GetDriverTemperature(outputNumber), TemperatureError::ok);
}

#endif
#endif

// End
