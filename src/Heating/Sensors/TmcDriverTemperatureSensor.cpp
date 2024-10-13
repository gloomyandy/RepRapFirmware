/*
 * TmcDriverTemperatureSensor.cpp
 *
 *  Created on: 8 Jun 2017
 *      Author: David
 */

#include "TmcDriverTemperatureSensor.h"
#include <Movement/Move.h>
#include <Platform/RepRap.h>

#if HAS_SMART_DRIVERS
#if STM32
#include <TMC22xx.h>
#endif

// Sensor type descriptors
TemperatureSensor::SensorTypeDescriptor TmcDriverTemperatureSensor::primaryTmcDriverSensorDescriptor(PrimaryTypeName, [](unsigned int sensorNum) noexcept -> TemperatureSensor *_ecv_from { return new TmcDriverTemperatureSensor(sensorNum, 0); } );

#if defined(DUET_NG) || defined(PCCB_10)
TemperatureSensor:: SensorTypeDescriptor TmcDriverTemperatureSensor::duexTmcDriverSensorDescriptor(DuexTypeName, [](unsigned int sensorNum) noexcept -> TemperatureSensor *_ecv_from { return new TmcDriverTemperatureSensor(sensorNum, 1); } );
#endif

TmcDriverTemperatureSensor::TmcDriverTemperatureSensor(unsigned int sensorNum, unsigned int chan) noexcept
	: TemperatureSensor(sensorNum, "Stepper driver temperature warnings"), channel(chan)
{
}

const char *_ecv_array TmcDriverTemperatureSensor::GetShortSensorType() const noexcept
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
	SetResult(reprap.GetMove().GetTmcDriversTemperature(channel), TemperatureError::ok);
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
