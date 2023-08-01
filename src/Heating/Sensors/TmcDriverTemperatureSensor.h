/*
 * TmcDriverTemperatureSensor.h
 *
 *  Created on: 8 Jun 2017
 *      Author: David
 */

#ifndef SRC_HEATING_SENSORS_TMCDRIVERTEMPERATURESENSOR_H_
#define SRC_HEATING_SENSORS_TMCDRIVERTEMPERATURESENSOR_H_

#include "TemperatureSensor.h"

#if HAS_SMART_DRIVERS

class TmcDriverTemperatureSensor : public TemperatureSensor
{
public:
	TmcDriverTemperatureSensor(unsigned int sensorNum, unsigned int chan) noexcept;

	int GetSmartDriversChannel() const noexcept override { return (int) channel; }		// Get the smart drivers channel that this sensor monitors, or -1 if it doesn't
	void Poll() noexcept override;
	const char *_ecv_array GetShortSensorType() const noexcept override;

	static constexpr const char *_ecv_array PrimaryTypeName = "drivers";
#if defined(DUET_NG) || defined(PCCB_10)
	static constexpr const char *_ecv_array DuexTypeName = "drivers-duex";
	static constexpr const char *_ecv_array DuexTypeShortName = "driversduex";
#endif
#if STM32
	const uint8_t GetNumAdditionalOutputs() const noexcept override { return totalSmartDrivers; }
#endif
private:
	unsigned int channel;
};

#if STM32
#include "AdditionalOutputSensor.h"
// This class represents a DHT humidity sensor
class TmcDriverActualTemperatureSensor : public AdditionalOutputSensor
{
public:
	explicit TmcDriverActualTemperatureSensor(unsigned int sensorNum) noexcept;
	~TmcDriverActualTemperatureSensor() noexcept;

	const char *_ecv_array GetShortSensorType() const noexcept override { return TypeName; }
	void Poll() noexcept override;
	static constexpr const char *_ecv_array TypeName = "drivertemp";
};
#endif

#endif

#endif /* SRC_HEATING_SENSORS_TMCDRIVERTEMPERATURESENSOR_H_ */
