/*
 * This is the public interface to TMC Smart drivers. Currently
 * We support TMC22xx and TMC51xx devices. 
 * 
 * Author: GA
 */


#ifndef SRC_MOVEMENT_STEPPERDRIVERS_SMARTDRIVERS_H_
#define SRC_MOVEMENT_STEPPERDRIVERS_SMARTDRIVERS_H_

#include "RepRapFirmware.h"

#if SUPPORT_TMC51xx || SUPPORT_TMC22xx

#include "StepperDrivers/DriverMode.h"

namespace SmartDrivers
{
	void Init(size_t numSmartDrivers) noexcept;
	void Exit() noexcept;
	void Spin(bool powered) noexcept;
	void TurnDriversOff() noexcept;
	bool IsReady() noexcept;

	void SetAxisNumber(size_t driver, uint32_t axisNumber) noexcept;
	uint32_t GetAxisNumber(size_t drive) noexcept;
	void SetCurrent(size_t driver, float current) noexcept;
	void EnableDrive(size_t driver, bool en) noexcept;
	bool SetMicrostepping(size_t drive, unsigned int microsteps, bool interpolation) noexcept;
	unsigned int GetMicrostepping(size_t drive, bool& interpolation) noexcept;
	bool SetDriverMode(size_t driver, unsigned int mode) noexcept;
	DriverMode GetDriverMode(size_t driver) noexcept;
	void SetStallThreshold(size_t driver, int sgThreshold) noexcept;
	void SetStallFilter(size_t driver, bool sgFilter) noexcept;
	void SetStallMinimumStepsPerSecond(size_t driver, unsigned int stepsPerSecond) noexcept;
	void AppendStallConfig(size_t driver, const StringRef& reply) noexcept;
	void AppendDriverStatus(size_t driver, const StringRef& reply) noexcept;
	float GetStandstillCurrentPercent(size_t driver) noexcept;
	void SetStandstillCurrentPercent(size_t driver, float percent) noexcept;
	bool SetRegister(size_t driver, SmartDriverRegister reg, uint32_t regVal) noexcept;
	uint32_t GetRegister(size_t driver, SmartDriverRegister reg) noexcept;
	GCodeResult GetAnyRegister(size_t driver, const StringRef& reply, uint8_t regNum) noexcept;
	GCodeResult SetAnyRegister(size_t driver, const StringRef& reply, uint8_t regNum, uint32_t regVal) noexcept;
	StandardDriverStatus GetStatus(size_t driver, bool accumulated, bool clearAccumulated) noexcept;
	bool IsReady() noexcept;
#if HAS_STALL_DETECT
	DriversBitmap GetStalledDrivers(DriversBitmap driversOfInterest) noexcept;
#endif
	void SetSenseResistor(size_t driver, float value) noexcept;
	float GetSenseResistor(size_t driver) noexcept;
	void SetMaxCurrent(size_t driver, float value) noexcept;
	float GetMaxCurrent(size_t driver) noexcept;
	float GetDriverTemperature(size_t driver) noexcept;
};

#endif

#endif /* SRC_MOVEMENT_STEPPERDRIVERS_SMARTDRIVERS_H_ */
