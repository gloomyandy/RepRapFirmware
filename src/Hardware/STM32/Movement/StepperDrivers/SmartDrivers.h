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
#include <Endstops/EndstopDefs.h>

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
	float GetStandstillCurrentPercent(size_t driver) noexcept;
	void SetStandstillCurrentPercent(size_t driver, float percent) noexcept;
	bool SetCurrentScaler(size_t driver, int8_t cs) noexcept;
	uint8_t GetIRun(size_t driver) noexcept;
	uint8_t GetIHold(size_t driver) noexcept;
	uint32_t GetGlobalScaler(size_t driver) noexcept;
	float GetCalculatedCurrent(size_t driver) noexcept;
	bool SetRegister(size_t driver, SmartDriverRegister reg, uint32_t regVal) noexcept;
	uint32_t GetRegister(size_t driver, SmartDriverRegister reg) noexcept;
	GCodeResult GetAnyRegister(size_t driver, const StringRef& reply, uint8_t regNum) noexcept;
	GCodeResult SetAnyRegister(size_t driver, const StringRef& reply, uint8_t regNum, uint32_t regVal) noexcept;
	StandardDriverStatus GetStatus(size_t driver, bool accumulated, bool clearAccumulated) noexcept;
	bool IsReady() noexcept;
#if HAS_STALL_DETECT
	void SetStallThreshold(size_t driver, int sgThreshold) noexcept;
	void SetStallFilter(size_t driver, bool sgFilter) noexcept;
	void SetStallMinimumStepsPerSecond(size_t driver, unsigned int stepsPerSecond) noexcept;
	void AppendStallConfig(size_t driver, const StringRef& reply) noexcept;
	void AppendDriverStatus(size_t driver, const StringRef& reply) noexcept;
	EndstopValidationResult CheckStallDetectionEnabled(size_t driver, float speed) noexcept;
	DriversBitmap GetStalledDrivers(DriversBitmap driversOfInterest) noexcept;
#endif
	void SetSenseResistor(size_t driver, float value) noexcept;
	float GetSenseResistor(size_t driver) noexcept;
	void SetMaxCurrent(size_t driver, float value) noexcept;
	float GetMaxCurrent(size_t driver) noexcept;
	float GetDriverTemperature(size_t driver) noexcept;
#if SUPPORT_PHASE_STEPPING
	bool EnablePhaseStepping(size_t driver, bool enable) noexcept;
	bool IsPhaseSteppingEnabled(size_t driver) noexcept;
#endif
#if SUPPORT_PHASE_STEPPING || SUPPORT_CLOSED_LOOP
	float GetCurrent(size_t driver) noexcept;
	unsigned int GetMicrostepShift(size_t driver) noexcept;
	uint16_t GetMicrostepPosition(size_t driver) noexcept;
	bool SetMotorPhases(size_t driver, uint32_t regVal) noexcept;
#endif
};

#endif

#endif /* SRC_MOVEMENT_STEPPERDRIVERS_SMARTDRIVERS_H_ */
