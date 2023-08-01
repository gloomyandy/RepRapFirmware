
#include "SmartDrivers.h"
#include "TmcDriverState.h"
#if HAS_SMART_DRIVERS
#include "TmcDriverState.h"
#include "Platform/Tasks.h"
#if SUPPORT_TMC51xx
#include "TMC51xxDriver.h"
#endif
#if SUPPORT_TMC22xx
#include "TMC22xxDriver.h"
#endif
static TmcDriverState **driverStates;
static size_t numDrivers;
//--------------------------- Public interface ---------------------------------
// Initialise the driver interface and the drivers, leaving each drive disabled.
// It is assumed that the drivers are not powered, so driversPowered(true) must be called after calling this before the motors can be moved.
void SmartDrivers::Init(size_t numSmartDrivers) noexcept
{
    numDrivers = min<size_t>(numSmartDrivers, MaxSmartDrivers);
	if (numDrivers == 0)
		driverStates = nullptr;
	else
		driverStates = (TmcDriverState **)	Tasks::AllocPermanent(sizeof(TmcDriverState *)*numDrivers);
    size_t drive = 0;
    size_t first = 0;
#if SUPPORT_TMC51xx
    Tmc51xxDriver::Init(first, num5160SmartDrivers);
    for(; drive < num5160SmartDrivers; drive++)
        driverStates[drive] = Tmc51xxDriver::GetDrive(drive);
    first = drive;
#endif
#if SUPPORT_TMC22xx
    Tmc22xxDriver::Init(first, numDrivers-first);
    for(; drive < numDrivers; drive++)
        driverStates[drive] = Tmc22xxDriver::GetDrive(drive - first);
#endif
}

// Shut down the drivers and stop any related interrupts. Don't call Spin() again after calling this as it may re-enable them.
void SmartDrivers::Exit() noexcept
{
#if SUPPORT_TMC51xx
    Tmc51xxDriver::Exit();
#endif
#if SUPPORT_TMC22xx
    Tmc22xxDriver::Exit();
#endif
}

void SmartDrivers::Spin(bool powered) noexcept
{
#if SUPPORT_TMC51xx
    Tmc51xxDriver::Spin(powered);
#endif
#if SUPPORT_TMC22xx
    Tmc22xxDriver::Spin(powered);
#endif
}

bool SmartDrivers::IsReady() noexcept
{
#if SUPPORT_TMC51xx
    return Tmc51xxDriver::IsReady()
#else
    return true 
#endif
#if SUPPORT_TMC22xx
    && Tmc22xxDriver::IsReady();
#else
    ;
#endif
}

// This is called from the tick ISR, possibly while Spin (with powered either true or false) is being executed
void SmartDrivers::TurnDriversOff() noexcept
{
#if SUPPORT_TMC51xx
    Tmc51xxDriver::TurnDriversOff();
#endif
#if SUPPORT_TMC22xx
    Tmc22xxDriver::TurnDriversOff();
#endif

}

void SmartDrivers::SetAxisNumber(size_t drive, uint32_t axisNumber) noexcept
{
	if (drive < numDrivers)
	{
		driverStates[drive]->SetAxisNumber(axisNumber);
	}
}

uint32_t SmartDrivers::GetAxisNumber(size_t drive) noexcept
{
	return (drive < numDrivers) ? driverStates[drive]->GetAxisNumber() : 0;
}

void SmartDrivers::SetCurrent(size_t drive, float current) noexcept
{
	if (drive < numDrivers)
	{
		driverStates[drive]->SetCurrent(current);
	}
}

void SmartDrivers::EnableDrive(size_t drive, bool en) noexcept
{
	if (drive < numDrivers)
	{
		driverStates[drive]->Enable(en);
	}
}

StandardDriverStatus SmartDrivers::GetStatus(size_t drive, bool accumulated, bool clearAccumulated) noexcept
{
	if (drive < numDrivers)
		return driverStates[drive]->ReadStatus(accumulated, clearAccumulated);
	StandardDriverStatus rslt;
	rslt.all = 0;
	rslt.notPresent = true;
	return rslt;
}

// Set microstepping or chopper control register
bool SmartDrivers::SetMicrostepping(size_t drive, unsigned int microsteps, bool interpolate) noexcept
{
	if (drive < numDrivers && microsteps > 0)
	{
		// Set the microstepping. We need to determine how many bits right to shift the desired microstepping to reach 1.
		unsigned int shift = 0;
		unsigned int uSteps = (unsigned int)microsteps;
		while ((uSteps & 1) == 0)
		{
			uSteps >>= 1;
			++shift;
		}
		if (uSteps == 1 && shift <= 8)
		{
			driverStates[drive]->SetMicrostepping(shift, interpolate);
			return true;
		}
	}
	return false;
}

// Get microstepping or chopper control register
unsigned int SmartDrivers::GetMicrostepping(size_t drive, bool& interpolation) noexcept
{
	return (drive < numDrivers) ? driverStates[drive]->GetMicrostepping(interpolation) : 1;
}

bool SmartDrivers::SetDriverMode(size_t driver, unsigned int mode) noexcept
{
	return driver < numDrivers && driverStates[driver]->SetDriverMode(mode);
}

DriverMode SmartDrivers::GetDriverMode(size_t driver) noexcept
{
	return (driver < numDrivers) ? driverStates[driver]->GetDriverMode() : DriverMode::unknown;
}

void SmartDrivers::SetStallThreshold(size_t driver, int sgThreshold) noexcept
{
#if HAS_STALL_DETECT
	if (driver < numDrivers)
	{
		driverStates[driver]->SetStallDetectThreshold(sgThreshold);
	}
#endif
}

void SmartDrivers::SetStallFilter(size_t driver, bool sgFilter) noexcept
{
#if HAS_STALL_DETECT
	if (driver < numDrivers)
	{
		driverStates[driver]->SetStallDetectFilter(sgFilter);
	}
#endif
}

void SmartDrivers::SetStallMinimumStepsPerSecond(size_t driver, unsigned int stepsPerSecond) noexcept
{
#if HAS_STALL_DETECT
	if (driver < numDrivers)
	{
		driverStates[driver]->SetStallMinimumStepsPerSecond(stepsPerSecond);
	}
#endif
}

void SmartDrivers::AppendStallConfig(size_t driver, const StringRef& reply) noexcept
{
#if HAS_STALL_DETECT
	if (driver < numDrivers)
	{
		driverStates[driver]->AppendStallConfig(reply);
	}
	else
	{
		reply.cat("no such driver");
	}
#endif
}

void SmartDrivers::AppendDriverStatus(size_t drive, const StringRef& reply) noexcept
{
	if (drive < numDrivers)
	{
		driverStates[drive]->AppendDriverStatus(reply);
	}
}

float SmartDrivers::GetStandstillCurrentPercent(size_t drive) noexcept
{
	return (drive < numDrivers) ? driverStates[drive]->GetStandstillCurrentPercent() : 0.0;
}

void SmartDrivers::SetStandstillCurrentPercent(size_t drive, float percent) noexcept
{
	if (drive < numDrivers)
	{
		driverStates[drive]->SetStandstillCurrentPercent(percent);
	}
}

bool SmartDrivers::SetRegister(size_t driver, SmartDriverRegister reg, uint32_t regVal) noexcept
{
	return (driver < numDrivers) && driverStates[driver]->SetRegister(reg, regVal);
}

uint32_t SmartDrivers::GetRegister(size_t driver, SmartDriverRegister reg) noexcept
{
	return (driver < numDrivers) ? driverStates[driver]->GetRegister(reg) : 0;
}

GCodeResult SmartDrivers::GetAnyRegister(size_t driver, const StringRef& reply, uint8_t regNum) noexcept
{
	if (driver < numDrivers)
	{
		return driverStates[driver]->GetAnyRegister(reply, regNum);
	}
	reply.copy("Invalid smart driver number");
	return GCodeResult::error;
}

GCodeResult SmartDrivers::SetAnyRegister(size_t driver, const StringRef& reply, uint8_t regNum, uint32_t regVal) noexcept
{
	if (driver < numDrivers)
	{
		return driverStates[driver]->SetAnyRegister(reply, regNum, regVal);
	}
	reply.copy("Invalid smart driver number");
	return GCodeResult::error;
}

#if HAS_STALL_DETECT

DriversBitmap SmartDrivers::GetStalledDrivers(DriversBitmap driversOfInterest) noexcept
{
#if SUPPORT_TMC22xx
    return Tmc22xxDriver::GetStalledDrivers(driversOfInterest);
#endif
}
#endif

void SmartDrivers::SetSenseResistor(size_t driver, float value) noexcept
{
	if (driver < numDrivers)
		driverStates[driver]->SetSenseResistor(value);
}

void SmartDrivers::SetMaxCurrent(size_t driver, float value) noexcept
{
	if (driver < numDrivers)
		driverStates[driver]->SetMaxCurrent(value);
}

float SmartDrivers::GetSenseResistor(size_t driver) noexcept
{
	return (driver < numDrivers ? driverStates[driver]->GetSenseResistor() : 0.0f);
}

float SmartDrivers::GetMaxCurrent(size_t driver) noexcept
{
	return (driver < numDrivers ? driverStates[driver]->GetMaxCurrent() : 0.0f);
}

float SmartDrivers::GetTemperature(size_t driver) noexcept
{
	return (driver < numDrivers ? driverStates[driver]->GetTemperature() : 0.0f);
}

#endif

