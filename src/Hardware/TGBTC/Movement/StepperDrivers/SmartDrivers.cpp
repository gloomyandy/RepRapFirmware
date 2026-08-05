
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

#if HAS_STALL_DETECT && SUPPORT_REMOTE_COMMANDS
# include <CAN/CanInterface.h>
LocalDriversBitmap SmartDrivers::stallEndstopsEnabled;
std::atomic<uint16_t> SmartDrivers::driverStallsToNotify(0);
#endif

static TmcDriverState **driverStates = nullptr;
static size_t numDrivers;

static bool IsDumbDriver(DriverType *dt, size_t driveNo) noexcept
{
	return dt[driveNo] >= DriverType::unknown && dt[driveNo] <= DriverType::stepdir;
}

static bool IsSPIDriver(DriverType *dt, size_t driveNo) noexcept
{
	return (dt[driveNo] >= DriverType::tmcspiauto && dt[driveNo] <= DriverType::tmc2240) || (dt[driveNo] >= DriverType::tmcauto);
}

static bool IsUARTDriver(DriverType *dt, size_t driveNo) noexcept
{
	return (dt[driveNo] >= DriverType::tmcuartauto && dt[driveNo] <= DriverType::tmc2660) || (dt[driveNo] >= DriverType::tmcauto);
}

static bool IsAutoDriver(DriverType *dt, size_t driveNo) noexcept
{
	return dt[driveNo] >= DriverType::tmcauto;
}

#if SUPPORT_TMC51xx
static void ConfigureSPIDrivers(size_t numDrives, DriverType *dt)
{
	size_t cnt = 0;
	for(size_t drive = 0; drive < numDrives; drive++)
	{
		if (IsSPIDriver(dt, drive))
		{
			cnt++;
		}
	}
	Tmc51xxDriver::Init(cnt);
	size_t slot = 0;
	for(size_t drive = 0; drive < numDrives; drive++)
	{
		if (IsSPIDriver(dt, drive))
		{
			driverStates[drive] = Tmc51xxDriver::InitDrive(slot, drive);
			slot++;
		}
	}
	// Run auto detect and complete install
	Tmc51xxDriver::Spin(true);
	Tmc51xxDriver::TurnDriversOff();
	// Allow time for the drivers to be disabled fully
	delay(10);
}
#endif

#if SUPPORT_TMC22xx
static void ConfigureUARTDrivers(size_t numDrives, DriverType *dt)
{
	size_t cnt = 0;
	for(size_t drive = 0; drive < numDrives; drive++)
	{
		if (IsUARTDriver(dt, drive) || IsDumbDriver(dt, drive))
		{
			cnt++;
		}
	}
	Tmc22xxDriver::Init(cnt);
	size_t slot = 0;
	for(size_t drive = 0; drive < numDrives; drive++)
	{
		if (IsUARTDriver(dt, drive) || IsDumbDriver(dt, drive))
		{
			driverStates[drive] = Tmc22xxDriver::InitDrive(slot, drive);
			slot++;
		}
	}
	// Run auto detect and complete install
	Tmc22xxDriver::Spin(true);
	Tmc22xxDriver::TurnDriversOff();
	// Allow time for the drivers to be disabled fully
	delay(10);
}
#endif

//--------------------------- Public interface ---------------------------------
// Initialise the driver interface and the drivers, leaving each drive disabled.
// It is assumed that the drivers are not powered, so driversPowered(true) must be called after calling this before the motors can be moved.
void SmartDrivers::Init(size_t numSmartDrivers) noexcept
{
	uint32_t numDrives = min<size_t>(numSmartDrivers, MaxSmartDrivers);
	DriverType driverType[NumDirectDrivers];
	memcpy((void *)driverType, (void *)TMC_DRIVER_TYPE, NumDirectDrivers*sizeof(DriverType));
	if (driverStates != nullptr)
	{
		delete driverStates;
	}
	if (numDrives == 0)
		driverStates = nullptr;
	else
		driverStates = (TmcDriverState **)	new TmcDriverState*[numDrives];
#if SUPPORT_TMC51xx
	ConfigureSPIDrivers(numDrives, driverType);
	// If we have any "full auto drivers" we need to check if they have been detected as SPI devices, if not
	// we need to remove that device and try it again as a UART device.
	bool reconfigure = false;
	for(size_t drive = 0; drive < numDrives; drive++)
	{
		if (IsAutoDriver(driverType, drive))
		{
			if (driverStates[drive]->GetStatus(false, false).notPresent)
			{
				// mark slot as a possible uart driver
				driverType[drive] = DriverType::tmcuartauto;
				reconfigure = true;
			}
			else
				driverType[drive] = DriverType::tmcspiauto;
		}
	}
	if (reconfigure)
	{
		ConfigureSPIDrivers(numDrives, driverType);
	}
#endif
#if SUPPORT_TMC22xx
	ConfigureUARTDrivers(numDrives, driverType);
#endif
	// make num drivers visible
	numDrivers = numDrives;
}

// Shut down the drivers and stop any related interrupts. Don't call Spin() again after calling this as it may re-enable them.
void SmartDrivers::Exit() noexcept
{
	if (numDrivers == 0)
		return;
#if SUPPORT_TMC51xx
	Tmc51xxDriver::Exit();
#endif
#if SUPPORT_TMC22xx
	Tmc22xxDriver::Exit();
#endif
}

void SmartDrivers::Spin(bool powered) noexcept
{
	if (numDrivers == 0)
		return;
#if SUPPORT_TMC22xx
	Tmc22xxDriver::Spin(powered);
#endif
#if SUPPORT_TMC51xx
	Tmc51xxDriver::Spin(powered);
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

void SmartDrivers::SetDriverType(size_t drive, DriverType typ) noexcept
{
	if (drive < numDrivers)
	{
		uint32_t numDrives = numDrivers;
		// make sure we don't call spin or perform any other requests
		numDrivers = 0;
		// stop eveything
		TurnDriversOff();
		// give them chance to shutdown
		delay(10);
		// set the new driver type
		TMC_DRIVER_TYPE[drive] = typ;
		// and re-init everything
		Init(numDrives);
	}
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
		return driverStates[drive]->GetStatus(accumulated, clearAccumulated);
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

const char *_ecv_array _ecv_null SmartDrivers::CheckStallDetectionEnabled(size_t driver, float speed) noexcept
{
#if HAS_STALL_DETECT
	if (driver < numDrivers)
	{
		return driverStates[driver]->CheckStallDetectionEnabled(speed);
	}
#endif
	return "driver %u does not support stall detection";
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

bool SmartDrivers::SetCurrentScaler(size_t driver, int8_t cs) noexcept
{
	return (driver < numDrivers) ? driverStates[driver]->SetCurrentScaler(cs) : false;
}

uint8_t SmartDrivers::GetIRun(size_t driver) noexcept
{
	return (driver < numDrivers) ? driverStates[driver]->GetIRun() : 0;
}

uint8_t SmartDrivers::GetIHold(size_t driver) noexcept
{
	return (driver < numDrivers) ? driverStates[driver]->GetIHold() : 0;
}

uint32_t SmartDrivers::GetGlobalScaler(size_t driver) noexcept
{
	return (driver < numDrivers) ? driverStates[driver]->GetGlobalScaler() : 0;
}

float SmartDrivers::GetCalculatedCurrent(size_t driver) noexcept
{
	return (driver < numDrivers) ? driverStates[driver]->CalculateCurrent() : 0.0;
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

LocalDriversBitmap SmartDrivers::GetStalledDrivers(LocalDriversBitmap driversOfInterest) noexcept
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

float SmartDrivers::GetDriverTemperature(size_t driver) noexcept
{
	return (driver < numDrivers ? driverStates[driver]->GetDriverTemperature() : 0.0f);
}

uint32_t SmartDrivers::GetDriverMaxClockFrequency(size_t driver) noexcept
{
	return (driver < numDrivers ? driverStates[driver]->GetDriverMaxClockFrequency() : 0);
}

uint32_t SmartDrivers::GetDriverNominalClockFrequency(size_t driver) noexcept
{
	return (driver < numDrivers ? driverStates[driver]->GetDriverNominalClockFrequency() : 0);
}

uint32_t SmartDrivers::GetDriverMinClockFrequency(size_t driver) noexcept
{
	return (driver < numDrivers ? driverStates[driver]->GetDriverMinClockFrequency() : 0);
}

#if SUPPORT_PHASE_STEPPING

bool SmartDrivers::EnablePhaseStepping(size_t driver, bool enable) noexcept
{
	return (driver < numDrivers ? driverStates[driver]->EnablePhaseStepping(enable) : false);
}

bool SmartDrivers::IsPhaseSteppingEnabled(size_t driver) noexcept
{
	return (driver < numDrivers ? driverStates[driver]->IsPhaseSteppingEnabled() : false);
}
#endif
#if SUPPORT_PHASE_STEPPING || SUPPORT_CLOSED_LOOP
// Get the configured motor current in mA
float SmartDrivers::GetCurrent(size_t driver) noexcept
{
	return (driver < numDrivers ? driverStates[driver]->GetCurrent() : 0.0f);
}

// Get the amount we have to shift 1 left by to get the microstepping
unsigned int SmartDrivers::GetMicrostepShift(size_t driver) noexcept
{
	return (driver < numDrivers ? driverStates[driver]->GetMicrostepShift() : 0);
}

// Get the coil A microstep position as a number in the range 0..1023
uint16_t SmartDrivers::GetMicrostepPosition(size_t driver) noexcept
{
	return (driver < numDrivers ? driverStates[driver]->GetMicrostepPosition() : 0);
}

// Schedules a request to update the motor phases using XDIRECT register.
// Returns true if request is scheduled. Will not schedule a request if it is equal to the current value.
bool SmartDrivers::SetMotorPhases(size_t driver, uint32_t regVal) noexcept
{
	return (driver < numDrivers ? driverStates[driver]->SetXdirect(regVal) : false);
}

#endif

#if SUPPORT_REMOTE_COMMANDS
GCodeResult SmartDrivers::SetStallEndstopReporting(uint16_t driver, float speed, const StringRef& reply) noexcept
{
	if (driver < numDrivers)
	{
		const char *_ecv_array _ecv_null const msg = driverStates[driver]->CheckStallDetectionEnabled(speed);
		if (msg == nullptr)
		{
			stallEndstopsEnabled.SetBit(driver);
			return GCodeResult::ok;
		}
		reply.printf(msg, driver);
		return GCodeResult::error;
	}
	else
	{
		stallEndstopsEnabled.Clear();
		driverStallsToNotify = 0;
		return GCodeResult::ok;
	}
}

void SmartDrivers::NotifyStalls() noexcept
{
	const LocalDriversBitmap stalledDrivers = Tmc22xxDriver::GetStalledDrivers(SmartDrivers::stallEndstopsEnabled);
	if (stalledDrivers.IsNonEmpty())
	{
		SmartDrivers::stallEndstopsEnabled &= ~stalledDrivers;
		SmartDrivers::driverStallsToNotify |= stalledDrivers.GetRaw();
		CanInterface::WakeAsyncSender();
	}
}

#endif

#endif

