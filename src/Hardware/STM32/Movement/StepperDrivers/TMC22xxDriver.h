#ifndef TMC22XXDRIVER_H
#define TMC22XXDRIVER_H
#include "TmcDriverState.h"



namespace Tmc22xxDriver
{
	void Init(size_t numDrivers) noexcept;
	void Exit() noexcept;
	void Spin(bool powered) noexcept;
	void TurnDriversOff() noexcept;
	bool IsReady() noexcept;
    TmcDriverState *InitDrive(size_t slot, size_t driveNo) noexcept;
    DriversBitmap GetStalledDrivers(DriversBitmap driversOfInterest) noexcept;
}
#endif