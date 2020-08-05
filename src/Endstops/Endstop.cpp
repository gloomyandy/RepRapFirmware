/*
 * Endstop.cpp
 *
 *  Created on: 4 Apr 2019
 *      Author: David
 */

#include "Endstop.h"
#if SUPPORT_TMC22xx && HAS_STALL_DETECT
# include "Movement/StepperDrivers/TMC22xx.h"

void EndstopOrZProbe::UpdateStalledDriversState(DriversBitmap drivers) noexcept
{
	// Poll all of the drivers we are interested in
	while(drivers.IsNonEmpty())
	{
		const unsigned int index = drivers.LowestSetBit();
		// calling GetLiveStatus will update the stalledDrivers bitmap
		SmartDrivers::GetLiveStatus(index);
		drivers.ClearBit(index);
	}
}
#endif

// Endstop base class
DriversBitmap EndstopOrZProbe::stalledDrivers;			// used to track which drivers are reported as stalled, for stall detect endstops and stall detect Z probes

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(Endstop, __VA_ARGS__)

constexpr ObjectModelTableEntry Endstop::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. Endstop members
	{ "triggered",	OBJECT_MODEL_FUNC(self->Stopped() == EndStopHit::atStop),		 	ObjectModelEntryFlags::live },
	{ "type",		OBJECT_MODEL_FUNC(self->GetEndstopType().ToString()), 				ObjectModelEntryFlags::none },
};

constexpr uint8_t Endstop::objectModelTableDescriptor[] = { 1, 2 };

DEFINE_GET_OBJECT_MODEL_TABLE(Endstop)

#endif

Endstop::Endstop(uint8_t p_axis, EndStopPosition pos) noexcept : axis(p_axis), atHighEnd(pos == EndStopPosition::highEndStop)
{
}

// End
