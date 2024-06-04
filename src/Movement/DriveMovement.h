/*
 * DriveMovement.h
 *
 *  Created on: 17 Jan 2015
 *      Author: David
 */

#ifndef DRIVEMOVEMENT_H_
#define DRIVEMOVEMENT_H_

#include <RepRapFirmware.h>
#include <Platform/Tasks.h>
#include "MoveSegment.h"
#include "ExtruderShaper.h"

class PrepParams;

enum class DMState : uint8_t
{
	idle = 0,

	// All states from stepError1 to just less than firstMotionState must be error states, see function DDA::HasStepEreor
	stepError1,
	stepError2,
	stepError3,

	// All higher values are various states of motion
	firstMotionState,
	starting = firstMotionState,					// interrupt scheduled for when the move should start
	ending,											// interrupt scheduled for when the move should end
	cartAccel,										// linear accelerating motion
	cartLinear,										// linear steady speed
	cartDecelNoReverse,
	cartDecelForwardsReversing,						// linear decelerating motion, expect reversal
	cartDecelReverse,								// linear decelerating motion, reversed
};

// This class describes a single movement of one drive
class DriveMovement
{
public:
	friend class Move;

	DriveMovement() noexcept { }
	void Init(size_t drv) noexcept;

	bool CalcNextStepTime(uint32_t now) noexcept SPEED_CRITICAL;
	void TakenStep() noexcept SPEED_CRITICAL;
	bool PrepareCartesianAxis(const DDA& dda, const PrepParams& params) noexcept SPEED_CRITICAL;
	bool PrepareExtruder(const DDA& dda, const PrepParams& params, float signedEffStepsPerMm) noexcept SPEED_CRITICAL;

	void DebugPrint() const noexcept;
	int32_t GetCurrentMotorPosition() const noexcept { return currentMotorPosition; }
	bool StopDriver(int32_t& netStepsTaken) noexcept;					// if the driver is moving, stop it, update the position and pass back the net steps taken
#if SUPPORT_REMOTE_COMMANDS
	void StopDriverFromRemote() noexcept;
#endif
	int32_t GetNetStepsTaken() const noexcept;							// return the number of steps taken in the current segment
	void SetMotorPosition(int32_t pos) noexcept;
	void AdjustMotorPosition(int32_t adjustment) noexcept;
	bool MotionPending() const noexcept { return segments != nullptr; }
	bool IsPrintingExtruderMovement() const noexcept;					// returns true if this is an extruder executing a printing move
	bool CheckingEndstops() const noexcept;								// returns true when executing a move that checks endstops or Z probe

	void AddSegment(uint32_t startTime, uint32_t duration, float distance, float u, float a, MovementFlags moveFlags) noexcept;
	void SetAsExtruder(bool p_isExtruder) noexcept { isExtruder = p_isExtruder; }

#if HAS_SMART_DRIVERS
	uint32_t GetStepInterval(uint32_t microstepShift) const noexcept;	// Get the current full step interval for this axis or extruder
#endif

	void ClearMovementPending() noexcept;

	bool HasError() const noexcept { return state != DMState::idle && state < DMState::firstMotionState; }

	static int32_t GetAndClearMaxStepsLate() noexcept;
	static int32_t GetAndClearMinStepInterval() noexcept;

private:
	bool CalcNextStepTimeFull(uint32_t now) noexcept SPEED_CRITICAL;
#if SUPPORT_CAN_EXPANSION
	void TakeStepsAndCalcStepTimeRarely(uint32_t clocksNow) noexcept SPEED_CRITICAL;
#endif
	MoveSegment *NewSegment(uint32_t now) noexcept SPEED_CRITICAL;
	bool ScheduleFirstSegment() noexcept;

	void CheckDirection(bool reversed) noexcept;
	void ReleaseSegments() noexcept;					// release the list of segments and set it to nullptr
	bool LogStepError() noexcept;						// tell the Move class that we had a step error

	static int32_t maxStepsLate;
	static int32_t minStepInterval;

	// Parameters common to Cartesian, delta and extruder moves

	DriveMovement *nextDM ;								// link to next DM that needs a step
	MoveSegment *volatile segments;						// pointer to the segment list for this driver

	DDA *homingDda;										// if we are checking endstops then this is the DDA that represents the move
	ExtruderShaper extruderShaper;						// pressure advance control

	DMState state;										// whether this is active or not
	uint8_t drive;										// the drive that this DM controls
	uint8_t direction : 1,								// true=forwards, false=backwards
			directionChanged : 1,						// set by CalcNextStepTime if the direction is changed
			directionReversed : 1,						// true if we have reversed the requested motion direction because of pressure advance
			isExtruder : 1,								// true if this DM is for an extruder
					: 2,								// padding to make the next field last
			stepsTakenThisSegment : 2;					// how many steps we have taken this phase, counts from 0 to 2. Last field in the byte so that we can increment it efficiently.
	uint8_t stepsTillRecalc;							// how soon we need to recalculate

	int32_t netStepsThisSegment;						// the (signed) net number of steps in the current segment
	int32_t segmentStepLimit;							// the first step number of the next phase, or the reverse start step if smaller
	int32_t reverseStartStep;							// the step number for which we need to reverse direction due to pressure advance or delta movement
	float q, t0, p;										// the movement parameters of the current segment
	MovementFlags segmentFlags;							// whether this segment checks endstops etc.
	float distanceCarriedForwards;						// the residual distance in microsteps (less than one) that was pending at the end of the previous segment
	int32_t currentMotorPosition;						// the current motor position in microsteps
#if SUPPORT_CAN_EXPANSION
	int32_t positionAtSegmentStart;						// the value of currentMotorPosition at the start of the current segment
#endif

	// These values change as the segment is executed
	int32_t nextStep;									// number of steps already done. For extruders this gets reset to the net steps already done at the start of each segment, so it can go negative.
	uint32_t nextStepTime;								// when the next step is due
	uint32_t stepInterval;								// how many clocks between steps

	uint32_t driversNormallyUsed;						// the local drivers that this axis or extruder uses
	uint32_t driversCurrentlyUsed;						// the bitmap of local drivers for this axis or extruder that we should step when the next step interrupt is due
	float movementAccumulator = 0.0;					// the accumulated movement since GetAccumulatedMovement was last called. Only used for extruders.
	uint32_t extruderPrintingSince = 0;					// the millis ticks when this extruder started doing printing moves
};

// Update the step counter because we have taken a step
inline void DriveMovement::TakenStep() noexcept
{
	const int32_t adjustment = (direction << 1) - 1;	// to avoid a conditional jump, calculate +1 or -1 according to direction
	currentMotorPosition += adjustment;					// adjust the current position
}

// Calculate and store the time since the start of the move when the next step for the specified DriveMovement is due.
// Return true if there are more steps to do. When finished, leave nextStep == totalSteps + 1 and state == DMState::idle.
// We inline this part to speed things up when we are doing double/quad/octal stepping.
inline bool DriveMovement::CalcNextStepTime(uint32_t now) noexcept
{
	++nextStep;
	if (stepsTillRecalc != 0)
	{
		--stepsTillRecalc;					// we are doing double/quad/octal stepping
		nextStepTime += stepInterval;
#ifdef DUET3_MB6HC							// we need to increase the minimum step pulse length to be long enough for the TMC5160
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
#endif
		return true;
	}
	return CalcNextStepTimeFull(now);
}

// Return the number of net steps already taken for the current segment in the forwards direction.
// We have already taken nextSteps - 1 steps
// Caller must disable interrupts before calling this
inline int32_t DriveMovement::GetNetStepsTaken() const noexcept
{
	if (segments == nullptr)
	{
		return 0;
	}

	const int32_t netStepsTaken = (directionReversed) 								// if started reverse phase
									? nextStep - (2 * reverseStartStep) + 1			// allowing for direction having changed
										: nextStep - 1;
	return (direction) ? netStepsTaken : -netStepsTaken;
}

// Return true if this is an extruder executing a printing move
// Call must disable interrupts before calling this
inline bool DriveMovement::IsPrintingExtruderMovement() const noexcept
{
	return !segmentFlags.nonPrintingMove;
}

inline void DriveMovement::CheckDirection(bool reversed) noexcept
{
	if (reversed != directionReversed)
	{
		directionReversed = !directionReversed;										// this can be done by an xor so hopefully more efficient than assignment
		direction = !direction;
		directionChanged = true;
	}
}

inline int32_t DriveMovement::GetAndClearMaxStepsLate() noexcept
{
	const int32_t ret = maxStepsLate;
	maxStepsLate = 0;
	return ret;
}

inline int32_t DriveMovement::GetAndClearMinStepInterval() noexcept
{
	const int32_t ret = minStepInterval;
	minStepInterval = 0;
	return ret;

}

// Clear any pending movement. This is called for extruders, mostly as an aid to debugging.
// Don't clear the extrusion pending if movement is in progress because this may lead to distanceCarriedForwards becoming out of range, resulting in step errors.
inline void DriveMovement::ClearMovementPending() noexcept
{
	AtomicCriticalSectionLocker lock;
	if (state == DMState::idle)
	{
		distanceCarriedForwards = 0.0;
	}
}

#if HAS_SMART_DRIVERS

// Get the current full step interval for this axis or extruder, or zero if no motion in progress
inline uint32_t DriveMovement::GetStepInterval(uint32_t microstepShift) const noexcept
{
	return (segments == nullptr || (nextStep >> microstepShift) == 0) ? 0
			: stepInterval << microstepShift;									// return the interval between steps converted to full steps
}

#endif

#endif /* DRIVEMOVEMENT_H_ */
