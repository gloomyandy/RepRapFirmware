/*
 * TMC22xx.cpp
 *
 *  Created on: 23 Jan 2016
 *      Author: David
 * Modified on: 1 Jun 2020 to support TMC2209 (based on Duet expansion board code) on the LPC platform
 *		Author: gloomyandy
 * NOTE: The Duet3d TMC22XX driver now supports TMC2209 devices. However it uses an extra task to run the
 * the driver code which requires an extra 400 bytes of RAM. For now we avoid this by continuing to use
 * Spin to drive the device. We may need to review this at some point.
 */

#include "RepRapFirmware.h"
#if SUPPORT_TMC22xx
#include "TMC22xx.h"
#include "Platform/RepRap.h"
#include <Platform/TaskPriorities.h>
#include "Movement/Move.h"
#include "Movement/StepTimer.h"
#include "Cache.h"
#include <General/Portability.h>
#include "AppNotifyIndices.h"
#if HAS_STALL_DETECT
#include "Endstops/Endstop.h"
#endif
#include "Platform/MessageType.h"
#include "TmcDriverState.h"
#include "DMABitIO.h"
#include "TMC22xxDriver.h"
#include <functional>

#ifndef TMC22xx_HAS_ENABLE_PINS
# error TMC22xx_HAS_ENABLE_PINS not defined
#endif

#ifndef TMC22xx_DEFAULT_STEALTHCHOP
# error TMC22xx_DEFAULT_STEALTHCHOP not defined
#endif

constexpr uint32_t TransferTimeout = 10;				// any transfer should complete within 100 ticks @ 1ms/tick

// Important note:
// The TMC2224 does handle a write request immediately followed by a read request.
// The TMC2224 does _not_ handle back-to-back read requests, it needs a short delay between them.

// Motor current calculations
// Note that we now allow both the sense resistor value and max current to be configued,
// so the following is a default value.
// The TMC datasheet shows an extra 0.02Ohms being added to RSense to account for additional resistance
// On typical TMC2209 driver boards RSense is 0.11Ohms on the Duet it is 0.082 Ohms
// The device can use two current control ranges (controlled by CHOPCONF_VSENSE_HIGH bit), with corresponding
// VRef values.
constexpr float RSense = 0.11; //Ohms
constexpr float RSenseExtra = 0.02; //Ohms
constexpr float VRefVS1 = 180.0; //mV
constexpr float VRefVS0 = 320.0; //mV

// Which gives iMax values in mA of...
constexpr int32_t DefaultiMax_VS1 = (int32_t)((VRefVS1/(RSense + RSenseExtra)) + 0.5);
constexpr int32_t DefaultiMax_VS0 = (int32_t)((VRefVS0/(RSense + RSenseExtra)) + 0.5);

constexpr float MaximumMotorCurrent = DefaultiMax_VS0;
constexpr float MaximumStandstillCurrent = 1400.0;
constexpr float MinimumOpenLoadMotorCurrent = 500;			// minimum current in mA for the open load status to be taken seriously
constexpr uint32_t DefaultMicrosteppingShift = 4;			// x16 microstepping
constexpr bool DefaultInterpolation = true;					// interpolation enabled
constexpr uint32_t DefaultTpwmthrsReg = 2000;				// low values (high changeover speed) give horrible jerk at the changeover from stealthChop to spreadCycle
constexpr uint32_t MaximumWaitTime = 10;					// Wait time for commands we need to complete
constexpr uint16_t DriverNotPresentTimeouts = 10;			// Number of timeouts before we decide to ignore the driver
constexpr size_t TmcTaskStackWords = 140;

#if HAS_STALL_DETECT
const int DefaultStallDetectThreshold = 1;
const unsigned int DefaultMinimumStepsPerSecond = 200;		// for stall detection: 1 rev per second assuming 1.8deg/step, as per the TMC5160 datasheet
#endif

static size_t numTmc22xxDrivers;

enum class DriversState : uint8_t
{
	shutDown = 0,
	noPower,				// no VIN power
	powerWait,				// waiting for power
	noDriver,				// no driver found or configured
	notInitialised,			// have VIN power but not started initialising drivers
	initialising,			// in the process of initialising the drivers
	ready					// drivers are initialised and ready
};

static DriversState driversState = DriversState::shutDown;

// GCONF register (0x00, RW)
constexpr uint8_t REGNUM_GCONF = 0x00;
constexpr uint32_t GCONF_USE_VREF = 1 << 0;					// use external VRef
constexpr uint32_t GCONF_INT_RSENSE = 1 << 1;				// use internal sense resistors
constexpr uint32_t GCONF_SPREAD_CYCLE = 1 << 2;				// use spread cycle mode (else stealthchop mode)
constexpr uint32_t GCONF_REV_DIR = 1 << 3;					// reverse motor direction
constexpr uint32_t GCONF_INDEX_OTPW = 1 << 4;				// INDEX output shows over temperature warning (else it shows first microstep position)
constexpr uint32_t GCONF_INDEX_PULSE = 1 << 5;				// INDEX output shows pulses from internal pulse generator, else as set by GCONF_INDEX_OTPW
constexpr uint32_t GCONF_UART = 1 << 6;						// PDN_UART used for UART interface (else used for power down)
constexpr uint32_t GCONF_MSTEP_REG = 1 << 7;				// microstep resolution set by MSTEP register (else by MS1 and MS2 pins)
constexpr uint32_t GCONF_MULTISTEP_FILT = 1 << 8;			// pulse generation optimised for >750Hz full stepping frequency
constexpr uint32_t GCONF_TEST_MODE = 1 << 9;				// test mode, do not set this bit for normal operation

constexpr uint32_t DefaultGConfReg =
#if TMC22xx_DEFAULT_STEALTHCHOP
									GCONF_UART | GCONF_MSTEP_REG | GCONF_MULTISTEP_FILT;
#else
									GCONF_UART | GCONF_MSTEP_REG | GCONF_MULTISTEP_FILT | GCONF_SPREAD_CYCLE;
#endif


// General configuration and status registers

// GSTAT register (0x01, RW). Write 1 bits to clear the flags.
constexpr uint8_t REGNUM_GSTAT = 0x01;
constexpr uint32_t GSTAT_RESET = 1 << 0;					// driver has been reset since last read
constexpr uint32_t GSTAT_DRV_ERR = 1 << 1;					// driver has been shut down due to over temp or short circuit
constexpr uint32_t GSTAT_UV_CP = 1 << 2;					// undervoltage on charge pump, driver disabled. Not latched so does not need to be cleared.

// IFCOUNT register (0x02, RO)
constexpr uint8_t REGNUM_IFCOUNT = 0x02;
constexpr uint32_t IFCOUNT_MASK = 0x000F;					// interface transmission counter

// SLAVECONF register (0x03, WO)
constexpr uint8_t REGNUM_SLAVECONF = 0x03;
constexpr uint32_t SLAVECONF_SENDDLY_8_BITS = 0 << 8;
constexpr uint32_t SLAVECONF_SENDDLY_24_BITS = 2 << 8;
constexpr uint32_t SLAVECONF_SENDDLY_40_BITS = 4 << 8;
constexpr uint32_t SLAVECONF_SENDDLY_56_BITS = 6 << 8;
constexpr uint32_t SLAVECONF_SENDDLY_72_BITS = 8 << 8;
constexpr uint32_t SLAVECONF_SENDDLY_88_BITS = 10 << 8;
constexpr uint32_t SLAVECONF_SENDDLY_104_BITS = 12 << 8;
constexpr uint32_t SLAVECONF_SENDDLY_120_BITS = 14 << 8;

constexpr uint32_t DefaultSlaveConfReg = SLAVECONF_SENDDLY_8_BITS;	// we don't need any delay between transmission and reception

// OTP_PROG register (0x04, WO)
constexpr uint8_t REGNUM_OTP_PROG = 0x04;
constexpr uint32_t OTP_PROG_BIT_SHIFT = 0;
constexpr uint32_t OTP_PROG_BIT_MASK = 7 << OTP_PROG_BIT_SHIFT;
constexpr uint32_t OTP_PROG_BYTE_SHIFT = 4;
constexpr uint32_t OTP_PROG_BYTE_MASK = 3 << OTP_PROG_BYTE_SHIFT;
constexpr uint32_t OTP_PROG_MAGIC = 0xBD << 8;

// OTP_READ register (0x05, RO)
constexpr uint8_t REGNUM_OTP_READ = 0x05;
constexpr uint32_t OTP_READ_BYTE0_SHIFT = 0;
constexpr uint32_t OTP_READ_BYTE0_MASK = 0xFF << OTP_READ_BYTE0_SHIFT;
constexpr uint32_t OTP_READ_BYTE1_SHIFT = 8;
constexpr uint32_t OTP_READ_BYTE1_MASK = 0xFF << OTP_READ_BYTE1_SHIFT;
constexpr uint32_t OTP_READ_BYTE2_SHIFT = 16;
constexpr uint32_t OTP_READ_BYTE2_MASK = 0xFF << OTP_READ_BYTE2_SHIFT;

// IOIN register (0x06, RO)
constexpr uint8_t REGNUM_IOIN = 0x06;
constexpr uint32_t IOIN_220x_ENN = 1 << 0;
constexpr uint32_t IOIN_222x_PDN_UART = 1 << 1;
constexpr uint32_t IOIN_220x_MS1 = 1 << 2;
constexpr uint32_t IOIN_222x_SPREAD = 2 << 1;
constexpr uint32_t IOIN_220x_MS2 = 1 << 3;
constexpr uint32_t IOIN_222x_DIR = 1 << 3;
constexpr uint32_t IOIN_220x_DIAG = 1 << 4;
constexpr uint32_t IOIN_222x_ENN = 1 << 4;
constexpr uint32_t IOIN_222x_STEP = 1 << 5;
constexpr uint32_t IOIN_220x_PDN_UART = 1 << 6;
constexpr uint32_t IOIN_222x_MS1 = 1 << 6;
constexpr uint32_t IOIN_220x_STEP = 1 << 7;
constexpr uint32_t IOIN_222x_MS2 = 1 << 7;
constexpr uint32_t IOIN_IS_220x = 1 << 8;					// 1 if TMC220x, 0 if TMC222x
constexpr uint32_t IOIN_2209_SPREAD_EN = 1 << 8;
constexpr uint32_t IOIN_220x_DIR = 1 << 9;
constexpr uint32_t IOIN_VERSION_SHIFT = 24;
constexpr uint32_t IOIN_VERSION_MASK = 0xFF << IOIN_VERSION_SHIFT;

constexpr uint32_t IOIN_VERSION_2208_2224 = 0x20;			// version for TMC2208/2224
constexpr uint32_t IOIN_VERSION_2209 = 0x21;				// version for TMC2209/2226

// FACTORY_CONF register (0x07, RW)
constexpr uint8_t REGNUM_FACTORY_CONF = 0x07;
constexpr uint32_t FACTORY_CONF_FCLKTRIM_SHIFT = 0;
constexpr uint32_t FACTORY_CONF_FCLKTRIM_MASK = 0x0F << FACTORY_CONF_FCLKTRIM_SHIFT;
constexpr uint32_t FACTORY_CONF_OTTRIM_SHIFT = 8;
constexpr uint32_t FACTORY_CONF_OTTRIM_MASK = 0x03 << FACTORY_CONF_OTTRIM_SHIFT;
constexpr uint32_t FACTORY_CONF_OTTRIM_143_120 = 0x00 << FACTORY_CONF_OTTRIM_SHIFT;
constexpr uint32_t FACTORY_CONF_OTTRIM_150_120 = 0x01 << FACTORY_CONF_OTTRIM_SHIFT;
constexpr uint32_t FACTORY_CONF_OTTRIM_150_143 = 0x02 << FACTORY_CONF_OTTRIM_SHIFT;
constexpr uint32_t FACTORY_CONF_OTTRIM_157_143 = 0x03 << FACTORY_CONF_OTTRIM_SHIFT;

// Velocity dependent control registers

// IHOLD_IRUN register (WO)
constexpr uint8_t REGNUM_IHOLDIRUN = 0x10;
constexpr uint32_t IHOLDIRUN_IHOLD_SHIFT = 0;				// standstill current
constexpr uint32_t IHOLDIRUN_IHOLD_MASK = 0x1F << IHOLDIRUN_IHOLD_SHIFT;
constexpr uint32_t IHOLDIRUN_IRUN_SHIFT = 8;
constexpr uint32_t IHOLDIRUN_IRUN_MASK = 0x1F << IHOLDIRUN_IRUN_SHIFT;
constexpr uint32_t IHOLDIRUN_IHOLDDELAY_SHIFT = 16;
constexpr uint32_t IHOLDIRUN_IHOLDDELAY_MASK = 0x0F << IHOLDIRUN_IHOLDDELAY_SHIFT;

constexpr uint32_t DefaultIholdIrunReg = (0 << IHOLDIRUN_IHOLD_SHIFT) | (0 << IHOLDIRUN_IRUN_SHIFT) | (2 << IHOLDIRUN_IHOLDDELAY_SHIFT);
															// approx. 0.5 sec motor current reduction to low power

constexpr uint8_t REGNUM_TPOWER_DOWN = 0x11;	// wo, 8 bits, sets delay from standstill detection to motor current reduction
constexpr uint8_t REGNUM_TSTEP = 0x12;			// ro, 20 bits, measured time between two 1/256 microsteps, in clocks
constexpr uint8_t REGNUM_TPWMTHRS = 0x13;		// wo, 20 bits, upper velocity for StealthChop mode
constexpr uint8_t REGNUM_VACTUAL = 0x22;		// wo, 24 bits signed, sets motor velocity for continuous rotation

// Stallguard registers (TMC2209 only)
constexpr uint8_t REGNUM_TCOOLTHRS = 0x14;		// wo, 20-bit lower threshold velocity. CoolStep and the StallGuard DIAG output are enabled above this speed.
constexpr uint8_t REGNUM_SGTHRS = 0x40;			// w0, 8-bit stall detection threshold. Stall is signalled when SG_RESULT <= SGTHRS * 2.
constexpr uint8_t REGNUM_SG_RESULT = 0x41;		// 10-bit StallGard result, read-only. Bits 0 and 9 are always 0.
constexpr uint8_t REGNUM_COOLCONF = 0x42;		// 16-bit CoolStep control

constexpr uint32_t SG_RESULT_MASK = 1023;

// Minimum StallGuard value. Current is increased if SGRESULT < SEMIN * 32.
constexpr unsigned int COOLCONF_SEMIN_SHIFT = 0;
constexpr uint32_t COOLCONF_SEMIN_MASK = 0x000F << COOLCONF_SEMIN_SHIFT;
// Current increment steps per measured SG_RESULT value: 1,2,4,8
constexpr unsigned int COOLCONF_SEUP_SHIFT = 5;
constexpr uint32_t COOLCONF_SEMUP_MASK = 0x0003 << COOLCONF_SEUP_SHIFT;
// Hysteresis value for smart current control. Motor current is reduced if SG_RESULT >= (SEMIN+SEMAX+1)*32.
constexpr unsigned int COOLCONF_SEMAX_SHIFT = 8;
constexpr uint32_t COOLCONF_SEMAX_MASK = 0x000F << COOLCONF_SEMAX_SHIFT;
// Current down step speed. For each {32,8,2,1} SG_RESULT value, decrease by one
constexpr unsigned int COOLCONF_SEDN_SHIFT = 13;
constexpr uint32_t COOLCONF_SEDN_MASK = 0x0003 << COOLCONF_SEDN_SHIFT;
// Minimum current for smart current control, 0 = half of IRUN, 1 = 1/4 of IRUN
constexpr unsigned int COOLCONF_SEIMIN_SHIFT = 15;
constexpr uint32_t COOLCONF_SEIMIN_MASK = 0x0001 << COOLCONF_SEIMIN_SHIFT;

// Sequencer registers (read only)
constexpr uint8_t REGNUM_MSCNT = 0x6A;
constexpr uint8_t REGNUM_MSCURACT = 0x6B;

// Chopper control registers

// CHOPCONF register
constexpr uint8_t REGNUM_CHOPCONF = 0x6C;
constexpr uint32_t CHOPCONF_TOFF_SHIFT = 0;					// off time setting, 0 = disable driver
constexpr uint32_t CHOPCONF_TOFF_MASK = 0x0F << CHOPCONF_TOFF_SHIFT;
constexpr uint32_t CHOPCONF_HSTRT_SHIFT = 4;				// hysteresis start
constexpr uint32_t CHOPCONF_HSTRT_MASK = 0x07 << CHOPCONF_HSTRT_SHIFT;
constexpr uint32_t CHOPCONF_HEND_SHIFT = 7;					// hysteresis end
constexpr uint32_t CHOPCONF_HEND_MASK = 0x0F << CHOPCONF_HEND_SHIFT;
constexpr uint32_t CHOPCONF_TBL_SHIFT = 15;					// blanking time
constexpr uint32_t CHOPCONF_TBL_MASK = 0x03 << CHOPCONF_TBL_SHIFT;
constexpr uint32_t CHOPCONF_VSENSE_HIGH = 1 << 17;			// use high sensitivity current scaling
constexpr uint32_t CHOPCONF_MRES_SHIFT = 24;				// microstep resolution
constexpr uint32_t CHOPCONF_MRES_MASK = 0x0F << CHOPCONF_MRES_SHIFT;
constexpr uint32_t CHOPCONF_INTPOL = 1 << 28;				// use interpolation
constexpr uint32_t CHOPCONF_DEDGE = 1 << 29;				// step on both edges
constexpr uint32_t CHOPCONF_DISS2G = 1 << 30;				// disable short to ground protection
constexpr uint32_t CHOPCONF_DISS2VS = 1 << 31;				// disable low side short protection

constexpr uint32_t DefaultChopConfReg = 0x00000053 | CHOPCONF_VSENSE_HIGH;	// this is the reset default + CHOPCONF_VSENSE_HIGH - CHOPCONF_INTPOL. Try it until we find something better.

// DRV_STATUS register
constexpr uint8_t REGNUM_DRV_STATUS = 0x6F;

// TMC22xx DRV_STATUS register bit assignments
constexpr uint32_t TMC_RR_OT_2209 = 1u << 1;			// over temperature shutdown
constexpr uint32_t TMC_RR_OTPW_2209 = 1u << 0;			// over temperature warning
constexpr uint32_t TMC_RR_S2G_2209 = 3u << 2;			// short to ground indicators (2 bits)
constexpr uint32_t TMC_RR_S2VS_2209 = 3u << 4;			// short to Vs indicators (2 bits)
constexpr uint32_t TMC_RR_OLA_2209 = 1u << 6;			// open load A
constexpr uint32_t TMC_RR_OLB_2209 = 1u << 7;			// open load B
constexpr uint32_t TMC_RR_OPW_2209_120 = 1u << 8;		// temperature threshold exceeded
constexpr uint32_t TMC_RR_OPW_2209_143 = 1u << 9;		// temperature threshold exceeded
constexpr uint32_t TMC_RR_OPW_2209_150 = 1u << 10;		// temperature threshold exceeded
constexpr uint32_t TMC_RR_OPW_2209_157 = 1u << 11;		// temperature threshold exceeded
constexpr uint32_t TMC_RR_TEMPBITS_2209 = 15u << 8;		// all temperature threshold bits
constexpr uint32_t TMC_RR_RESERVED_2209 = (15u << 12) | (0x01FF << 21);	// reserved bits

constexpr uint32_t TMC_RR_STST = 1u << 31;				// standstill detected (same bit on 2209 and 2240)
constexpr unsigned int TMC_RR_STST_BIT_POS = 31;		// same position for 2209 and 2240

// PWMCONF register
constexpr uint8_t REGNUM_PWMCONF = 0x70;

constexpr uint32_t DefaultPwmConfReg = 0xC10D0024;		// this is the reset default - try it until we find something better

constexpr uint8_t REGNUM_PWM_SCALE = 0x71;
constexpr uint8_t REGNUM_PWM_AUTO = 0x72;

static constexpr uint32_t MaxValidSgLoadRegister = 1023;
static constexpr uint32_t InvalidSgLoadRegister = 1024;

// Send/receive data and CRC stuff

// Data format to write a driver register:
// Byte 0 sync byte, 0x05 (4 LSBs are don't cares but included in CRC)
// Byte 1 slave address, 0x00
// Byte 2 register register address to write | 0x80
// Bytes 3-6 32-bit data, MSB first
// Byte 7 8-bit CRC of bytes 0-6

// Data format to read a driver register:
// Byte 0 sync byte, 0x05 (4 LSBs are don't cares but included in CRC)
// Byte 1 slave address, 0x00
// Byte 2 register address to read (top bit clear)
// Byte 3 8-bit CRC of bytes 0-2

// Reply to a read request:
// Byte 0 sync byte, 0x05
// Byte 1 master address, 0xFF
// Byte 2 register address (top bit clear)
// Bytes 3-6 32-bit data, MSB first
// Byte 7 8-bit CRC

// Fast table-driven CRC-8. The result after we have taken the CRC of all bytes needs to be reflected.
// The CRC polynomial used by the TMC drivers is: X^8 + X^2 + X + 1 which is CRC-8-CCITT
static constexpr uint8_t crc_table[256] =
{
	0x00, 0x91, 0xE3, 0x72, 0x07, 0x96, 0xE4, 0x75, 0x0E, 0x9F, 0xED, 0x7C, 0x09, 0x98, 0xEA, 0x7B,
	0x1C, 0x8D, 0xFF, 0x6E, 0x1B, 0x8A, 0xF8, 0x69, 0x12, 0x83, 0xF1, 0x60, 0x15, 0x84, 0xF6, 0x67,
	0x38, 0xA9, 0xDB, 0x4A, 0x3F, 0xAE, 0xDC, 0x4D, 0x36, 0xA7, 0xD5, 0x44, 0x31, 0xA0, 0xD2, 0x43,
	0x24, 0xB5, 0xC7, 0x56, 0x23, 0xB2, 0xC0, 0x51, 0x2A, 0xBB, 0xC9, 0x58, 0x2D, 0xBC, 0xCE, 0x5F,
	0x70, 0xE1, 0x93, 0x02, 0x77, 0xE6, 0x94, 0x05, 0x7E, 0xEF, 0x9D, 0x0C, 0x79, 0xE8, 0x9A, 0x0B,
	0x6C, 0xFD, 0x8F, 0x1E, 0x6B, 0xFA, 0x88, 0x19, 0x62, 0xF3, 0x81, 0x10, 0x65, 0xF4, 0x86, 0x17,
	0x48, 0xD9, 0xAB, 0x3A, 0x4F, 0xDE, 0xAC, 0x3D, 0x46, 0xD7, 0xA5, 0x34, 0x41, 0xD0, 0xA2, 0x33,
	0x54, 0xC5, 0xB7, 0x26, 0x53, 0xC2, 0xB0, 0x21, 0x5A, 0xCB, 0xB9, 0x28, 0x5D, 0xCC, 0xBE, 0x2F,
	0xE0, 0x71, 0x03, 0x92, 0xE7, 0x76, 0x04, 0x95, 0xEE, 0x7F, 0x0D, 0x9C, 0xE9, 0x78, 0x0A, 0x9B,
	0xFC, 0x6D, 0x1F, 0x8E, 0xFB, 0x6A, 0x18, 0x89, 0xF2, 0x63, 0x11, 0x80, 0xF5, 0x64, 0x16, 0x87,
	0xD8, 0x49, 0x3B, 0xAA, 0xDF, 0x4E, 0x3C, 0xAD, 0xD6, 0x47, 0x35, 0xA4, 0xD1, 0x40, 0x32, 0xA3,
	0xC4, 0x55, 0x27, 0xB6, 0xC3, 0x52, 0x20, 0xB1, 0xCA, 0x5B, 0x29, 0xB8, 0xCD, 0x5C, 0x2E, 0xBF,
	0x90, 0x01, 0x73, 0xE2, 0x97, 0x06, 0x74, 0xE5, 0x9E, 0x0F, 0x7D, 0xEC, 0x99, 0x08, 0x7A, 0xEB,
	0x8C, 0x1D, 0x6F, 0xFE, 0x8B, 0x1A, 0x68, 0xF9, 0x82, 0x13, 0x61, 0xF0, 0x85, 0x14, 0x66, 0xF7,
	0xA8, 0x39, 0x4B, 0xDA, 0xAF, 0x3E, 0x4C, 0xDD, 0xA6, 0x37, 0x45, 0xD4, 0xA1, 0x30, 0x42, 0xD3,
	0xB4, 0x25, 0x57, 0xC6, 0xB3, 0x22, 0x50, 0xC1, 0xBA, 0x2B, 0x59, 0xC8, 0xBD, 0x2C, 0x5E, 0xCF
};

// Add a byte to a CRC
static inline constexpr uint8_t CRCAddByte(uint8_t crc, uint8_t currentByte) noexcept
{
	return crc_table[crc ^ currentByte];
}

// Version of Reflect that can be declared constexpr so that we can use it in a static_assert
static inline constexpr uint8_t SlowReflect(uint8_t b) noexcept
{
	b = (b & 0b11110000) >> 4 | (b & 0b00001111) << 4;
	b = (b & 0b11001100) >> 2 | (b & 0b00110011) << 2;
	b = (b & 0b10101010) >> 1 | (b & 0b01010101) << 1;
	return b;
}

// Reverse the order of the bits
static inline uint8_t Reflect(uint8_t b) noexcept
{
#if SAMC21
	return SlowReflect(b);
#else
	uint32_t temp = b;
	asm("rbit %0,%1" : "=r" (temp) : "r" (temp));
	return temp >> 24;
#endif
}

static inline constexpr uint8_t CRCAddFinalByte(uint8_t crc, uint8_t finalByte) noexcept
{
	return SlowReflect(CRCAddByte(crc, finalByte));
}

static_assert(CRCAddFinalByte(CRCAddByte(CRCAddByte(0, 1), 2), 3) == 0x1E);

// CRC of the first 2 bytes we send in any request
static constexpr uint8_t InitialSendCRC = CRCAddByte(CRCAddByte(0, 0x05), 0x00);

// CRC of a request to read the IFCOUNT register
static constexpr uint8_t ReadIfcountCRC = CRCAddFinalByte(InitialSendCRC, REGNUM_IFCOUNT);

// CRC of the first two bytes we receive in any reply
static constexpr uint8_t InitialReceiveCrc = CRCAddByte(CRCAddByte(0, 0x05), 0xFF);

//----------------------------------------------------------------------------------------------------------------------------------
// Private types and methods
class Tmc22xxDriverState: public TmcDriverState
{
public:
	Tmc22xxDriverState() noexcept;
	void Init(uint8_t p_driverNumber, Pin p_enablePin
#if HAS_STALL_DETECT
							, Pin p_diagPin
#endif
			 ) noexcept;
	void SetAxisNumber(size_t p_axisNumber) noexcept;
	uint32_t GetAxisNumber() const noexcept { return axisNumber; }
	void WriteAll() noexcept;
	bool SetMicrostepping(uint8_t shift, bool interpolate) noexcept;
	unsigned int GetMicrostepping(bool& interpolation) const noexcept;
#if SUPPORT_PHASE_STEPPING || SUPPORT_CLOSED_LOOP
	unsigned int GetMicrostepShift() const noexcept { return 0; }
	uint16_t GetMicrostepPosition() const noexcept { return 0; }
	bool SetXdirect(uint32_t regVal) noexcept {return false;}
	uint32_t GetPhaseToSet() const noexcept { return 0; }
	float GetCurrent() const noexcept { return (float)0.0f; }
#endif	
#if SUPPORT_PHASE_STEPPING
	bool EnablePhaseStepping(bool enable) noexcept { return false; }
	bool IsPhaseSteppingEnabled() const noexcept { return false; }
#endif
	bool SetDriverMode(unsigned int mode) noexcept;
	DriverMode GetDriverMode() const noexcept;
	void SetCurrent(float current) noexcept;
	void Enable(bool en) noexcept;
#if HAS_STALL_DETECT
	void SetStallDetectThreshold(int sgThreshold) noexcept;
	void SetStallMinimumStepsPerSecond(unsigned int stepsPerSecond) noexcept;
	void SetStallDetectFilter(bool sgFilter) noexcept {};
	void AppendStallConfig(const StringRef& reply) const noexcept;
#endif
	StandardDriverStatus GetStatus(bool accumulated, bool clearAccumulated) noexcept;
	float GetSenseResistor() const noexcept;
	void SetSenseResistor(float value) noexcept;
	float GetMaxCurrent() const noexcept;
	void SetMaxCurrent(float value) noexcept;
	void AppendDriverStatus(const StringRef& reply) noexcept;
	float GetDriverTemperature() noexcept;
	uint8_t GetDriverNumber() const noexcept { return driverNumber; }
	bool UpdatePending() const noexcept;
	
	bool SetRegister(SmartDriverRegister reg, uint32_t regVal) noexcept;
	uint32_t GetRegister(SmartDriverRegister reg) const noexcept;

	GCodeResult GetAnyRegister(const StringRef& reply, uint8_t regNum) noexcept;
	GCodeResult SetAnyRegister(const StringRef& reply, uint8_t regNum, uint32_t regVal) noexcept;

	float GetStandstillCurrentPercent() const noexcept;
	void SetStandstillCurrentPercent(float percent) noexcept;

	void TransferDone() noexcept __attribute__ ((hot));				// called by the ISR when the SPI transfer has completed
	bool StartTransfer() noexcept __attribute__ ((hot));				// called to start a transfer
	void TransferTimedOut() noexcept { ++numTimeouts; }

	DriversState SetupDriver(bool reset) noexcept;
	bool inline IsActive() noexcept {return state >= DriversState::initialising;}
	bool inline IsReady() noexcept {return state == DriversState::ready;}

private:
	bool SetChopConf(uint32_t newVal) noexcept;
	void UpdateRegister(size_t regIndex, uint32_t regVal) noexcept;
	void UpdateChopConfRegister() noexcept;						// calculate the chopper control register and flag it for sending
	void UpdateCurrent() noexcept;
	void UpdateMaxOpenLoadStepInterval() noexcept;
	bool IsTmc2209() const noexcept { return typ == DriverType::tmc2209; }
	int32_t IdentifyDriver() noexcept;
	void ResetReadRegisters() noexcept;
#if HAS_STALL_DETECT
	void ResetLoadRegisters() noexcept
	{
		minSgLoadRegister = InvalidSgLoadRegister;				// value InvalidSgLoadRegister indicates that it hasn't been read
	}
#endif

	bool DMASend(uint8_t regnum, uint32_t outVal) noexcept __attribute__ ((hot));
	bool DMAReceive(uint8_t regnum, uint8_t crc) noexcept __attribute__ ((hot));

#if HAS_STALL_DETECT
	static constexpr unsigned int NumWriteRegisters = 9;		// the number of registers that we write to on a TMC2209
	static constexpr unsigned int NumWriteRegistersNon09 = 6;	// the number of registers that we write to on a TMC2208/2224
#else
	static constexpr unsigned int NumWriteRegisters = 6;		// the number of registers that we write to on a TMC2208/2224
#endif
	static const uint8_t WriteRegNumbers[NumWriteRegisters];	// the register numbers that we write to

	// Write register numbers are in priority order, most urgent first, in same order as WriteRegNumbers
	static constexpr unsigned int WriteGConf = 0;				// microstepping
	static constexpr unsigned int WriteSlaveConf = 1;			// read response timing
	static constexpr unsigned int WriteChopConf = 2;			// enable/disable and microstep setting
	static constexpr unsigned int WriteIholdIrun = 3;			// current setting
	static constexpr unsigned int WritePwmConf = 4;				// read register select, sense voltage high/low sensitivity
	static constexpr unsigned int WriteTpwmthrs = 5;			// upper step rate limit for stealthchop
#if HAS_STALL_DETECT
	static constexpr unsigned int WriteTcoolthrs = 6;			// coolstep and stall DIAG output lower speed threshold
	static constexpr unsigned int WriteSgthrs = 7;				// stallguard threshold
	static constexpr unsigned int WriteCoolconf = 8;			// coolstep configuration
#endif
	static constexpr unsigned int WriteSpecial = NumWriteRegisters;

#if HAS_STALL_DETECT
	static constexpr unsigned int NumReadRegisters = 6;			// the number of registers that we read from on a TMC2209
#else
	static constexpr unsigned int NumReadRegisters = 5;			// the number of registers that we read from on a TMC2208/2224
#endif
	static const uint8_t ReadRegNumbers[NumReadRegisters];		// the register numbers that we read from

	// Read register numbers, in same order as ReadRegNumbers
	static constexpr unsigned int ReadGStat = 0;			// global status
	static constexpr unsigned int ReadDrvStat = 1;			// drive status
	static constexpr unsigned int ReadMsCnt = 2;			// microstep counter
	static constexpr unsigned int ReadPwmScale = 3;			// PWM scaling
	static constexpr unsigned int ReadPwmAuto = 4;			// PWM scaling
#if HAS_STALL_DETECT
	static constexpr unsigned int ReadSgResult = 5;			// stallguard result, TMC2209 only
#endif
	static constexpr unsigned int ReadSpecial = NumReadRegisters;

	volatile uint32_t writeRegisters[NumWriteRegisters + 1];	// the values we want the TMC22xx writable registers to have
	volatile uint32_t readRegisters[NumReadRegisters + 1];	// the last values read from the TMC22xx readable registers
	volatile uint32_t accumulatedDriveStatus;

	uint32_t configuredChopConfReg;							// the configured chopper control register, in the Enabled state, without the microstepping bits

	float motorCurrent;										// the configured motor current
	uint32_t maxOpenLoadStepInterval;						// the maximum step pulse interval for which we consider open load detection to be reliable

	volatile uint16_t registersToUpdate;					// bitmap of register indices whose values need to be sent to the driver chip

#if HAS_STALL_DETECT
	uint16_t minSgLoadRegister;								// the maximum value of the StallGuard bits we read
#endif
	uint8_t axisNumber;									// the axis number of this driver as used to index the DriveMovements in the DDA
	uint8_t microstepShiftFactor;							// how much we need to shift 1 left by to get the current microstepping

	// To write a register, we send one 8-byte packet to write it, then a 4-byte packet to ask for the IFCOUNT register, then we receive an 8-byte packet containing IFCOUNT.
	// This is the message we send - volatile because we care about when it is written
	static volatile uint8_t sendData[12];

	// Buffer for the message we receive when reading data. The first 4 or 12 bytes bytes are our own transmitted data.
	static volatile uint8_t receiveData[20];

	uint16_t readErrors;									// how many read errors we had
	uint16_t writeErrors;									// how many write errors we had
	uint16_t numReads;										// how many successful reads we had
	uint16_t numWrites;										// how many successful writes we had
	uint16_t numTimeouts;									// how many times a transfer timed out
	Pin enablePin;											// the enable pin of this driver, if it has its own
#if HAS_STALL_DETECT
	Pin diagPin;
#endif
	uint8_t driverNumber;									// the number of this driver as addressed by the UART multiplexer
	uint8_t standstillCurrentFraction;						// divide this by 256 to get the motor current standstill fraction
	uint8_t registerToRead;									// the next register we need to read
	uint8_t regnumBeingUpdated;								// which register we are sending
	uint8_t lastIfCount;									// the value of the IFCNT register last time we read it
	DriversState state;
	DriverType typ;
	static const uint8_t ReadRegCRCs[NumReadRegisters];		// CRCs of the messages needed to read the registers
	volatile uint8_t specialReadRegisterNumber;				// the special register number we are reading
	volatile uint8_t specialWriteRegisterNumber;			// the special register number we are writing
	bool enabled;											// true if driver is enabled

	float senseResistor;
	float maxCurrent;
};

// Static data members of class Tmc22xxDriverState

// To write a register, we send one 8-byte packet to write it, then a 4-byte packet to ask for the IFCOUNT register, then we receive an 8-byte packet containing IFCOUNT.
// This is the message we send - volatile because we care about when it is written
volatile uint8_t Tmc22xxDriverState::sendData[12] =
{
	0x05, 0x00,							// sync byte and slave address
	0x00,								// register address and write flag (filled in)
	0x00, 0x00, 0x00, 0x00,				// value to write (if writing), or 1 byte of CRC if read request (filled in)
	0x00,								// CRC of write request (filled in)
	0x05, 0x00,							// sync byte and slave address
	REGNUM_IFCOUNT,						// register we want to read
	ReadIfcountCRC						// CRC
};

// Buffer for the message we receive when reading data. The first 4 or 12 bytes bytes are our own transmitted data.
volatile uint8_t Tmc22xxDriverState::receiveData[20];

constexpr uint8_t Tmc22xxDriverState::WriteRegNumbers[NumWriteRegisters] =
{
	REGNUM_GCONF,
	REGNUM_SLAVECONF,
	REGNUM_CHOPCONF,
	REGNUM_IHOLDIRUN,
	REGNUM_PWMCONF,
	REGNUM_TPWMTHRS,
#if HAS_STALL_DETECT
	// The rest are on TMC2209 only
	REGNUM_TCOOLTHRS,
	REGNUM_SGTHRS,
	REGNUM_COOLCONF
#endif
};

constexpr uint8_t Tmc22xxDriverState::ReadRegNumbers[NumReadRegisters] =
{
	REGNUM_GSTAT,
	REGNUM_DRV_STATUS,
	REGNUM_MSCNT,
	REGNUM_PWM_SCALE,
	REGNUM_PWM_AUTO,
#if HAS_STALL_DETECT
	REGNUM_SG_RESULT					// TMC2209 only
#endif
};

constexpr uint8_t Tmc22xxDriverState::ReadRegCRCs[NumReadRegisters] =
{
	CRCAddFinalByte(InitialSendCRC, ReadRegNumbers[0]),
	CRCAddFinalByte(InitialSendCRC, ReadRegNumbers[1]),
	CRCAddFinalByte(InitialSendCRC, ReadRegNumbers[2]),
	CRCAddFinalByte(InitialSendCRC, ReadRegNumbers[3]),
	CRCAddFinalByte(InitialSendCRC, ReadRegNumbers[4]),
#if HAS_STALL_DETECT
	CRCAddFinalByte(InitialSendCRC, ReadRegNumbers[5])
#endif
};

Tmc22xxDriverState::Tmc22xxDriverState() noexcept : TmcDriverState(), configuredChopConfReg(0),
 motorCurrent(0.0f), maxOpenLoadStepInterval(0), registersToUpdate(0), minSgLoadRegister(0), axisNumber(0), microstepShiftFactor(0)
{
}

inline bool Tmc22xxDriverState::UpdatePending() const noexcept
{
	return registersToUpdate != 0;
}

// Set up the PDC or DMAC to send a register
inline bool Tmc22xxDriverState::DMASend(uint8_t regNum, uint32_t regVal) noexcept
{
	uint8_t crc = InitialSendCRC;
	{
		const uint8_t byte =  regNum | 0x80;
		sendData[2] = byte;
		crc = CRCAddByte(crc, byte);
	}
	{
		const uint8_t byte = (uint8_t)(regVal >> 24);
		sendData[3] = byte;
		crc = CRCAddByte(crc, byte);
	}
	{
		const uint8_t byte = (uint8_t)(regVal >> 16);
		sendData[4] = byte;
		crc = CRCAddByte(crc, byte);
	}
	{
		const uint8_t byte = (uint8_t)(regVal >> 8);
		sendData[5] = byte;
		crc = CRCAddByte(crc, byte);
	}
	{
		const uint8_t byte = (uint8_t)regVal;
		sendData[6] = byte;
		crc = CRCAddByte(crc, byte);
	}
	sendData[7] = Reflect(crc);
	receiveData[12] = 0xAA;
	receiveData[13] = 0x55;
	return TMCSoftUARTTransfer(TMC_PINS[driverNumber], sendData, 12, receiveData + 12, 8, TransferTimeout);
}

// Set up the PDC or DMAC to send a register and receive the status
inline bool Tmc22xxDriverState::DMAReceive(uint8_t regNum, uint8_t crc) noexcept
{
	sendData[2] = regNum;
	sendData[3] = crc;
	receiveData[4] = 0xAA;
	receiveData[5] = 0x55;
	return TMCSoftUARTTransfer(TMC_PINS[driverNumber], sendData, 4, receiveData + 4, 8, TransferTimeout);
}

// Update the maximum step pulse interval at which we consider open load detection to be reliable
void Tmc22xxDriverState::UpdateMaxOpenLoadStepInterval() noexcept
{
	const uint32_t defaultMaxInterval = StepClockRate/MinimumOpenLoadFullStepsPerSec;
	if ((writeRegisters[WriteGConf] & GCONF_SPREAD_CYCLE) != 0)
	{
		maxOpenLoadStepInterval = defaultMaxInterval;
	}
	else
	{
		// In stealthchop mode open load detection in unreliable, so disable it below the speed at which we switch to spreadCycle
		const uint32_t tpwmthrs = writeRegisters[WriteTpwmthrs] & 0x000FFFFF;
		// tpwmthrs is the 20-bit interval between 1/256 microsteps threshold, in clock cycles @ 12MHz.
		// We need to convert it to the interval between full steps, measured in our step clocks, less about 20% to allow some margin.
		// So multiply by the step clock rate divided by 12MHz, also multiply by 256 less 20%.
		constexpr uint32_t conversionFactor = ((256 - 51) * (StepClockRate/1000000))/12;
		const uint32_t fullStepClocks = tpwmthrs * conversionFactor;
		maxOpenLoadStepInterval = min<uint32_t>(fullStepClocks, defaultMaxInterval);
	}
}

// Set a register value and flag it for updating
void Tmc22xxDriverState::UpdateRegister(size_t regIndex, uint32_t regVal) noexcept
{
	{
		AtomicCriticalSectionLocker lock;
		writeRegisters[regIndex] = regVal;
		registersToUpdate |= (1u << regIndex);								// flag it for sending
	}
	if (regIndex == WriteGConf || regIndex == WriteTpwmthrs)
	{
		UpdateMaxOpenLoadStepInterval();
	}
}

// Calculate the chopper control register and flag it for sending
void Tmc22xxDriverState::UpdateChopConfRegister() noexcept
{
	UpdateRegister(WriteChopConf, (enabled) ? configuredChopConfReg : configuredChopConfReg & ~CHOPCONF_TOFF_MASK);
}

// Initialise the state of the driver and its CS pin
void Tmc22xxDriverState::Init(uint8_t p_driverNumber, Pin p_enablePin
#if HAS_STALL_DETECT
							, Pin p_diagPin
#endif
) noexcept
pre(!driversPowered)
{
	driverNumber = p_driverNumber;
	state = DriversState::noPower;
	axisNumber = p_driverNumber;										// assume straight-through axis mapping initially
	enablePin = p_enablePin;											// this is NoPin for the built-in drivers
	IoPort::SetPinMode(p_enablePin, OUTPUT_HIGH);

#if HAS_STALL_DETECT
	diagPin = p_diagPin;
	IoPort::SetPinMode(p_diagPin, INPUT_PULLUP);
#endif
	state = DriversState::noPower;
	enabled = false;
	registersToUpdate = 0;
	specialReadRegisterNumber = specialWriteRegisterNumber = 0xFF;
	motorCurrent = 0.0;
	standstillCurrentFraction = (256 * 3)/4;							// default to 75%
	maxCurrent = MaximumMotorCurrent;
	senseResistor = RSense;
	UpdateRegister(WriteGConf, DefaultGConfReg);
	UpdateRegister(WriteSlaveConf, DefaultSlaveConfReg);
	configuredChopConfReg = DefaultChopConfReg;
	SetMicrostepping(DefaultMicrosteppingShift, DefaultInterpolation);	// this also updates the chopper control register
	UpdateRegister(WriteIholdIrun, DefaultIholdIrunReg);
	UpdateRegister(WritePwmConf, DefaultPwmConfReg);
	UpdateRegister(WriteTpwmthrs, DefaultTpwmthrsReg);
#if HAS_STALL_DETECT
	SetStallDetectThreshold(DefaultStallDetectThreshold);
	SetStallMinimumStepsPerSecond(DefaultMinimumStepsPerSecond);
	UpdateRegister(WriteCoolconf, 0);									// coolStep disabled
#endif

	for (size_t i = 0; i < NumReadRegisters; ++i)
	{
		readRegisters[i] = 0;				// clear all read registers so that we don't use dud values, in particular we don't know the driver type yet
	}
	accumulatedDriveStatus = 0;
	regnumBeingUpdated = 0xFF;
	registerToRead = 0;
	lastIfCount = 0;
	readErrors = writeErrors = numReads = numWrites = numTimeouts = 0;
#if HAS_STALL_DETECT
	ResetLoadRegisters();
#endif
}
// State structures for all drivers
static Tmc22xxDriverState *driverStates;

#if HAS_STALL_DETECT

void Tmc22xxDriverState::SetStallDetectThreshold(int sgThreshold) noexcept
{
	// TMC2209 stall threshold is 0 to 255 with 255 being most sensitive.
	// RRF is normally -63 to 64 with -63 being most sensitive
	// We expand the RRF range but adjust it for TMC2209
	const uint32_t sgthrs = 255 - (uint32_t)(constrain<int>(sgThreshold, -128, 127) + 128);
	UpdateRegister(WriteSgthrs, sgthrs);
}

void Tmc22xxDriverState::SetStallMinimumStepsPerSecond(unsigned int stepsPerSecond) noexcept
{
	UpdateRegister(WriteTcoolthrs, (12000000 + (128 * stepsPerSecond))/(256 * stepsPerSecond));
}

void Tmc22xxDriverState::AppendStallConfig(const StringRef& reply) const noexcept
{
	const int threshold = (int)((255 - writeRegisters[WriteSgthrs]) - 128);
	reply.catf("stall threshold %d, steps/sec %" PRIu32 ", coolstep %" PRIx32,
				threshold, 12000000 / (256 * writeRegisters[WriteTcoolthrs]), writeRegisters[WriteCoolconf] & 0xFFFF);
}

#endif

inline void Tmc22xxDriverState::SetAxisNumber(size_t p_axisNumber) noexcept
{
	axisNumber = p_axisNumber;
}

// Write all registers. This is called when the drivers are known to be powered up.
inline void Tmc22xxDriverState::WriteAll() noexcept
{
	AtomicCriticalSectionLocker lock;
	registersToUpdate = (1 << (IsTmc2209() ? NumWriteRegisters : NumWriteRegistersNon09)) - 1;
}

float Tmc22xxDriverState::GetStandstillCurrentPercent() const noexcept
{
	return (float)(standstillCurrentFraction * 100)/256;
}

void Tmc22xxDriverState::SetStandstillCurrentPercent(float percent) noexcept
{
	standstillCurrentFraction = (uint8_t)constrain<long>(lrintf((percent * 256)/100), 0, 255);
	UpdateCurrent();
}

// Set the microstepping and microstep interpolation. The desired microstepping is (1 << shift).
bool Tmc22xxDriverState::SetMicrostepping(uint8_t shift, bool interpolate) noexcept
{
	microstepShiftFactor = shift;
	configuredChopConfReg = (configuredChopConfReg & ~(CHOPCONF_MRES_MASK | CHOPCONF_INTPOL)) | ((8 - shift) << CHOPCONF_MRES_SHIFT);
	if (interpolate)
	{
		configuredChopConfReg |= CHOPCONF_INTPOL;
	}
	UpdateChopConfRegister();
	return true;
}

// Get microstepping or chopper control register
unsigned int Tmc22xxDriverState::GetMicrostepping(bool& interpolation) const noexcept
{
	interpolation = (writeRegisters[WriteChopConf] & CHOPCONF_INTPOL) != 0;
	return 1u << microstepShiftFactor;
}

bool Tmc22xxDriverState::SetRegister(SmartDriverRegister reg, uint32_t regVal) noexcept
{
	switch(reg)
	{
	case SmartDriverRegister::chopperControl:
		return SetChopConf(regVal);

	case SmartDriverRegister::toff:
		return SetChopConf((configuredChopConfReg & ~CHOPCONF_TOFF_MASK) | ((regVal << CHOPCONF_TOFF_SHIFT) & CHOPCONF_TOFF_MASK));

	case SmartDriverRegister::tblank:
		return SetChopConf((configuredChopConfReg & ~CHOPCONF_TBL_MASK) | ((regVal << CHOPCONF_TBL_SHIFT) & CHOPCONF_TBL_MASK));

	case SmartDriverRegister::hstart:
		return SetChopConf((configuredChopConfReg & ~CHOPCONF_HSTRT_MASK) | ((regVal << CHOPCONF_HSTRT_SHIFT) & CHOPCONF_HSTRT_MASK));

	case SmartDriverRegister::hend:
		return SetChopConf((configuredChopConfReg & ~CHOPCONF_HEND_MASK) | ((regVal << CHOPCONF_HEND_SHIFT) & CHOPCONF_HEND_MASK));

	case SmartDriverRegister::tpwmthrs:
		UpdateRegister(WriteTpwmthrs, regVal & ((1u << 20) - 1));
		return true;

#if HAS_STALL_DETECT
	case SmartDriverRegister::coolStep:
		UpdateRegister(WriteCoolconf, regVal & ((1u << 16) - 1));
		return true;
#endif

	case SmartDriverRegister::hdec:
	default:
		return false;
	}
}

uint32_t Tmc22xxDriverState::GetRegister(SmartDriverRegister reg) const noexcept
{
	switch(reg)
	{
	case SmartDriverRegister::chopperControl:
		return configuredChopConfReg & 0x01FFFF;

	case SmartDriverRegister::toff:
		return (configuredChopConfReg & CHOPCONF_TOFF_MASK) >> CHOPCONF_TOFF_SHIFT;

	case SmartDriverRegister::tblank:
		return (configuredChopConfReg & CHOPCONF_TBL_MASK) >> CHOPCONF_TBL_SHIFT;

	case SmartDriverRegister::hstart:
		return (configuredChopConfReg & CHOPCONF_HSTRT_MASK) >> CHOPCONF_HSTRT_SHIFT;

	case SmartDriverRegister::hend:
		return (configuredChopConfReg & CHOPCONF_HEND_MASK) >> CHOPCONF_HEND_SHIFT;

	case SmartDriverRegister::tpwmthrs:
		return writeRegisters[WriteTpwmthrs] & 0x000FFFFF;

	case SmartDriverRegister::mstepPos:
		return readRegisters[ReadMsCnt];

	case SmartDriverRegister::pwmScale:
		return readRegisters[ReadPwmScale];

	case SmartDriverRegister::pwmAuto:
		return readRegisters[ReadPwmAuto];

#if HAS_STALL_DETECT
	case SmartDriverRegister::coolStep:
		return writeRegisters[WriteCoolconf];
#endif

	case SmartDriverRegister::hdec:
	default:
		return 0;
	}
}

// Read any register from a driver
// This will return GCodeResult:notFinished for at least the first call, so it must be called repeatedly until it returns a different value.
GCodeResult Tmc22xxDriverState::GetAnyRegister(const StringRef& reply, uint8_t regNum) noexcept
{
	if (specialReadRegisterNumber == 0xFE)
	{
		reply.printf("Register 0x%02x value 0x%08" PRIx32, regNum, readRegisters[ReadSpecial]);
		specialReadRegisterNumber = 0xFF;
		return GCodeResult::ok;
	}

	if (specialReadRegisterNumber == 0xFF)
	{
		specialReadRegisterNumber = regNum;
	}
	return GCodeResult::notFinished;
}

GCodeResult Tmc22xxDriverState::SetAnyRegister(const StringRef& reply, uint8_t regNum, uint32_t regVal) noexcept
{
	for (size_t i = 0; i < NumWriteRegisters; ++i)
	{
		if (regNum == WriteRegNumbers[i])
		{
			UpdateRegister(i, regVal);
			return GCodeResult::ok;
		}
	}
	specialWriteRegisterNumber = regNum;
	UpdateRegister(WriteSpecial, regVal);
	return GCodeResult::ok;
}


// Set the chopper control register to the settings provided by the user. We allow only the lowest 17 bits to be set.
bool Tmc22xxDriverState::SetChopConf(uint32_t newVal) noexcept
{
	const uint32_t offTime = (newVal & CHOPCONF_TOFF_MASK) >> CHOPCONF_TOFF_SHIFT;
	if (offTime == 0 || (offTime == 1 && (newVal & CHOPCONF_TBL_MASK) < (2 << CHOPCONF_TBL_SHIFT)))
	{
		return false;
	}
	const uint32_t hstrt = (newVal & CHOPCONF_HSTRT_MASK) >> CHOPCONF_HSTRT_SHIFT;
	const uint32_t hend = (newVal & CHOPCONF_HEND_MASK) >> CHOPCONF_HEND_SHIFT;
	if (hstrt + hend > 16)
	{
		return false;
	}
	const uint32_t userMask = CHOPCONF_TBL_MASK | CHOPCONF_HSTRT_MASK | CHOPCONF_HEND_MASK | CHOPCONF_TOFF_MASK;	// mask of bits the user is allowed to change
	configuredChopConfReg = (configuredChopConfReg & ~userMask) | (newVal & userMask);
	UpdateChopConfRegister();
	return true;
}

// Set the driver mode
bool Tmc22xxDriverState::SetDriverMode(unsigned int mode) noexcept
{
	switch (mode)
	{
	case (unsigned int)DriverMode::spreadCycle:
		UpdateRegister(WriteGConf, writeRegisters[WriteGConf] | GCONF_SPREAD_CYCLE);
		return true;

	case (unsigned int)DriverMode::stealthChop:
		UpdateRegister(WriteGConf, writeRegisters[WriteGConf] & ~GCONF_SPREAD_CYCLE);
		return true;

	default:
		return false;
	}
}

// Get the driver mode
DriverMode Tmc22xxDriverState::GetDriverMode() const noexcept
{
	return ((writeRegisters[WriteGConf] & GCONF_SPREAD_CYCLE) != 0) ? DriverMode::spreadCycle : DriverMode::stealthChop;
}

// Set the motor current
void Tmc22xxDriverState::SetCurrent(float current) noexcept
{
	motorCurrent = static_cast<float>(constrain<float>(current, 50.0, maxCurrent));
	UpdateCurrent();
}

void Tmc22xxDriverState::UpdateCurrent() noexcept
{

	// New calc based on current Duet3D code, not sure about the 0.2 rounding!
	uint32_t vsense = CHOPCONF_VSENSE_HIGH;
	const float DriverFullScaleCurrent = VRefVS1/(senseResistor + RSenseExtra);	// in mA
	const float DriverCsMultiplier = 32.0/DriverFullScaleCurrent;
	float idealIRunCs = DriverCsMultiplier * motorCurrent;
	if ((unsigned int)(idealIRunCs + 0.2) > 32)
	{
		//Can't use high sensitivity scale
		vsense = 0;
		idealIRunCs *= VRefVS1/VRefVS0;
	}
	const uint32_t iRunCsBits = constrain<uint32_t>((unsigned int)(idealIRunCs + 0.2), 1, 32) - 1;
	const float idealIHoldCs = idealIRunCs * standstillCurrentFraction * (1.0/256.0);
	const uint32_t iHoldCsBits = constrain<uint32_t>((unsigned int)(idealIHoldCs + 0.2), 1, 32) - 1;
	if (reprap.Debug(Module::Driver))
		debugPrintf("TMC current set I %d IH %d csBits 0x%x 0x%x vsense 0x%x\n", (int)motorCurrent, (int)idealIHoldCs, (unsigned)iRunCsBits, (unsigned)iHoldCsBits, (unsigned)vsense);

	UpdateRegister(WriteIholdIrun,
					(writeRegisters[WriteIholdIrun] & ~(IHOLDIRUN_IRUN_MASK | IHOLDIRUN_IHOLD_MASK))
					| (iRunCsBits << IHOLDIRUN_IRUN_SHIFT) 
					| (iHoldCsBits << IHOLDIRUN_IHOLD_SHIFT));
	configuredChopConfReg = (configuredChopConfReg & ~CHOPCONF_VSENSE_HIGH) | vsense;
	UpdateChopConfRegister();
}

// Enable or disable the driver
void Tmc22xxDriverState::Enable(bool en) noexcept
{
	if (enabled != en)
	{
		enabled = en;
		if (enablePin != NoPin)
		{
			digitalWrite(enablePin, !en);			// we assume that smart drivers always have active low enables
		}
		UpdateChopConfRegister();
	}
}

float Tmc22xxDriverState::GetSenseResistor() const noexcept
{
	return senseResistor;
}

void Tmc22xxDriverState::SetSenseResistor(float value) noexcept
{
	if (value > 0.0f) senseResistor = value;
	// Max current may have changed due to sense resistor change
	SetMaxCurrent(maxCurrent);
}

float Tmc22xxDriverState::GetMaxCurrent() const noexcept
{
	return maxCurrent;
}

void Tmc22xxDriverState::SetMaxCurrent(float value) noexcept
{
	if (value > 0.0f) maxCurrent = value;
	const int32_t iMax_VS0 = (int32_t)((VRefVS0/(RSense + RSenseExtra)) + 0.5);
	if (maxCurrent > iMax_VS0) maxCurrent = iMax_VS0;
	SetCurrent(motorCurrent);
}

StandardDriverStatus Tmc22xxDriverState::GetStatus(bool accumulated, bool clearAccumulated) noexcept
{
	StandardDriverStatus rslt;
	if (IsReady())
	{
		uint32_t status;
		if (accumulated)
		{
			AtomicCriticalSectionLocker lock;

			status = accumulatedDriveStatus;
			if (clearAccumulated)
			{
				accumulatedDriveStatus = readRegisters[ReadDrvStat];
			}
		}
		else
		{
			status = readRegisters[ReadDrvStat];
			if (!enabled)
			{
				status &= ~(TMC_RR_OLA_2209 | TMC_RR_OLB_2209);
			}
		}
		// The lowest 8 bits of StandardDriverStatus have the same meanings as for the TMC2209 status
		rslt.all = status & 0x000000FF;
		rslt.all |= ExtractBit(status, TMC_RR_STST_BIT_POS, StandardDriverStatus::StandstillBitPos);	// put the standstill bit in the right place
#if HAS_STALL_DETECT
		if (IoPort::ReadPin(diagPin))
		{
			rslt.stall = true;
		}
		rslt.sgresultMin = minSgLoadRegister;
#endif
	}
	else
	{
		rslt.all = 0;
		rslt.notPresent = typ <= DriverType::none;
	}
	return rslt;
}

// Append the driver status to a string, and reset the min/max load values
void Tmc22xxDriverState::AppendDriverStatus(const StringRef& reply) noexcept
{
	if (typ == DriverType::stepdir)
	{
		reply.cat(" stepdir");
		return;
	}
	if (!IsReady())
	{
		return;
	}
	if (IsTmc2209())
		reply.cat(" 2209");
	else
		reply.cat(" 2208");

#if HAS_STALL_DETECT
	if (IsTmc2209())
	{
		if (minSgLoadRegister <= MaxValidSgLoadRegister)
		{
			reply.catf(", SG min %u", minSgLoadRegister);
		}
		else
		{
			reply.cat(", SG min n/a");
		}
	}
	ResetLoadRegisters();
#endif
	reply.catf(", reads %u, writes %u", numReads, numWrites);
	if(readErrors != 0 || writeErrors != 0 || numTimeouts != 0)
		reply.catf(", error r/w %u/%u, ifcnt %u, timeout %u",
						readErrors, writeErrors, lastIfCount, numTimeouts);

	readErrors = writeErrors = numReads = numWrites = numTimeouts = 0;
}

float Tmc22xxDriverState::GetDriverTemperature() noexcept
{
	uint32_t status = readRegisters[ReadDrvStat];

	return (status & TMC_RR_OT_2209 ? 150.0f : status & TMC_RR_OTPW_2209 ? 100.0f : 0.0f);

}

// This is called by the ISR when the SPI transfer has completed
inline void Tmc22xxDriverState::TransferDone() noexcept
{
	if (sendData[2] & 0x80)								// if we were writing a register
	{
		const uint8_t currentIfCount = receiveData[18];
		if (regnumBeingUpdated < NumWriteRegisters 
		    && currentIfCount == (uint8_t)(lastIfCount + 1)
			&& (sendData[2] & 0x7F) == ((regnumBeingUpdated == WriteSpecial) ? specialWriteRegisterNumber : WriteRegNumbers[regnumBeingUpdated])
			&& receiveData[12] == 0x05
			&& receiveData[13] == 0xFF
			&& Reflect(CRCAddByte(CRCAddByte(CRCAddByte(CRCAddByte(CRCAddByte(InitialReceiveCrc, receiveData[14]), receiveData[15]), receiveData[16]), receiveData[17]), receiveData[18])) == receiveData[19]
		   )
		{
			++numWrites;
			{
				AtomicCriticalSectionLocker lock;
				registersToUpdate &= ~(1u << regnumBeingUpdated);
				// The value to be written may have changed since we sent it, so check that we wrote the latest data
				if (LoadBEU32(const_cast<const uint8_t *>(sendData + 3)) != writeRegisters[regnumBeingUpdated])
				{
					registersToUpdate |= 1u << regnumBeingUpdated;
				}
			}
		}
		else
		{
			// mark this to retry
			{
				AtomicCriticalSectionLocker lock;

				registersToUpdate |= (1u << regnumBeingUpdated);
			}
			++writeErrors;
		}
		lastIfCount = currentIfCount;
		regnumBeingUpdated = 0xFF;
	}
	else if (driversState != DriversState::noPower)		// we don't check the CRC, so only accept the result if power is still good
	{
		const uint8_t readRegNumber = (registerToRead >= NumReadRegisters) ? specialReadRegisterNumber
												: ReadRegNumbers[registerToRead];
		if (sendData[2] == readRegNumber
		    && readRegNumber == receiveData[6]
			&& receiveData[4] == 0x05
			&& receiveData[5] == 0xFF
			&& Reflect(CRCAddByte(CRCAddByte(CRCAddByte(CRCAddByte(CRCAddByte(InitialReceiveCrc, receiveData[6]), receiveData[7]), receiveData[8]), receiveData[9]), receiveData[10])) == receiveData[11]
		   )
		{
			// We asked to read the scheduled read register, and the sync byte, slave address and register number in the received message match
			//TODO here we could check the CRC of the received message, but for now we assume that we won't get any corruption in the 32-bit received data
			uint32_t regVal = ((uint32_t)receiveData[7] << 24) | ((uint32_t)receiveData[8] << 16) | ((uint32_t)receiveData[9] << 8) | receiveData[10];

			if (registerToRead == ReadDrvStat)
			{
				uint32_t interval;
				if (   (regVal & TMC_RR_STST) != 0
					|| (interval = reprap.GetMove().GetStepInterval(axisNumber, microstepShiftFactor)) == 0		// get the full step interval
					|| interval > maxOpenLoadStepInterval
					|| motorCurrent < MinimumOpenLoadMotorCurrent
				   )
				{
					regVal &= ~(TMC_RR_OLA_2209 | TMC_RR_OLB_2209);				// open load bits are unreliable at standstill and low speeds
				}
			}
#if HAS_STALL_DETECT
			else if (registerToRead == ReadSgResult)
			{
				const uint16_t sgResult = regVal & SG_RESULT_MASK;
				if (sgResult < minSgLoadRegister)
				{
					minSgLoadRegister = sgResult;
				}
			}
#endif
			readRegisters[registerToRead] = regVal;
			if (registerToRead == ReadSpecial)
			{
				specialReadRegisterNumber = 0xFE;						// set it to 0xFE to indicate that we have read it and to prevent it being read again
				registerToRead = 0;
			}
			else
			{
				if (registerToRead == ReadDrvStat)
				{
					accumulatedDriveStatus |= regVal;
				}
				++registerToRead;
				if (registerToRead == ReadSpecial && specialReadRegisterNumber >= 0x80)
				{
					registerToRead = 0;
				}
			}
			++numReads;
		}
		else
		{
			++readErrors;
		}
	}
}


// This is called from the ISR or elsewhere to start a new SPI transfer. Inlined for ISR speed.
inline bool Tmc22xxDriverState::StartTransfer() noexcept
{
	// Find which register to send. The common case is when no registers need to be updated.
	if (registersToUpdate != 0)
	{
		// Write a register
		const size_t regNum = LowestSetBit(registersToUpdate);
		// Kick off a transfer for the register to write
		uint8_t regNumber;
		uint32_t regData;
		{
			AtomicCriticalSectionLocker lock;
			regnumBeingUpdated = regNum;
			regNumber = (regNum < WriteSpecial) ? WriteRegNumbers[regNum] : specialWriteRegisterNumber;
			regData = writeRegisters[regNum];
		}
		return DMASend(regNumber, regData);	// set up the PDC
	}
	else
	{
		// Read a register
		regnumBeingUpdated = 0xFF;
		if (registerToRead >= NumReadRegisters)
		{
			return DMAReceive(specialReadRegisterNumber, CRCAddFinalByte(InitialSendCRC, specialReadRegisterNumber));		
		}
		else
		{
			return DMAReceive(ReadRegNumbers[registerToRead], ReadRegCRCs[registerToRead]);	// set up the PDC
		}
	}
}

int32_t Tmc22xxDriverState::IdentifyDriver() noexcept
{
	if (specialReadRegisterNumber == 0xFE)
	{
		specialReadRegisterNumber = 0xFF;
		return (readRegisters[ReadSpecial] & IOIN_VERSION_MASK) >> IOIN_VERSION_SHIFT;
	}

	if (specialReadRegisterNumber == 0xFF)
	{
		// Force next read to be this one
		registerToRead = ReadSpecial;
		specialReadRegisterNumber = REGNUM_IOIN;
	}
	return 0xffffffff;
}

void Tmc22xxDriverState::ResetReadRegisters() noexcept
{
	for (size_t i = 0; i < NumReadRegisters; ++i)
	{
		readRegisters[i] = 0;				// clear all read registers so that we don't use dud values, in particular we don't know the driver type yet
	}
	accumulatedDriveStatus = 0;
}

DriversState Tmc22xxDriverState::SetupDriver(bool reset) noexcept
{
	// Step the driver through the setup process and report current state
	//debugPrintf("Setup driver %d cnt %d/%d", GetDriverNumber(), numReads, numWrites);
	if (reset)
	{
		if (TMC_PINS[driverNumber] == NoPin || TMC_DRIVER_TYPE[driverNumber] <= DriverType::stepdir)
		{
			state = DriversState::noDriver;
			typ = TMC_DRIVER_TYPE[driverNumber] == DriverType::stepdir ? DriverType::stepdir : DriverType::none;
		}
		else
		{
			// set initial state send updates and read registers
			readErrors = writeErrors = numReads = numWrites = numTimeouts = 0;
			ResetReadRegisters();
			typ = DriverType::unknown;
			registersToUpdate = 0;
			IdentifyDriver();
			state = DriversState::initialising;
		}
		return state;
	}
	if (state == DriversState::noDriver)
	{
		return state;
	}
	// check for device not present
	if (numTimeouts > DriverNotPresentTimeouts || readErrors > DriverNotPresentTimeouts)
	{
		//debugPrintf(" disabling driver %d\n", driverNumber);
		if (reprap.Debug(Module::Driver))
			debugPrintf("TMCUART: Too many errors drive %d driver disabled\n", driverNumber);
		ResetReadRegisters();
		state = DriversState::noDriver;
		typ = DriverType::none;
		return state;
	}

	// Idenitfy device during initial reads
	if (typ == DriverType::unknown)
	{
		int32_t version = IdentifyDriver();
		if (version >= 0)
		{
			if (numReads < 3)
			{
				// Start another read
				IdentifyDriver();
			}
			else
			{
				if (version == IOIN_VERSION_2209)
					typ = DriverType::tmc2209;
				else if (version == IOIN_VERSION_2208_2224)
					typ = DriverType::tmc2208;
				else
				{
					// We could potentially stop here and declare the driver unknown, but for now
					// we issue a warning and assume it is a 2209.
					if (reprap.Debug(Module::Driver))
						debugPrintf("TMCUART:: Warning driver %d unknown version number 0x%x\n", driverNumber, (unsigned)version);
					typ = DriverType::tmc2209;
				}
				// did our discovery match the request driver type?
				if (TMC_DRIVER_TYPE[driverNumber] != DriverType::tmcuartauto && typ != TMC_DRIVER_TYPE[driverNumber])
				{
					if (reprap.Debug(Module::Driver))
						debugPrintf("TMCUART:: Warning driver %d type mismatch requested %s actual %s\n", driverNumber, TMC_DRIVER_TYPE[driverNumber].ToString(), typ.ToString());
					typ = TMC_DRIVER_TYPE[driverNumber];
				}
				WriteAll();
			}
		}
		return state;
	}		

	if (UpdatePending())
	{
		//debugPrintf(" write pending %x\n", registersToUpdate);
		return state;
	}
	if (numReads >= NumReadRegisters + 3)
	{
		//debugPrintf(" ready\n");
		state = DriversState::ready;
	}
	return state;
}

static TASKMEM Task<TmcTaskStackWords> tmc22Task;

extern "C" [[noreturn]] void Tmc22Loop(void *) noexcept
{
	for (;;)
	{
		if (driversState <= DriversState::noDriver)
		{
			if (driversState != DriversState::noDriver) driversState = DriversState::powerWait;
			TaskBase::TakeIndexed(NotifyIndices::Tmc);
		}
		else
		{
			if (driversState == DriversState::notInitialised)
			{
				for (size_t i = 0; i < numTmc22xxDrivers; ++i)
				{
					driverStates[i].SetupDriver(true);
				}
				driversState = DriversState::initialising;
			}
			else
			{
				if (driversState == DriversState::initialising)
				{
					// If all drivers that share the global enable have been initialised, set the global enable
					bool allInitialised = true;
					for (size_t i = 0; i < numTmc22xxDrivers; ++i)
					{
						if (driverStates[i].SetupDriver(false) == DriversState::initialising)
						{
							allInitialised = false;
						}
					}

					if (allInitialised)
					{
						size_t readyCnt = 0;
						for (size_t i = 0; i < numTmc22xxDrivers; ++i)
						{
							if (driverStates[i].IsReady())
							{
								digitalWrite(ENABLE_PINS[driverStates[i].GetDriverNumber()], false);
								readyCnt++;
							}
						}
						driversState = (readyCnt ? DriversState::ready : DriversState::noDriver);
					}
				}
			}
			uint32_t activeCnt = 0;
			for (size_t i = 0; i < numTmc22xxDrivers; ++i)
			{
				if (driverStates[i].IsActive())
				{
					activeCnt++;
					if (driverStates[i].StartTransfer())
						driverStates[i].TransferDone();
					else
						driverStates[i].TransferTimedOut();

				}
			}
			// Give other tasks a chance to run.
			delay((activeCnt <= 1 ? 3 : 1));
		}
	}
}

//--------------------------- Public interface ---------------------------------
// Initialise the driver interface and the drivers, leaving each drive disabled.
// It is assumed that the drivers are not powered, so driversPowered(true) must be called after calling this before the motors can be moved.
void Tmc22xxDriver::Init(size_t numDrivers) noexcept
{
	numTmc22xxDrivers = min<size_t>(numDrivers, MaxSmartDrivers);
	if (numTmc22xxDrivers == 0)
	{
		driversState = DriversState::ready;
		return;
	}		
	driverStates = (Tmc22xxDriverState *)	Tasks::AllocPermanent(sizeof(Tmc22xxDriverState)*numTmc22xxDrivers);
	memset((void *)driverStates, 0, sizeof(Tmc22xxDriverState)*numTmc22xxDrivers);
	
	driversState = DriversState::noPower;
	tmc22Task.Create(Tmc22Loop, "TMC22xx", nullptr, TaskPriority::TmcPriority);
}

// Shut down the drivers and stop any related interrupts. Don't call Spin() again after calling this as it may re-enable them.
void Tmc22xxDriver::Exit() noexcept
{
	if (numTmc22xxDrivers > 0)
	{
		TurnDriversOff();
		tmc22Task.TerminateAndUnlink();
	}
	driversState = DriversState::noPower;
}


// Flag that the the drivers have been powered up or down and handle any timeouts
// Before the first call to this function with 'powered' true, you must call Init()
void Tmc22xxDriver::Spin(bool powered) noexcept
{
	if (numTmc22xxDrivers == 0) return;
	TaskCriticalSectionLocker lock;

	if (powered)
	{
		if (driversState == DriversState::powerWait)
		{
			driversState = DriversState::notInitialised;
			tmc22Task.Give(NotifyIndices::Tmc);									// wake up the TMC task because the drivers need to be initialised
		}
	}
	else if (driversState > DriversState::powerWait)
	{
		TurnDriversOff();
	}
}

bool Tmc22xxDriver::IsReady() noexcept
{
	return driversState == DriversState::ready || driversState == DriversState::noDriver;
}

// This is called from the tick ISR, possibly while Spin (with powered either true or false) is being executed
void Tmc22xxDriver::TurnDriversOff() noexcept
{
	for (size_t i = 0; i < numTmc22xxDrivers; ++i)
	{
		digitalWrite(ENABLE_PINS[driverStates[i].GetDriverNumber()], true);
	}
	driversState = (driversState == DriversState::noDriver ? DriversState::powerWait : DriversState::noPower);
}

TmcDriverState* Tmc22xxDriver::InitDrive(size_t slot, size_t driveNo) noexcept
{
	new(&driverStates[slot]) Tmc22xxDriverState();
	driverStates[slot].Init(driveNo
#if TMC22xx_HAS_ENABLE_PINS
							, ENABLE_PINS[driveNo]
#endif
#if HAS_STALL_DETECT
							, DriverDiagPins[driveNo]
#endif
							);
	return &(driverStates[slot]);
}

#if HAS_STALL_DETECT

DriversBitmap Tmc22xxDriver::GetStalledDrivers(DriversBitmap driversOfInterest) noexcept
{
	DriversBitmap rslt;
	driversOfInterest.Iterate([&rslt](unsigned int driverNumber, unsigned int count)
								{
									if (driverNumber < ARRAY_SIZE(DriverDiagPins) && digitalRead(DriverDiagPins[driverNumber]))
									{
										rslt.SetBit(driverNumber);
									}
								}
							 );
	return rslt;
}
#endif

#endif

// End
