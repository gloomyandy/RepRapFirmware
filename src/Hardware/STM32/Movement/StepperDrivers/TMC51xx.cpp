/*
 * TMC51xx.cpp
 *
 *  Created on: 26 Aug 2018
 *      Author: David
 *  Purpose:
 *  	Support for TMC5130, TMC5160 and TMC5161 stepper drivers
 * 		Andy added support for TMC2240 drivers
 */

#include "SmartDrivers.h"

#if SUPPORT_TMC51xx
#include <RTOSIface/RTOSIface.h>
#include <Platform/Platform.h>
#include <Platform/RepRap.h>
#include <Movement/Move.h>
#include <Hardware/Spi/SharedSpiDevice.h>
#include <Hardware/Spi/SharedSpiClient.h>
#include <Platform/TaskPriorities.h>
#include <General/Portability.h>
#include <AppNotifyIndices.h>
#include <Endstops/Endstop.h>
#include "TmcDriverState.h"
#include "TMC51xxDriver.h"
#if HAS_STALL_DETECT && SUPPORT_REMOTE_COMMANDS
# include <CAN/CanInterface.h>
#endif
// On some processors we need to ensure that memory mapped I/O operations are synced to the hardware
# if STM32H7
# define SYNC_GPIO() __DSB()
#else
# define SYNC_GPIO() 
#endif

static inline Move& GetMoveInstance() noexcept { return reprap.GetMove(); }

//#define TMC_TYPE	5130
#define TMC_TYPE	5160
#define DEBUG_DRIVER_TIMEOUT	0

constexpr float MinimumMotorCurrent = 50.0;
constexpr float MinimumOpenLoadMotorCurrent = 500;			// minimum current in mA for the open load status to be taken seriously
constexpr uint32_t DefaultMicrosteppingShift = 4;			// x16 microstepping
constexpr bool DefaultInterpolation = true;					// interpolation enabled
constexpr uint32_t DefaultTpwmthrsReg = 2000;				// low values (high changeover speed) give horrible jerk at the changeover from stealthChop to spreadCycle
constexpr int DefaultStallDetectThreshold = 1;
constexpr bool DefaultStallDetectFiltered = false;
constexpr unsigned int DefaultMinimumStepsPerSecond = 200;	// for stall detection: 1 rev per second assuming 1.8deg/step, as per the TMC5160 datasheet
constexpr uint32_t DefaultTcoolthrs = 2000;					// max interval between 1/256 microsteps for stall detection to be enabled
constexpr uint32_t DefaultThigh = 200;
constexpr uint32_t LowestTmcClockSpeed =  11500000;			// the lowest speed at which the TMC driver is clocked internally
constexpr uint32_t NominalTmcClockSpeed = 12000000;			// the nominal speed at which the TMC driver is clocked internally
constexpr uint32_t HighestTmcClockSpeed = 12600000;			// the highest speed at which the TMC driver is clocked internally
#if SUPPORT_CLOSED_LOOP
constexpr size_t TmcTaskStackWords = 430;					// we need extra stack to handle closed loop tuning and writing to NVM
#elif SUPPORT_PHASE_STEPPING
constexpr size_t TmcTaskStackWords = 430;					// we need extra stack to handle phase stepping (amount not calculated yet, just taken from 1HCL)
#else
constexpr size_t TmcTaskStackWords = 140;					// with 100 stack words, deckingman's M122 on the main board after a major axis shift showed just 10 words left
#endif

constexpr float Default5160SenseResistor = 0.075;			// Typical value used on step sticks
constexpr float DefaultMaxTmc5160Current = 6300.0;			// The maximum current we allow the TMC5160/5161 drivers to be set to

constexpr uint32_t Tmc2240CurrentRange = 0x3;				// TMC2240 Current Range max 3A
constexpr uint32_t Tmc2240SlopeControl = 0x01;				// which slope control we set the TMC2240 to (200V/us)
constexpr float DefaultTmc2240Rref = 12300.0;				// TMC2240 reference resistor on Fly boards, in ohms
constexpr float DefaultMaxTmc2240Current = 2500.0;

// Max current and sense resistor values can now be set via gcode, so we calculate other values as needed
//constexpr float MaximumStandstillCurrent = MaxTmc5160Current * 0.707;
//constexpr float RecipFullScaleCurrent = SenseResistor/325.0;		// 1.0 divided by full scale current in mA
constexpr float Vfs = 325.0;										// Full scale voltage from 5160 datasheet

// The SPI clock speed is a compromise:
// - too high and polling the driver chips takes too much of the CPU time
// - too low and we won't detect stalls quickly enough
#if SUPPORT_PHASE_STEPPING
constexpr uint32_t DefaultDriversSpiClockFrequency = 2000000;		// 2MHz SPI clock, this is speed used in older version of RRF
// Note for reasons I don't currently understand running some 5160 driver modules on some
// boards results in invalid data being read (drivers report multiple errors). For this
// reason the spi clock frequency actually used is curently 3.75MHz (see CoreN2G variant.cpp
// for details). This probably needs further investigation.
// Also note that on H7 (and probably F4) mcus the setup time for hardware spi is such that a
// spi transaction takes longer than the minimum datalen/freq by around 7us. This also needs
// further investigation/optimisation
constexpr uint32_t PhaseStepDriversSpiClockFrequency = 4000000;		// 4MHz SPI clock, this is the maximum rate the TMC5160/2160 support using the internal clock
constexpr uint32_t DefaultSpiSleepMicroseconds = 1000;				// Sleep time used for tmcTask when not phase stepping
constexpr uint32_t PhaseStepSpiSleepMicroseconds = 125;				// Sleep time used for tmcTask when phase stepping
																	// there is 1 write + 1 read/write per motor current setting.
constexpr uint32_t DefaultSpiSleepClocks = (StepClockRate * DefaultSpiSleepMicroseconds)/1000000;
constexpr uint32_t PhaseStepSpiSleepClocks = (StepClockRate * PhaseStepSpiSleepMicroseconds)/1000000;

static uint32_t DriversDirectSleepClocks = DefaultSpiSleepClocks;	// how long the phase stepping task sleeps for in each cycle. Max SPI message frequency is ~16.7 kHz
																	// there is 1 write + 1 read/write per motor current setting.
#else
constexpr uint32_t DefaultDriversSpiClockFrequency = 2000000;		// 2MHz SPI clock
constexpr uint32_t DefaultSpiSleepMicroseconds = 1000;				// Sleep time used for tmcTask when not phase stepping
constexpr uint32_t DefaultSpiSleepClocks = (StepClockRate * DefaultSpiSleepMicroseconds)/1000000;
constexpr uint32_t DriversDirectSleepClocks = DefaultSpiSleepClocks;
#endif
constexpr uint32_t TransferTimeout = 3;						// any transfer should complete within 2 ticks @ 1ms/tick. Need to allow one more in case a tick is about to happen.

// GCONF register (0x00, RW)
constexpr uint8_t REGNUM_GCONF = 0x00;

constexpr uint32_t GCONF_5130_USE_VREF = 1 << 0;			// use external VRef
constexpr uint32_t GCONF_5130_INT_RSENSE = 1 << 1;			// use internal sense resistors
constexpr uint32_t GCONF_5130_END_COMMUTATION = 1 << 3;		// Enable commutation by full step encoder (DCIN_CFG5 = ENC_A, DCEN_CFG4 = ENC_B)

constexpr uint32_t GCONF_5160_RECAL = 1 << 0;				// Zero crossing recalibration during driver disable (via ENN or via TOFF setting)
constexpr uint32_t GCONF_5160_FASTSTANDSTILL = 1 << 1;		// Timeout for step execution until standstill detection: 1: Short time: 2^18 clocks, 0: Normal time: 2^20 clocks
constexpr uint32_t GCONF_5160_MULTISTEP_FILT = 1 << 3;		// Enable step input filtering for stealthChop optimization with external step source (default=1)

constexpr uint32_t GCONF_STEALTHCHOP = 1 << 2;				// use stealthchop mode (else spread cycle mode)
constexpr uint32_t GCONF_REV_DIR = 1 << 4;					// reverse motor direction
constexpr uint32_t GCONF_DIAG0_ERROR = 1 << 5;				// Enable DIAG0 active on driver errors: Over temperature (ot), short to GND (s2g), undervoltage chargepump (uv_cp)
															// DIAG0 always shows the reset-status, i.e. is active low during reset condition.
constexpr uint32_t GCONF_DIAG0_OTPW = 1 << 6;				// Enable DIAG0 active on driver over temperature prewarning (otpw)
constexpr uint32_t GCONF_DIAG0_STALL = 1 << 7;				// Enable DIAG0 active on motor stall (set TCOOLTHRS before using this feature)
constexpr uint32_t GCONF_DIAG1_STALL = 1 << 8;				// Enable DIAG1 active on motor stall (set TCOOLTHRS before using this feature)
constexpr uint32_t GCONF_DIAG1_INDEX = 1 << 9;				// Enable DIAG1 active on index position (microstep look up table position 0)
constexpr uint32_t GCONF_DIAG1_ONSTATE = 1 << 10;			// Enable DIAG1 active when chopper is on (for the coil which is in the second half of the fullstep)
constexpr uint32_t GCONF_DIAG1_STEPS_SKIPPED = 1 << 11;		// Enable output toggle when steps are skipped in dcStep mode (increment of LOST_STEPS). Do not enable in conjunction with other DIAG1 options.
constexpr uint32_t GCONF_DIAG0_PUSHPULL = 1 << 12;			// 0: SWN_DIAG0 is open collector output (active low), 1: Enable SWN_DIAG0 push pull output (active high)
constexpr uint32_t GCONF_DIAG1_PUSHPULL = 1 << 13;			// 0: SWN_DIAG1 is open collector output (active low), 1: Enable SWN_DIAG1 push pull output (active high)
constexpr uint32_t GCONF_SMALL_HYSTERESIS = 1 << 14;		// 0: Hysteresis for step frequency comparison is 1/16, 1: Hysteresis for step frequency comparison is 1/32
constexpr uint32_t GCONF_STOP_ENABLE = 1 << 15;				// 0: Normal operation, 1: Emergency stop: ENCA_DCIN stops the sequencer when tied high (no steps become executed by the sequencer, motor goes to standstill state)
constexpr uint32_t GCONF_DIRECT_MODE = 1 << 16;				// 0: Normal operation, 1: Motor coil currents and polarity directly programmed via serial interface:
															// Register XTARGET (0x2D) specifies signed coil A current (bits 8..0) and coil B current (bits 24..16).
															// In this mode, the current is scaled by IHOLD setting. Velocity based current regulation of stealthChop
															// is not available in this mode. The automatic stealthChop current regulation will work only for low stepper motor velocities.
constexpr uint32_t GCONF_TEST_MODE = 1 << 17;				// 0: Normal operation, 1: Enable analog test output on pin ENCN_DCO. IHOLD[1..0] selects the function of ENCN_DCO: 0…2: T120, DAC, VDDH

#if TMC_TYPE == 5130
constexpr uint32_t DefaultGConfReg = GCONF_DIAG0_STALL | GCONF_DIAG0_PUSHPULL;
#elif TMC_TYPE == 5160
constexpr uint32_t DefaultGConfReg = GCONF_5160_RECAL | GCONF_5160_MULTISTEP_FILT | GCONF_DIAG0_STALL | GCONF_DIAG0_PUSHPULL;
#endif
constexpr uint32_t DefaultGConfReg2240 = GCONF_5160_MULTISTEP_FILT | GCONF_DIAG0_STALL | GCONF_DIAG0_PUSHPULL;

// General configuration and status registers

// GSTAT register (0x01, RW). Write 1 bits to clear the flags.
constexpr uint8_t REGNUM_GSTAT = 0x01;
constexpr uint32_t GSTAT_RESET = 1 << 0;					// driver has been reset since last read
constexpr uint32_t GSTAT_DRV_ERR = 1 << 1;					// driver has been shut down due to over temp or short circuit
constexpr uint32_t GSTAT_UV_CP = 1 << 2;					// undervoltage on charge pump, driver disabled while it persists. This bit is latched for information.

constexpr uint32_t DefaultGstatReg = 0x07;					// this value clear all bits

// IOIN register (0x04, RO) reads the state of all input pins and version. We use it for device identification.
constexpr uint8_t REGNUM_IOIN = 0x04;
constexpr uint32_t IOIN_VERSION_SHIFT = 24;
constexpr uint32_t IOIN_VERSION_MASK = 0xff << IOIN_VERSION_SHIFT;
constexpr uint32_t IOIN_VERSION_5160 = 0x30;				// version for TMC5160
constexpr uint32_t IOIN_VERSION_2240 = 0x40;				// version for TMC2240 in spi mode

// IFCOUNT register (0x02, RO) is not used in SPI mode
// SLAVECONF register (0x03, WO) is not used in SPI mode
// IOIN register (0x04, RO) reads the state of all input pins. We use it for device identification.
// OUTPUT register (0x04, WO) is not used in SPI mode
// X_COMPARE register (0x05, WO) allows us to get a pulse on DIAG1 when an index is passed. We don't use it.
// OTP_PROG register (0x06, WO, 5160 only) is not used in this firmware
// OTP_READ register (0x07, RO, 5160 only) is not used in this firmware
// FACTORY_CONF register (0x08, RW, 5160 only) trims the clock frequency and is preset for 12MHz

#if TMC_TYPE == 5160

// SHORT_CONF register
constexpr uint8_t REGNUM_5160_SHORTCONF = 0x09;

constexpr uint32_t SHORTCONF_S2VS_LEVEL_SHIFT = 0;
constexpr uint32_t SHORTCONF_S2VS_LEVEL_MASK = 15;			// Short to VS detector level for lowside FETs. Checks for voltage drop in LS MOSFET and sense resistor.
															// 4 (highest sensitivity) … 15 (lowest sensitivity); 10 recommended for normal operation (Reset default 12 via OTP)
															// Hint: Settings from 1 to 3 will trigger during normal operation due to voltage drop on sense resistor.
constexpr uint32_t SHORTCONF_S2G_LEVEL_SHIFT = 8;
constexpr uint32_t SHORTCONF_S2G_LEVEL_MASK = (15 << 8);	// Short to GND detector level for highside FETs. Checks for voltage drop on high side MOSFET
															// 2 (highest sensitivity) … 15 (lowest sensitivity) 6 to 10 recommended (Reset Default: 12 via OTP)
constexpr uint32_t SHORTCONF_FILTER_SHIFT = 16;
constexpr uint32_t SHORTCONF_FILTER_MASK = (3 << 16);		// Spike filtering bandwidth for short detection 0 (lowest, 100ns), 1 (1us), 2 (2us) 3 (3us)
															// Hint: A good PCB layout will allow using setting 0. Increase value, if erroneous short detection occurs. Reset Default = 1
constexpr uint32_t SHORTCONF_DELAY = (1 << 18);				// Short detection delay 0=750ns: normal, 1=1500ns: high The short detection delay shall cover the bridge switching time.
															// 0 will work for most applications. (Reset Default = 0)
constexpr uint32_t DefaultShortConfReg = (10 << SHORTCONF_S2VS_LEVEL_SHIFT) | (6 << SHORTCONF_S2G_LEVEL_SHIFT) | (0 << SHORTCONF_FILTER_SHIFT);

// DRV_CONF register
constexpr uint8_t REGNUM_5160_DRVCONF = 0x0A;
constexpr uint32_t DRVCONF_BBMTIME_SHIFT = 0;
constexpr uint32_t DRVCONF_BBMTIME_MASK = 31;				// Break-Before make delay 0=shortest (100ns) … 16 (200ns) … 24=longest (375ns) >24 not recommended, use BBMCLKS instead
															// Hint: 0 recommended due to fast switching MOSFETs (Reset Default = 0)
constexpr uint32_t DRVCONF_BBMCLKS_SHIFT = 8;
constexpr uint32_t DRVCONF_BBMCLKS_MASK = (15 << 8);		// Digital BBM time in clock cycles (typ. 83ns). The longer setting rules (BBMTIME vs. BBMCLKS).
															// Reset Default: 2 via OTP. Hint: 2, or down to 0 recommended due to fast switching MOSFETs
constexpr uint32_t DRVCONF_OTSELECT_SHIFT = 16;
constexpr uint32_t DRVCONF_OTSELECT_MASK = (3 << 16);		// Selection of over temperature level for bridge disable, switch on after cool down to 120°C / OTPW level. Reset Default = 0.
															// 00: 150°C (not recommended – MOSFET might overheat); 01: 143°C 10: 136°C (Recommended); 11: 120°C (not recommended, no hysteresis)
															// Hint: Adapt overtemperature threshold as required to protect the MOSFETs or other components on the PCB.
constexpr uint32_t DRVCONF_STRENGTH_SHIFT = 18;
constexpr uint32_t DRVCONF_STRENGTH_MASK = (3 << 18);		// Selection of gate driver current. Adapts the gate driver current to the gate charge of the external MOSFETs.
															// 00: Normal slope (Recommended), 01: Normal+TC (medium above OTPW level), 10: Fast slope. Reset Default = 10.
constexpr uint32_t DRVCONF_FILT_ISENSE_SHIFT = 20;
constexpr uint32_t DRVCONF_FILT_ISENSE_MASK = (3 << 20);	// Filter time constant of sense amplifier to suppress ringing and coupling from second coil operation
															// 00: low – 100ns 01: – 200ns 10: – 300ns 11: high – 400ns
															// Hint: Increase setting if motor chopper noise occurs due to cross-coupling of both coils. Reset Default = 0.
constexpr uint32_t DefaultDrvConfReg = (2 << DRVCONF_BBMCLKS_SHIFT) | (2 << DRVCONF_OTSELECT_SHIFT);

// TMC2040 has different DRV_CONF settings
constexpr unsigned int DRV_CONF2240_CURRENT_RANGE_SHIFT = 0;
constexpr uint32_t DRV_CONF2240_CURRENT_RANGE_MASK = 0x03;										// 0 = 1A, 1 = 2A, 2 = 3A, 3 = 3A peak current
constexpr unsigned int DRV_CONF2240_SLOPE_CONTROL_SHIFT = 4;
constexpr uint32_t DRV_CONF2240_SLOPE_CONTROL_MASK = 0x03 << DRV_CONF2240_SLOPE_CONTROL_SHIFT;		// 0 = 100V/us, 1 = 200V/us, 2 = 400V/us, 3 - 800V/us
constexpr uint32_t DefaultDrvConfReg2240 = (Tmc2240CurrentRange << DRV_CONF2240_CURRENT_RANGE_SHIFT) | (Tmc2240SlopeControl << DRV_CONF2240_SLOPE_CONTROL_SHIFT);

constexpr uint8_t REGNUM_5160_GLOBAL_SCALER = 0x0B;			// Global scaling of Motor current. This value is multiplied to the current scaling in order to adapt a drive to a
															// certain motor type. This value should be chosen before tuning other settings, because it also influences chopper hysteresis.
															// 0: Full Scale (or write 256) 1 … 31: Not allowed for operation 32 … 255: 32/256 … 255/256 of maximum current.
															// Hint: Values >128 recommended for best results. Reset Default 0.
constexpr uint32_t DefaultGlobalScalerReg = 0;				// until we use it as part of the current setting

constexpr uint8_t REGNUM_5160_OFFSET_READ = 0x0B;			// Bits 8..15: Offset calibration result phase A (signed). Bits 0..7: Offset calibration result phase B (signed).

constexpr uint8_t REGNUM_5160_X_DIRECT = 0x2D;				// Coil currents for direct mode. Bits 8..0: signed coil A current. Bits 24..16: signed coil B current.
															// A maximal value of 255 in this register corresponds to a current of IHOLD
															// Note: Reg GCONF bit 16 (direct_mode) must be set to use this register

// TMC2040 Temperature ADC regs
constexpr uint8_t REGNUM_ADC_TEMP = 0x51;
constexpr unsigned int ADC_TEMP_SHIFT = 0;
constexpr uint32_t ADC_TEMP_MASK = 0x01FFF << ADC_TEMP_SHIFT;								// ADC temperature reading

#endif

// Velocity dependent control registers

// IHOLD_IRUN register (WO)
constexpr uint8_t REGNUM_IHOLDIRUN = 0x10;
constexpr uint32_t IHOLDIRUN_IHOLD_SHIFT = 0;				// standstill current
constexpr uint32_t IHOLDIRUN_IHOLD_MASK = 0x1F << IHOLDIRUN_IHOLD_SHIFT;
constexpr uint32_t IHOLDIRUN_IRUN_SHIFT = 8;
constexpr uint32_t IHOLDIRUN_IRUN_MASK = 0x1F << IHOLDIRUN_IRUN_SHIFT;
constexpr uint32_t IHOLDIRUN_IHOLDDELAY_SHIFT = 16;
constexpr uint32_t IHOLDIRUN_IHOLDDELAY_MASK = 0x0F << IHOLDIRUN_IHOLDDELAY_SHIFT;
constexpr unsigned int IHOLDIRUN2240_IRUNDELAY_SHIFT = 24;
constexpr uint32_t IHOLDIRUN2240_IRUNDELAY_MASK = 0x0F << IHOLDIRUN2240_IRUNDELAY_SHIFT;

constexpr uint32_t DefaultIholdIrunReg = (0 << IHOLDIRUN_IHOLD_SHIFT) | (0 << IHOLDIRUN_IRUN_SHIFT) | (2 << IHOLDIRUN_IHOLDDELAY_SHIFT);
															// approx. 0.5 sec motor current reduction to half power

constexpr uint8_t REGNUM_TPOWER_DOWN = 0x11;
constexpr uint8_t REGNUM_TSTEP = 0x12;
constexpr uint8_t REGNUM_TPWMTHRS = 0x13;

constexpr uint8_t REGNUM_TCOOLTHRS = 0x14;
const uint32_t DefaultTcoolthrsReg = DefaultTcoolthrs;

constexpr uint8_t REGNUM_THIGH = 0x15;
const uint32_t DefaultThighReg = DefaultThigh;

constexpr uint8_t REGNUM_VACTUAL = 0x22;

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
constexpr uint32_t CHOPCONF_5130_RNDTOFF = 1 << 13;			// random off time
constexpr uint32_t CHOPCONF_CHM = 1 << 14;					// fixed off time
constexpr uint32_t CHOPCONF_TBL_SHIFT = 15;					// blanking time
constexpr uint32_t CHOPCONF_TBL_MASK = 0x03 << CHOPCONF_TBL_SHIFT;
constexpr uint32_t CHOPCONF_5130_VSENSE_HIGH = 1 << 17;		// use high sensitivity current scaling
constexpr uint32_t CHOPCONF_MRES_SHIFT = 24;				// microstep resolution
constexpr uint32_t CHOPCONF_MRES_MASK = 0x0F << CHOPCONF_MRES_SHIFT;
constexpr uint32_t CHOPCONF_INTPOL = 1 << 28;				// use interpolation
constexpr uint32_t CHOPCONF_DEDGE = 1 << 29;				// step on both edges
constexpr uint32_t CHOPCONF_DISS2G = 1 << 30;				// disable short to ground protection
constexpr uint32_t CHOPCONF_DISS2VS = 1 << 31;				// disable low side short protection

#if TMC_TYPE == 5130
constexpr uint32_t DefaultChopConfReg = (1 << CHOPCONF_TBL_SHIFT) | (3 << CHOPCONF_TOFF_SHIFT) | (5 << CHOPCONF_HSTRT_SHIFT) | CHOPCONF_5130_VSENSE_HIGH;
#elif TMC_TYPE == 5160
constexpr uint32_t DefaultChopConfReg = (1 << CHOPCONF_TBL_SHIFT) | (3 << CHOPCONF_TOFF_SHIFT) | (5 << CHOPCONF_HSTRT_SHIFT);
#endif

constexpr uint8_t REGNUM_COOLCONF = 0x6D;
constexpr uint32_t COOLCONF_SGFILT = 1 << 24;				// set to update stallGuard status every 4 full steps instead of every full step
constexpr uint32_t COOLCONF_SGT_SHIFT = 16;
constexpr uint32_t COOLCONF_SGT_MASK = 127 << COOLCONF_SGT_SHIFT;	// stallguard threshold (signed)
constexpr uint32_t COOLCONF_COOL_MASK = (1u << 16) - 1;

constexpr uint32_t DefaultCoolConfReg = 0;

// DRV_STATUS register
constexpr uint8_t REGNUM_DRV_STATUS = 0x6F;
constexpr uint32_t TMC_RR_S2VS = 3 << 12;				// short to VS indicator (1 bit for each phase)
constexpr uint32_t TMC_RR_SG = 1 << 24;					// stall detected
constexpr uint32_t TMC_RR_OT = 1 << 25;					// over temperature shutdown
constexpr uint32_t TMC_RR_OTPW = 1 << 26;				// over temperature warning
constexpr uint32_t TMC_RR_S2G = 3 << 27;				// short to ground indicator (1 bit for each phase)
constexpr uint32_t TMC_RR_OL = 3 << 29;					// open load (1 bit for each phase)
constexpr uint32_t TMC_RR_STST = 1 << 31;				// standstill detected
constexpr uint32_t TMC_RR_SGRESULT = 0x3FF;				// 10-bit stallGuard2 result

constexpr unsigned int TMC_RR_S2VS_BIT_POS = 12;
constexpr unsigned int TMC_RR_SG_BIT_POS = 24;
constexpr unsigned int TMC_RR_OT_BIT_POS = 25;
constexpr unsigned int TMC_RR_OTPW_BIT_POS = 26;
constexpr unsigned int TMC_RR_S2G_BIT_POS = 27;
constexpr unsigned int TMC_RR_OL_BIT_POS = 29;
constexpr unsigned int TMC_RR_STST_BIT_POS = 31;

// PWMCONF register
constexpr uint8_t REGNUM_PWMCONF = 0x70;

constexpr uint32_t DefaultPwmConfReg = 0xC40C001E;			// this is the reset default - try it until we find something better
constexpr uint32_t DefaultPwmConfReg2240 = 0xC40C001D;			// this is the reset default - try it until we find something better

constexpr uint8_t REGNUM_PWM_SCALE = 0x71;
constexpr uint8_t REGNUM_PWM_AUTO = 0x72;

// Common data
static size_t numTmc51xxDrivers = 0;
static size_t numDriversToPoll = 0;

static constexpr uint32_t MaxValidSgLoadRegister = 1023;
static constexpr uint32_t InvalidSgLoadRegister = 1024;

inline uint32_t GetHighestTmcClockSpeed() noexcept
{
	return HighestTmcClockSpeed;
}

inline uint32_t GetLowestTmcClockSpeed() noexcept
{
	return LowestTmcClockSpeed;
}


enum class DriversState : uint8_t
{
	shutDown = 0,
	powerWait,				// waiting for power
	noDriver,				// no driver found or configured
	notInitialised,			// have VIN power but not started initialising drivers
	initialising,			// in the process of initialising the drivers
	ready					// drivers are initialised and ready
};

static DriversState driversState = DriversState::shutDown;

#if SUPPORT_REMOTE_COMMANDS
static LocalDriversBitmap stallEndstopsEnabled;
std::atomic<uint16_t> SmartDrivers::driverStallsToNotify(0);
#endif

//----------------------------------------------------------------------------------------------------------------------------------
// Private types and methods

class Tmc51xxDriverState : public TmcDriverState
{
public:
	Tmc51xxDriverState() noexcept;
	void Init(uint32_t p_driverNumber) noexcept;
	void SetAxisNumber(size_t p_axisNumber) noexcept;
	uint32_t GetAxisNumber() const noexcept { return axisNumber; }
	void WriteAll() noexcept;
	bool SetMicrostepping(uint8_t shift, bool interpolate) noexcept;
	unsigned int GetMicrostepping(bool& interpolation) const noexcept;
#if SUPPORT_PHASE_STEPPING || SUPPORT_CLOSED_LOOP
	unsigned int GetMicrostepShift() const noexcept { return microstepShiftFactor; }
	uint16_t GetMicrostepPosition() const noexcept { return readRegisters[ReadMsCnt] & 1023; }
	bool SetXdirect(uint32_t regVal) noexcept;
	float GetCurrent() const noexcept { return (float)motorCurrent; }
#endif	
#if SUPPORT_PHASE_STEPPING
	bool EnablePhaseStepping(bool enable) noexcept;
	bool inline IsPhaseSteppingEnabled() const noexcept { return phaseStepEnabled; }
#endif
	bool SetDriverMode(unsigned int mode) noexcept;
	DriverMode GetDriverMode() const noexcept;
	void SetCurrent(float current) noexcept;
	void Enable(bool en) noexcept;
	StandardDriverStatus GetStatus(bool accumulated, bool clearAccumulated) noexcept;
	float GetSenseResistor() const noexcept;
	void SetSenseResistor(float value) noexcept;
	float GetMaxCurrent() const noexcept;
	void SetMaxCurrent(float value) noexcept;
	void AppendDriverStatus(const StringRef& reply) noexcept;
	float GetDriverTemperature() noexcept;
	uint32_t GetDriverMaxClockFrequency() noexcept;
	uint32_t GetDriverNominalClockFrequency() noexcept;
	uint32_t GetDriverMinClockFrequency() noexcept;
	bool UpdatePending() const noexcept { return (registersToUpdate.load() | newRegistersToUpdate.load()) != 0; }
#if HAS_STALL_DETECT
	void SetStallDetectThreshold(int sgThreshold) noexcept;
	void SetStallDetectFilter(bool sgFilter) noexcept;
	void SetStallMinimumStepsPerSecond(unsigned int stepsPerSecond) noexcept;
	void AppendStallConfig(const StringRef& reply) const noexcept;
	const char *_ecv_array _ecv_null  CheckStallDetectionEnabled(float speed) noexcept;
#endif

	bool SetRegister(SmartDriverRegister reg, uint32_t regVal) noexcept;
	uint32_t GetRegister(SmartDriverRegister reg) const noexcept;

	GCodeResult GetAnyRegister(const StringRef& reply, uint8_t regNum) noexcept;
	GCodeResult SetAnyRegister(const StringRef& reply, uint8_t regNum, uint32_t regVal) noexcept;

	float GetStandstillCurrentPercent() const noexcept;
	void SetStandstillCurrentPercent(float percent) noexcept;
	bool SetCurrentScaler(int8_t cs) noexcept;
	uint8_t GetIRun() const noexcept { return (writeRegisters[WriteIholdIrun] & IHOLDIRUN_IRUN_MASK) >> IHOLDIRUN_IRUN_SHIFT; }
	uint8_t GetIHold() const noexcept { return (writeRegisters[WriteIholdIrun] & IHOLDIRUN_IHOLD_MASK) >> IHOLDIRUN_IHOLD_SHIFT; }
	uint32_t GetGlobalScaler() const noexcept { return writeRegisters[Write5160GlobalScaler]; }
	float CalculateCurrent() const noexcept;				// calculate what current the driver is actually using based on register values


	bool GetSpiCommand(uint8_t *sendDataBlock, bool forceRead = false) noexcept;
	void TransferSucceeded(const uint8_t *rcvDataBlock, bool checkWrite = false) noexcept;
	void TransferFailed() noexcept;
	bool DoIo(bool forceRead = false, bool checkWrite = false) noexcept;
	void ResetDriver() noexcept;
	DriversState SetupDriver() noexcept;
	bool inline IsActive() noexcept {return state >= DriversState::initialising;}
	bool inline IsReady() noexcept {return state == DriversState::ready;}
	uint8_t inline GetDriverNumber() const noexcept { return driverNumber; }
	void inline EnableChipSelect() noexcept {fastDigitalWriteLow(TMC_PINS[driverNumber]); SYNC_GPIO();}
	void inline DisableChipSelect() noexcept {fastDigitalWriteHigh(TMC_PINS[driverNumber]); SYNC_GPIO();}
#if SUPPORT_PHASE_STEPPING
	bool inline static NeedCoilCurrentSet() noexcept {bool ret = needToSetCoilCurrents; needToSetCoilCurrents = false; return ret;}
#endif
private:
	DriversState state;
	uint8_t driverNumber;
	bool SetChopConf(uint32_t newVal) noexcept;
	void UpdateRegister(size_t regIndex, uint32_t regVal) noexcept;
	void UpdateChopConfRegister() noexcept;							// calculate the chopper control register and flag it for sending
	void UpdateCurrent() noexcept;
	int32_t IdentifyDriver() noexcept;
	bool IsTmc2240() const noexcept { return typ == DriverType::tmc2240; }
	void ResetReadRegisters() noexcept;
	void ResetLoadRegisters() noexcept { minSgLoadRegister = InvalidSgLoadRegister; }

	// Write register numbers are in priority order, most urgent first, in same order as WriteRegNumbers
	static constexpr unsigned int WriteXDirect = 0;			// microstepping
	static constexpr unsigned int WriteGConf = 1;			// microstepping
	static constexpr unsigned int WriteIholdIrun = 2;		// current setting
	static constexpr unsigned int WriteTpwmthrs = 3;		// upper step rate limit for stealthchop
	static constexpr unsigned int WriteTcoolthrs = 4;		// lower velocity for coolStep and stallGuard
	static constexpr unsigned int WriteThigh = 5;			// upper velocity for coolStep and stealthChop
	static constexpr unsigned int WriteChopConf = 6;		// chopper control
	static constexpr unsigned int WriteCoolConf = 7;		// coolstep control
	static constexpr unsigned int WritePwmConf = 8;			// stealthchop and freewheel control
	static constexpr unsigned int WriteGstat = 9;			// global status register (writing it resets status bits)
#if TMC_TYPE == 5160
	static constexpr unsigned int Write5160ShortConf = 10;	// short circuit detection configuration
	static constexpr unsigned int Write5160DrvConf = 11;		// driver timing
	static constexpr unsigned int Write5160GlobalScaler = 12; // motor current scaling

	static constexpr unsigned int NumWriteRegisters = 13;	// the number of registers that we write to
#else
	static constexpr unsigned int NumWriteRegisters = 10;	// the number of registers that we write to
#endif
	static constexpr unsigned int WriteSpecial = NumWriteRegisters;
	static constexpr unsigned int WriteAll5160 = ((1u << NumWriteRegisters) - 1) & ~(1 << WriteXDirect);
	static constexpr unsigned int WriteAll2240 = ((1u << NumWriteRegisters) - 1) & ~((1 << Write5160ShortConf) | (1 << WriteXDirect));


	static const uint8_t WriteRegNumbers[NumWriteRegisters];	// the register numbers that we write to

	static constexpr unsigned int NumReadRegisters = 6;		// the number of registers that we read from
	static const uint8_t ReadRegNumbers[NumReadRegisters];	// the register numbers that we read from

	// Read register numbers, in same order as ReadRegNumbers
	static constexpr unsigned int ReadGStat = 0;
	static constexpr unsigned int ReadDrvStat = 1;
	static constexpr unsigned int ReadMsCnt = 2;
	static constexpr unsigned int ReadPwmScale = 3;
	static constexpr unsigned int ReadPwmAuto = 4;
	static constexpr unsigned int ReadAdcTemp = 5;
	static constexpr unsigned int ReadSpecial = NumReadRegisters;

	static constexpr uint8_t NoRegIndex = 0xFF;				// this means no register updated, or no register requested

	volatile uint32_t writeRegisters[NumWriteRegisters+1];	// the values we want the TMC22xx writable registers to have
	volatile uint32_t readRegisters[NumReadRegisters+1];	// the last values read from the TMC22xx readable registers
	volatile uint32_t accumulatedDriveStatus;				// the accumulated drive status bits

	uint32_t configuredChopConfReg;							// the configured chopper control register, in the Enabled state, without the microstepping bits
	uint32_t maxStallStepInterval;							// maximum interval between full steps to take any notice of stall detection
	uint32_t minSgLoadRegister;								// the minimum value of the StallGuard bits we read

	std::atomic<uint32_t> newRegistersToUpdate;				// bitmap of register indices whose values need to be sent to the driver chip
	std::atomic<uint32_t> registersToUpdate;								// bitmap of register indices whose values need to be sent to the driver chip
	LocalDriversBitmap driverBit;								// a bitmap containing just this driver number
	uint32_t axisNumber;									// the axis number of this driver as used to index the DriveMovements in the DDA
	uint32_t microstepShiftFactor;							// how much we need to shift 1 left by to get the current microstepping
	uint32_t motorCurrent;									// the configured motor current in mA

	uint16_t numReads, numWrites;							// how many successful reads and writes we had
	uint16_t numWriteErrors;								// how many write errors do we have

	int8_t currentScaler = -1;								// CS if manually specified, otherwise -1 to indicate auto calculate
	uint16_t standstillCurrentFraction;						// divide this by 256 to get the motor current standstill fraction
	uint8_t regIndexBeingUpdated;							// which register we are sending
	uint8_t regIndexRequested;								// the register we asked to read in the previous transaction, or 0xFF
	uint8_t previousRegIndexRequested;						// the register we asked to read in the previous transaction, or 0xFF
	volatile uint8_t specialReadRegisterNumber;
	volatile uint8_t specialWriteRegisterNumber;
	bool enabled;											// true if driver is enabled
	DriverType typ;

#if SUPPORT_PHASE_STEPPING
	inline static bool needToSetCoilCurrents = false;
	bool phaseStepEnabled = false;
	DriverMode currentMode;									// stepper driver mode if not using phase stepping
#endif

	float maxCurrent;
	float senseResistor;
};

const uint8_t Tmc51xxDriverState::WriteRegNumbers[NumWriteRegisters] =
{
	REGNUM_5160_X_DIRECT,
	REGNUM_GCONF,
	REGNUM_IHOLDIRUN,
	REGNUM_TPWMTHRS,
	REGNUM_TCOOLTHRS,
	REGNUM_THIGH,
	REGNUM_CHOPCONF,
	REGNUM_COOLCONF,
	REGNUM_PWMCONF,
	REGNUM_GSTAT,
#if TMC_TYPE == 5160
	REGNUM_5160_SHORTCONF,
	REGNUM_5160_DRVCONF,
	REGNUM_5160_GLOBAL_SCALER
#endif
};

const uint8_t Tmc51xxDriverState::ReadRegNumbers[NumReadRegisters] =
{
	REGNUM_GSTAT,
	REGNUM_DRV_STATUS,
	REGNUM_MSCNT,
	REGNUM_PWM_SCALE,
	REGNUM_PWM_AUTO,
	REGNUM_ADC_TEMP
};

Tmc51xxDriverState::Tmc51xxDriverState() noexcept : TmcDriverState(), accumulatedDriveStatus(0), configuredChopConfReg(0), maxStallStepInterval(0),
	minSgLoadRegister(0), registersToUpdate(0), axisNumber(0), microstepShiftFactor(0),
	motorCurrent(0), enabled(0), maxCurrent(0), senseResistor(0)
{
}

// Initialise the state of the driver and its CS pin
void Tmc51xxDriverState::Init(uint32_t p_driverNumber) noexcept
pre(!driversPowered)
{
	driverNumber = p_driverNumber;
	state = DriversState::powerWait;
	axisNumber = p_driverNumber;										// axes are mapped straight through to drivers initially
	driverBit = LocalDriversBitmap::MakeFromBits(p_driverNumber);
	enabled = false;
	registersToUpdate.store(0);
	newRegistersToUpdate.store(0);
	specialReadRegisterNumber = specialWriteRegisterNumber = 0xFF;
	motorCurrent = 0;
	senseResistor = Default5160SenseResistor;
	maxCurrent = DefaultMaxTmc5160Current;
	standstillCurrentFraction = (uint16_t)min<uint32_t>((DefaultStandstillCurrentPercent * 256)/100, 256);

#if SUPPORT_PHASE_STEPPING
	currentMode = DriverMode::spreadCycle;
#endif

	if (TMC_PINS[driverNumber] != NoPin)
		DisableChipSelect();

	// Set default values for all registers and flag them to be updated
	UpdateRegister(WriteGConf, DefaultGConfReg);
#if TMC_TYPE == 5160
	UpdateRegister(Write5160ShortConf, DefaultShortConfReg);
	UpdateRegister(Write5160DrvConf, DefaultDrvConfReg);
	UpdateRegister(Write5160GlobalScaler, DefaultGlobalScalerReg);
#endif
	UpdateRegister(WriteIholdIrun, DefaultIholdIrunReg);
	UpdateRegister(WriteTpwmthrs, DefaultTpwmthrsReg);
	UpdateRegister(WriteTcoolthrs, DefaultTcoolthrsReg);
	UpdateRegister(WriteThigh, DefaultThighReg);
	UpdateRegister(WriteGstat, DefaultGstatReg);
	configuredChopConfReg = DefaultChopConfReg;
	SetMicrostepping(DefaultMicrosteppingShift, DefaultInterpolation);	// this also updates the chopper control register
	writeRegisters[WriteCoolConf] = DefaultCoolConfReg;
#if HAS_STALL_DETECT
	SetStallDetectThreshold(DefaultStallDetectThreshold);				// this also updates the CoolConf register
	SetStallMinimumStepsPerSecond(DefaultMinimumStepsPerSecond);
#endif
	UpdateRegister(WritePwmConf, DefaultPwmConfReg);

	ResetReadRegisters();

	regIndexBeingUpdated = regIndexRequested = previousRegIndexRequested = NoRegIndex;
	numReads = numWrites = numWriteErrors = 0;
	ResetLoadRegisters();
}

// Set a register value and flag it for updating
void Tmc51xxDriverState::UpdateRegister(size_t regIndex, uint32_t regVal) noexcept
{
	writeRegisters[regIndex] = regVal;
	newRegistersToUpdate.fetch_or(1u << regIndex);							// flag it for sending
}

// Calculate the chopper control register and flag it for sending
void Tmc51xxDriverState::UpdateChopConfRegister() noexcept
{
	UpdateRegister(WriteChopConf, (enabled) ? configuredChopConfReg : configuredChopConfReg & ~CHOPCONF_TOFF_MASK);
}

inline void Tmc51xxDriverState::SetAxisNumber(size_t p_axisNumber) noexcept
{
	axisNumber = p_axisNumber;
}

// Write all registers. This is called when the drivers are known to be powered up.
inline void Tmc51xxDriverState::WriteAll() noexcept
{
	newRegistersToUpdate.store(IsTmc2240() ? WriteAll2240 : WriteAll5160);
}

float Tmc51xxDriverState::GetStandstillCurrentPercent() const noexcept
{
	return (float)(standstillCurrentFraction * 100)/256;
}

void Tmc51xxDriverState::SetStandstillCurrentPercent(float percent) noexcept
{
	standstillCurrentFraction = (uint16_t)constrain<long>(lrintf((percent * 256)/100.0), 0, 256);
	UpdateCurrent();
}

bool Tmc51xxDriverState::SetCurrentScaler(int8_t cs) noexcept
{
	if (cs > 31)
	{
		return false;
	}

	if (cs < 0)
	{
		cs = -1;
	}

	currentScaler = cs;
	UpdateCurrent();

	return true;
}

// Set the microstepping and microstep interpolation. The desired microstepping is (1 << shift) where shift is in 0..8.
bool Tmc51xxDriverState::SetMicrostepping(uint8_t shift, bool interpolate) noexcept
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
unsigned int Tmc51xxDriverState::GetMicrostepping(bool& interpolation) const noexcept
{
	interpolation = (configuredChopConfReg & CHOPCONF_INTPOL) != 0;
	return 1u << microstepShiftFactor;
}

// Check that stall detection can occur at the specified speed
const char *_ecv_array _ecv_null Tmc51xxDriverState::CheckStallDetectionEnabled(float speed) noexcept
{
	if (GetDriverMode() > DriverMode::spreadCycle)			// if in stealthChop or direct mode
	{
		return "driver %u is not in spreadCycle mode";
	}
	if (speed * (float)maxStallStepInterval < (float)(1u << microstepShiftFactor))
	{
		return "move is too slow for driver %u to detect stall (increase speed or reduce M915 H parameter)";
	}
#if 0	// the Tpwmthrs setting affects the DIAG pin output but not the stall detection that we read over SPI, so we must not check the following
	if (speed * (float)StepClockRate * (float)writeRegisters[WriteTpwmthrs] > (float)((GetLowestTmcClockSpeed()/256) << microstepShiftFactor))
	{
		return "move is too fast for driver %u to detect stall (reduce speed or M569 V parameter)";
	}
#endif
	return nullptr;
}

bool Tmc51xxDriverState::SetRegister(SmartDriverRegister reg, uint32_t regVal) noexcept
{
	switch (reg)
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

	case SmartDriverRegister::thigh:
		UpdateRegister(WriteThigh, regVal & ((1u << 20) - 1));
		return true;

	case SmartDriverRegister::coolStep:
		UpdateRegister(WriteCoolConf, (writeRegisters[WriteCoolConf] & ~COOLCONF_COOL_MASK) | (regVal & COOLCONF_COOL_MASK));
		return true;

	case SmartDriverRegister::hdec:
	default:
		return false;
	}
}

uint32_t Tmc51xxDriverState::GetRegister(SmartDriverRegister reg) const noexcept
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

	case SmartDriverRegister::tcoolthrs:
		return writeRegisters[WriteTcoolthrs] & 0x000FFFFF;

	case SmartDriverRegister::coolStep:
		return writeRegisters[WriteCoolConf];

	case SmartDriverRegister::mstepPos:
		return readRegisters[ReadMsCnt];

	case SmartDriverRegister::pwmScale:
		return readRegisters[ReadPwmScale];

	case SmartDriverRegister::pwmAuto:
		return readRegisters[ReadPwmAuto];

	case SmartDriverRegister::hdec:
	default:
		return 0;
	}
}

// This will return GCodeResult:notFinished for at least the first call if the driver number is valid, so it must be called repeatedly until it returns a different value.
GCodeResult Tmc51xxDriverState::GetAnyRegister(const StringRef& reply, uint8_t regNum) noexcept
{
	if (specialReadRegisterNumber == 0xFE)		// this value indicates that the register has been read and the value stored
	{
		reply.printf("Register 0x%02x value 0x%08" PRIx32, regNum, readRegisters[ReadSpecial]);
		specialReadRegisterNumber = 0xFF;
		return GCodeResult::ok;
	}

	if (specialReadRegisterNumber == 0xFF)
	{
		specialReadRegisterNumber = regNum;
	}
	return GCodeResult::notFinished;			// else a read is already in progress
}

GCodeResult Tmc51xxDriverState::SetAnyRegister(const StringRef& reply, uint8_t regNum, uint32_t regVal) noexcept
{
	for (size_t i = 0; i < NumWriteRegisters; ++i)
	{
		if (regNum == WriteRegNumbers[i])
		{
			if (i == WriteChopConf)
			{
				// We keep a copy of the contents of ChopConf so we need to update that to make the new
				// value "stick"
				configuredChopConfReg = regVal;
			}
			UpdateRegister(i, regVal);
			return GCodeResult::ok;
		}
	}
	specialWriteRegisterNumber = regNum;
	UpdateRegister(WriteSpecial, regVal);
	return GCodeResult::ok;
}


// Set the chopper control register to the settings provided by the user. We allow only the lowest 17 bits to be set.
bool Tmc51xxDriverState::SetChopConf(uint32_t newVal) noexcept
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

// Set the driver mode, returning true if successful
bool Tmc51xxDriverState::SetDriverMode(unsigned int mode) noexcept
{
	switch (mode)
	{
	case (unsigned int)DriverMode::spreadCycle:
		UpdateRegister(WriteGConf, writeRegisters[WriteGConf] & ~(GCONF_DIRECT_MODE | GCONF_STEALTHCHOP));
#if TMC_TYPE == 5130
		configuredChopConfReg = &= ~(CHOPCONF_CHM | CHOPCONF_5130_RNDTOFF);
#else
		configuredChopConfReg &= ~CHOPCONF_CHM;
#endif
		UpdateChopConfRegister();
		break;

	case (unsigned int)DriverMode::stealthChop:
		UpdateRegister(WriteGConf, (writeRegisters[WriteGConf] & ~GCONF_DIRECT_MODE) | GCONF_STEALTHCHOP);
#if TMC_TYPE == 5130
		configuredChopConfReg = &= ~(CHOPCONF_CHM | CHOPCONF_5130_RNDTOFF);
#else
		configuredChopConfReg &= ~CHOPCONF_CHM;
#endif
		UpdateChopConfRegister();
		break;

	case (unsigned int)DriverMode::constantOffTime:
		UpdateRegister(WriteGConf, writeRegisters[WriteGConf] & ~(GCONF_DIRECT_MODE | GCONF_STEALTHCHOP));
#if TMC_TYPE == 5130
		configuredChopConfReg = (configuredChopConfReg & ~CHOPCONF_5130_RNDTOFF) | CHOPCONF_CHM;
#else
		configuredChopConfReg |= CHOPCONF_CHM;
#endif
		UpdateChopConfRegister();
		break;

#if TMC_TYPE == 5130
	case (unsigned int)DriverMode::randomOffTime:
		UpdateRegister(WriteGConf, writeRegisters[WriteGConf] & ~GCONF_STEALTHCHOP);
		configuredChopConfReg |= CHOPCONF_CHM | CHOPCONF_5130_RNDTOFF;
		UpdateChopConfRegister();
		break;
#endif

	default:
		return false;
	}
#if SUPPORT_PHASE_STEPPING
	currentMode = (DriverMode)mode;
#endif
	return true;
}

// Get the driver mode
DriverMode Tmc51xxDriverState::GetDriverMode() const noexcept
{
	return ((writeRegisters[WriteGConf] & GCONF_STEALTHCHOP) != 0) ? DriverMode::stealthChop
		: ((configuredChopConfReg & CHOPCONF_CHM) == 0) ? DriverMode::spreadCycle
#if TMC_TYPE == 5130
			: ((configuredChopConfReg & CHOPCONF_5130_RNDTOFF) != 0) ? DriverMode::randomOffTime
#endif
				: DriverMode::constantOffTime;
}


// Set the motor current
void Tmc51xxDriverState::SetCurrent(float current) noexcept
{
	motorCurrent = static_cast<uint32_t>(constrain<float>(current, MinimumMotorCurrent, maxCurrent));
	UpdateCurrent();
}

float Tmc51xxDriverState::CalculateCurrent() const noexcept
{
	float RecipFullScaleCurrent;
	if (IsTmc2240())
	{
		constexpr float Tmc2240Kifs = (Tmc2240CurrentRange == 0b00) ? 11.75f : (Tmc2240CurrentRange == 0b01) ? 24.0f : 36.0f;
		// Note datasheet uses rRef/sense ik KOhms we hold it in Ohms hence / 1000.0f
		RecipFullScaleCurrent = (senseResistor / 1000.0f) / (Tmc2240Kifs * 1000.0f);	// reciprocal of full scale current in mA
	}
	else
		RecipFullScaleCurrent = senseResistor/Vfs;
	const uint32_t globalScaler = GetGlobalScaler();
	const uint32_t gs = (globalScaler == 0) ? 256 : globalScaler;
	return (float)(gs * (GetIRun() + 1)) / (256 * 32 * RecipFullScaleCurrent);
}

void Tmc51xxDriverState::UpdateCurrent() noexcept
{
#if TMC_TYPE == 5130
	// Assume a current sense resistor of 0.082 ohms, to which we must add 0.025 ohms internal resistance.
	// Full scale peak motor current in the high sensitivity range is give by I = 0.18/(R+0.03) = 0.18/0.105 ~= 1.6A
	// This gives us a range of 50mA to 1.6A in 50mA steps in the high sensitivity range (VSENSE = 1)
	const uint32_t iRunCsBits = (32 * motorCurrent - 800)/1615;		// formula checked by simulation on a spreadsheet
	const uint32_t iHoldCurrent = (motorCurrent * standstillCurrentFraction)/256;	// set standstill current
	const uint32_t iHoldCsBits = (32 * iHoldCurrent - 800)/1615;	// formula checked by simulation on a spreadsheet
	UpdateRegister(WriteIholdIrun,
					(writeRegisters[WriteIholdIrun] & ~(IHOLDIRUN_IRUN_MASK | IHOLDIRUN_IHOLD_MASK)) | (iRunCsBits << IHOLDIRUN_IRUN_SHIFT) | (iHoldCsBits << IHOLDIRUN_IHOLD_SHIFT));
#elif TMC_TYPE == 5160
	float RecipFullScaleCurrent;
	if (IsTmc2240())
	{
		constexpr float Tmc2240Kifs = (Tmc2240CurrentRange == 0b00) ? 11.75f : (Tmc2240CurrentRange == 0b01) ? 24.0f : 36.0f;
		// Note datasheet uses rRef/sense ik KOhms we hold it in Ohms hence / 1000.0f
		RecipFullScaleCurrent = (senseResistor / 1000.0f) / (Tmc2240Kifs * 1000.0f);	// reciprocal of full scale current in mA
	}
	else
		RecipFullScaleCurrent = senseResistor/Vfs;
	// See if we can set IRUN to 31 (or user defined value) and do the current adjustment in the global scaler
	uint8_t iRun = (currentScaler < 0) ? 31 : (uint8_t)currentScaler;

	const float csRecip = (iRun == 31) ? 1.0f : 32.0f / (float)(iRun + 1);
	uint32_t globalScaler = lrintf(motorCurrent * 256 * RecipFullScaleCurrent * csRecip);
	if (globalScaler >= 256)
	{
		const uint32_t prod = globalScaler * (iRun + 1);
		globalScaler = 0;
		iRun = (uint8_t)constrain<float>(rintf(float(prod) / 256u) - 1, 0, 31);		// globalscaler = 0 means 256
	}
	else if (globalScaler < 32)
	{
		// We can't regulate the current just through the global scaler because it has a minimum value of 32
		const uint32_t prod = globalScaler * (iRun + 1);
		globalScaler = 32;
		iRun = (uint8_t)constrain<float>(rintf(float(prod) / globalScaler) - 1, 0, 31);
	}

	// At high motor currents, limit the standstill current fraction to avoid overheating particular pairs of mosfets. Avoid dividing by zero if motorCurrent is zero.
	const uint32_t desiredStandstillCurrentFraction =
#if SUPPORT_PHASE_STEPPING
				(phaseStepEnabled) ? 256 : standstillCurrentFraction;
#else
				standstillCurrentFraction;
#endif

	const uint32_t MaxStandstillCurrentTimes256 = 256 * (uint32_t)(maxCurrent * 0.707); // *0.707 to convert peak to RMS ;
	const uint32_t limitedStandstillCurrentFraction = (motorCurrent * desiredStandstillCurrentFraction <= MaxStandstillCurrentTimes256)
														? desiredStandstillCurrentFraction
															: MaxStandstillCurrentTimes256/motorCurrent;
	const uint8_t iHold = (iRun * limitedStandstillCurrentFraction)/256;
	UpdateRegister(WriteIholdIrun, (writeRegisters[WriteIholdIrun] & ~(IHOLDIRUN_IRUN_MASK | IHOLDIRUN_IHOLD_MASK)) | (iRun << IHOLDIRUN_IRUN_SHIFT) | (iHold << IHOLDIRUN_IHOLD_SHIFT));
	UpdateRegister(Write5160GlobalScaler, globalScaler);
#else
# error unknown device
#endif
}

// Enable or disable the driver
void Tmc51xxDriverState::Enable(bool en) noexcept
{
	if (enabled != en)
	{
		enabled = en;
		UpdateChopConfRegister();
	}
}

float Tmc51xxDriverState::GetSenseResistor() const noexcept
{
	return senseResistor;
}

void Tmc51xxDriverState::SetSenseResistor(float value) noexcept
{
	if (value > 0.0f) senseResistor = value;
	// Max current may have changed due to sense resistor change
	SetMaxCurrent(maxCurrent);
}

float Tmc51xxDriverState::GetMaxCurrent() const noexcept
{
	return maxCurrent;
}

void Tmc51xxDriverState::SetMaxCurrent(float value) noexcept
{
	if (IsTmc2240())
		maxCurrent = constrain<float>(value, 0.0f, (Tmc2240CurrentRange == 0b00) ? 1000.0f : (Tmc2240CurrentRange == 0b01) ? 2000.0f : 3000.0f);
	else
		maxCurrent = constrain<float>(value, 0.0f, Vfs/senseResistor);
	SetCurrent(motorCurrent);
}


// Read the status
StandardDriverStatus Tmc51xxDriverState::GetStatus(bool accumulated, bool clearAccumulated) noexcept
{
	if (!IsReady())
	{
		StandardDriverStatus rslt;
		rslt.all = 0;
		rslt.notPresent = true;
		return rslt;
	}

	uint32_t status;
	if (accumulated)
	{
		AtomicCriticalSectionLocker lock;

		// In the following we must or-in the current drive status, otherwise an error such as S2G may appear to go away between two successive calls
		status = accumulatedDriveStatus | readRegisters[ReadDrvStat];
		if (clearAccumulated)
		{
			// In the following we can't just copy readRegisters[ReadDrvStat] into accumulatedDriveStatus, because we only want to set bits in accumulatedDriveStatus
			// when they occur in 2 successive samples. So clear it instead.
			accumulatedDriveStatus = 0;
		}
	}
	else
	{
		status = readRegisters[ReadDrvStat];
	}

	// The lowest 8 bits of StandardDriverStatus have the same meanings as for the TMC2209 status, but the TMC51xx uses different bit assignments
	StandardDriverStatus rslt;
	rslt.all =  ExtractBit(status, TMC_RR_OTPW_BIT_POS, StandardDriverStatus::OtpwBitPos);
	rslt.all |= ExtractBit(status, TMC_RR_OT_BIT_POS, StandardDriverStatus::OtBitPos);
	rslt.all |= ExtractTwoBits(status, TMC_RR_S2G_BIT_POS, StandardDriverStatus::S2gBitsPos);		// put the s2ga and s2gb bits in the right place
	rslt.all |= ExtractTwoBits(status, TMC_RR_S2VS_BIT_POS, StandardDriverStatus::S2vsBitsPos);		// put s2vsa and s2vsb in the right place
	rslt.all |= ExtractTwoBits(status, TMC_RR_OL_BIT_POS, StandardDriverStatus::OpenLoadBitsPos);	// put ola and olb in the right place
	rslt.all |= ExtractBit(status, TMC_RR_STST_BIT_POS, StandardDriverStatus::StandstillBitPos);	// put the standstill bit in the right place
	rslt.all |= ExtractBit(status, TMC_RR_SG_BIT_POS, StandardDriverStatus::StallBitPos);			// put the stall bit in the right place
	rslt.sgresultMin = minSgLoadRegister;
	return rslt;
}


// Append any additonal driver status to a string, and reset the min/max load values
void Tmc51xxDriverState::AppendDriverStatus(const StringRef& reply) noexcept
{
	if (!IsReady())
	{
		return;
	}
	reply.catf(IsTmc2240() ? " 2240" : " 5160");
	if (minSgLoadRegister <= MaxValidSgLoadRegister)
	{
		reply.catf(", SG min %u", (unsigned)minSgLoadRegister);
	}
	else
	{
		reply.cat(", SG min n/a");
	}
	ResetLoadRegisters();
	if (IsTmc2240())
	{
		reply.catf(", temp %.1fC", (double)GetDriverTemperature());
	}

	reply.catf(", mspos %u, reads %u, writes %u", (unsigned int)(readRegisters[ReadMsCnt] & 1023), numReads, numWrites);
	if (numWriteErrors > 0)
		reply.catf(", write errors %u", numWriteErrors);
	numReads = numWrites = numWriteErrors = 0;
}

float Tmc51xxDriverState::GetDriverTemperature() noexcept
{
	if (IsTmc2240())
	{
		return((float)(((readRegisters[ReadAdcTemp] & ADC_TEMP_MASK) >> ADC_TEMP_SHIFT) - 2038)/7.7);
	}
	else
	{
		uint32_t status = readRegisters[ReadDrvStat];

		return (status & TMC_RR_OT ? 150.0f : status & TMC_RR_OTPW ? 100.0f : 0.0f);
	}
}

uint32_t Tmc51xxDriverState::GetDriverMaxClockFrequency() noexcept
{
	return HighestTmcClockSpeed;
}

uint32_t Tmc51xxDriverState::GetDriverNominalClockFrequency() noexcept
{
	return NominalTmcClockSpeed;
}

uint32_t Tmc51xxDriverState::GetDriverMinClockFrequency() noexcept
{
	return LowestTmcClockSpeed;
}

#if HAS_STALL_DETECT
void Tmc51xxDriverState::SetStallDetectThreshold(int sgThreshold) noexcept
{
	const uint32_t sgVal = ((uint32_t)constrain<int>(sgThreshold, -64, 63)) & 127u;
	writeRegisters[WriteCoolConf] = (writeRegisters[WriteCoolConf] & ~COOLCONF_SGT_MASK) | (sgVal << COOLCONF_SGT_SHIFT);
	newRegistersToUpdate.fetch_or(1u << WriteCoolConf);
}

void Tmc51xxDriverState::SetStallDetectFilter(bool sgFilter) noexcept
{
	if (sgFilter)
	{
		writeRegisters[WriteCoolConf] |= COOLCONF_SGFILT;
	}
	else
	{
		writeRegisters[WriteCoolConf] &= ~COOLCONF_SGFILT;
	}
	newRegistersToUpdate.fetch_or(1u << WriteCoolConf);
}

void Tmc51xxDriverState::SetStallMinimumStepsPerSecond(unsigned int stepsPerSecond) noexcept
{
	if (stepsPerSecond == 0) { stepsPerSecond = 1; }					// avoid divide-by-zero errors
	maxStallStepInterval = StepClockRate/stepsPerSecond;
	UpdateRegister(WriteTcoolthrs, (GetHighestTmcClockSpeed() + (128 * stepsPerSecond))/(256 * stepsPerSecond));
}

void Tmc51xxDriverState::AppendStallConfig(const StringRef& reply) const noexcept
{
	const bool filtered = ((writeRegisters[WriteCoolConf] & COOLCONF_SGFILT) != 0);
	int threshold = (int)((writeRegisters[WriteCoolConf] & COOLCONF_SGT_MASK) >> COOLCONF_SGT_SHIFT);
	if (threshold >= 64)
	{
		threshold -= 128;
	}
	const uint32_t fullstepsPerSecond = StepClockRate/maxStallStepInterval;
	const float stepsPerMm = GetMoveInstance().DriveStepsPerMm(axisNumber);
	const float speed1 = (float)(fullstepsPerSecond << microstepShiftFactor)/stepsPerMm;
	const uint32_t tcoolthrs = writeRegisters[WriteTcoolthrs] & ((1ul << 20) - 1u);
	bool bdummy;
	const float speed2 = ((float)GetHighestTmcClockSpeed() * GetMicrostepping(bdummy))/(256 * tcoolthrs * stepsPerMm);
	reply.catf("stall threshold %d, filter %s, full steps/sec %" PRIu32 " (%.1f mm/sec), coolstep threshold %" PRIu32 " (%.1f mm/sec)",
				threshold, ((filtered) ? "on" : "off"), fullstepsPerSecond, (double)speed1, tcoolthrs, (double)speed2);
}

#endif

// In the following, only byte accesses to sendDataBlock are allowed, because accesses to non-cacheable memory must be aligned
bool Tmc51xxDriverState::GetSpiCommand(uint8_t *sendDataBlock, bool forceRead) noexcept
{
	// Find which register to send. The common case is when no registers need to be updated.
	const uint32_t locRegistersToUpdate = (registersToUpdate |= newRegistersToUpdate.exchange(0));
	if (locRegistersToUpdate == 0 || forceRead)
	{
		// Read a register
		regIndexBeingUpdated = NoRegIndex;
		if (regIndexRequested >= ReadSpecial)
		{
			regIndexRequested = 0;
		}
		else
		{
			++regIndexRequested;
			if (regIndexRequested == ReadSpecial && specialReadRegisterNumber >= 0x80)
			{
				regIndexRequested = 0;
			}
		}

		sendDataBlock[0] = (regIndexRequested == ReadSpecial) ? specialReadRegisterNumber : ReadRegNumbers[regIndexRequested];
		sendDataBlock[1] = 0;
		sendDataBlock[2] = 0;
		sendDataBlock[3] = 0;
		sendDataBlock[4] = 0;
		return false;
	}
	else
	{
		// Write a register
		const size_t regNum = LowestSetBit(locRegistersToUpdate);
		regIndexBeingUpdated = regNum;
		sendDataBlock[0] = ((regNum == WriteSpecial) ? specialWriteRegisterNumber : WriteRegNumbers[regNum]) | 0x80;
		StoreBEU32(sendDataBlock + 1, writeRegisters[regNum]);
		return true;
	}
}

void Tmc51xxDriverState::TransferSucceeded(const uint8_t *rcvDataBlock, bool checkWrite) noexcept
{
	// If we wrote a register, mark it up to date
	if (regIndexBeingUpdated < NumWriteRegisters)
	{
		registersToUpdate &= ~(1u << regIndexBeingUpdated);
		++numWrites;
	}

	// Get the full step interval, we will need it later
	const uint32_t interval = GetMoveInstance().GetStepInterval(axisNumber, microstepShiftFactor);		// get the full step interval

	// If we read a register, update our copy
	if (previousRegIndexRequested <= NumReadRegisters)
	{
		++numReads;
		uint32_t regVal = LoadBEU32(rcvDataBlock + 1);
		if (previousRegIndexRequested == ReadDrvStat)
		{
			// We treat the DRV_STATUS register separately
			if ((regVal & TMC_RR_STST) == 0)							// in standstill, SG_RESULT returns the chopper on-time instead
			{
				const uint16_t sgResult = regVal & TMC_RR_SGRESULT;
				if (sgResult < minSgLoadRegister)
				{
					minSgLoadRegister = sgResult;
				}
			}

			if ((regVal & TMC_RR_OL) != 0)
			{
				if (   (regVal & TMC_RR_STST) != 0
					|| interval == 0
					|| interval > StepClockRate/MinimumOpenLoadFullStepsPerSec
					|| motorCurrent < MinimumOpenLoadMotorCurrent
				   )
				{
					regVal &= ~TMC_RR_OL;				// open load bits are unreliable at standstill, low speeds, and low current
				}
			}

			// Only add bits to the accumulator if they appear in 2 successive samples. This is to avoid seeing transient S2G, S2VS, STST and open load errors.
			const uint32_t oldDrvStat = readRegisters[ReadDrvStat];
			readRegisters[ReadDrvStat] = regVal;
			regVal &= oldDrvStat;
			accumulatedDriveStatus |= regVal;
		}
		else
		{
			readRegisters[previousRegIndexRequested] = regVal;
			if (previousRegIndexRequested == ReadSpecial)
			{
				specialReadRegisterNumber = 0xFE;
			}
		}
	}
	else if (checkWrite && previousRegIndexRequested != NoRegIndex)
	{
		// we have read the result from a previous write, validate it
		uint32_t regVal = LoadBEU32(rcvDataBlock + 1);
		previousRegIndexRequested -= (NumReadRegisters + 1);
		if (writeRegisters[previousRegIndexRequested] != regVal)
		{
			debugPrintf("TMC5160: Write error driver %d register index %d expected %x got %x\n", driverNumber, previousRegIndexRequested, (unsigned)writeRegisters[previousRegIndexRequested], (unsigned)regVal); 
			numWriteErrors++;
			// retry the write
			registersToUpdate |= (1u << previousRegIndexRequested);
		}

	}

	// Deal with the stall status. Note that the TCoolThrs setting prevents us getting a DIAG output at low speeds, but it doesn't seem to affect the stall status
	if (   (rcvDataBlock[0] & (1u << 2)) != 0							// if the status indicates stalled
		&& interval != 0
		&& interval <= maxStallStepInterval								// if the motor speed is high enough to get a reliable stall indication
	   )
	{
		readRegisters[ReadDrvStat] |= TMC_RR_SG;
		accumulatedDriveStatus |= TMC_RR_SG;
#if SUPPORT_REMOTE_COMMANDS
		if (CanInterface::InExpansionMode())
		{
			if (stallEndstopsEnabled.IsBitSet(driverNumber))
			{
				SmartDrivers::stallEndstopsEnabled.ClearBit(driverNumber);
				SmartDrivers::driverStallsToNotify |= 1u << driverNumber;
				CanInterface::WakeAsyncSender();
			}
		}
		else
#endif
		{
			EndstopOrZProbe::SetDriversStalled(driverBit);
		}
	}
	else
	{
		readRegisters[ReadDrvStat] &= ~TMC_RR_SG;
		EndstopOrZProbe::SetDriversNotStalled(driverBit);
	}

	previousRegIndexRequested = (regIndexBeingUpdated == NoRegIndex) ? regIndexRequested : regIndexBeingUpdated + NumReadRegisters + 1;
}

void Tmc51xxDriverState::TransferFailed() noexcept
{
	regIndexRequested = previousRegIndexRequested = NoRegIndex;
}

// State structures for all drivers
static Tmc51xxDriverState *driverStates = nullptr;
// TMC51xx management task
static TASKMEM Task<TmcTaskStackWords> tmcTask;

#if SUPPORT_PHASE_STEPPING || SUPPORT_CLOSED_LOOP
static bool usePhaseStepping = false;
#endif

static SharedSpiClient *spiDevice;
static StepTimer tmcTimer;

// Declare the DMA buffers with the __nocache attribute. Access to these must be aligned.
__nocache uint8_t sendData[5];
__nocache uint8_t rcvData[5];


#if SUPPORT_PHASE_STEPPING || SUPPORT_CLOSED_LOOP
inline bool Tmc51xxDriverState::SetXdirect(uint32_t regVal) noexcept
{
	if (regVal != writeRegisters[WriteXDirect])
	{
		UpdateRegister(WriteXDirect, regVal);
		needToSetCoilCurrents = true;
		return true;
	}
	return false;
}

#endif

bool Tmc51xxDriverState::DoIo(bool forceRead, bool checkWrite) noexcept
{
	if (IsActive())
	{
		EnableChipSelect();
		bool isWrite = GetSpiCommand(sendData, forceRead);
		spiDevice->TransceivePacket(sendData, rcvData, 5);
		DisableChipSelect();
		TransferSucceeded(rcvData, checkWrite);
		return isWrite;
	}
	return false;
}


int32_t Tmc51xxDriverState::IdentifyDriver() noexcept
{
	if (specialReadRegisterNumber == 0xFE)
	{
		specialReadRegisterNumber = 0xFF;
		return (readRegisters[ReadSpecial] & IOIN_VERSION_MASK) >> IOIN_VERSION_SHIFT;
	}

	if (specialReadRegisterNumber == 0xFF)
	{
		// Force next read to be this one
		regIndexRequested = ReadSpecial - 1;
		specialReadRegisterNumber = REGNUM_IOIN;
	}
	return -1;
}

void Tmc51xxDriverState::ResetReadRegisters() noexcept
{
	accumulatedDriveStatus = 0;
	for(size_t i = 0; i < NumReadRegisters; i++)
		readRegisters[i] = 0;
}

void Tmc51xxDriverState::ResetDriver() noexcept
{
	// Start the identification/setup process
	accumulatedDriveStatus = 0;
	if (TMC_PINS[driverNumber] == NoPin || TMC_DRIVER_TYPE[driverNumber] <= DriverType::stepdir)
	{
		state = DriversState::noDriver;
		typ = DriverType::none;
	}
	else
	{
		numReads = numWrites = numWriteErrors = 0;
		ResetReadRegisters();
		newRegistersToUpdate.store(0);
		typ = DriverType::unknown;
		// Ask for hardware version
		IdentifyDriver();
		state = DriversState::initialising;
	}
}

DriversState Tmc51xxDriverState::SetupDriver() noexcept
{
	if (state == DriversState::notInitialised)
	{
		debugPrintf("SetupDriver invalid state\n");
		return state;
	}
	if (state == DriversState::noDriver)
		return state;
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
				if (version == IOIN_VERSION_2240)
				{
					// We have a TMC2240, adjust settings from assumed 5160
					typ = DriverType::tmc2240;
					UpdateRegister(WriteGConf, writeRegisters[WriteGConf] & ~(GCONF_5160_RECAL));
					UpdateRegister(WriteIholdIrun, writeRegisters[WriteIholdIrun] | (0x4 << IHOLDIRUN2240_IRUNDELAY_SHIFT));
					UpdateRegister(Write5160DrvConf, DefaultDrvConfReg2240);
					UpdateRegister(WritePwmConf, DefaultPwmConfReg2240);
					senseResistor = DefaultTmc2240Rref;
					maxCurrent = DefaultMaxTmc2240Current;

				}
				else if (version == IOIN_VERSION_5160)
				{
					typ = DriverType::tmc5160;
				}
				else
				{
					// We could potentially stop here and declare the driver unknown, but for now
					// we issue a warning and assume it is a 5160.
					debugPrintf("TMCSPI:: Warning driver %d unknown version number 0x%x\n", driverNumber, (unsigned)version);
					typ = DriverType::tmc5160;
				}
				// did our discovery match the request driver type?
				if (TMC_DRIVER_TYPE[driverNumber] != DriverType::tmcspiauto && TMC_DRIVER_TYPE[driverNumber] != DriverType::tmcauto && typ != TMC_DRIVER_TYPE[driverNumber])
				{
					debugPrintf("TMCSPI:: Warning driver %d type mismatch requested %s actual %s\n", driverNumber, TMC_DRIVER_TYPE[driverNumber].ToString(), typ.ToString());
					typ = TMC_DRIVER_TYPE[driverNumber];
				}
				WriteAll();
			}
		}
	}		

	if (DoIo(false, true))
	{
		// For some reason on the tmc2240, the value returned on the next operation after a write
		// is corrupted if there is any spi bus activity between the write and the following operation
		// to allow us to verify the write, we issue a read operation immeadiately following the write.
		DoIo(true, true);
	}

	// check for errors
	if (numWriteErrors > NumWriteRegisters)
	{
		debugPrintf("TMCSPI: Too many write errors drive %d error cnt %d driver disabled\n", driverNumber, numWriteErrors);
		// Too many write errors, probably means no driver or config error
		ResetReadRegisters();
		state = DriversState::noDriver;
		return state;
	}
	if (numReads >= NumReadRegisters + NumWriteRegisters + 3)
	{
		state = DriversState::ready;
	}
	return state;
}

static void TmcTimerCallback(CallbackParameter) noexcept
{
	tmcTask.GiveFromISR(NotifyIndices::Tmc);
}

extern "C" [[noreturn]] void TmcLoop(void *) noexcept
{
	tmcTimer.SetCallback(TmcTimerCallback, (CallbackParameter)0);
	uint32_t lastWakeupTime;
	size_t driverToPoll = 0;
	for (;;)
	{
		lastWakeupTime = StepTimer::GetTimerTicks();
		if (driversState == DriversState::ready)
		{
			spiDevice->Select(100);
#if SUPPORT_PHASE_STEPPING || SUPPORT_CLOSED_LOOP
			if (usePhaseStepping)
			{
				// Set the motor phase currents before we write them
				GetMoveInstance().PhaseStepControlLoop();
				// If we have any steps to make do them.
				if (Tmc51xxDriverState::NeedCoilCurrentSet())
				{
					for (size_t i = 0; i < numTmc51xxDrivers; ++i)
					{
						Tmc51xxDriverState& drv = driverStates[i];
						if (drv.IsPhaseSteppingEnabled())
							drv.DoIo();
					}
				}
			}
#endif
			// Do normal I/O operations
			for (size_t i = 0; i < numDriversToPoll; ++i)
			{
				Tmc51xxDriverState& drv = driverStates[driverToPoll];
				drv.DoIo();
				driverToPoll = (driverToPoll + 1) % numTmc51xxDrivers;
			}
			spiDevice->Deselect();
		}
		else if (driversState <= DriversState::noDriver)
		{
			if (driversState != DriversState::noDriver) driversState = DriversState::powerWait;
			TaskBase::TakeIndexed(NotifyIndices::Tmc);
			lastWakeupTime = StepTimer::GetTimerTicks();
			driverToPoll = 0;
		}
		else if (driversState == DriversState::notInitialised)
		{
			for (size_t i = 0; i < numTmc51xxDrivers; ++i)
			{
				driverStates[i].ResetDriver();
			}
			driversState = DriversState::initialising;
		}
		else if (driversState == DriversState::initialising)
		{
			// If all drivers that share the global enable have been initialised, set the global enable
			bool allInitialised = true;
			spiDevice->Select(100);
			for (size_t i = 0; i < numTmc51xxDrivers; ++i)
			{
				if (driverStates[i].SetupDriver() == DriversState::initialising)
				{
					allInitialised = false;
				}
			}
			spiDevice->Deselect();
			delay(1);
			if (allInitialised)
			{
				size_t readyCnt = 0;
				for (size_t i = 0; i < numTmc51xxDrivers; ++i)
				{
					if (driverStates[i].IsReady())
					{
						digitalWrite(DriverEnablePins[driverStates[i].GetDriverNumber()], false);
						readyCnt++;
					}
				}
				driversState = (readyCnt ? DriversState::ready : DriversState::noDriver);
			}
		}
		// Give other tasks a chance to run.
		// We run the SPI bus at high speeds so that motor currents get updated as quickly as possible.
		// If we wake up as soon as the transfer has completed then we will use too much of the available CPU time.
		// So schedule a wakeup call instead. Try to make the wakeup interval regular.
		lastWakeupTime += DriversDirectSleepClocks;
		if (!tmcTimer.ScheduleCallback(lastWakeupTime))
		{
			TaskBase::TakeIndexed(NotifyIndices::Tmc);
		}
	}
}

#if SUPPORT_PHASE_STEPPING

bool Tmc51xxDriverState::EnablePhaseStepping(bool enable) noexcept
{
	// We can't do phase stepping if we need delays on cs
	if (SmartDriversSpiCsDelay > 0)
	{
		return false;
	}
	bool anyDriversUsingPhaseStepping = false;
	bool ret = false;
	phaseStepEnabled = enable;
	if (enable)
	{
		UpdateRegister(WriteGConf, (writeRegisters[WriteGConf] & ~GCONF_STEALTHCHOP) | GCONF_DIRECT_MODE);
		ret = true;;
	}
	else
	{
		ret = SetDriverMode((unsigned int)currentMode);
	}
	UpdateCurrent();		// when entering direct mode we need to update the standstill current
	if (!ret)
	{
		return false;
	}
	if (enable)
	{
		anyDriversUsingPhaseStepping = true;
	}
	else
	{
		for (size_t i = 0; i < numTmc51xxDrivers; i++)
		{
			if (driverStates[i].IsPhaseSteppingEnabled())
			{
				anyDriversUsingPhaseStepping = true;
			}
		}
	}

	usePhaseStepping = anyDriversUsingPhaseStepping;
	DriversDirectSleepClocks = anyDriversUsingPhaseStepping ? PhaseStepSpiSleepClocks : DefaultSpiSleepClocks;
	numDriversToPoll = anyDriversUsingPhaseStepping ? 1 : numTmc51xxDrivers;
	spiDevice->SetClockFrequency(anyDriversUsingPhaseStepping ? PhaseStepDriversSpiClockFrequency : DefaultDriversSpiClockFrequency);
	tmcTask.SetPriority(anyDriversUsingPhaseStepping ? TaskPriority::TmcPhaseStepPriority : TaskPriority::TmcPriority);
	return ret;
}
#endif

static void DisableAllDrivers() noexcept
{
	for (size_t i = 0; i < numTmc51xxDrivers; ++i)
	{
		digitalWrite(DriverEnablePins[driverStates[i].GetDriverNumber()], true);
	}
}

// Members of namespace SmartDrivers

// Initialise the driver interface and the drivers, leaving each drive disabled.
// It is assumed that the drivers are not powered, so driversPowered(true) must be called after calling this before the motors can be moved.
void Tmc51xxDriver::Init(size_t numDrivers) noexcept
{
	numTmc51xxDrivers = min<size_t>(numDrivers, MaxSmartDrivers);
	numDriversToPoll = numTmc51xxDrivers;
	if (driverStates != nullptr)
	{
		delete (uint8_t *)driverStates;
		driverStates = nullptr;
	}
	if (numTmc51xxDrivers == 0)
	{
		driversState = DriversState::noDriver;
		return;
	}

	driverStates = (Tmc51xxDriverState *)	new uint8_t[(sizeof(Tmc51xxDriverState )*numTmc51xxDrivers)];
	memset((void *)driverStates, 0, sizeof(Tmc51xxDriverState)*numTmc51xxDrivers);
	if (SmartDriversSpiChannel == SSPNONE)
	{
		debugPrintf("TMC5160 stepper.spiChannel has not been configured\n");
		numTmc51xxDrivers = 0;
		driversState = DriversState::ready;
		return;
	}
	if (!tmcTask.IsRunning())
	{
		spiDevice = new SharedSpiClient(SharedSpiDevice::GetSharedSpiDevice(SmartDriversSpiChannel), DefaultDriversSpiClockFrequency, SPI_MODE_3, NoPin, false);
		tmcTask.Create(TmcLoop, "TMC51xx", nullptr, TaskPriority::TmcPriority);
	}
	driversState = DriversState::powerWait;
}

// Shut down the drivers and stop any related interrupts
void Tmc51xxDriver::Exit() noexcept
{
	if (numTmc51xxDrivers > 0 && driversState != DriversState::shutDown)
	{
		DisableAllDrivers();
		// make sure that the task does not own the spi device when we kill it
		if (spiDevice->Select(100))
		{
			tmcTask.TerminateAndUnlink();
			spiDevice->Deselect();
		}
		else
		{
			debugPrintf("TMC51xx: Failed to select spi device on exit\n");
		}
	}
	driversState = DriversState::shutDown;						// prevent Spin() calls from doing anything
}


// Flag that the the drivers have been powered up or down
void Tmc51xxDriver::Spin(bool powered) noexcept
{
	if (numTmc51xxDrivers == 0) return;
	//TaskCriticalSectionLocker lock;

	if (powered)
	{
		if (driversState == DriversState::powerWait)
		{
			driversState = DriversState::notInitialised;
			tmcTask.Give(NotifyIndices::Tmc);									// wake up the TMC task because the drivers need to be initialised
			// Wait for them to be ready
			while (!IsReady() && driversState > DriversState::powerWait)
				delay(10);
		}
	}
	else if (driversState > DriversState::powerWait)
	{
		TurnDriversOff();
	}
}

// This is called from the tick ISR, possibly while Spin (with powered either true or false) is being executed
void Tmc51xxDriver::TurnDriversOff() noexcept
{
	if (numTmc51xxDrivers > 0 && driversState >= DriversState::noDriver)
	{
		DisableAllDrivers();
		driversState = DriversState::powerWait;
	}
}

bool Tmc51xxDriver::IsReady() noexcept
{
	return driversState == DriversState::ready || driversState == DriversState::noDriver;
}


TmcDriverState* Tmc51xxDriver::InitDrive(size_t slot, size_t driveNo) noexcept
{
	// init everything and return pointer to driver
	new(&driverStates[slot]) Tmc51xxDriverState();
	driverStates[slot].Init(driveNo);
	return &(driverStates[slot]);
}

#endif

// End
