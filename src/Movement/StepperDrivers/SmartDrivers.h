/*
 * SmartDrivers.h
 *
 *  Created on: 10 Jan 2025
 *      Author: David
 */

#ifndef SRC_MOVEMENT_STEPPERDRIVERS_SMARTDRIVERS_H_
#define SRC_MOVEMENT_STEPPERDRIVERS_SMARTDRIVERS_H_
// Ugly hack to make sure we use the TGBTC version
#if TGBTC
#include "Hardware/TGBTC/Movement/StepperDrivers/SmartDrivers.h"
#else
#if SUPPORT_TMC2660
# include "TMC2660.h"
#endif
#if SUPPORT_TMC22xx
# include "TMC22xx.h"
#endif
#if SUPPORT_TMC51xx || SUPPORT_TMC2240_SPI
# include "TMC51xx.h"
#endif
#endif
#endif /* SRC_MOVEMENT_STEPPERDRIVERS_SMARTDRIVERS_H_ */
