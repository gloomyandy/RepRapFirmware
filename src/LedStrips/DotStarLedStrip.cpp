/*
 * DotStarLedStrip.cpp
 *
 *  Created on: 30 Apr 2023
 *      Author: David
 */

#include <LedStrips/DotStarLedStrip.h>

#if SUPPORT_LED_STRIPS

DotStarLedStrip::DotStarLedStrip(uint32_t p_freq) noexcept : LocalLedStrip(p_freq)
{
	// TODO Auto-generated constructor stub

}

GCodeResult DotStarLedStrip::HandleM150(GCodeBuffer &gb, const StringRef &reply) THROWS(GCodeException)
{
	//TODO
	return GCodeResult::errorNotSupported;
}

#endif

// End
