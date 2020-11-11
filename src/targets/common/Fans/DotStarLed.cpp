/*
 * DotStarLed.cpp
 *
 *  Created on: 18 Jul 2018
 *      Author: David/GA
 * This is a simplified version of the standard Duet3D code that only supports NeoPixels
 * and which reduces the memory required.
 */

#include "DotStarLed.h"

#if SUPPORT_DOTSTAR_LED

#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Movement/StepTimer.h>
#include <RepRap.h>
#include <GCodes/GCodes.h>


namespace DotStarLed
{
	constexpr uint32_t MinNeoPixelResetTicks = (50 * StepTimer::StepClockRate)/1000000;		// 50us minimum Neopixel reset time

	constexpr size_t ChunkBufferSize = 180;								// the size of our buffer NeoPixels use 3 bytes per pixel

	static uint32_t ledType = 1;										// 0 = DotStar (not supported on Duet 3 Mini), 1 = NeoPixel, 2 = NeoPixel on Mini 12864 display (Duet 3 Mini only)
	static unsigned int numAlreadyInBuffer = 0;							// number of pixels already store in the buffer (NeoPixel only)
	static uint8_t *chunkBuffer = nullptr;								// buffer for sending data to LEDs
	constexpr size_t MaxLeds = ChunkBufferSize/3;


// This was the original delay function used. It works fine on the STM32F4, but on the LPC
// it can occasionally generate delays that are too long (and this results in glitches in the
// NeoPixel display). So for now we use the loop based timer. Which is not very accurate but
// seems to work for both MCUs
#if 0
	// Delay for a specified number of CPU clock cycles from the starting time. Return the time at which we actually stopped waiting.
	[[gnu::always_inline, gnu::optimize("03")]] static inline uint32_t DelayCycles(const uint32_t start, const uint32_t cycles, const uint32_t reload) noexcept
	{
		//const uint32_t reload = (SysTick->LOAD & 0x00FFFFFF) + 1;
		uint32_t now = start;

		// Wait for the systick counter to cycle round until we need to wait less than the reload value
		while (cycles >= reload)
		{
			const uint32_t last = now;
			now = SysTick->VAL & 0x00FFFFFF;
			if (now > last)
			{
				cycles -= reload;
			}
		}

		uint32_t when;
		if (start >= cycles)
		{
			when = start - cycles;
		}
		else
		{
			when = start + reload - cycles;

			// Wait for the counter to cycle again
			while (true)
			{
				const uint32_t last = now;
				now = SysTick->VAL & 0x00FFFFFF;
				if (now > last)
				{
					break;
				}
			}
		}

		// Wait until the counter counts down to 'when' or below, or cycles again
		while (true)
		{
			const uint32_t last = now;
			now = SysTick->VAL & 0x00FFFFFF;
			if (now <= when || now > last)
			{
				return now;
			}
		}
	}

#else

	[[gnu::always_inline, gnu::optimize("O3")]] static inline void __delay_4cycles(uint32_t cy) { // +1 cycle
	__asm__ __volatile__(
		"  .syntax unified\n\t" // is to prevent CM0,CM1 non-unified syntax
		"1:\n\t"
		"  subs %[cnt],#1\n\t" // 1
		"  mov r0, r0\n\t"            // 1
		"  bne 1b\n\t"         // 1 + (1? reload)
		: [cnt]"+r"(cy)   // output: +r means input+output
		:                 // input:
		: "cc"            // clobbers:
	);
	}

	// Delay in cycles
	[[gnu::always_inline, gnu::optimize("03")]] static inline uint32_t DelayCycles(const uint32_t start, const uint32_t cycles, const uint32_t reload) noexcept 
	{
		if (cycles >> 2) __delay_4cycles(cycles >> 2);
		return 0;
	}
#endif


	uint32_t NanosecondsToCycles(uint32_t ns) noexcept
	{
		return (ns * (uint64_t)SystemCoreClock)/1000000000u;
	}

	// Send data to NeoPixel LEDs by bit banging
	static GCodeResult BitBangNeoPixelData(uint8_t red, uint8_t green, uint8_t blue, uint32_t numLeds, bool following) noexcept
	{
		if (NeopixelOutPin == NoPin) return GCodeResult::ok;

		uint8_t *p = chunkBuffer + (3 * numAlreadyInBuffer);
		while (numLeds != 0 && p <= chunkBuffer + ChunkBufferSize - 3)
		{
			*p++ = green;
			*p++ = red;
			*p++ = blue;
			--numLeds;
			++numAlreadyInBuffer;
		}

		if (!following)
		{
			const uint32_t T0H = NanosecondsToCycles(300);
			const uint32_t T0L = NanosecondsToCycles(850);
			const uint32_t T1H = NanosecondsToCycles(800);
			const uint32_t T1L = NanosecondsToCycles(475);
			const uint8_t *q = chunkBuffer;
			uint32_t nextDelay = T0L;
			cpu_irq_disable();
			uint32_t lastTransitionTime = SysTick->VAL & 0x00FFFFFF;
			const uint32_t reload = (SysTick->LOAD & 0x00FFFFFF) + 1;

			while (q < p)
			{
				uint8_t c = *q++;
				for (unsigned int i = 0; i < 8; ++i)
				{
					if (c & 0x80)
					{
						lastTransitionTime = DelayCycles(lastTransitionTime, nextDelay, reload);
						fastDigitalWriteHigh(NeopixelOutPin);
						lastTransitionTime = DelayCycles(lastTransitionTime, T1H, reload);
						fastDigitalWriteLow(NeopixelOutPin);
						nextDelay = T1L;
					}
					else
					{
						lastTransitionTime = DelayCycles(lastTransitionTime, nextDelay, reload);
						fastDigitalWriteHigh(NeopixelOutPin);
						lastTransitionTime = DelayCycles(lastTransitionTime, T0H, reload);
						fastDigitalWriteLow(NeopixelOutPin);
						nextDelay = T0L;
					}
					c <<= 1;
				}
			}
			cpu_irq_enable();
			numAlreadyInBuffer = 0;
		}
		return GCodeResult::ok;
	}
}

void DotStarLed::Init() noexcept
{
	if (NeopixelOutPin != NoPin)
	{
		IoPort::SetPinMode(NeopixelOutPin, PinMode::OUTPUT_LOW);
		chunkBuffer = new uint8_t[ChunkBufferSize];
	}
}

// This function handles M150
// For DotStar LEDs:
// 	We can handle an unlimited length LED strip, because we can send the data in multiple chunks.
//	So whenever we receive a m150 command, we send the data immediately, in multiple chunks if our DMA buffer is too small to send it as a single chunk.
//	To send multiple chunks, we process the command once per chunk, using numRemaining to keep track of how many more LEDs need to be written to
// For NeoPixel LEDs:
//	If there is a gap or more then about 9us in transmission, the string will reset and the next command will be taken as applying to the start of the strip.
//  Therefore we need to DMA the data for all LEDs in one go. So the maximum strip length is limited by the size of our DMA buffer.
//	We buffer up incoming data until we get a command with the Following parameter missing or set to zero, then we DMA it all.
GCodeResult DotStarLed::SetColours(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	bool seen = false;

	// Deal with changing the LED type first
	if (gb.Seen('X'))
	{
		seen = true;
		const uint32_t newType = gb.GetLimitedUIValue('X', 3, 2 ); 
		const bool typeChanged = (newType != ledType);
		if (newType != 2)
		{
			return GCodeResult::ok;
		}

		if (typeChanged)
		{
			ledType = newType;
			numAlreadyInBuffer = 0;
			if (ledType == 2)
			{
				return GCodeResult::notFinished;
			}
		}
	}

	if (ledType == 2)
	{
		// Interrupts are disabled while bit-banging the data, so make sure movement has stopped
		if (!reprap.GetGCodes().LockMovementAndWaitForStandstill(gb))
		{
			return GCodeResult::notFinished;
		}
	}

	// Get the RGB and brightness values
	uint32_t red = 0, green = 0, blue = 0, brightness = 128, numLeds = MaxLeds;
	bool following = false;
	gb.TryGetLimitedUIValue('R', red, seen, 256);
	gb.TryGetLimitedUIValue('U', green, seen, 256);
	gb.TryGetLimitedUIValue('B', blue, seen, 256);
	if (gb.Seen('P'))
	{
		brightness = gb.GetLimitedUIValue('P', 256);						// valid P values are 0-255
	}
	else if (gb.Seen('Y'))
	{
		brightness = gb.GetLimitedUIValue('Y',  32) << 3;					// valid Y values are 0-31
	}
	gb.TryGetUIValue('S', numLeds, seen);
	gb.TryGetBValue('F', following, seen);
	if (!seen || (numLeds == 0 && !following))
	{
		return GCodeResult::ok;
	}

	switch (ledType)
	{
	case 2:
		// Scale RGB by the brightness
		return BitBangNeoPixelData(	(uint8_t)((red * brightness + 255) >> 8),
									(uint8_t)((green * brightness + 255) >> 8),
									(uint8_t)((blue * brightness + 255) >> 8),
									numLeds, following
							      );
	}
	return GCodeResult::ok;													// should never get here
}

#endif

// End
