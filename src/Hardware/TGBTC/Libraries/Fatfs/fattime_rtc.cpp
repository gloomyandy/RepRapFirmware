
#include "RepRapFirmware.h"
#include "Platform/RepRap.h"
#include "Platform/Platform.h"

/**
 * \brief Current time returned is packed into a DWORD value.
 *
 * The bit field is as follows:
 *
 * bit31:25  Year from 1980 (0..127)
 *
 * bit24:21  Month (1..12)
 *
 * bit20:16  Day in month(1..31)
 *
 * bit15:11  Hour (0..23)
 *
 * bit10:5   Minute (0..59)
 *
 * bit4:0    Second/2 (0..29)
 *
 * \return Current time.
 */
extern "C" uint32_t get_fattime() noexcept
{
	const Platform& platform = reprap.GetPlatform();
	if (!platform.IsDateTimeSet())
	{
		// Date and time have not been set, return default timestamp instead
		return 0x210001;
	}

	// Retrieve current date and time from RTC
	time_t timeNow = platform.GetDateTime();
	struct tm timeInfo;
	gmtime_r(&timeNow, &timeInfo);

	uint32_t ul_time = ((timeInfo.tm_year + 1900 - 1980) << 25)
						| ((timeInfo.tm_mon + 1) << 21)
						| (timeInfo.tm_mday << 16)
						| (timeInfo.tm_hour << 11)
						| (timeInfo.tm_min << 5)
						| (timeInfo.tm_sec >> 1);
	return ul_time;
}

