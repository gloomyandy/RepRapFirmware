/*
 * Stream.cpp
 *
 *  Created on: 20 Jun 2020
 *      Author: David
 */

#include "Stream.h"

// read characters from stream into buffer
// returns the number of characters placed in the buffer
#if __LPC17xx__
size_t Stream::readBytes(char *buffer, size_t length) noexcept
#else
size_t Stream::readBytes(char *buffer, size_t length) noexcept
#endif
{
	size_t count = 0;
	while (count < length)
	{
		const int c = read();
		if (c < 0)
		{
			break;
		}
		*buffer++ = (char) c;
		count++;
	}
	return count;
}

// End
