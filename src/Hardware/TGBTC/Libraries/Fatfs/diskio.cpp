/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2007        */
/*-----------------------------------------------------------------------*/
/* This is a stub disk I/O module that acts as front end of the existing */
/* disk I/O modules and attach it to FatFs module with common interface. */
/*-----------------------------------------------------------------------*/
//SD :: Modified to work with RRF
//SD :: Updated for RTOS


#include <stdio.h>
#include <string.h>

#include "RepRapFirmware.h"
#include "Platform/RepRap.h"
#include "Platform/Tasks.h"
#include <Movement/StepTimer.h>
#include "SDCard.h"
#include "ff.h"
#include "diskio.h"


extern SDCard *_ffs[_DRIVES]; //Defined in CoreLPC

static unsigned int highestSdRetriesDone = 0;
static uint32_t longestWriteTime = 0;
static uint32_t longestReadTime = 0;
static uint32_t sectorsRead = 0;
static uint32_t sectorsWritten = 0;


void DiskioAppendStats(const StringRef& reply) noexcept
{
	reply.lcatf("SD card longest read time %.1fms, write time %.1fms, max retries %u, sectors read/written %" PRIu32 "/%" PRIu32,
					(double)((float)longestReadTime * StepClocksToMillis), (double)((float)longestWriteTime * StepClocksToMillis), highestSdRetriesDone, sectorsRead, sectorsWritten);
	longestReadTime = longestWriteTime = 0;
	highestSdRetriesDone = 0;
	sectorsRead = sectorsWritten = 0;
}



/* drv - Physical drive nmuber (0..) */
DSTATUS disk_initialize (BYTE pdrv) noexcept
{
	return (DSTATUS)_ffs[pdrv]->disk_initialize();
}

/* drv - Physical drive nmuber (0..) */
DSTATUS disk_status (BYTE pdrv) noexcept
{
	return (DSTATUS)_ffs[pdrv]->disk_status();
}

/* drv - Physical drive nmuber (0..) */
/* buff - Data buffer to store read data */
/* sector - Sector address (LBA) */
/* count - Number of sectors to read (1..255) */
DRESULT disk_read (BYTE pdrv, BYTE* buff, LBA_t sector, UINT count) noexcept
{
    if (reprap.Debug(Module::Storage))
    {
        debugPrintf("Read %u %u %lu\n", pdrv, count, sector);
    }
    
    unsigned int retryNumber = 0;
    uint32_t retryDelay = SdCardRetryDelay;
    for(;;)
    {
        uint32_t time = StepTimer::GetTimerTicks();
        DRESULT res = _ffs[pdrv]->disk_read(buff, sector, count);
		time = StepTimer::GetTimerTicks() - time;
		if (time > longestReadTime)
		{
			longestReadTime = time;
		}
        
        if (res == RES_OK) break;
        ++retryNumber;
        if (retryNumber == MaxSdCardTries)
        {
            delay(retryDelay);
            _ffs[pdrv]->disk_initialize();
        }           
        if (retryNumber > MaxSdCardTries)
        {
            highestSdRetriesDone = MaxSdCardTries + 1;
            return RES_ERROR;
        }
        delay(retryDelay);
        retryDelay *= 2;
    }
    
    if (retryNumber > highestSdRetriesDone)
    {
        highestSdRetriesDone = retryNumber;
    }

    sectorsRead += count;
    return RES_OK;
}

#if _READONLY == 0
/* drv - Physical drive nmuber (0..) */
/* buff - Data to be written */
/* sector - Sector address (LBA) */
/* count - Number of sectors to write (1..255) */

DRESULT disk_write (BYTE pdrv, const BYTE* buff, LBA_t sector, UINT count) noexcept
{
    if (reprap.Debug(Module::Storage))
    {
        debugPrintf("Write %u %u %lu\n", pdrv, count, sector);
    }
    
    /* Write the data */
    unsigned int retryNumber = 0;
    uint32_t retryDelay = SdCardRetryDelay;
    for(;;)
    {
        uint32_t time = StepTimer::GetTimerTicks();
        DRESULT res = _ffs[pdrv]->disk_write(buff, sector, count);
		time = StepTimer::GetTimerTicks() - time;
        if (time > longestWriteTime)
		{
			longestWriteTime = time;
		}        
        if (res == RES_OK) break;
        ++retryNumber;
        if (retryNumber == MaxSdCardTries)
        {
            delay(retryDelay);
            _ffs[pdrv]->disk_initialize();
        }           
        if (retryNumber > MaxSdCardTries)
        {
            highestSdRetriesDone = MaxSdCardTries + 1;
            return RES_ERROR;
        }
        delay(retryDelay);
        retryDelay *= 2;
    }
    
    if (retryNumber > highestSdRetriesDone)
    {
        highestSdRetriesDone = retryNumber;
    }
    
    sectorsWritten += count;
    return RES_OK;
}
#endif /* _READONLY */

/* drv - Physical drive nmuber (0..) */
/* ctrl - Control code */
/* buff - Buffer to send/receive control data */

DRESULT disk_ioctl (BYTE pdrv, BYTE ctrl, void *buff) noexcept
{
    //MutexLocker lock(Tasks::GetSpiMutex());
    return _ffs[pdrv]->disk_ioctl(ctrl, buff);
}


