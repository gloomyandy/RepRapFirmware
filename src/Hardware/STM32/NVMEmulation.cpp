
//GA
#include "SoftwareReset.h"
#include "NVMEmulation.h"
#include "RepRapFirmware.h"
#include "Cache.h"

// Flash write/erase
// Note the STM32H7 flash memory device has additional protection/error detection.
// This can mean that extra exceptions can be thrown when reading memory that has
// not been correctly erased and programmed. The Flash read code will ignore such
// errors.
// This protection also means that the H7 does not seem to like writing to flash
// lines that have already been written to. Because of the alignment of the
// postmortum data this means that we need to use a new 512byte block for each write.

#if STM32H7
// We write in 256 bit alignment!
#define IS_FLASH_ALIGNED(addr) (((uint32_t)(addr) & (32-1)) == 0)
#else
// we write with 32bit alignment
#define IS_FLASH_PROGRAM_ADDRESS(addr) (((addr) >= FLASH_BASE) && ((addr) <= FLASH_END))
#define IS_FLASH_ALIGNED(addr) (((uint32_t)(addr) & (sizeof(uint32_t)-1)) == 0)
#endif
#define IS_ALIGNED(addr) (((uint32_t)(addr) & (sizeof(uint32_t)-1)) == 0)
constexpr uint32_t IAP_BAD_SECTOR = 0xffffffff;

static void FlashClearError()
{
	// Clear pending flags (if any)
#if STM32H7
	__HAL_FLASH_CLEAR_FLAG_BANK1(FLASH_FLAG_WRPERR_BANK1 | FLASH_FLAG_PGSERR_BANK1 | FLASH_FLAG_STRBERR_BANK1 | \
                            		FLASH_FLAG_INCERR_BANK1 | FLASH_FLAG_OPERR_BANK1 | FLASH_FLAG_SNECCERR_BANK1 | \
                                    FLASH_IT_DBECCERR_BANK1);
#if STM32H743xx
	__HAL_FLASH_CLEAR_FLAG_BANK2((FLASH_FLAG_WRPERR_BANK2 | FLASH_FLAG_PGSERR_BANK2 | FLASH_FLAG_STRBERR_BANK2 | \
									FLASH_FLAG_INCERR_BANK2 | FLASH_FLAG_SNECCERR_BANK2 | FLASH_IT_DBECCERR_BANK2) & 0x7FFFFFFFU);
#endif
#else
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |\
							FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR| FLASH_FLAG_PGSERR);
#endif
}


static bool isErased(const uint32_t addr, const size_t len) noexcept
{
#if STM32H7
    // On the STM32H7 if the flash has not been correctly erased then simply reading
    // it can cause a bus fault (due to multiple ECC errors). We avoid this by disaabling
    // the fault mechanism while checking the flash memory.
    const coreIrqflags_t flags = IrqSave();

    __set_FAULTMASK(1);
    SCB->CCR |= SCB_CCR_BFHFNMIGN_Msk;
    __DSB();
    __ISB();
#endif
	HAL_FLASH_Unlock();
	FlashClearError();

    bool blank = true;
	// Check that the sector really is erased
	for (uint32_t p = addr; p < addr + len && blank; p += sizeof(uint32_t))
	{
		if (*reinterpret_cast<const uint32_t*>(p) != 0xFFFFFFFF)
		{
			blank = false;
		}
	}

	FlashClearError();
	HAL_FLASH_Lock();

#if STM32H7
    // restore bus fault logic
	__set_FAULTMASK(0);
	SCB->CCR &= ~SCB_CCR_BFHFNMIGN_Msk;
	__DSB();
	__ISB();
	IrqRestore(flags);
#endif
	return blank;
}

static uint32_t FlashGetSector(const uint32_t addr) noexcept
{
	if (!IS_FLASH_PROGRAM_ADDRESS(addr))
	{
		debugPrintf("Bad flash address %x\n", (unsigned)addr);
		return IAP_BAD_SECTOR;
	}
	// Flash memory on STM32F4 is 4 sectors of 16K + 1 sector of 64K + 8 sectors of 128K
    // on the H7 all sectors are 128Kb
	uint32_t offset = addr - FLASH_BASE;
#if STM32H7
	return offset/0x20000;
#else
	if (offset < 4*0x4000)
		return offset / 0x4000;
	else if (offset < 4*0x4000 + 0x10000)
		return 4;
	else
		return offset / 0x20000 + 4;
#endif
}

#if 0
static size_t FlashGetSectorLength(const uint32_t addr) noexcept
{
	uint32_t sector = FlashGetSector(addr);
	if (sector == IAP_BAD_SECTOR)
		return 0;
#if STM32H7
	return 0x20000;
#else
	if (sector < 4)
		return 0x4000;
	else if (sector < 5)
		return 0x10000;
	else
		return 0x20000;
#endif
}
#endif

static bool FlashEraseSector(const uint32_t sector) noexcept
{
	WatchdogReset();
	FLASH_EraseInitTypeDef eraseInfo;
	uint32_t SectorError;
	bool ret = true;
	eraseInfo.TypeErase = FLASH_TYPEERASE_SECTORS;
#if STM32H7
    if (sector < FLASH_SECTOR_TOTAL)
    {
	    eraseInfo.Banks = FLASH_BANK_1;
        eraseInfo.Sector = sector;
    }
    else
    {
#if STM32H743xx
	    eraseInfo.Banks = FLASH_BANK_2;
        eraseInfo.Sector = sector - FLASH_SECTOR_TOTAL;
#endif
    }
#else
	eraseInfo.Sector = sector;
#endif
	eraseInfo.NbSectors = 1;
	eraseInfo.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	HAL_FLASH_Unlock();
	FlashClearError();
	if (HAL_FLASHEx_Erase(&eraseInfo, &SectorError) != HAL_OK)
	{
		ret = false;
	}
	HAL_FLASH_Lock();
	if (!ret)
		debugPrintf("Flash erase failed sector %d error %x\n", (int)sector, (unsigned)SectorError);
	return ret;
}

static bool FlashWrite(const uint32_t addr, const uint8_t *data, const size_t len) noexcept
{
	uint32_t *dst = (uint32_t *)addr;
	uint32_t *src = (uint32_t *)data;
	if (!IS_FLASH_ALIGNED(dst) || !IS_ALIGNED(src) || !IS_ALIGNED(len))
	{
		debugPrintf("FlashWrite alignment error dst %x, data %d len %d\n", (unsigned)dst, (unsigned)src, (int)len);
		return false;
	}
	bool ret = true;
	//debugPrintf("Write flash addr %x len %d\n", (unsigned)addr, (int)len);
	WatchdogReset();
    bool cacheEnabled = Cache::Disable();
	HAL_FLASH_Unlock();
	FlashClearError();
	uint32_t cnt = 0;
	while(cnt < len)
	{
#if STM32H7
#define FLASH_TYPEPROGRAM_WORD FLASH_TYPEPROGRAM_FLASHWORD
		// We write 256 bits == 8 32bit words at a time
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, (uint32_t) dst, (uint64_t) src) != HAL_OK)
		{
			ret = false;
			break;
		}
		dst += 8;
		src += 8;
		cnt += 32;
#else
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t) dst, (uint64_t) *src) != HAL_OK)
		{
			ret = false;
			break;
		}
		dst++;
		src++;
		cnt += 4;
#endif
	}
	HAL_FLASH_Lock();
    if (cacheEnabled) Cache::Enable();
	if (!ret)
		debugPrintf("Flash write failed cnt %d\n", (int)((int)dst - addr));

	return ret; 
}

static bool FlashRead(const uint32_t addr, uint8_t *data, const size_t len) noexcept
{
#if STM32H7
    // On the STM32H7 if the flash has not been correctly erased then simply reading
    // it can cause a bus fault (due to multiple ECC errors). We avoid this by disaabling
    // the fault mechanism while checking the flash memory.
    const coreIrqflags_t flags = IrqSave();

    __set_FAULTMASK(1);
    SCB->CCR |= SCB_CCR_BFHFNMIGN_Msk;
    __DSB();
    __ISB();
#endif
	HAL_FLASH_Unlock();
	FlashClearError();
    // Do the actual read from flash
    memcpy((void *)data, (void *)addr, len);
    // Clear any errors
	FlashClearError();
	HAL_FLASH_Lock();
#if STM32H7
    // restore bus fault logic
	__set_FAULTMASK(0);
	SCB->CCR &= ~SCB_CCR_BFHFNMIGN_Msk;
	__DSB();
	__ISB();
	IrqRestore(flags);
#endif
    return true;
}

__attribute__((__section__(".reset_data"), used)) uint32_t ResetData[FLASH_DATA_LENGTH/sizeof(uint32_t)];
/*
 
On STM32 we store reset data in flash. We have allocated a flash sector for this purpose and the above
array maps to it.

*/

constexpr uint32_t SlotSize = 512/sizeof(uint32_t); // in 32 bit words
constexpr uint32_t MAX_SLOT = FLASH_DATA_LENGTH/sizeof(uint32_t)/SlotSize - 1;
static uint32_t currentSlot     = MAX_SLOT+1;

static uint32_t *GetSlotPtr(uint8_t slot)
{
    return (uint32_t *) ResetData + (slot*SlotSize);
}


//When the Sector is erased, all the bits will be high
//the first 2 bytes of a used reset slot will have the magic number in it.
bool IsSlotVacant(uint8_t slot)
{
    return isErased((uint32_t) GetSlotPtr(slot), SlotSize);
}

void NVMEmulationRead(void *data, uint32_t dataLength)
{
    // find the most recently written data or slot 0 if all free
    currentSlot = MAX_SLOT;
    while (currentSlot > 0 && IsSlotVacant(currentSlot))
        currentSlot--;
    uint32_t *slotStartAddress = GetSlotPtr(currentSlot);

    FlashRead((const uint32_t)slotStartAddress, (uint8_t *)data, dataLength);
}

bool NVMEmulationErase()
{
// On the H7 we always write to a new slot so nothing to do here.
#if !STM32H7
    // Have we reached the last slot yet?
    if (currentSlot < MAX_SLOT)
    {
        currentSlot++;
        return true;
    }
    FlashEraseSector(FlashGetSector((uint32_t)ResetData));
    if (!isErased((uint32_t)ResetData, sizeof(ResetData)))
        debugPrintf("Sector not erased\n");
    currentSlot = 0;
#endif    
    return true;  
}

bool NVMEmulationWrite(const void *data, uint32_t dataLength){
    if (dataLength != SlotSize*sizeof(uint32_t))
    {
        debugPrintf("Bad flash data size\n");
        return false;
    }
    if (currentSlot > MAX_SLOT)
    {
        debugPrintf("Write to flash slot that has not been read slot is %d\n", (int)currentSlot);
        return false;
    }
#if STM32H7
    if (++currentSlot > MAX_SLOT)
    {
        // All slots have been used, erase entire sector
        FlashEraseSector(FlashGetSector((uint32_t)ResetData));
        debugPrintf("Erase complete\n");
        if (!isErased((uint32_t)ResetData, sizeof(ResetData)))
        {
           debugPrintf("Sector not erased\n");
           return false;
        }
        currentSlot = 0;
    }
#endif

    FlashWrite((uint32_t)GetSlotPtr(currentSlot), (const uint8_t*)data, dataLength);
    return true;  
}

