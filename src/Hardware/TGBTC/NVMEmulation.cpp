
//GA
#include "SoftwareReset.h"
#include "NVMEmulation.h"
#include "RepRapFirmware.h"
#include "Cache.h"
#include "Flash.h"



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
bool IsSlotVacant(uint8_t slot) noexcept
{
    return Flash::FlashIsErased((uint32_t) GetSlotPtr(slot), SlotSize);
}

void NVMEmulationRead(void *data, uint32_t dataLength) noexcept
{
    // find the most recently written data or slot 0 if all free
    currentSlot = MAX_SLOT;
    while (currentSlot > 0 && IsSlotVacant(currentSlot))
        currentSlot--;
    uint32_t *slotStartAddress = GetSlotPtr(currentSlot);

    Flash::FlashRead((const uint32_t)slotStartAddress, (uint8_t *)data, dataLength);
}

bool NVMEmulationErase() noexcept
{
// On the H7 we always write to a new slot so nothing to do here.
#if !STM32H7
    // Have we reached the last slot yet?
    if (currentSlot < MAX_SLOT)
    {
        currentSlot++;
        return true;
    }
    Flash::FlashEraseSector(Flash::FlashGetSector((uint32_t)ResetData));
    if (!Flash::FlashIsErased((uint32_t)ResetData, sizeof(ResetData)))
        debugPrintf("Sector not erased\n");
    currentSlot = 0;
#endif    
    return true;  
}

bool NVMEmulationWrite(const void *data, uint32_t dataLength) noexcept
{
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
        Flash::FlashEraseSector(Flash::FlashGetSector((uint32_t)ResetData));
        debugPrintf("Erase complete\n");
        if (!Flash::FlashIsErased((uint32_t)ResetData, sizeof(ResetData)))
        {
           debugPrintf("Sector not erased\n");
           return false;
        }
        currentSlot = 0;
    }
#endif

    Flash::FlashWrite((uint32_t)GetSlotPtr(currentSlot), (const uint8_t*)data, dataLength);
    return true;  
}

