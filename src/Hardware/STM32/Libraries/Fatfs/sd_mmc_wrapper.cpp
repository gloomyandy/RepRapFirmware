//SD: Wrapper to work with RRF

//Wraps RRF "Slot 0" to SDCard on SSP1 (internal SDCard on smoothie)
//wraps RRF "Slot 1" to SDCard on SSP0

#include "RepRapFirmware.h"
#include "sd_mmc_wrapper.h"
#if USE_SSP1 || USE_SSP2 || USE_SSP3 || USE_SSP4 || USE_SSP5 || USE_SSP6
#include "SDCardSPI.h"
#if STM32H7
// We need to make sure the command buffers are stored in un cached memory
static __nocache SDCardSPI spiBasedCards[_DRIVES];
#endif
#endif
#if USE_SDIO
#include "SDCardSDIO.h"
#endif


SDCard *_ffs[_DRIVES]; //also used by FatFS

//writeProtect pins and ChipSelect Pins for the SDCards
void sd_mmc_init(Pin const wpPins[_DRIVES],Pin const spiCsPins[_DRIVES]) noexcept{
    // STM32 we do nothing here, device and pins are defined later
}

//reinit to support setting cs/freq from config
void sd_mmc_reinit_slot(uint8_t slot, Pin csPin, uint32_t spiFrequency) noexcept
{
    if(slot < _DRIVES && _ffs[slot])
    {
        _ffs[slot]->set_max_frequency(spiFrequency);
    }
}

void sd_mmc_setSSPChannel(uint8_t slot, SSPChannel channel, Pin cs) noexcept
{
    if (_ffs[slot] != nullptr)
    {
#if !STM32H7
        delete _ffs[slot];
#endif
        _ffs[slot] = nullptr;
    }
    if (channel == SSPNONE)
        return;
#if USE_SDIO
    if (channel == SSPSDIO)
        _ffs[slot] = new SDCardSDIO();
    else
#endif
#if USE_SSP1 || USE_SSP2 || USE_SSP3 || USE_SSP4 || USE_SSP5 || USE_SSP6
    if (channel != SSPNONE)
    {
#if STM32H7
        _ffs[slot] = &spiBasedCards[slot];
#else
        _ffs[slot] = new SDCardSPI();
#endif
        ((SDCardSPI *)_ffs[slot])->init(channel, cs);
    }
    else
#endif
        debugPrintf("SD card driver not configured %d\n", channel);
}


void sd_mmc_unmount(uint8_t slot) noexcept
{
    if(slot < _DRIVES && _ffs[slot])
    {
        _ffs[slot]->unmount();
    }
}


sd_mmc_err_t sd_mmc_check(uint8_t slot) noexcept{
    
    if(slot < _DRIVES && _ffs[slot] && _ffs[slot]->disk_initialize() == 0)
    {
        return SD_MMC_OK;
    }
    else
    {
        return SD_MMC_ERR_UNUSABLE;
    }
}


uint32_t sd_mmc_get_capacity(uint8_t slot) noexcept{

    if(slot < _DRIVES && _ffs[slot])
    {
        uint64_t s = _ffs[slot]->disk_sectors();
        uint32_t b = _ffs[slot]->disk_blocksize();

        s = (s/1024)*b;

        return (uint32_t) s; //return in kB
    }
    return 0;
}

card_type_t sd_mmc_get_type(uint8_t slot) noexcept
{
    if(slot < _DRIVES && _ffs[slot])
    {
        CARD_TYPE type = _ffs[slot]->card_type();
        
        card_type_t result = 0;
        
        if( (type & CT_SD1) || (type & CT_SD2) ) result = CARD_TYPE_SD; //V1 or V2 SDCard
        if(type & CT_BLOCK) result |= CARD_TYPE_HC; //High Capacity card
        
        return result;
    }
    
    return CARD_TYPE_UNKNOWN;
}

uint32_t sd_mmc_get_interface_speed(uint8_t slot) noexcept
{
    if(slot < _DRIVES && _ffs[slot])
    {
        // Get the speed of the SPI SD card interface for reporting purposes, in bytes/sec
        return _ffs[slot]->interface_speed();
    }
    
    return 0;
}

