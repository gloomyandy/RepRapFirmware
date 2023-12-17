#ifndef BOARD_H__
#define BOARD_H__

#include <Pins_STM32.h>

struct PinEntry
{
    Pin GetPin() const  noexcept{ return pin; }
    const char* GetNames() const  noexcept{ return names; }
    
    const Pin pin;
    const char *names;
};

constexpr size_t MaxSignatures = 3;
typedef enum {
    SD_SPI1_A,
    SD_SPI1_B,
    SD_SDIO,
    SD_SPI3_A,
    SD_SPI3_B,
    SD_SPI2_A,
    SD_UNKNOWN = 0xfe,
    SD_NONE = 0xff
} SDConfigs;

constexpr uint32_t UNKNOWN_BOARD = 0;

struct BoardDefaults
{
    const uint32_t signatures[MaxSignatures];
    const SDConfigs SDConfig;
    const Pin spiPins[NumSPIDevices][NumSPIPins];
    const uint32_t numDrivers;
    const Pin enablePins[NumDirectDrivers];
    const Pin stepPins[NumDirectDrivers];
    const Pin dirPins[NumDirectDrivers];
#if HAS_SMART_DRIVERS
    const Pin uartPins[NumDirectDrivers];
    const uint32_t numSmartDrivers;
#endif
    const float digipotFactor;
#if HAS_VOLTAGE_MONITOR
    const Pin vinDetectPin;
#endif
    const Pin stepperPowerEnablePin;
#if HAS_SBC_INTERFACE
    Pin SbcTfrReadyPin;
    Pin SbcCsPin;
    SSPChannel SbcSpiChannel;
#endif     
};

struct BoardEntry
{
    const char *boardName[3];
    const PinEntry *boardPinTable;
    const size_t numNamedEntries;
    const BoardDefaults &defaults;
};

#endif
