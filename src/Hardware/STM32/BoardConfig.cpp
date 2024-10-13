/*
 * Board Config
 *
 *  Created on: 3 Feb 2019
 *      Author: sdavi
 */



#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <stdint.h>
#include "Version.h"
#include "BoardConfig.h"
#include "Devices.h"
#include "RepRapFirmware.h"
#include "SbcInterface.h"
#include "sd_mmc.h"
#include "SPI.h"
#include "HardwareSPI.h"
#include "HardwareSDIO.h"
#include "Platform/Platform.h"

#include "HybridPWM.h"
#include "ff.h"
#include "Hardware/SoftwareReset.h"
#include "Hardware/ExceptionHandlers.h"

#include "pinmap.h"

#if STM32H7
# include "stm32h7xx_ll_system.h"
# include "core_cm7.h"
#else
# include "stm32f4xx_ll_system.h"
# include "core_cm4.h"
#endif

#if SUPPORT_TMC22xx || SUPPORT_DMA_NEOPIXEL
# include "DMABitIO.h"
#endif
#ifdef TEST_CACHE
extern uint32_t _nocache_ram_start;
extern uint32_t _nocache_ram_end;
#if STM32H7
extern uint32_t _nocache2_ram_start;
extern uint32_t _nocache2_ram_end;
#endif
#include <Cache.h>
#endif


char BoardName[MaxBoardNameLength] = "unknown";
char BoardLongName[MaxBoardNameLength] = "unknown";
char iapFirmwareFile[MaxBoardNameLength*2] = "unknown";
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

SDConfigs sdConfig = SD_UNKNOWN;

static constexpr const char* boardConfigFile = "0:/sys/board.txt";
static constexpr const char* bootConfigFile = "0:/rrfboot.txt";
static constexpr const char* pinsConfigFile = "0:/rrfpins.txt";

// board configs
static const boardConfigEntry_t boardConfigs[]=
{
    {"board", &BoardName, 1, cvStringType},
    {"board.longName", &BoardLongName, 1, cvStringType},
    {"leds.diagnostic", &DiagPin, 1, cvPinType},
    {"leds.diagnosticOn", &DiagOnPolarity, 1, cvBoolType},
    {"leds.activity", &ActLedPin, 1, cvPinType},
    {"leds.activityOn", &ActOnPolarity, 1, cvBoolType},

    // initial pin states
    {"pins.SetHigh", PinsSetHigh, MaxInitialPins, cvPinType},
    {"pins.SetLow", PinsSetLow, MaxInitialPins, cvPinType},

    //Steppers
    {"stepper.powerEnablePin", &StepperPowerEnablePin, 1, cvPinType},
    {"stepper.enablePins", ENABLE_PINS, NumDirectDrivers, cvPinType},
    {"stepper.stepPins", STEP_PINS, NumDirectDrivers, cvPinType},
    {"stepper.directionPins", DIRECTION_PINS, NumDirectDrivers, cvPinType},
    {"stepper.digipotFactor", &digipotFactor, 1, cvFloatType},
#if HAS_SMART_DRIVERS
    {"stepper.TmcUartPins", TMC_PINS, NumDirectDrivers, cvPinType},
    {"stepper.DriverType", TMC_DRIVER_TYPE, NumDirectDrivers, cvDriverType},
    {"stepper.numSmartDrivers", &totalSmartDrivers, 1, cvUint32Type},
#if SUPPORT_TMC51xx
    {"stepper.num5160Drivers", &num5160SmartDrivers, 1, cvUint32Type},
    {"stepper.spiChannel", &SmartDriversSpiChannel, 1, cvUint8Type},
    {"stepper.csDelay", &SmartDriversSpiCsDelay, 1, cvUint32Type},
#endif
#if HAS_STALL_DETECT && SUPPORT_TMC22xx
    {"stepper.TmcDiagPins", DriverDiagPins, NumDirectDrivers, cvPinType},
#endif
#endif
    //Heater sensors
    {"heat.tempSensePins", TEMP_SENSE_PINS, NumThermistorInputs, cvPinType},
    {"heat.spiTempSensorCSPins", SpiTempSensorCsPins, MaxSpiTempSensors, cvPinType},
    {"heat.spiTempSensorChannel", &TempSensorSSPChannel, 1, cvUint8Type},
    {"heat.thermistorSeriesResistor", &DefaultThermistorSeriesR, 1, cvFloatType},
    
    //ATX Power
    {"atx.powerPin", &ATX_POWER_PIN, 1, cvPinType},
    {"atx.powerPinInverted", &ATX_POWER_INVERTED, 1, cvBoolType},
    {"atx.initialPowerOn", &ATX_INITIAL_POWER_ON, 1, cvBoolType},

    //SDCards
    {"sdcard.internal.type", &sdConfig, 1, cvUint8Type},
    {"sdCard.internal.spiFrequencyHz", &InternalSDCardFrequency, 1, cvUint32Type},
    {"sdCard.external.csPin", &SdSpiCSPins[1], 1, cvPinType},
    {"sdCard.external.cardDetectPin", &SdCardDetectPins[1], 1, cvPinType},
    {"sdCard.external.spiFrequencyHz", &ExternalSDCardFrequency, 1, cvUint32Type},
    {"sdCard.external.spiChannel", &ExternalSDCardSSPChannel, 1, cvUint8Type},

#if SUPPORT_12864_LCD
    {"lcd.lcdCSPin", &LcdCSPin, 1, cvPinType},
    {"lcd.lcdBeepPin", &LcdBeepPin, 1, cvPinType},
    {"lcd.encoderPinA", &EncoderPinA, 1, cvPinType},
    {"lcd.encoderPinB", &EncoderPinB, 1, cvPinType},
    {"lcd.encoderPinSw", &EncoderPinSw, 1, cvPinType},
    {"lcd.lcdDCPin", &LcdA0Pin, 1, cvPinType},
    {"lcd.panelButtonPin", &PanelButtonPin, 1, cvPinType},
    {"lcd.spiChannel", &LcdSpiChannel, 1, cvUint8Type},
#endif
    
    {"SPI0.pins", SPIPins[0], NumSPIPins, cvPinType}, //SCK, MISO, MOSI
    {"SPI1.pins", SPIPins[1], NumSPIPins, cvPinType}, //SCK, MISO, MOSI
    {"SPI2.pins", SPIPins[2], NumSPIPins, cvPinType}, //SCK, MISO, MOSI
    {"SPI3.pins", SPIPins[3], NumSPIPins, cvPinType}, //SCK, MISO, MOSI
    {"SPI4.pins", SPIPins[4], NumSPIPins, cvPinType}, //SCK, MISO, MOSI
    {"SPI5.pins", SPIPins[5], NumSPIPins, cvPinType}, //SCK, MISO, MOSI
#if STM32H7
    {"SPI6.pins", SPIPins[6], NumSPIPins, cvPinType}, //SCK, MISO, MOSI
    {"SPI7.pins", SPIPins[7], NumSPIPins, cvPinType}, //SCK, MISO, MOSI
    {"SPI8.pins", SPIPins[8], NumSPIPins, cvPinType}, //SCK, MISO, MOSI
#endif
    
#if HAS_WIFI_NETWORKING
    {"8266wifi.espDataReadyPin", &EspDataReadyPin, 1, cvPinType},
    {"8266wifi.TfrReadyPin", &SamTfrReadyPin, 1, cvPinType},
    {"8266wifi.lpcTfrReadyPin", &SamTfrReadyPin, 1, cvPinType},
    {"8266wifi.espResetPin", &EspResetPin, 1, cvPinType},
    {"8266wifi.csPin", &SamCsPin, 1, cvPinType},
    {"8266wifi.serialRxTxPins", &WifiSerialRxTxPins, NumberSerialPins, cvPinType},
    {"8266wifi.spiChannel", &WiFiSpiChannel, 1, cvUint8Type},    
    {"8266wifi.clockReg", &WiFiClockReg, 1, cvUint32Type},
    {"8266wifi.moduleType", &NetworkModule, 1, cvModuleType},
    {"wifi.espDataReadyPin", &EspDataReadyPin, 1, cvPinType},
    {"wifi.TfrReadyPin", &SamTfrReadyPin, 1, cvPinType},
    {"wifi.espResetPin", &EspResetPin, 1, cvPinType},
    {"wifi.csPin", &SamCsPin, 1, cvPinType},
    {"wifi.serialRxTxPins", &WifiSerialRxTxPins, NumberSerialPins, cvPinType},
    {"wifi.spiChannel", &WiFiSpiChannel, 1, cvUint8Type},    
    {"wifi.clockReg", &WiFiClockReg, 1, cvUint32Type},
    {"wifi.moduleType", &NetworkModule, 1, cvModuleType},
#endif

#if HAS_SBC_INTERFACE
    {"sbc.TfrReadyPin", &SbcTfrReadyPinConfig, 1, cvPinType},
    {"sbc.csPin", &SbcCsPinConfig, 1, cvPinType},
    {"sbc.spiChannel", &SbcSpiChannel, 1, cvUint8Type},
    {"sbc.loadConfig", &SbcLoadConfig, 1, cvBoolType},
    {"sbc.SBCMode", &SbcMode, 1, cvBoolType},
#endif

#if defined(SERIAL_AUX_DEVICE)
    {"serial.aux.rxTxPins", &AuxSerialRxTxPins, NumberSerialPins, cvPinType},
#endif
#if defined(SERIAL_AUX2_DEVICE)
    {"serial.aux2.rxTxPins", &Aux2SerialRxTxPins, NumberSerialPins, cvPinType},
#endif
    
#if SUPPORT_LED_STRIPS
    {"led.neopixelPin", &NeopixelOutPin, 1, cvPinType},
#endif

#if HAS_VOLTAGE_MONITOR
    {"power.VInDetectPin", &PowerMonitorVinDetectPin, 1, cvPinType},
    {"power.voltage", &VInDummyReading, 1, cvUint32Type},
#endif
#if SUPPORT_ACCELEROMETERS
    {"accelerometer.spiChannel", &AccelerometerSpiChannel, 1, cvUint8Type},
#endif
#if SUPPORT_SPICAN
    {"can.spiChannel", &CanSpiChannel, 1, cvUint8Type},
    {"can.csPin", &CanCsPin, 1, cvPinType},
    {"can.spiFrequencyHz", &CanSpiFrequency, 1, cvUint32Type},
#endif
#if STM32H7
    {"can.readPin", &CanReadPin, 1, cvPinType},
    {"can.writePin", &CanWritePin, 1, cvPinType},
#endif
};


static void ClearConfig() noexcept
{
    const size_t numConfigs = ARRAY_SIZE(boardConfigs);
    for(size_t i=0; i<numConfigs; i++)
    {
        boardConfigEntry_t next = boardConfigs[i];
        for(size_t p=0; p<(next.numItems); p++)
        {
            switch(next.type)
            {
                case cvPinType:
                    ((Pin *)(next.variable))[p] = NoPin;
                    break;
                case cvDriverType:
                    ((DriverType *)(next.variable))[p] = DriverType::unknown;
                    break;
#if HAS_WIFI_NETWORKING
                case cvModuleType:
                    ((NetworkModuleType *)(next.variable))[p] = NetworkModuleType::espauto;
                    break;
#endif
                case cvBoolType:
                    ((bool *)(next.variable))[p] = false;
                    break;
                case cvFloatType:
                    ((float *)(next.variable))[p] = 0.0f;
                    break;
                case cvUint8Type:
                    ((uint8_t *)(next.variable))[p] = 0;
                    break;
                case cvUint16Type:
                    ((uint16_t *)(next.variable))[p] = 0;
                   break;
                case cvUint32Type:
                    ((uint32_t *)(next.variable))[p] = 0;
                    break;
                case cvStringType:
                    strcpy((char *)(next.variable), "");
                    break;
                default:
                    break;
            }
        }
    }
    // We have a few settings that have "special" default values
    TempSensorSSPChannel = SSPNONE;
    DefaultThermistorSeriesR = 4700.0;
    ATX_INITIAL_POWER_ON = true;
    SdSpiCSPins[0] = PA_4;
    SdCardDetectPins[0] = NoPin;
    SdSpiCSPins[1] = NoPin;
    SdCardDetectPins[1] = NoPin;
    ExternalSDCardFrequency = 4000000;
    ExternalSDCardSSPChannel = SSPNONE;
    InternalSDCardFrequency = 25000000;
    LcdSpiChannel = SSPNONE;
    DiagOnPolarity = true;
    ActOnPolarity = true;
    SmartDriversSpiChannel = SSPNONE;
#if HAS_WIFI_NETWORKING
    SamCsPin = PB_12;
    WiFiSpiChannel = SSP2;
#endif
#if defined(SERIAL_AUX_DEVICE)
    AuxSerialRxTxPins[0] = PA_10;
    AuxSerialRxTxPins[1] = PA_9;
#endif
#if HAS_SBC_INTERFACE
    SbcCsPinConfig = PB_12;
    SbcSpiChannel = SSP2;
#endif
#if SUPPORT_SPICAN
    CanSpiFrequency = 15000000;
#endif
#if STM32H7
    CanReadPin = PB_8;
    CanWritePin = PB_9;
#endif
#if HAS_VOLTAGE_MONITOR
    VInDummyReading = 24;
#endif
#if SUPPORT_ACCELEROMETERS
    AccelerometerSpiChannel = SSPNONE;
#endif
    sdConfig = SD_UNKNOWN;
}


static void InitDiagPin()
{
    bool dummy;
    LogicalPin pin;
    if (LookupPinName("status", pin, dummy))
    {
        DiagPin = (Pin)pin;
    }
}


uint32_t crc32_for_byte(uint32_t r) 
{
    for(int j = 0; j < 8; ++j)
        r = (r & 1? 0: (uint32_t)0xEDB88320L) ^ r >> 1;
    return r ^ (uint32_t)0xFF000000L;
}

uint32_t crc32(const void *data, size_t n_bytes) 
{
    uint32_t table[0x100];
    uint32_t crc = 0;

    for(size_t i = 0; i < 0x100; ++i)
        table[i] = crc32_for_byte(i);
    for(size_t i = 0; i < n_bytes; ++i)
        crc = table[(uint8_t)crc ^ ((uint8_t*)data)[i]] ^ crc >> 8;
    return crc;
}


#if HAS_SBC_INTERFACE
class InMemoryBoardConfiguration
{
public:
    InMemoryBoardConfiguration() : signature(invalidSignature), length(0) {}
    bool isValid() noexcept;
    void setConfiguration() noexcept;
    void getConfiguration() noexcept;
    void saveToBackupRAM() noexcept;
    void loadFromBackupRAM() noexcept;
    bool isEqual(InMemoryBoardConfiguration& other) noexcept;
private:
    static constexpr uint32_t validSignature = 0xBCBC0002;
    static constexpr uint32_t invalidSignature = 0xBAD0BAD1;
    static constexpr int maxData = 512;
    uint32_t signature;
    uint32_t configEntrySig;
    uint32_t length;
    uint8_t data[maxData];
};

bool InMemoryBoardConfiguration::isValid() noexcept
{
    //debugPrintf("Memory at %x signature %x confSig %x\n", (unsigned)this, (unsigned) signature, (unsigned)configEntrySig);
    return signature == validSignature && configEntrySig == crc32(boardConfigs, sizeof(boardConfigs));
}

void InMemoryBoardConfiguration::setConfiguration() noexcept
{
    if (!isValid()) debugPrintf("Warning: Setting configuration from invalid memory\n");
    uint8_t *pmem = data;
    const size_t numConfigs = ARRAY_SIZE(boardConfigs);
    for(size_t i=0; i<numConfigs; i++)
    {
        const boardConfigEntry_t item = boardConfigs[i];
        uint32_t len = item.numItems*CVTLengths[item.type];
        memcpy(item.variable, pmem, len);
        pmem += len;
    }
}

void InMemoryBoardConfiguration::getConfiguration() noexcept
{
    uint8_t *pmem = data;
    const size_t numConfigs = ARRAY_SIZE(boardConfigs);
    for(size_t i=0; i<numConfigs; i++)
    {
        const boardConfigEntry_t item = boardConfigs[i];
        uint32_t len = item.numItems*CVTLengths[item.type];
        memcpy(pmem, item.variable, len);
        pmem += len;
        if (pmem - data > maxData)
        {
            debugPrintf("Warning configuration data too large\n");
            return;
        }
    }
    length = pmem - data;
    signature = validSignature;
    configEntrySig = crc32(boardConfigs, sizeof(boardConfigs));
    //debugPrintf("config length %d sig %x\n", (int)length, (unsigned)configEntrySig);
}

void InMemoryBoardConfiguration::saveToBackupRAM() noexcept
{
    // enable access to backup RAM
#if STM32H7
    __HAL_RCC_BKPRAM_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    memcpy((void *)D3_BKPSRAM_BASE, this, sizeof(InMemoryBoardConfiguration));
#else
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_BKPSRAM_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    memcpy((void *)BKPSRAM_BASE, this, sizeof(InMemoryBoardConfiguration));
#endif
}

void InMemoryBoardConfiguration::loadFromBackupRAM() noexcept
{
    // enable access to backup RAM
#if STM32H7
    __HAL_RCC_BKPRAM_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    memcpy(this, (void *)D3_BKPSRAM_BASE, sizeof(InMemoryBoardConfiguration));
#else
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_BKPSRAM_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    memcpy(this, (void *)BKPSRAM_BASE, sizeof(InMemoryBoardConfiguration));
#endif
    //if (isValid())
        //debugPrintf("Loaded valid configuration data from backup RAM\n");
}


bool InMemoryBoardConfiguration::isEqual(InMemoryBoardConfiguration& other) noexcept
{
    return (isValid() && other.isValid() && length == other.length && !memcmp(data, other.data, length));
}
#endif


static inline bool isSpaceOrTab(char c) noexcept
{
    return (c == ' ' || c == '\t');
}
    
BoardConfig::BoardConfig() noexcept
{
}

static void ConfigureGPIOPins() noexcept
{
    initInterruptPins();
    for(size_t i = 0; i < MaxInitialPins; i++)
    {
        pinMode(PinsSetLow[i], OUTPUT_LOW);
        pinMode(PinsSetHigh[i], OUTPUT_HIGH);
        pinMode(SpiTempSensorCsPins[i], INPUT_PULLUP);
    }
#if HAS_SMART_DRIVERS
    for(size_t i = 0; i < NumDirectDrivers; i++)
        pinMode(TMC_PINS[i], OUTPUT_HIGH);
#endif
    // Handle special cases
    //Init pins for LCD
    //make sure to init ButtonPin as input incase user presses button
    pinMode(PanelButtonPin, INPUT);
    pinMode(LcdA0Pin, OUTPUT_HIGH);
    pinMode(LcdBeepPin, OUTPUT_LOW);
    // Set the 12864 display CS pin low to prevent it from receiving garbage due to other SPI traffic
    pinMode(LcdCSPin, OUTPUT_LOW);
#if SUPPORT_SPICAN
    pinMode(CanCsPin, OUTPUT_HIGH);
#endif

    //Init Diagnostcs Pin
#if STARTUP_DELAY
    if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0)
    {
        // Debugger is active do not allow use of those pins for diag light
        if (DiagPin == SWDIO_PIN || DiagPin == SWCLK_PIN)
            DiagPin = NoPin;
    } 
#endif

    pinMode(DiagPin, OUTPUT_LOW);

    // Configure ATX power control
    ATX_POWER_STATE = ATX_INITIAL_POWER_ON;
    pinMode(StepperPowerEnablePin, (ATX_POWER_STATE ? OUTPUT_HIGH : OUTPUT_LOW));
}

static void ConfigureSPIPins(SSPChannel dev, Pin clk, Pin miso, Pin mosi) noexcept
{
    SPI::getSSPDevice(dev)->initPins(clk, miso, mosi, NvicPrioritySpi);
}

static void ConfigureDriveType() noexcept
{
    // first check to see if we have any explcit settings
    for(size_t i = 0; i < NumDirectDrivers; i++)
    {
        if (TMC_DRIVER_TYPE[i] != DriverType::unknown)
        {
            // at least one driver has been set, so we assume all have been
            return;
        }
    }
    // individual drivers have not been set, so we configure using "legacy" settings
    for(size_t i = 0; i < num5160SmartDrivers; i++)
        TMC_DRIVER_TYPE[i] = DriverType::tmcspiauto;
    for(size_t i = num5160SmartDrivers; i < totalSmartDrivers; i++)
        TMC_DRIVER_TYPE[i] = DriverType::tmcuartauto;
    for(size_t i = totalSmartDrivers; i < NumDirectDrivers; i++)
        TMC_DRIVER_TYPE[i] = DriverType::stepdir;
}

static void FatalError(const char* fmt, ...) noexcept
{
    for(;;)
    {
        va_list vargs;
        va_start(vargs, fmt);
        reprap.GetPlatform().DebugMessage(fmt, vargs);
        va_end(vargs);
        delay(2000);
    }
}

static void MessageF(MessageType mtype, const char* fmt, ...) noexcept
{
    va_list vargs;
    va_start(vargs, fmt);
    reprap.GetPlatform().MessageV(mtype, fmt, vargs);
    va_end(vargs);
}

static void FlushMessages() noexcept
{
    uint32_t start = millis();
    while(reprap.GetPlatform().FlushMessages() && (millis() - start < 5000))
        ;
}

#if 0
static void CheckDriverPins() noexcept
{
    for(size_t i=0; i<NumDirectDrivers; i++)
    {
        if (ENABLE_PINS[i] != NoPin && STEP_PINS[i] != NoPin && DIRECTION_PINS[i] != NoPin)
        {
            pinMode(ENABLE_PINS[i], INPUT);
            bool state1 = IoPort::ReadPin(ENABLE_PINS[i]);
            pinMode(STEP_PINS[i], OUTPUT_LOW);
            delay(50);
            bool state2 = IoPort::ReadPin(ENABLE_PINS[i]);
            pinMode(STEP_PINS[i], INPUT);
            if (state1 != state2 && state2 == false)
            {
                FatalError("Possible short between step and enable pins on driver %d.\nPlease check driver installation/configuration.\n", i);
            }
        }
    }
}
#endif


static void UnknownHardware() noexcept
{
    for(;;)
    {
        debugPrintf("\nRRF has been unable to identify the current hardware.\n");
        debugPrintf("This may be because it is a new board or has a new bootloader installed.\n");
        debugPrintf("To register the hardware configuration please contact TeamGloomy via our\n");
        debugPrintf("discord server (https://discord.gg/uS97Qs7) and supply details of\n");
        debugPrintf("the board and the board.\n");
        delay(2000);
    }
}


static const char *GetBootloaderString() noexcept
{
    const uint32_t *BootVectors = (const uint32_t *)0x8000000;
    const char *BootloaderString = (const char *) BootVectors[8];
    // make sure it looks valid...
    if (BootloaderString == nullptr)
        return nullptr;
    if (Strnlen(BootloaderString, 128) >= 128)
        return nullptr;
    if (strstr(BootloaderString, " version ") == nullptr)
        return nullptr;
    return BootloaderString;
}

// Determine how to access the SD card
typedef struct {
    SSPChannel device;
    Pin pins[6];
    uint32_t mode[6];
} SDCardConfig;

// These are our known SD card configurations
static constexpr SDCardConfig SDCardConfigs[] = {
    {SSP1, {PA_5, PA_6, PB_5, PA_4, NoPin, NoPin}, {0x502, 0x502, 0x502, 0x1}}, // SKR Pro
    {SSP1, {PA_5, PA_6, PA_7, PA_4, NoPin, NoPin}, {0x502, 0x502, 0x502, 0x1}}, // GTR
    {SSPSDIO, {PC_8, PC_9, PC_10, PC_11, PC_12, PD_2}, {0xc02, 0xc02, 0xc02, 0xc02, 0xc02, 0xc02}}, // Fly/SDIO
    {SSP3, {PC_10, PC_11, PC_12, PC_9, NoPin, NoPin}, {0x602, 0x602, 0x602, 0x1}}, // MKS?
    {SSP3, {PC_10, PC_11, PC_12, PA_15, NoPin, NoPin}, {0x602, 0x602, 0x602, 0x1}}, // BTT BX
    {SSP2, {PB_13, PB_14, PB_15, PB_12, NoPin, NoPin}, {0x502, 0x502, 0x502, 0x1}}, // BTT kraken?
};

static bool TryConfig(uint32_t config, bool mount) noexcept
{
    const SDCardConfig *conf = &SDCardConfigs[config];
    if (conf->device != SSPSDIO)
    {
        ConfigureSPIPins(conf->device, conf->pins[0], conf->pins[1], conf->pins[2]);
        sd_mmc_setSSPChannel(0, conf->device, conf->pins[3]);
    }
    else
    {
        HardwareSDIO::SDIO1.InitPins(NvicPrioritySDIO);
        sd_mmc_setSSPChannel(0, conf->device, NoPin);
    }

    if (!mount) return true;

    GCodeResult rslt;
    String<100> reply;
    do
    {
        MassStorage::Spin();
        rslt = MassStorage::Mount(0, reply.GetRef(), false, 100);
    } while (rslt == GCodeResult::notFinished);
    if (rslt == GCodeResult::ok)
        return true;
    // mount failed, reset the hardware
    if (conf->device != SSPSDIO)
        ((HardwareSPI *)(SPI::getSSPDevice(conf->device)))->disable();
    for (size_t i = 0; i < ARRAY_SIZE(conf->pins); i++)
        pinMode(conf->pins[i], INPUT);    
    sd_mmc_setSSPChannel(0, SSPNONE, NoPin);
    return false;
}    

static bool LoadBoardDefaults() noexcept
{
    ClearConfig();
    // Load the configuration from the embedded file system
    if (BoardConfig::LoadBoardConfigFromFile(bootConfigFile, false))
    {
        MessageF(UsbMessage, "Found boot config for board: %s\n", BoardName);
        // we use the name configured here for the firmware file
        SafeSnprintf(iapFirmwareFile, sizeof(iapFirmwareFile), "firmware_%s.bin", BoardName);
        InitDiagPin();
        // for now ignore settings in boot file
        NetworkModule = NetworkModuleType::espauto;
        return true;
    }
    return false;
}


static SSPChannel InitSDCard(SDConfigs conf, bool mount, bool needed) noexcept
{
    if (conf == SD_UNKNOWN)
    {
        UnknownHardware();
    }
    else if (conf == SD_NONE)
    {
        if (needed)
            FatalError("SD card is not supported on this board\n");
        return SSPNONE;
    }
    else
    {
        if (TryConfig(conf, mount))
        {
            return SDCardConfigs[conf].device;
        }
        if (needed)
            FatalError("Unable to mount SD card, config %d.\n", conf);
        else
        {
            MessageF(UsbMessage, "Unable to mount SD card using config %d\n", conf);
            return SDCardConfigs[conf].device;
        }
    }
    FatalError("Failed to mount SD card\n");
    return SSPNONE;
}


void BoardConfig::Init() noexcept
{
    SSPChannel sdChannel = SSPNONE;
	String<100> reply;
#if !HAS_MASS_STORAGE
#error "Invalid board configuration HAS_MASS_STORAGE is required"
#endif

#if STARTUP_DELAY
    for(int i = 0; i < STARTUP_DELAY; i++)
    {
        delay(1);
        if (SERIAL_MAIN_DEVICE.IsConnected())
        {
            // For some reason if we start using USB straightaway we end up reading back some data we just
            // sent. Adding a small delay seems to fix the problem.
            delay(100);
            debugPrintf("RRF Started....\n");
            break;
        }
    }
#ifdef TEST_CACHE
    debugPrintf("Testing cache configuration\n");
    debugPrintf("_nocache_ram_start %x/%x _nocache2_ram_start %x/%x\n", _nocache_ram_start, &_nocache_ram_start, _nocache2_ram_start, &_nocache2_ram_start);
    Cache::FlushBeforeDMAReceive(&_nocache_ram_start, 256);
    Cache::FlushBeforeDMAReceive(&_nocache2_ram_start, 256);
    Cache::InvalidateAfterDMAReceive(&_nocache_ram_start, 256);
    Cache::InvalidateAfterDMAReceive(&_nocache2_ram_start, 256);
    debugPrintf("Following two should fail\n");
    Cache::FlushBeforeDMAReceive(BoardName, 256);
    Cache::InvalidateAfterDMAReceive(BoardName, 256);
#endif
#endif
    if (!LoadBoardDefaults())
    {
        // On test builds we may not have embedded data. We assume the Sd card is SDIO and
        // look for the config data on the card
        MessageF(UsbMessage, "Embdded board config not found, please check you have the correct firmware file installed.\nAttempting to fall back to SDIO\n");
        sdChannel = InitSDCard(SD_SDIO, true, true);
        if (!LoadBoardDefaults())
        {
            FatalError("Unable to load board defaults.\n");
        }
        MassStorage::Unmount(0, reply.GetRef());
    }
#if HAS_SBC_INTERFACE
    // See if there is an (optional) config file on the SD card
    sdChannel = InitSDCard(sdConfig, true, false);
    if (sdChannel == SSPNONE)
    {
        // Device does not have an SD card so if in sbc mode load config from sbc
        SbcLoadConfig = true;
    }
    else if (!BoardConfig::LoadBoardConfigFromFile(boardConfigFile))
    {
        // No board.txt
        MessageF(UsbMessage, "Warning: unable to load board configuration from file\n");
        // Enable loading of config from the SBC
        SbcLoadConfig = true;
    }
    if (SbcLoadConfig)
    {
        MessageF(UsbMessage, "Checking for SBC based configuration files\n");
        // Check for a configuration stored in RAM (supplied by the SBC),
        // if found use it and override any config from the card
        InMemoryBoardConfiguration inMemoryConfig;
        inMemoryConfig.loadFromBackupRAM();
        if (inMemoryConfig.isValid())
        {
            MessageF(UsbMessage, "Using RAM based configuration data\n");
            inMemoryConfig.setConfiguration();
            // Set SD config if we haven't already
            if (!MassStorage::IsDriveMounted(0))
                sdChannel = InitSDCard(sdConfig, false, false);
        }
    }
#else
    // Try and mount the sd card and read the board.txt file, error if not present
    sdChannel = InitSDCard(sdConfig, true, true);
    if (sdChannel == SSPNONE)
    {
        FatalError("Failed to load board configuration\n");
        return;
    }
    if (!BoardConfig::LoadBoardConfigFromFile(boardConfigFile))
    {
        // failed to load a valid configuration
        MessageF(UsbMessage, "Warning: unable to load board configuration from file\n");
    }
#endif
    if (MassStorage::IsDriveMounted(0))
        MassStorage::Unmount(0, reply.GetRef());

#if HAS_SBC_INTERFACE
    if (SbcMode && (SbcCsPinConfig == NoPin || SbcTfrReadyPinConfig == NoPin || SbcSpiChannel == SSPNONE))
    {
        FatalError("No SBC configuration\n");
        return;
    }
#endif
    //Calculate STEP_DRIVER_MASK (used for parallel writes)
    STEP_DRIVER_MASK = 0;
    // Currently not implemented for STM32
    #if 0
    for(size_t i=0; i<NumDirectDrivers; i++)
    {
        //It is assumed all pins will be on Port 2
        const Pin stepPin = STEP_PINS[i];
        if( stepPin != NoPin && (stepPin >> 5) == 2) // divide by 32 to get port number
        {
            STEP_DRIVER_MASK |= (1 << (stepPin & 0x1f)); //this is a bitmask of all the stepper pins on Port2 used for Parallel Writes
        }
        else
        {
            if(stepPin != NoPin)
            {
                // configured step pins are not on the same port - not using parallel writes
                hasStepPinsOnDifferentPorts = true;
            }
        }
    }
    #endif
    hasStepPinsOnDifferentPorts = true;
    
    //Does board have built in current control via digipots?
    if(digipotFactor > 1)
    {
        hasDriverCurrentControl = true;
    }
#if 0
    // anti-rotation detection feature disabled for now due to potential to damage some drivers
    CheckDriverPins();
#endif
#if HAS_SMART_DRIVERS
    ConfigureDriveType();
#endif       
    //Setup the SPI Pins, note that the SD SPI device may already have been configured
    for(size_t i = 0; i < ARRAY_SIZE(SPIPins); i++)
        if (sdChannel != (SSPChannel)i)
            ConfigureSPIPins((SSPChannel)i, SPIPins[i][0], SPIPins[i][1], SPIPins[i][2]);
#if HAS_MASS_STORAGE
    // Set internal SD card frequency (if supported)
    sd_mmc_reinit_slot(0, NoPin, InternalSDCardFrequency);
    if (ExternalSDCardSSPChannel != SSPNONE && SdSpiCSPins[1] != NoPin)
    {
        sd_mmc_setSSPChannel(1, ExternalSDCardSSPChannel, SdSpiCSPins[1]);
        sd_mmc_reinit_slot(1, NoPin, ExternalSDCardFrequency);
    }
    // Update RRF settings
    MassStorage::Init2();
#endif
#if HAS_SBC_INTERFACE
    SbcTfrReadyPin = SbcTfrReadyPinConfig;
    SbcCsPin = SbcCsPinConfig;
#endif

#if HAS_WIFI_NETWORKING
    // Setup WiFi pins for compatibility
    APIN_ESP_SPI_MOSI = SPIPins[WiFiSpiChannel][2];
    APIN_ESP_SPI_MISO = SPIPins[WiFiSpiChannel][1];
    APIN_ESP_SPI_SCK = SPIPins[WiFiSpiChannel][0];

    if(WifiSerialRxTxPins[0] != NoPin && WifiSerialRxTxPins[1] != NoPin)
    {
        //Setup the Serial Port for ESP Wifi
        APIN_SerialWiFi_RXD = WifiSerialRxTxPins[0];
        APIN_SerialWiFi_TXD = WifiSerialRxTxPins[1];
        
        if(!serialWiFi.Configure(WifiSerialRxTxPins[0], WifiSerialRxTxPins[1]))
        {
            reprap.GetPlatform().MessageF(UsbMessage, "Failed to set WIFI Serial with pins %c.%d and %c.%d.\n", 'A'+(WifiSerialRxTxPins[0] >> 4), (WifiSerialRxTxPins[0] & 0xF), 'A'+(WifiSerialRxTxPins[1] >> 4), (WifiSerialRxTxPins[1] & 0xF) );
        }
    }
#endif


#if defined(SERIAL_AUX_DEVICE)
    //Configure Aux Serial
    if(AuxSerialRxTxPins[0] != NoPin && AuxSerialRxTxPins[1] != NoPin)
    {
        if(!SERIAL_AUX_DEVICE.Configure(AuxSerialRxTxPins[0], AuxSerialRxTxPins[1]))
        {
            reprap.GetPlatform().MessageF(UsbMessage, "Failed to set AUX Serial with pins %c.%d and %c.%d.\n", 'A'+(AuxSerialRxTxPins[0] >> 4), (AuxSerialRxTxPins[0] & 0xF), 'A'+(AuxSerialRxTxPins[1] >> 4), (AuxSerialRxTxPins[1] & 0xF) );
        }

    }
#endif

#if defined(SERIAL_AUX2_DEVICE)
    //Configure Aux2 Serial
    if(Aux2SerialRxTxPins[0] != NoPin && Aux2SerialRxTxPins[1] != NoPin)
    {
        if(!SERIAL_AUX2_DEVICE.Configure(Aux2SerialRxTxPins[0], Aux2SerialRxTxPins[1]))
        {
            reprap.GetPlatform().MessageF(UsbMessage, "Failed to set AUX2 Serial with pins %d.%d and %d.%d.\n", (Aux2SerialRxTxPins[0] >> 5), (Aux2SerialRxTxPins[0] & 0x1F), (Aux2SerialRxTxPins[1] >> 5), (Aux2SerialRxTxPins[1] & 0x1F) );
        }

    }
#endif

    ConfigureGPIOPins();

#if SUPPORT_TMC22xx || SUPPORT_DMA_NEOPIXEL
    DMABitIOInit();
#endif
}

// Function to look up a pin name pass back the corresponding index into the pin table
// On this platform, the mapping from pin names to pins is fixed, so this is a simple lookup
bool LookupPinName(const char*pn, LogicalPin& lpin, bool& hardwareInverted) noexcept
{
    if (StringEqualsIgnoreCase(pn, NoPinName) || StringEqualsIgnoreCase(pn, "NoPin"))
    {
        lpin = NoLogicalPin;
        hardwareInverted = false;
        return true;
    }
    FileStore * const configFile = MassStorage::OpenFile(pinsConfigFile, OpenMode::read, 0);
    if (configFile == nullptr)
    {
        FatalError("Pins file not found\n");
        return false;
    }
    constexpr size_t maxLineLength = 120;
    char line[maxLineLength];
    while(configFile->ReadLine(line, maxLineLength) >= 0)
    {
        // debugPrintf("Got line %s\n", line);
        // skip pin number
        char *q = line;
        while(*q && *q != ' ')
        {
            q++;
        }
        if (*q == ' ')
        {
            *q++ = 0;
        }
        while (*q != 0)
        {
            // Try the next alias in the list of names for this pin
            const char *p = pn;
            // skip hardware pin options
            if (*q == '+' || *q == '-' || *q == '^')
                ++q;
            bool hwInverted = (*q == '!');
            if (hwInverted)
            {
                ++q;
            }
            // skip leading "_"
            while (*p == '_') p++;
            while (*q != ',' && *q != 0 && tolower(*p) == tolower(*q))
            {
                ++p;
                ++q;
                while (*p == '_' || *p =='-') p++;
            }
            if ((*p == 0 || *p == ',') && (*q == 0 || *q == ','))
            {
                // Found a match
                lpin = BoardConfig::StringToPin(line);
                hardwareInverted = hwInverted;
                configFile->Close();
                return true;
            }
            
            // Skip to the start of the next alias
            while (*q != 0 && *q != ',')
            {
                ++q;
            }
            if (*q == ',')
            {
                ++q;
            }
        }
    }
    configFile->Close();
    //pn did not match a label in the lookup table, so now
    //look up by classic port.pin format
    const Pin lpcPin = BoardConfig::StringToPin(pn);
    if(lpcPin != NoPin){
        lpin = (LogicalPin)lpcPin;
        hardwareInverted = false;
        return true;
    }
    return false;
}

// Return the string names associated with a pin
const char *GetPinNames(LogicalPin lp) noexcept
{
    static char name[32];
    FileStore * const configFile = MassStorage::OpenFile(pinsConfigFile, OpenMode::read, 0);
    if (configFile == nullptr)
    {
        FatalError("Pins file not found\n");
        return NULL;
    }
    constexpr size_t maxLineLength = 120;
    char line[maxLineLength];
    while(configFile->ReadLine(line, maxLineLength) >= 0)
    {
        char *q = line;
        while(*q && *q != ' ')
        {
            q++;
        }
        if (*q == ' ')
        {
            *q++ = 0;
        }
        if (BoardConfig::StringToPin(line) == lp)
        {
            configFile->Close();
            SafeStrncpy(name, q, sizeof(name));
            return name;
        }
    }
    configFile->Close();
    // not found manufascture a name
    name[0] = 'A' + (lp >> 4);
    name[1] = '.';
    if ((lp & 0xf) > 9)
    {
        name[2] = '1';
        name[3] = '0' + (lp & 0xf) - 10;
        name[4] = '\0';
    }
    else
    {
        name[2] = '0' + (lp & 0xf);
        name[3] = '\0';
    }
    // Next is very, very iffy, but ok for current usage!
    return (const char *)name;
}

//Convert a pin string into a RRF Pin
//Handle formats such as A.13, A_13, PA_13 or PA.13
Pin BoardConfig::StringToPin(const char *strvalue) noexcept
{
    if(strvalue == nullptr) return NoPin;
    
    if(tolower(*strvalue) == 'p') strvalue++; //skip P
    //check size.. should be 2, 3 or 4 chars i.e. A1 A.1, A.16, A_16.
    uint8_t len = strlen(strvalue);
    if(len >= 2 && len <= 4)
    {
        const char *ptr = nullptr;
        const char ch = toupper(*strvalue);
        uint8_t port = ch - 'A';
        if(port <= 8)
        {
            // skip "." or "_"
            if ((*(strvalue+1) == '.' || *(strvalue+1) == '_'))
                strvalue += 2;
            else
                strvalue += 1;
            uint8_t pin = StrToI32(strvalue, &ptr);          
            if(ptr > strvalue && pin < 16)
            {
                //Convert the Port and Pin to match the arrays in CoreSTM
                Pin lpcpin = (Pin) ( (port << 4) | pin);
                return lpcpin;
            }
        }
    }
    
    return NoPin;
}

Pin BoardConfig::LookupPin(char *strvalue) noexcept
{
    //Lookup a pin by name
    LogicalPin lp;
    bool hwInverted;
    
    //convert string to lower case for LookupPinName
    for(char *l = strvalue; *l; l++) *l = tolower(*l);
    
    if(LookupPinName(strvalue, lp, hwInverted))
    {
        return (Pin)lp; //lookup succeeded, return the Pin
    }
                     
    //pin may not be in the pintable so check if the format is a correct pin (returns NoPin if not)
    return StringToPin(strvalue);
}



void BoardConfig::PrintValue(MessageType mtype, configValueType configType, void *variable) noexcept
{
    switch(configType)
    {
        case cvPinType:
            {
                Pin pin = *(Pin *)(variable);
                if(pin == NoPin)
                {
                    MessageF(mtype, "NoPin");
                }
                else
                {
                    MessageF(mtype, "%c.%d", 'A' + (pin >> 4), (pin & 0xF) );
                }
            }
            break;
        case cvBoolType:
            MessageF(mtype, "%s", (*(bool *)(variable) == true)?"true":"false" );
            break;
        case cvFloatType:
            MessageF(mtype, "%.2f",  (double) *(float *)(variable) );
            break;
        case cvUint8Type:
            MessageF(mtype, "%u",  *(uint8_t *)(variable) );
            break;
        case cvUint16Type:
            MessageF(mtype, "%d",  *(uint16_t *)(variable) );
            break;
        case cvUint32Type:
            MessageF(mtype, "%lu",  *(uint32_t *)(variable) );
            break;
        case cvStringType:
            MessageF(mtype, "%s",  (char *)(variable) );
            break;
        case cvDriverType:
            {
                DriverType dt = *(DriverType *)(variable);
                MessageF(mtype, "%s", dt.ToString());
            }
            break;
#if HAS_WIFI_NETWORKING
        case cvModuleType:
            {
                NetworkModuleType mt = *(NetworkModuleType *)(variable);
                MessageF(mtype, "%s", mt.ToString());
            }
            break;
#endif
        default:{
            
        }
    }
}


extern "C" uint32_t USBReadOverrun;
extern uint32_t _sdata;
extern uint32_t _estack;
#if STM32F4
extern uint32_t _sccmram;
extern uint32_t _ccmramend;
#elif STM32H7
extern uint32_t _nocache_ram_start;
extern uint32_t _nocache_ram_end;
extern uint32_t _nocache2_ram_start;
extern uint32_t _nocache2_ram_end;
#endif

//Information printed by M122 P200
void BoardConfig::Diagnostics(MessageType mtype) noexcept
{
    MessageF(mtype, "=== Diagnostics ===\n");
#if HAS_SBC_INTERFACE
	MessageF(mtype, "%s version %s running on %s (%s mode) at %dMhz\n", FIRMWARE_NAME, VERSION, reprap.GetPlatform().GetElectronicsString(),
						(reprap.UsingSbcInterface()) ? "SBC" : "standalone", (int)SystemCoreClock/1000000);
#else
	MessageF(mtype, "%s (%s) version %s running on %s at %dMhz\n", FIRMWARE_NAME, BoardName, VERSION, reprap.GetPlatform().GetElectronicsString(), (int)SystemCoreClock/1000000);
#endif
    const char *Bootloader = GetBootloaderString();
    MessageF(mtype, "Bootloader: %s\n", Bootloader == nullptr ? "Unknown" : Bootloader);


    MessageF(mtype, "\n== Configurable Board.txt Settings ==\n");
    //Print rest of board configurations
    const size_t numConfigs = ARRAY_SIZE(boardConfigs);
    for(size_t i=0; i<numConfigs; i++)
    {
        boardConfigEntry_t next = boardConfigs[i];

        MessageF(mtype, "%s = ", next.key );
        if(next.numItems > 1)
        {
            MessageF(mtype, "{");
            for(size_t p=0; p<(next.numItems); p++)
            {
                if (p > 0)
                    MessageF(mtype, ", ");
                switch(next.type)
                {
                    case cvPinType:
                        BoardConfig::PrintValue(mtype, next.type, (void *)&((Pin *)(next.variable))[p]);
                        break;
                    case cvDriverType:
                        BoardConfig::PrintValue(mtype, next.type, (void *)&((DriverType *)(next.variable))[p]);
                        break;
                    default:
                        break;
                }
            }
            MessageF(mtype, "}\n");
        }
        else
        {
            BoardConfig::PrintValue(mtype, next.type, next.variable);
            MessageF(mtype, "\n");

        }
    }

    // Display all pins
    MessageF(mtype, "\n== Defined Pins ==\n");
    for (uint8_t p = 0; p < (size_t)NoPin; p++)
    {
        const char *name = GetPinNames((Pin)p);
        if (name[0] != 0 && islower(name[0]))
        {
            MessageF(mtype, "%s = ", name );
            BoardConfig::PrintValue(mtype, cvPinType, (void *)&p);
            MessageF(mtype, "\n");
        }
    }
    
#if defined(SERIAL_AUX_DEVICE) || defined(SERIAL_AUX2_DEVICE) || HAS_WIFI_NETWORKING
    MessageF(mtype, "\n== Hardware Serial ==\n");
    #if defined(SERIAL_AUX_DEVICE)
        MessageF(mtype, "AUX Serial: %s%c\n", ((SERIAL_AUX_DEVICE.GetUARTPortNumber() == -1)?"Disabled": "UART "), (SERIAL_AUX_DEVICE.GetUARTPortNumber() == -1)?' ': ('0' + SERIAL_AUX_DEVICE.GetUARTPortNumber()));
    #endif
    #if defined(SERIAL_AUX2_DEVICE)
        MessageF(mtype, "AUX2 Serial: %s%c\n", ((SERIAL_AUX2_DEVICE.GetUARTPortNumber() == -1)?"Disabled": "UART "), (SERIAL_AUX2_DEVICE.GetUARTPortNumber() == -1)?' ': ('0' + SERIAL_AUX2_DEVICE.GetUARTPortNumber()));
    #endif
    #if HAS_WIFI_NETWORKING
        MessageF(mtype, "WIFI Serial: %s%c\n", ((serialWiFi.GetUARTPortNumber() == -1)?"Disabled": "UART "), (serialWiFi.GetUARTPortNumber() == -1)?' ': ('0' + serialWiFi.GetUARTPortNumber()));
    #endif
#endif
    
    

    MessageF(mtype, "\n== PWM ==\n");
    for(uint8_t i=0; i<MaxPWMChannels; i++)
    {
		String<StringLength256> status;
		PWMPins[i].appendStatus(status.GetRef());
		MessageF(mtype, "%u: %s\n", i, status.c_str());
	}

    MessageF(mtype, "\n== Attached interrupt pins ==\n");
    for(uint32_t i = 0; i < 16; i++)
    {
        Pin p = getAttachedPin(i);
        MessageF(mtype, "%u: ", (unsigned)i);
        if (p != NoPin)
            BoardConfig::PrintValue(mtype, cvPinType, (void *)&p);
        MessageF(mtype, "\n");
    }

    MessageF(mtype, "\n== MCU ==\n");
    MessageF(mtype, "AdcBits = %d\n", (int) LegacyAnalogIn::AdcBits);
    MessageF(mtype, "TS_CAL1 (30C) = %d\n", (int) (*TEMPSENSOR_CAL1_ADDR));
    MessageF(mtype, "TS_CAL2 (110C) = %d\n", (int) (*TEMPSENSOR_CAL2_ADDR));
    MessageF(mtype, "V_REFINCAL (30C 3.3V) = %d\n\n", (int) (*VREFINT_CAL_ADDR));
    uint32_t vrefintraw = AnalogInReadChannel(LegacyAnalogIn::GetVREFAdcChannel());
    float vref = 3.3f*((float)(GET_ADC_CAL(VREFINT_CAL_ADDR, VREFINT_CAL_DEF)))/(float)(vrefintraw >> (LegacyAnalogIn::AdcBits - 12));
    MessageF(mtype, "V_REFINT raw %d\n", (int) vrefintraw);
    MessageF(mtype, "V_REF  %f\n\n", (double)vref);
    float tmcuraw = (float)AnalogInReadChannel(LegacyAnalogIn::GetTemperatureAdcChannel());
    MessageF(mtype, "T_MCU raw %d\n", (int) tmcuraw);
    MessageF(mtype, "T_MCU cal %f\n", (double)(((110.0f - 30.0f)/(((float)(GET_ADC_CAL(TEMPSENSOR_CAL2_ADDR, TEMPSENSOR_CAL2_DEF))) - ((float)(GET_ADC_CAL(TEMPSENSOR_CAL1_ADDR, TEMPSENSOR_CAL1_DEF))))) * ((float)(tmcuraw / (float) (1 << (LegacyAnalogIn::AdcBits - 12))) - ((float)(GET_ADC_CAL(TEMPSENSOR_CAL1_ADDR, TEMPSENSOR_CAL1_DEF)))) + 30.0f)); 
    MessageF(mtype, "T_MCU calc %f\n\n", (double)(((tmcuraw*3.3f)/(float)((1 << LegacyAnalogIn::AdcBits) - 1) - 0.76f)/0.0025f + 25.0f));
    tmcuraw = tmcuraw*vref/3.3f; 
    MessageF(mtype, "T_MCU raw (corrected) %d\n", (int) tmcuraw);
    MessageF(mtype, "T_MCU cal (corrected) %f\n", (double)(((110.0f - 30.0f)/(((float)(GET_ADC_CAL(TEMPSENSOR_CAL2_ADDR, TEMPSENSOR_CAL2_DEF))) - ((float)(GET_ADC_CAL(TEMPSENSOR_CAL1_ADDR, TEMPSENSOR_CAL1_DEF))))) * ((float)(tmcuraw / (float) (1 << (LegacyAnalogIn::AdcBits - 12))) - ((float)(GET_ADC_CAL(TEMPSENSOR_CAL1_ADDR, TEMPSENSOR_CAL1_DEF)))) + 30.0f)); 
    MessageF(mtype, "T_MCU calc (corrected) %f\n", (double)(((tmcuraw*3.3f)/(float)((1 << LegacyAnalogIn::AdcBits) - 1) - 0.76f)/0.0025f + 25.0f));
    MessageF(mtype, "Device Id %x Revison Id %x CPUId r%dp%d \n", (unsigned)LL_DBGMCU_GetDeviceID(), (unsigned)LL_DBGMCU_GetRevisionID(),  
                                            (unsigned)((SCB->CPUID >> 20) & 0x0F), (unsigned)(SCB->CPUID & 0x0F));
    MessageF(mtype, "\n== RAM ==\n");
    MessageF(mtype, "RAM start 0x%x end 0x%x\n", (unsigned)&_sdata, (unsigned)&_estack);
#if STM32F4
    MessageF(mtype, "CCMRAM start 0x%x end 0x%x\n", (unsigned)&_sccmram, (unsigned)&_ccmramend);
#elif STM32H7
    MessageF(mtype, "No cache RAM 1 start 0x%x end 0x%x\n", (unsigned)&_nocache_ram_start, (unsigned)&_nocache_ram_end);
    MessageF(mtype, "No cache RAM 2 start 0x%x end 0x%x\n", (unsigned)&_nocache2_ram_start, (unsigned)&_nocache2_ram_end);
#endif

    MessageF(mtype, "\n== USB ==\n");
    //MessageF(mtype, "Read overrun %d\n", (int)USBReadOverrun);
    //USBReadOverrun = 0;
}

//Set a variable from a string using the specified data type
void BoardConfig::SetValueFromString(configValueType type, void *variable, char *valuePtr) noexcept
{
    switch(type)
    {
        case cvPinType:
            *(Pin *)(variable) = LookupPin(valuePtr);
            break;
        case cvBoolType:
            {
                bool res = false;
                
                if(strlen(valuePtr) == 1)
                {
                    //check for 0 or 1
                    if(valuePtr[0] == '1') res = true;
                }
                else if(strlen(valuePtr) == 4 && StringEqualsIgnoreCase(valuePtr, "true"))
                {
                    res = true;
                }
                *(bool *)(variable) = res;
            }
            break;
        case cvFloatType:
            {
                const char *ptr = nullptr;
                *(float *)(variable) = SafeStrtof(valuePtr, &ptr);
            }
            break;
        case cvUint8Type:
            {
                const char *ptr = nullptr;
                uint8_t val = StrToU32(valuePtr, &ptr);
                if(val < 0) val = 0;
                if(val > 0xFF) val = 0xFF;
                
                *(uint8_t *)(variable) = val;
            }
            break;
        case cvUint16Type:
            {
                const char *ptr = nullptr;
                uint16_t val = StrToU32(valuePtr, &ptr);
                if(val < 0) val = 0;
                if(val > 0xFFFF) val = 0xFFFF;
                
                *(uint16_t *)(variable) = val;
                    
            }
            break;
        case cvUint32Type:
            {
                const char *ptr = nullptr;
                *(uint32_t *)(variable) = StrToU32(valuePtr, &ptr);
            }
            break;
        case cvStringType:
            {
                
                //TODO:: string Type only handles Board Name variable
                if(strlen(valuePtr)+1 < MaxBoardNameLength)
                {
                    strcpy((char *)(variable), valuePtr);
                }
            }
            break;
#if HAS_WIFI_NETWORKING
        case cvModuleType:
            *(NetworkModuleType *)(variable) = NetworkModuleType(valuePtr);
            break;
#endif
        default:
            debugPrintf("Unhandled ValueType %d\n", (int)type);
    }
}

bool BoardConfig::LoadBoardConfigFromFile(const char *filePath, bool restricted) noexcept
{
    FileStore * const configFile = MassStorage::OpenFile(filePath, OpenMode::read, 0);
    if (configFile == nullptr)
    {
        MessageF(UsbMessage, "Configuration file %s not found\n", filePath );
        FlushMessages();
        return false;
    }
    MessageF(UsbMessage, "Loading config from %s...\n", filePath );
    char savedBoardName[MaxBoardNameLength];
    memcpy(savedBoardName, BoardName, MaxBoardNameLength);
    BoardConfig::GetConfigKeys(configFile);
    if (restricted)
    {
        // Don't allow changes to the board name
        memcpy(BoardName, savedBoardName, MaxBoardNameLength);
    }
    configFile->Close();
    FlushMessages();
    return true;
}

#if HAS_SBC_INTERFACE
bool BoardConfig::LoadBoardConfigFromSBC() noexcept
{
    // Is this feature disabled?
    if (!SbcLoadConfig) return false;
    // get existing config
    // Force Sbc mode on
    SbcMode = true;
    InMemoryBoardConfiguration oldConfig, newConfig;
    oldConfig.getConfiguration();
    // now ask sbc for new config
    LoadBoardDefaults();
    BoardConfig::LoadBoardConfigFromFile(boardConfigFile);
#if HAS_SMART_DRIVERS
    ConfigureDriveType();
#endif
    // SbcLoadConfig/SbcMode may have been reset force it back on
    SbcMode = SbcLoadConfig = true;
    newConfig.getConfiguration();
    if (oldConfig.isEqual(newConfig))
        MessageF(UsbMessage, "Configurations match\n");
    else
    {
        // store new config into memory that will survive a reboot
        newConfig.saveToBackupRAM();
        MessageF(UsbMessage, "Configurations do not match rebooting to load new settings\n");
        FlushMessages();
        reprap.EmergencyStop();
        delay(1000);
        SoftwareReset(SoftwareResetReason::erase); // Reboot
    }
    return true;
}

void BoardConfig::InvalidateBoardConfiguration() noexcept
{
    InMemoryBoardConfiguration emptyConfig;
    emptyConfig.saveToBackupRAM();
}
#endif

static
const boardConfigEntry_t* FindConfigKey(const char *key)
{
    const size_t numConfigs = ARRAY_SIZE(boardConfigs);
    for(size_t i=0; i<numConfigs; i++)
    {
        if(StringEqualsIgnoreCase(key, boardConfigs[i].key))
        {
            return &boardConfigs[i];
        }
    }
    return nullptr;
}

static
size_t SkipWhitespace(size_t pos, const char *line, size_t len)
{
    while(pos < len && isSpaceOrTab(line[pos])) pos++; //eat leading whitespace
    // if we hit a comment we skip to the end of the line
    if (line[pos] == '/' || line[pos] == '#' || line[pos] == ';')
    {
        pos = len;
    }
    return pos;
}

static
bool IsValidChar(char c)
{
    c = tolower(c);
    return isalpha(c) || isdigit(c) || c == '.' || c == '_';
}

bool BoardConfig::GetConfigKeys(FileStore * const configFile) noexcept
{
    constexpr size_t maxLineLength = 120;
    char line[maxLineLength];

    int readLen = configFile->ReadLine(line, maxLineLength);
    while(readLen >= 0)
    {
        //debugPrintf("ReadLine returns %d %s\n", readLen, line);
        size_t len = (size_t) readLen;
        size_t pos = 0;
        pos = SkipWhitespace(pos, line, len);

        if(pos < len){

            //check for comments
            if(line[pos] == '/' || line[pos] == '#' || line[pos] == ';')
            {
                //Comment - Skipping
            }
            else
            {
                const char* key = line + pos;
                // find the end of the key
                while(pos < len && IsValidChar(line[pos])) pos++;
                char* endKey = line + pos;
                pos = SkipWhitespace(pos, line, len);
                if (pos < len && line[pos] == '=')
                {
                    // terminate the key
                    *endKey = 0;
                    // skip the '='
                    pos++;

                    pos = SkipWhitespace(pos, line, len);

                    //debugPrintf("Key: %s", key);

                    if(pos < len && line[pos] == '{')
                    {
                        // { indicates the start of an array
                        //debugPrintf(" { ");
                        pos++; //skip the {

                        //Array of Values:
                        const boardConfigEntry_t* confEntry = FindConfigKey(key);
                        if(confEntry != nullptr && confEntry->numItems > 1 && (confEntry->type == cvPinType || confEntry->type == cvDriverType))
                        {
                            //matched an entry in boardConfigEntryArray

                            //create a temp array to read into. Only copy the array entries into the final destination when we know the array is properly defined
                            const size_t maxArraySize = confEntry->numItems;
                            
                            //Pin Array Type
                            Pin readArray[maxArraySize];
                            DriverType readArrayDT[maxArraySize];

                            pos = SkipWhitespace(pos, line, len);
                            bool searching = true;
                            size_t arrIdx = 0;

                            //search for values in Array
                            while( searching )
                            {
                                if(pos < len)
                                {
                                    pos = SkipWhitespace(pos, line, len);
                                    if(pos >= len)
                                    {
                                        debugPrintf("Got to end of line before end of array, line must be longer than maxLineLength");
                                        searching = false;
                                        break;
                                    }

                                    bool closedSuccessfully = false;
                                    //check brace isnt closed
                                    if(pos < len && line[pos] == '}')
                                    {
                                        closedSuccessfully = true;
                                        arrIdx--; // we got the closing brace before getting a value this round, decrement arrIdx
                                    }
                                    else
                                    {

                                        if(arrIdx >= maxArraySize )
                                        {
                                            debugPrintf("Error : Too many entries defined in config for array\n");
                                            searching = false;
                                            break;
                                        }

                                        //Try to Read the next Value

                                        //should be at first char of value now
                                        char *valuePtr = line+pos;

                                        //read until end condition - space,comma,}
                                        while(pos < len && IsValidChar(line[pos]))
                                        {
                                            line[pos] = tolower(line[pos]);
                                            pos++;
                                        }
                                        char *endValue = line+pos;
                                        // Skip trailing whitespace
                                        pos = SkipWhitespace(pos, line, len);

                                        // make sure we ended on a valid character
                                        if(pos >= len || (line[pos] != '}' && line[pos] != ','))
                                        {
                                            debugPrintf("Error: invalid array\n");
                                            searching = false;
                                            break;
                                        }

                                        //check if there is a closing brace after value without any whitespace, before it gets overwritten with a null
                                        if(line[pos] == '}')
                                        {
                                            closedSuccessfully = true;
                                        }

                                        *endValue = 0; // null terminate the string

                                        //debugPrintf("%s ", valuePtr);

                                        //Put into the Temp Array
                                        if(arrIdx >= 0 && arrIdx<maxArraySize)
                                        {
                                            switch(confEntry->type)
                                            {
                                                case cvPinType:
                                                    readArray[arrIdx] = LookupPin(valuePtr);
                                                    break;
                                                case cvDriverType:
                                                    readArrayDT[arrIdx] = DriverType(valuePtr);
                                                    break;
                                                default:
                                                    break;
                                            }
                                        }
                                    }

                                    if(closedSuccessfully == true)
                                    {
                                        //debugPrintf("}\n");
                                        //Array Closed - Finished Searching
                                        if(arrIdx >= 0 && arrIdx < maxArraySize) //arrIndx will be -1 if closed before reading any values
                                        {
                                            //All values read successfully, copy temp array into Final destination
                                            //dest array may be larger, dont overrite the default values
                                            for(size_t i=0; i<(arrIdx+1); i++ )
                                            {
                                                switch(confEntry->type)
                                                {
                                                    case cvPinType:
                                                        ((Pin *)(confEntry->variable))[i] = readArray[i];
                                                        break;
                                                    case cvDriverType:
                                                        ((DriverType *)(confEntry->variable))[i] = readArrayDT[i];
                                                        break;
                                                    default:
                                                        break;
                                                }
                                            }
                                            //Success!
                                            searching = false;
                                            break;

                                        }
                                        //failed to set values
                                        searching = false;
                                        break;
                                    }
                                    arrIdx++;
                                    pos++;
                                }
                                else
                                {
                                    debugPrintf("Unable to find values for Key\n");
                                    searching = false;
                                    break;
                                }
                            }//end while(searching)
                        }//end if matched key
                        else
                        {
                            debugPrintf("Key not found or wrong type %s\n", key);
                        }
                    }
                    else
                    {
                        //single value
                        if(pos < len)
                        {
                            //should be at first char of value now
                            char *valuePtr = line+pos;
                            if (*valuePtr == '"')
                            {
                                // quoted string
                                pos++;
                                valuePtr++;
                                while (pos < len && line[pos] != '"') pos++;
                            }
                            else
                            {
                                //read until end condition - space, ;, comment, null,etc
                                while(pos < len && IsValidChar(line[pos])) pos++;
                            }

                            //overrite the end condition with null....
                            line[pos] = 0; // null terminate the string (the "value")
                            //debugPrintf(" value is %s\n", valuePtr);
                            const boardConfigEntry_t* confEntry = FindConfigKey(key);
                            //Single Value config entries have 1 for numItems
                            if(confEntry != nullptr && confEntry->numItems == 1)
                            {
                                //debugPrintf("Setting value\n");
                                //match
                                BoardConfig::SetValueFromString(confEntry->type, confEntry->variable, valuePtr);
                            }
                            else
                            {
                                debugPrintf("Key not found or wrong type %s\n", key);
                            }
                        }
                    }
                }
                else
                {
                    debugPrintf("Missing equals %s\n", line);
                }
            }
        }
        else
        {
            //Empty Line - Nothing to do here
        }

        readLen = configFile->ReadLine(line, maxLineLength); //attempt to read the next line
    }
    return false;
}

void assert_failed(uint8_t *file, uint32_t line)
{
    debugPrintf("Assert failed file %s line %d\n", file, (int)line);
    delay(1000);
}
