#include "RepRapFirmware.h"
#include "Platform/Platform.h"
#include "BoardConfig.h"
#include "Boards/BIQU_SKR.h"
#include "Boards/FLY.h"
#include "Boards/FYSETC.h"
#include "Boards/Generic.h"
//Known boards with built in stepper configurations and pin table
// Note the generic entry must be the first entry in the table.
constexpr BoardEntry LPC_Boards[] =
{
    {{"generic"},      PinTable_Generic,    ARRAY_SIZE(PinTable_Generic),    Generic_Defaults},
#if STM32H7
# if STM32H743xx
    {{"fly_super5"},      PinTable_FLY_SUPER5,    ARRAY_SIZE(PinTable_FLY_SUPER5),    fly_super5_Defaults},
    {{"fly_super8h7", "fly_super8_pro"},      PinTable_FLY_SUPER8H7,    ARRAY_SIZE(PinTable_FLY_SUPER8H7),    fly_super8h7_Defaults},
    {{"biquskr_se_bx_2.0"},      PinTable_BIQU_SKR_SE_BX_v2_0,    ARRAY_SIZE(PinTable_BIQU_SKR_SE_BX_v2_0),    biqu_skr_se_bx_v2_0_Defaults},
    {{"biquskr_3", "biquskr_3_ez"},      PinTable_BTT_SKR_3,    ARRAY_SIZE(PinTable_BTT_SKR_3),    btt_skr_3_Defaults},
# elif STM32H723xx
    {{"fly_super5_h723"},      PinTable_FLY_SUPER5,    ARRAY_SIZE(PinTable_FLY_SUPER5),    fly_super5_Defaults},
    {{"fly_super8_pro_h723"},      PinTable_FLY_SUPER8H7,    ARRAY_SIZE(PinTable_FLY_SUPER8H7),    fly_super8h7_Defaults},
    {{"fly_openpnp_tool"},      PinTable_FLY_OPENPNP_TOOL,    ARRAY_SIZE(PinTable_FLY_OPENPNP_TOOL),    fly_openpnp_tool_Defaults},
    {{"biquskr_3_h723", "biquskr_3_ez_h723"},      PinTable_BTT_SKR_3,    ARRAY_SIZE(PinTable_BTT_SKR_3),    btt_skr_3_Defaults},
    {{"fysetc_spider_king723"}, PinTable_FYSETC_SPIDER_KING407, ARRAY_SIZE(PinTable_FYSETC_SPIDER_KING407), fysetc_spider_king407_Defaults},
# endif
#else
    {{"biquskrpro_1.1"},      PinTable_BIQU_SKR_PRO_v1_1,    ARRAY_SIZE(PinTable_BIQU_SKR_PRO_v1_1),    biquskr_pro_1_1_Defaults},
    {{"biqugtr_1.0"},      PinTable_BIQU_GTR_v1_0,    ARRAY_SIZE(PinTable_BIQU_GTR_v1_0),    biqu_gtr_1_0_Defaults},
    {{"fly_e3_pro"},      PinTable_FLY_E3_PRO,    ARRAY_SIZE(PinTable_FLY_E3_PRO),    fly_e3_pro_Defaults},
    {{"fly_e3_prov3"},      PinTable_FLY_E3_PROV3,    ARRAY_SIZE(PinTable_FLY_E3_PROV3),    fly_e3_prov3_Defaults},
    {{"fly_f407zg"},      PinTable_FLY_F407ZG,    ARRAY_SIZE(PinTable_FLY_F407ZG),    fly_f407zg_Defaults},
    {{"fly_e3"},      PinTable_FLY_E3,    ARRAY_SIZE(PinTable_FLY_E3),    fly_e3_Defaults},
    {{"fly_e3_v2"},      PinTable_FLY_E3_V2,    ARRAY_SIZE(PinTable_FLY_E3_V2),    fly_e3_v2_Defaults},
    {{"fly_cdyv2", "fly_cdyv3"},      PinTable_FLY_CDYV2,    ARRAY_SIZE(PinTable_FLY_CDYV2),    fly_cdyv2_Defaults},
    {{"fly_super8"},      PinTable_FLY_SUPER8,    ARRAY_SIZE(PinTable_FLY_SUPER8),    fly_super8_Defaults},    
    {{"fly_gemini"},      PinTable_FLY_GEMINI,    ARRAY_SIZE(PinTable_FLY_GEMINI),    fly_gemini_Defaults},    
    {{"fly_geminiv1.1"},      PinTable_FLY_GEMINI_V1_1,    ARRAY_SIZE(PinTable_FLY_GEMINI_V1_1),    fly_gemini_v1_1_Defaults},    
    {{"fly_geminiv2.0"},      PinTable_FLY_GEMINI_V2_0,    ARRAY_SIZE(PinTable_FLY_GEMINI_V2_0),    fly_gemini_v2_0_Defaults},    
    {{"biquskr_rrf_e3_1.1"},      PinTable_BTT_RRF_E3_v1_1,    ARRAY_SIZE(PinTable_BTT_RRF_E3_v1_1),    btt_rrf_e3_1_1_Defaults},
    {{"biquskr_2"}, PinTable_BTT_SKR_2, ARRAY_SIZE(PinTable_BTT_SKR_2), btt_skr_2_Defaults},
    {{"biqoctopus_1.1", "biquoctopus_1.1"}, PinTable_BTT_OCTOPUS, ARRAY_SIZE(PinTable_BTT_OCTOPUS), btt_octopus_Defaults},
    {{"biqoctopuspro_1.0", "biquoctopuspro_1.0"}, PinTable_BTT_OCTOPUSPRO, ARRAY_SIZE(PinTable_BTT_OCTOPUSPRO), btt_octopuspro_Defaults},
    {{"biquoctopus_x7", "troodon_v2"}, PinTable_BIQU_OCTOPUS_X7, ARRAY_SIZE(PinTable_BIQU_OCTOPUS_X7), biquoctopus_x7_Defaults},
    {{"fysetc_spider"}, PinTable_FYSETC_SPIDER, ARRAY_SIZE(PinTable_FYSETC_SPIDER), fysetc_spider_Defaults},
    {{"fysetc_spider_king407"}, PinTable_FYSETC_SPIDER_KING407, ARRAY_SIZE(PinTable_FYSETC_SPIDER_KING407), fysetc_spider_king407_Defaults},
#endif
};
constexpr size_t NumBoardEntries = ARRAY_SIZE(LPC_Boards);


//Default values for configurable variables.


//All I/Os default to input and are floating
Pin PinsSetHigh[MaxInitialPins];
Pin PinsSetLow[MaxInitialPins];

Pin TEMP_SENSE_PINS[NumThermistorInputs];
Pin SpiTempSensorCsPins[MaxSpiTempSensors]; // Used to deselect all devices at boot
SSPChannel TempSensorSSPChannel = SSPNONE;  // Off by default
float DefaultThermistorSeriesR = 4700.0;

Pin ATX_POWER_PIN = NoPin;                  // Pin to use to control external power
bool ATX_POWER_INVERTED = false;            // Should the state of this pin be inverted
bool ATX_INITIAL_POWER_ON = true;           // Should external power be on/off at startup
bool ATX_POWER_STATE = true;                // We may not have an actual pin so use this to track state

//SDCard pins and settings
Pin SdCardDetectPins[NumSdCards] = { NoPin, NoPin };
Pin SdSpiCSPins[NumSdCards] =      { PA_4,  NoPin };    // Internal, external. Note:: ("slot" 0 in CORE is configured to be LCP SSP1 to match default RRF behaviour)
uint32_t ExternalSDCardFrequency = 4000000;             //default to 4MHz
SSPChannel ExternalSDCardSSPChannel = SSPNONE;          // Off by default
uint32_t InternalSDCardFrequency = 25000000;            //default to 25MHz


Pin LcdCSPin = NoPin;               //LCD Chip Select
Pin LcdA0Pin = NoPin;               //DataControl Pin (A0) if none used set to NoPin
Pin LcdBeepPin = NoPin;
Pin EncoderPinA = NoPin;
Pin EncoderPinB = NoPin;
Pin EncoderPinSw = NoPin;           //click
Pin PanelButtonPin = NoPin;         //Extra button on Viki and RRD Panels (reset/back etc)
SSPChannel LcdSpiChannel = SSPNONE; //Off by default

Pin DiagPin = NoPin;
bool DiagOnPolarity = true;
Pin ActLedPin = NoPin;
bool ActOnPolarity = true;

//Stepper settings
Pin ENABLE_PINS[NumDirectDrivers];
Pin STEP_PINS[NumDirectDrivers];
Pin DIRECTION_PINS[NumDirectDrivers];
#if HAS_SMART_DRIVERS
#if HAS_STALL_DETECT && SUPPORT_TMC22xx
    Pin DriverDiagPins[NumDirectDrivers];
#endif
Pin TMC_PINS[NumDirectDrivers];
size_t totalSmartDrivers;
size_t num5160SmartDrivers = 0;
SSPChannel SmartDriversSpiChannel = SSPNONE;
uint32_t SmartDriversSpiCsDelay = 0;
#endif

uint32_t STEP_DRIVER_MASK = 0;                          //SD: mask of the step pins on Port 2 used for writing to step pins in parallel
bool hasStepPinsOnDifferentPorts = false;               //for boards that don't have all step pins on same port
bool hasDriverCurrentControl = false;                   //Supports digipots to set stepper current
float digipotFactor = 0.0;                              //defualt factor for converting current to digipot value


Pin SPIPins[NumSPIDevices][NumSPIPins];                 //GPIO pins for hardware/software SPI (used with SharedSPI)


#if HAS_WIFI_NETWORKING
    Pin EspDataReadyPin = NoPin;
    Pin SamTfrReadyPin = NoPin;
    Pin EspResetPin = NoPin;
    Pin EspEnablePin = NoPin;
    Pin SamCsPin = PB_12;
    Pin APIN_SerialWiFi_TXD = NoPin;
    Pin APIN_SerialWiFi_RXD = NoPin;
    SSPChannel WiFiSpiChannel = SSP2;
    Pin APIN_ESP_SPI_MOSI = NoPin;
    Pin APIN_ESP_SPI_MISO = NoPin;
    Pin APIN_ESP_SPI_SCK = NoPin;
    uint32_t WiFiClockReg = 0;

    Pin WifiSerialRxTxPins[NumberSerialPins] = {NoPin, NoPin};
#endif
    
//Aux Serial

#if defined(SERIAL_AUX_DEVICE)
    #if defined(__MBED__)
        Pin AuxSerialRxTxPins[NumberSerialPins] = {NoPin, NoPin}; //UART0 is the Main Serial on MBED so set Aux to nopin
    #else
        Pin AuxSerialRxTxPins[NumberSerialPins] = {PA_10, PA_9}; //Default to UART0
    #endif
#endif
#if defined(SERIAL_AUX2_DEVICE)
    Pin Aux2SerialRxTxPins[NumberSerialPins] = {NoPin, NoPin};
#endif

#if HAS_SBC_INTERFACE
    Pin SbcTfrReadyPin = NoPin;
    Pin SbcCsPin = PB_12;
    SSPChannel SbcSpiChannel = SSP2;
    bool SbcLoadConfig = false;
#endif

bool ADCEnablePreFilter = true;

#if SUPPORT_LED_STRIPS
Pin NeopixelOutPin = NoPin;
#endif

#if HAS_VOLTAGE_MONITOR
Pin PowerMonitorVinDetectPin = NoPin;
uint32_t VInDummyReading = 24;
#endif
Pin StepperPowerEnablePin = NoPin;

#if SUPPORT_ACCELEROMETERS
SSPChannel AccelerometerSpiChannel = SSPNONE;
#endif

//BrownOut Detection
//The brownout interrupt is triggered when the supply voltage drops below approx 2.2V
//If the voltage falls below approx 1.8V the BOD will reset the CPU (and Brownout will be
//shown in M122 as the reset cause when it restarts).
//If the voltage falls below 1V this will trigger a Power-On reset (power on will be shown
//in M122 as the reset cause)
//Initial revision CPUs require Vdd to be above 3.0V as per the Errata sheet Rev. 10.4 — 17 March 2020
volatile uint32_t BrownoutEvents = 0;
void BOD_IRQHandler()
{
    BrownoutEvents++;
}

//Default to the Generic PinTable
PinEntry *PinTable = (PinEntry *) PinTable_Generic;
size_t NumNamedLPCPins = ARRAY_SIZE(PinTable_Generic);
char lpcBoardName[MaxBoardNameLength] = "generic";

// Copy the default src pin array to dst array
static void SetDefaultPinArray(const Pin *src, Pin *dst, size_t len) noexcept
{
    //array is empty from board.txt config, set to defaults
    for(size_t i=0; i<len; i++)
    {
        dst[i] = src[i];
    }
}

static void InitPinArray(Pin *dst, size_t len) noexcept
{
    for(size_t i=0; i<len; i++)
        dst[i] = NoPin;
}

void ClearPinArrays() noexcept
{
    InitPinArray(SpiTempSensorCsPins, MaxSpiTempSensors);
    InitPinArray(PinsSetHigh, MaxInitialPins);
    InitPinArray(PinsSetLow, MaxInitialPins);
    InitPinArray(ENABLE_PINS, NumDirectDrivers);
    InitPinArray(STEP_PINS, NumDirectDrivers);
    InitPinArray(DIRECTION_PINS, NumDirectDrivers);
#if HAS_SMART_DRIVERS
    InitPinArray(TMC_PINS, NumDirectDrivers);
#endif
#if HAS_STALL_DETECT && SUPPORT_TMC22xx
    InitPinArray(DriverDiagPins, NumDirectDrivers);
#endif
    InitPinArray(TEMP_SENSE_PINS, NumThermistorInputs);
    for(size_t i = 0; i < ARRAY_SIZE(SPIPins); i++)
        InitPinArray(SPIPins[i], NumSPIPins);
    InitPinArray(SpiTempSensorCsPins, MaxSpiTempSensors);
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

//Find Board settings from string
bool SetBoard(const char* bn) noexcept
{
    const size_t numBoards = ARRAY_SIZE(LPC_Boards);

    for(size_t i=0; i<numBoards; i++)
    {
        for(size_t j=0; j < ARRAY_SIZE(LPC_Boards[0].boardName); j++)
            if(LPC_Boards[i].boardName[j] && StringEqualsIgnoreCase(bn, LPC_Boards[i].boardName[j]))
            {
                SafeStrncpy(lpcBoardName, bn, sizeof(lpcBoardName));
                PinTable = (PinEntry *)LPC_Boards[i].boardPinTable;
                NumNamedLPCPins = LPC_Boards[i].numNamedEntries;
                // Clear everything
                ClearPinArrays();
                //copy default settings
                for(size_t j = 0; j < ARRAY_SIZE(SPIPins); j++)
                    SetDefaultPinArray(LPC_Boards[i].defaults.spiPins[j], SPIPins[j], NumSPIPins);
                SetDefaultPinArray(LPC_Boards[i].defaults.enablePins, ENABLE_PINS, LPC_Boards[i].defaults.numDrivers);
                SetDefaultPinArray(LPC_Boards[i].defaults.stepPins, STEP_PINS, LPC_Boards[i].defaults.numDrivers);
                SetDefaultPinArray(LPC_Boards[i].defaults.dirPins, DIRECTION_PINS, LPC_Boards[i].defaults.numDrivers);
    #if HAS_SMART_DRIVERS
                SetDefaultPinArray(LPC_Boards[i].defaults.uartPins, TMC_PINS, LPC_Boards[i].defaults.numDrivers);
                totalSmartDrivers = LPC_Boards[i].defaults.numSmartDrivers;
    #endif
                digipotFactor = LPC_Boards[i].defaults.digipotFactor;
    #if HAS_VOLTAGE_MONITOR
                PowerMonitorVinDetectPin = LPC_Boards[i].defaults.vinDetectPin;
    #endif
                StepperPowerEnablePin = LPC_Boards[i].defaults.stepperPowerEnablePin;
    #if HAS_SBC_INTERFACE
                SbcTfrReadyPin = LPC_Boards[i].defaults.SbcTfrReadyPin;
                SbcCsPin = LPC_Boards[i].defaults.SbcCsPin;
                SbcSpiChannel = LPC_Boards[i].defaults.SbcSpiChannel;
    #endif
                InitDiagPin();
                return true;
            }
    }
    return false;
}

void PrintBoards(MessageType mtype) noexcept
{
    const size_t numBoards = ARRAY_SIZE(LPC_Boards);
    for(size_t i=0; i<numBoards; i++)
    {
        for(size_t j=0; j < ARRAY_SIZE(LPC_Boards[0].boardName); j++)
            if(LPC_Boards[i].boardName[j])
            {
                reprap.GetPlatform().MessageF(mtype, "Board %d.%d: %s iomode %d Signatures:", i, j, LPC_Boards[i].boardName[j], LPC_Boards[i].defaults.SDConfig);
                for (size_t k = 0; k < MaxSignatures; k++)
                    if (LPC_Boards[i].defaults.signatures[k] != 0)
                        reprap.GetPlatform().MessageF(mtype, " 0x%x", (unsigned)LPC_Boards[i].defaults.signatures[k]);
                reprap.GetPlatform().MessageF(mtype, "\n");
            }
    }
}




// Function to look up a pin name pass back the corresponding index into the pin table
// On this platform, the mapping from pin names to pins is fixed, so this is a simple lookup
bool LookupPinName(const char*pn, LogicalPin& lpin, bool& hardwareInverted) noexcept
{
    if (StringEqualsIgnoreCase(pn, NoPinName))
    {
        lpin = NoLogicalPin;
        hardwareInverted = false;
        return true;
    }

    for (size_t lp = 0; lp < NumNamedLPCPins; ++lp)
    {
        const char *q = PinTable[lp].names;
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
                lpin = (LogicalPin)PinTable[lp].pin;
                hardwareInverted = hwInverted;
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
    for (size_t i = 0; i < NumNamedLPCPins; ++i)
    {
        if ((LogicalPin)(PinTable[i].pin) == lp)
            return PinTable[i].names;
    }
    // not found manufascture a name
    static char name[5];
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

