#include "RepRapFirmware.h"
#include "Pins_LPC.h"
#include "BoardConfig.h"


//Default values for configurable variables.


//All I/Os default to input with pullup after reset (9.2.1 from manual)
//The Smoothie Bootloader turns off Pins 2.4, 2.5, 2.6 and 2.7 which are used as Heater pins

Pin TEMP_SENSE_PINS[NumThermistorInputs] = {NoPin, NoPin, NoPin, NoPin};
Pin SpiTempSensorCsPins[MaxSpiTempSensors] = { NoPin, NoPin };
SSPChannel TempSensorSSPChannel = SSP0;   //default SPI Temp sensor on SSP0

Pin ATX_POWER_PIN = NoPin;
bool ATX_POWER_INVERTED = false;

//SDCard pins and settings
Pin SdCardDetectPins[NumSdCards] = { NoPin, NoPin };
Pin SdSpiCSPins[NumSdCards] =      { P0_6,  NoPin };    // Internal, external. Note:: ("slot" 0 in CORE is configured to be LCP SSP1 to match default RRF behaviour)
uint32_t ExternalSDCardFrequency = 4000000;             //default to 4MHz
#if HAS_LINUX_INTERFACE || HAS_WIFI_NETWORKING
    SSPChannel ExternalSDCardSSPChannel = SSPNONE;          // SSP0 used for network
#else
    SSPChannel ExternalSDCardSSPChannel = SSP0;             //default to SSP0
#endif
uint32_t InternalSDCardFrequency = 25000000;            //default to 25MHz


Pin LcdCSPin = NoPin;               //LCD Chip Select
Pin LcdDCPin = NoPin;               //DataControl Pin (A0) if none used set to NoPin
Pin LcdBeepPin = NoPin;
Pin EncoderPinA = NoPin;
Pin EncoderPinB = NoPin;
Pin EncoderPinSw = NoPin;           //click
Pin PanelButtonPin = NoPin;         //Extra button on Viki and RRD Panels (reset/back etc)
SSPChannel LcdSpiChannel = SSP0;

Pin DiagPin = NoPin;

//Stepper settings
Pin ENABLE_PINS[NumDirectDrivers] =     {NoPin, NoPin, NoPin, NoPin, NoPin};
Pin STEP_PINS[NumDirectDrivers] =       {NoPin, NoPin, NoPin, NoPin, NoPin};
Pin DIRECTION_PINS[NumDirectDrivers] =  {NoPin, NoPin, NoPin, NoPin, NoPin};
#if HAS_STALL_DETECT
    Pin DIAG_PINS[NumDirectDrivers] =     {NoPin, NoPin, NoPin, NoPin, NoPin};
#endif
#if LPC_TMC_SOFT_UART
    Pin TMC_UART_PINS[NumDirectDrivers] = {NoPin, NoPin, NoPin, NoPin, NoPin};
    size_t lpcSmartDrivers;
#endif
uint32_t STEP_DRIVER_MASK = 0;                          //SD: mask of the step pins on Port 2 used for writing to step pins in parallel
bool hasStepPinsOnDifferentPorts = false;               //for boards that don't have all step pins on port2
bool hasDriverCurrentControl = false;                   //Supports digipots to set stepper current
float digipotFactor = 0.0;                              //defualt factor for converting current to digipot value


Pin SoftwareSPIPins[3] = {NoPin, NoPin, NoPin};         //GPIO pins for softwareSPI (used with SharedSPI)
Pin SSP0Pins[4] = {SPI0_SCK, SPI0_MISO, SPI0_MOSI, SPI0_SSEL}; //GPIO pins for SSP0 (used with SharedSPI)


#if HAS_WIFI_NETWORKING
    Pin EspDataReadyPin = NoPin;
    Pin SamTfrReadyPin = NoPin;
    Pin EspResetPin = NoPin;

    Pin APIN_Serial1_TXD = NoPin;
    Pin APIN_Serial1_RXD = NoPin;

    Pin WifiSerialRxTxPins[NumberSerialPins] = {NoPin, NoPin};
#endif
    
//Aux Serial

#if defined(SERIAL_AUX_DEVICE)
    #if defined(__MBED__)
        Pin AuxSerialRxTxPins[NumberSerialPins] = {NoPin, NoPin}; //UART0 is the Main Serial on MBED so set Aux to nopin
    #else
        Pin AuxSerialRxTxPins[NumberSerialPins] = {P0_3, P0_2}; //Default to UART0
    #endif
#endif
#if defined(SERIAL_AUX2_DEVICE)
    Pin Aux2SerialRxTxPins[NumberSerialPins] = {NoPin, NoPin};
#endif

#if HAS_LINUX_INTERFACE
    Pin LinuxTfrReadyPin = NoPin;
#endif

bool ADCEnablePreFilter = false;
uint8_t ADCPreFilterNumberSamples = 8; //8 Samples per channel
uint32_t ADCPreFilterSampleRate = 10000; //10KHz


//Default to the Generic PinTable
PinEntry *PinTable = (PinEntry *) PinTable_Generic;
size_t NumNamedLPCPins = ARRAY_SIZE(PinTable_Generic);
char lpcBoardName[MaxBoardNameLength] = "generic";

bool IsEmptyPinArray(Pin *arr, size_t len) noexcept
{
    for(size_t i=0; i<len; i++)
    {
        if(arr[i] != NoPin) return false;
    }
    
    return true;
}

//If the dst array is empty, then copy the src array to dst array
void SetDefaultPinArray(const Pin *src, Pin *dst, size_t len) noexcept
{
    if(IsEmptyPinArray(dst, len))
    {
        //array is empty from board.txt config, set to defaults
        for(size_t i=0; i<len; i++)
        {
            dst[i] = src[i];
        }
    }
}


//Find Board settings from string
bool SetBoard(const char* bn) noexcept
{
    const size_t numBoards = ARRAY_SIZE(LPC_Boards);

    for(size_t i=0; i<numBoards; i++)
    {
        if(StringEqualsIgnoreCase(bn, LPC_Boards[i].boardName))
        {
            PinTable = (PinEntry *)LPC_Boards[i].boardPinTable;
            NumNamedLPCPins = LPC_Boards[i].numNamedEntries;

            //copy default settings (if not set in board.txt)
            SetDefaultPinArray(LPC_Boards[i].defaults.enablePins, ENABLE_PINS, MaxTotalDrivers);
            SetDefaultPinArray(LPC_Boards[i].defaults.stepPins, STEP_PINS, MaxTotalDrivers);
            SetDefaultPinArray(LPC_Boards[i].defaults.dirPins, DIRECTION_PINS, MaxTotalDrivers);
#if LPC_TMC_SOFT_UART
            SetDefaultPinArray(LPC_Boards[i].defaults.uartPins, TMC_UART_PINS, MaxTotalDrivers);
            lpcSmartDrivers = LPC_Boards[i].defaults.numSmartDrivers;
#endif


            digipotFactor = LPC_Boards[i].defaults.digipotFactor;
                        
            return true;
        }
    }
    return false;
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
            bool hwInverted = (*q == '!');
            if (hwInverted)
            {
                ++q;
            }
            while (*q != ',' && *q != 0 && *p == *q)
            {
                ++p;
                ++q;
            }
            if (*p == 0 && (*q == 0 || *q == ','))
            {
                // Found a match
                lpin = (LogicalPin)lp;
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
    //Note: only pins in the selected board lookup table are suported.
    const Pin lpcPin = BoardConfig::StringToPin(pn);
    if(lpcPin != NoPin){
        //find pin in lookup table
        for (size_t lp = 0; lp < NumNamedLPCPins; ++lp){
            if(lpcPin == PinTable[lp].pin){
                lpin = (LogicalPin)lp;
                hardwareInverted = false;
                return true;
            }
        }
    }
    return false;
}

