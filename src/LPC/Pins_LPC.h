#ifndef PINS_LPC_H__
#define PINS_LPC_H__

#include "Microstepping.h"
#include "sd_mmc.h"
#include "RepRapFirmware.h"
#include "chip.h"
#include "SoftwareResetData.h"

#ifndef UNUSED
#define UNUSED(x) (void)(x)
#endif

#define FIRMWARE_NAME "RepRapFirmware for LPC176x based Boards"

// Default board type
#define DEFAULT_BOARD_TYPE BoardType::Lpc
#define ELECTRONICS "LPC176x"
#define LPC_ELECTRONICS_STRING "LPC176x"
#define LPC_BOARD_STRING "LPC176x"

#define FIRMWARE_FILE       "firmware.bin"
#define WIFI_FIRMWARE_FILE  "DuetWiFiServer.bin" // Firmware to be loaded onto the ESP board

//needed to compile
#define IAP_FIRMWARE_FILE "firmware.bin"
#define IAP_UPDATE_FILE "N/A"

#define FLASH_DATA_LENGTH (32*1024) //size of the Software Reset Data in Flash (Last Sector = 32K)


#if defined(ESP8266WIFI)
    constexpr size_t NumFirmwareUpdateModules = 4;        // 3 modules, plus one for manual upload to WiFi module (module 2 is now unused)
#else
    constexpr size_t NumFirmwareUpdateModules = 1;
#endif

// Features definition
#define SUPPORT_OBJECT_MODEL             1
#define HAS_CPU_TEMP_SENSOR		         0	// enabling the CPU temperature sensor disables Due pin 13 due to bug in SAM3X
#define HAS_HIGH_SPEED_SD		         0
#define HAS_VOLTAGE_MONITOR		         0
#define ACTIVE_LOW_HEAT_ON		         0
#define HAS_VREF_MONITOR                 0

#define SUPPORT_INKJET		             0	// set nonzero to support inkjet control
#define SUPPORT_ROLAND		             0	// set nonzero to support Roland mill
#define SUPPORT_SCANNER		             0	// set nonzero to support FreeLSS scanners
#define SUPPORT_IOBITS		             0	// set to support P parameter in G0/G1 commands
#define SUPPORT_DHT_SENSOR	             0	// set nonzero to support DHT temperature/humidity sensors
#define SUPPORT_WORKPLACE_COORDINATES    1
#define SUPPORT_LASER                    1

#define SUPPORT_TELNET                   0
#define SUPPORT_FTP                      0
#define HAS_MASS_STORAGE                 1


#define ALLOCATE_DEFAULT_PORTS           0

#if defined(LPC_NETWORKING)
    //LPC Ethernet
    #define HAS_RTOSPLUSTCP_NETWORKING   1
    #define SUPPORT_12864_LCD            1
    #define HAS_WIFI_NETWORKING          0

    #define BOARD_NAME          "LPC176x Ethernet"
    #define BOARD_SHORT_NAME    "LPCEth"
#elif defined(ESP8266WIFI)
    #define HAS_RTOSPLUSTCP_NETWORKING   0
    #define SUPPORT_12864_LCD            0
    #define HAS_WIFI_NETWORKING          1

    #define BOARD_NAME          "LPC176x WiFi"
    #define BOARD_SHORT_NAME    "LPCWiFi"
#else
    #define HAS_RTOSPLUSTCP_NETWORKING   0
    #define SUPPORT_12864_LCD            1
    #define HAS_WIFI_NETWORKING          0

    #define BOARD_NAME          "LPC176x"
    #define BOARD_SHORT_NAME    "LPC"

#endif


// The physical capabilities of the machine
constexpr size_t NumDirectDrivers = 5;               // The maximum number of drives supported by the electronics
#if defined(SUPPORT_TMC22xx)
    constexpr size_t MaxSmartDrivers = 5;            // The maximum number of smart drivers
    constexpr size_t NumTmcDriversSenseChannels = 2;
    #define LPC_TMC_SOFT_UART 1
#else
    constexpr size_t MaxSmartDrivers = 0;            // The maximum number of smart drivers
    #define LPC_TMC_SOFT_UART 0
#endif

constexpr size_t MaxSensors = 32;

constexpr size_t MaxHeaters = 3;                     // The maximum number of heaters in the machine
constexpr size_t MaxExtraHeaterProtections = 3;      // The number of extra heater protection instances

constexpr size_t MaxBedHeaters = 1;
constexpr size_t MaxChamberHeaters = 1;
constexpr int8_t DefaultBedHeater = 0;
constexpr int8_t DefaultE0Heater = 1;                // Index of the default first extruder heater, used only for the legacy status response

constexpr size_t NumThermistorInputs = 4;

constexpr size_t MaxZProbes = 1;
constexpr size_t MaxGpioPorts = 10;

constexpr size_t MinAxes = 3;                        // The minimum and default number of axes
constexpr size_t MaxAxes = 5;                        // The maximum number of movement axes in the machine, usually just X, Y and Z, <= DRIVES
constexpr size_t MaxDriversPerAxis = 2;              // The maximum number of stepper drivers assigned to one axis

constexpr size_t MaxExtruders = 2;                   // The maximum number of extruders
constexpr size_t NumDefaultExtruders = 1;            // The number of drivers that we configure as extruders by default

constexpr size_t MaxAxesPlusExtruders = 5;

constexpr size_t MaxHeatersPerTool = 2;
constexpr size_t MaxExtrudersPerTool = 2;

constexpr size_t MaxFans = 3;

constexpr unsigned int MaxTriggers = 16;            // Must be <= 32 because we store a bitmap of pending triggers in a uint32_t

//Steppers
extern Pin ENABLE_PINS[NumDirectDrivers];
extern Pin STEP_PINS[NumDirectDrivers];
extern Pin DIRECTION_PINS[NumDirectDrivers];
#if LPC_TMC_SOFT_UART
    extern Pin TMC_UART_PINS[NumDirectDrivers];
    constexpr Pin GlobalTmc22xxEnablePin = NoPin;			// The pin that drives ENN of all drivers
    constexpr uint32_t DriversBaudRate = 9600;
    constexpr uint32_t TransferTimeout = 100;				// any transfer should complete within 100 ticks @ 1ms/tick

#endif

extern uint32_t STEP_DRIVER_MASK; // Mask for parallel write to all steppers on port 2 (calculated in after loading board.txt)
extern bool hasStepPinsOnDifferentPorts;
extern bool hasDriverCurrentControl;
extern float digipotFactor;

constexpr uint32_t DefaultStandstillCurrentPercent = 100;

//Smoothie uses MCP4451 for current control
// Indices for motor current digipots (X,Y,Z,E) - E is on 2nd digipot chip
constexpr uint8_t POT_WIPES[5] = { 0, 1, 2, 3, 0};


// HEATERS - The bed is assumed to be the at index 0
extern Pin TEMP_SENSE_PINS[NumThermistorInputs];

// Default thermistor betas
constexpr float BED_R25 = 100000.0;
constexpr float BED_BETA = 3988.0;
constexpr float BED_SHC = 0.0;
constexpr float EXT_R25 = 100000.0;
constexpr float EXT_BETA = 4388.0;
constexpr float EXT_SHC = 0.0;

// Thermistor series resistor value in Ohms
constexpr float DefaultThermistorSeriesR = 4700.0;

constexpr size_t MaxSpiTempSensors = 2;
extern Pin SpiTempSensorCsPins[MaxSpiTempSensors];  // Digital pins the 31855s have their select lines tied to
constexpr SSPChannel TempSensorSSPChannel = SSP0;   //Connect SPI Temp sensor to SSP0

#if defined(ESP8266WIFI)
    extern Pin EspDataReadyPin;
    extern Pin SamTfrReadyPin;
    extern Pin EspResetPin;
    constexpr Pin SamCsPin = P0_16; //CS for SSP0
    constexpr LPC175X_6X_IRQn_Type ESP_SPI_IRQn = SSP0_IRQn;
#endif


//Hardware LPC Timers
//Timer 0 is used for the Step Generation
//Timer 1 is unused
//Timer 2 is fixed at 50Hz for servos
//Timer 3 is software pwm


extern Pin ATX_POWER_PIN;// Digital pin number that controls the ATX power on/off
extern bool ATX_POWER_INVERTED;

//SD: Internal SDCard is on SSP1
//    MOSI, MISO, SCLK, CS
//    P0_9, P0_8, P0_7, P0_6

//SD:: 2nd SDCard can be connected to SSP0
//    MOSI, MISO, SCLK
//    P0_18 P0_17 P0_15

// SD cards
constexpr size_t NumSdCards = _DRIVES; //_DRIVES is defined in CoreLPC (and used by FatFS) allow one internal and one external
extern Pin SdCardDetectPins[NumSdCards];
constexpr Pin SdWriteProtectPins[NumSdCards] = { NoPin, NoPin }; //unused on LPC boards
extern Pin SdSpiCSPins[NumSdCards];
extern uint32_t InternalSDCardFrequency;
extern uint32_t ExternalSDCardFrequency;
extern SSPChannel ExternalSDCardSSPChannel;


//LCD
constexpr uint32_t LcdSpiClockFrequency = 2000000;    // 2.0MHz
extern SSPChannel LcdSpiChannel;
extern Pin LcdCSPin;
extern Pin LcdDCPin;
extern Pin LcdBeepPin;
extern Pin EncoderPinA;
extern Pin EncoderPinB;
extern Pin EncoderPinSw;
extern Pin PanelButtonPin;

extern Pin DiagPin;

extern bool ADCEnablePreFilter;
extern uint8_t ADCPreFilterNumberSamples;
extern uint32_t ADCPreFilterSampleRate;

constexpr size_t NumSoftwareSPIPins = 3;
extern Pin SoftwareSPIPins[3]; //GPIO pins for softwareSPI (used with SharedSPI)

#include "usart.h"
// Use TX0/RX0 for the auxiliary serial line
#if defined(__MBED__)
    #define SERIAL_MAIN_DEVICE  Serial0 //TX0/RX0 connected to via seperate USB
    #define SERIAL_AUX_DEVICE   Serial  //USB pins unconnected.
    constexpr size_t NUM_SERIAL_CHANNELS = 2;

    #if defined(ESP8266WIFI)
        #define SERIAL_WIFI_DEVICE Serial3 //TXD3/RXD3 as uart0 is in use
        static const Pin APIN_Serial1_TXD = USART3->channel->TxPin;
        static const Pin APIN_Serial1_RXD = USART3->channel->RxPin;
    #endif

#else
    #define SERIAL_MAIN_DEVICE  Serial  //USB
    #if defined(ESP8266WIFI)
        //No AUX Serial, Serial0 is connected to the ESP8266
        constexpr size_t NUM_SERIAL_CHANNELS = 1;
        #define SERIAL_WIFI_DEVICE Serial0

        //Compatibility with RRF code
        static const Pin APIN_Serial1_TXD = USART0->channel->TxPin;
        static const Pin APIN_Serial1_RXD = USART0->channel->RxPin;
    #else
        #define SERIAL_AUX_DEVICE   Serial0
        constexpr size_t NUM_SERIAL_CHANNELS = 2;
    #endif

#endif


//Timer 0 is used for Step Generation
#define STEP_TC             (LPC_TIMER0)
#define STEP_TC_IRQN        TIMER0_IRQn
#define STEP_TC_HANDLER     TIMER0_IRQHandler




// Enum to represent allowed types of pin access
// We don't have a separate bit for servo, because Duet PWM-capable ports can be used for servos if they are on the Duet main board
enum class PinCapability: uint8_t
{
    // Individual capabilities
    read = 1,
    ain = 2,
    write = 4,
    pwm = 8,
    
    // Combinations
    ainr = 1|2,
    rw = 1|4,
    wpwm = 4|8,
    rwpwm = 1|4|8,
    ainrw = 1|2|4,
    ainrwpwm = 1|2|4|8
};

constexpr inline PinCapability operator|(PinCapability a, PinCapability b)
{
    return (PinCapability)((uint8_t)a | (uint8_t)b);
}

struct PinEntry
{
    bool CanDo(PinAccess access) const noexcept;
    Pin GetPin() const  noexcept{ return pin; }
    PinCapability GetCapability() const  noexcept{ return cap; }
    const char* GetNames() const  noexcept{ return names; }
    
    Pin pin;
    PinCapability cap;
    const char *names;
};

extern PinEntry *PinTable;



bool LookupPinName(const char *pn, LogicalPin& lpin, bool& hardwareInverted) noexcept;

constexpr const char *DefaultEndstopPinNames[] = { "nil", "nil", "nil" };
constexpr const char *DefaultZProbePinNames = "nil";
constexpr const char *DefaultFanPinNames[] = { "nil", "nil" };
constexpr PwmFrequency DefaultFanPwmFrequencies[] = { DefaultFanPwmFreq };



//Boards

bool SetBoard(const char* bn)  noexcept;

constexpr size_t MaxBoardNameLength = 20;
extern char lpcBoardName[MaxBoardNameLength];

struct BoardDefaults
{
    const Pin enablePins[NumDirectDrivers];
    const Pin stepPins[NumDirectDrivers];
    const Pin dirPins[NumDirectDrivers];
#if LPC_TMC_SOFT_UART
    const Pin uartPins[NumDirectDrivers];
#endif
    const float digipotFactor;    
};

struct BoardEntry
{
    const char *boardName;
    const PinEntry *boardPinTable;
    const size_t numNamedEntries;
    const BoardDefaults defaults;
};


#include "Boards/AzteegX5Mini.h"
#include "Boards/ReArm.h"
#include "Boards/Mbed.h"
#include "Boards/Smoothieboard.h"
#include "Boards/MKSSBase.h"
#include "Boards/AzsmzMini.h"
#include "Boards/BIQU_SKR.h"
#include "Boards/Generic.h"



//Known boards with built in stepper configurations and pin table 
constexpr BoardEntry LPC_Boards[] =
{
    {"mbed",             PinTable_Mbed,             ARRAY_SIZE(PinTable_Mbed),             mbedDefaults}, //for debugging
    {"generic",          PinTable_Generic,          ARRAY_SIZE(PinTable_Generic),          genericDefaults},

    //known boards
    {"smoothieboard",    PinTable_Smoothieboard,    ARRAY_SIZE(PinTable_Smoothieboard),    smoothieBoardDefaults},
    {"rearm",            PinTable_Rearm,            ARRAY_SIZE(PinTable_Rearm),            rearmDefaults},
    {"mkssbase_1.3",     PinTable_MKSSbase1_3,      ARRAY_SIZE(PinTable_MKSSbase1_3),      mkssbase1_3_Defaults},
    {"azsmzmini",        PinTable_AZSMZ,            ARRAY_SIZE(PinTable_AZSMZ),            azsmzDefaults},
    {"biquskr_1.1",      PinTable_BIQU_SKR_v1_1,    ARRAY_SIZE(PinTable_BIQU_SKR_v1_1),    biquskr_1_1_Defaults},
    {"biquskr_1.3",      PinTable_BIQU_SKR_v1_3,    ARRAY_SIZE(PinTable_BIQU_SKR_v1_3),    biquskr_1_3_Defaults},
    {"biquskr_1.4",      PinTable_BIQU_SKR_v1_4,    ARRAY_SIZE(PinTable_BIQU_SKR_v1_4),    biquskr_1_4_Defaults},
    {"azteegx5mini_1.1", PinTable_AzteegX5MiniV1_1, ARRAY_SIZE(PinTable_AzteegX5MiniV1_1), azteegX5Mini1_1Defaults},
};


//This needs to be const as its used in other places to create arrays
//Use the largest size which is the "generic" pintable
constexpr unsigned int NumNamedPins = ARRAY_SIZE(PinTable_Generic);



namespace StepPins
{
    // *** These next three functions must use the same bit assignments in the drivers bitmap ***
    // Each stepper driver must be assigned one bit in a 32-bit word, in such a way that multiple drivers can be stepped efficiently
    // and more or less simultaneously by doing parallel writes to several bits in one or more output ports.
    // All our step pins are on port D, so the bitmap is just the map of step bits in port D.
    
    // Calculate the step bit for a driver. This doesn't need to be fast. It must return 0 if the driver is remote.
    static inline uint32_t CalcDriverBitmap(size_t driver)  noexcept
    {
        if (driver >= NumDirectDrivers)
        {
            return 0;
        }

        if(STEP_PINS[driver] == NoPin ) return 0;
        if(hasStepPinsOnDifferentPorts == true )
        {
            //treat these pins one by one instead of parallel writes for now. Using driver pos in bitmap
            return 1u << driver ;
        }
        //Pins are on the same port,
        return 1u << (STEP_PINS[driver] & 0x1f); //lower 5-bits contains the bit number of a 32bit port
    }
    
    // Set the specified step pins high
    // This needs to be as fast as possible, so we do a parallel write to the port(s).
    static inline void StepDriversHigh(uint32_t driverMap)  noexcept
    {
        if(hasStepPinsOnDifferentPorts == true )
        {
            //Using driver pos in bitmap to match position in STEP_PINS
            uint8_t pos=0;
            while (driverMap!=0 && pos < NumDirectDrivers)
            {
                if(driverMap & 0x01)
                {
                    if(STEP_PINS[pos] != NoPin) GPIO_PinWrite(STEP_PINS[pos], 1); //set high
                }
                driverMap = driverMap >> 1;
                pos++;
            }
        }
        else
        {
            //pins all on port 2, parallel write
            LPC_GPIO2->SET = driverMap;
        }
    }
    
    // Set all step pins low
    // This needs to be as fast as possible, so we do a parallel write to the port(s).
    // We rely on only those port bits that are step pins being set in the STEP_DRIVER_MASK variable
    static inline void StepDriversLow() noexcept
    {
        if(hasStepPinsOnDifferentPorts == true )
        {
            for(size_t d=0; d<NumDirectDrivers; d++)
            {
                if(STEP_PINS[d] != NoPin) GPIO_PinWrite(STEP_PINS[d], 0); //set low
            }
        }
        else
        {
            //pins all on port 2, parallel write
            LPC_GPIO2->CLR = STEP_DRIVER_MASK; //clear only pins that are 1 in the mask
        }
    }
}


// From section 3.12.7 of http://infocenter.arm.com/help/topic/com.arm.doc.dui0553b/DUI0553.pdf:
// When you write to BASEPRI_MAX, the instruction writes to BASEPRI only if either:
// - Rn is non-zero and the current BASEPRI value is 0
// - Rn is non-zero and less than the current BASEPRI value
__attribute__( ( always_inline ) ) __STATIC_INLINE void __set_BASEPRI_MAX(uint32_t value)
{
  __ASM volatile ("MSR basepri_max, %0" : : "r" (value) : "memory");
}


#endif
