#ifndef PINS_STM32_H__
#define PINS_STM32_H__
#include "Core.h"
#include "sd_mmc.h"
#include "NVMEmulation.h"

#if !defined(COMBINEDFW)
# error "unsupported network build options"
#endif

#ifndef UNUSED
#define UNUSED(x) (void)(x)
#endif


// Default board type
#if STM32H7
# define FIRMWARE_NAME "RepRapFirmware for STM32H7 based Boards"
# define DEFAULT_BOARD_TYPE BoardType::Stm32H7
# define SUPPORT_CAN_EXPANSION       1
# define DUAL_CAN                    0
# define SUPPORT_SPICAN              0
# define FLASH_DATA_LENGTH (128*1024) //size of the Software Reset Data in Flash

# if STM32H743xx
#  define STM_ELECTRONICS_STRING "STM32H743"
#  define STM_BOARD_STRING "STM32H743"
#  define IAP_UPDATE_FILE         "stm32h743_iap_SD.bin"
#  define IAP_UPDATE_FILE_SBC     "stm32h743_iap_SBC.bin"
#  define IAP_CAN_LOADER_FILE     "stm32h743_iap_CAN.bin"

#  define IAP_IMAGE_START         0x24060000
# elif STM32H723xx
  // For now we use the generic stm32h7 name for stm32h743 based systems, this is compatible with released versions
#  define STM_ELECTRONICS_STRING "STM32H723"
#  define STM_BOARD_STRING "STM32H723"
#  define IAP_UPDATE_FILE         "stm32h723_iap_SD.bin"
#  define IAP_UPDATE_FILE_SBC     "stm32h723_iap_SBC.bin"
#  define IAP_CAN_LOADER_FILE     "stm32h723_iap_CAN.bin"

#  define IAP_IMAGE_START         0x24038000
# else
#  error "Processor undfined"
# endif
#else
# define FIRMWARE_NAME "RepRapFirmware for STM32F4 based Boards"
# define DEFAULT_BOARD_TYPE BoardType::Stm32F4
# define SUPPORT_CAN_EXPANSION       1
# define DUAL_CAN                    0
# define SUPPORT_SPICAN              1
# define FLASH_DATA_LENGTH (16*1024) //size of the Software Reset Data in Flash

# define STM_ELECTRONICS_STRING "STM32F4"
# define STM_BOARD_STRING "STM32F4"
# define IAP_UPDATE_FILE         "stm32f4_iap_SD.bin"
# define IAP_UPDATE_FILE_SBC     "stm32f4_iap_SBC.bin"
# define IAP_CAN_LOADER_FILE     "stm32f4_iap_CAN.bin"
# define IAP_IMAGE_START         0x20018000

#endif
// The name of the file used by the board bootloader, boot file is renamed to this
#define FIRMWARE_FILE       "0:/firmware.bin"

// Firmware to be loaded onto the ESP board
#define WIFI_FIRMWARE_FILE  (ModuleFiles[NetworkModule.ToBaseType()])


constexpr size_t NumFirmwareUpdateModules = 5;        // 3 modules, plus one for manual upload to WiFi module (module 2 is now unused)

// Features definition
#define SUPPORT_OBJECT_MODEL             1
#define HAS_CPU_TEMP_SENSOR              1	// enabling the CPU temperature sensor disables Due pin 13 due to bug in SAM3X
#define HAS_HIGH_SPEED_SD                0
#define HAS_VOLTAGE_MONITOR              1
#define ACTIVE_LOW_HEAT_ON               0
#define HAS_VREF_MONITOR                 0

#define SUPPORT_INKJET                   0	// set nonzero to support inkjet control
#define SUPPORT_ROLAND                   0	// set nonzero to support Roland mill
#define SUPPORT_SCANNER                  0	// set nonzero to support FreeLSS scanners
#define SUPPORT_IOBITS                   0	// set to support P parameter in G0/G1 commands
#define SUPPORT_DHT_SENSOR               1	// set nonzero to support DHT temperature/humidity sensors
#define SUPPORT_WORKPLACE_COORDINATES    1
#define SUPPORT_LASER                    1

#define HAS_ATX_POWER_MONITOR            1

#define SUPPORT_LED_STRIPS               1
#define SUPPORT_BITBANG_NEOPIXEL         1
#define SUPPORT_DMA_NEOPIXEL             1
#define SUPPORT_ASYNC_MOVES              1
#define ALLOCATE_DEFAULT_PORTS           0
#define TRACK_OBJECT_NAMES               1
#define HAS_DEFAULT_PSON_PIN             0
#define SUPPORT_BME280                   1
#define SUPPORT_12864_LCD                1
#define SUPPORT_PROBE_POINTS_FILE        1
#define SUPPORT_KEEPOUT_ZONES            1
#define HAS_EMBEDDED_CONFIG              1
#define BOARD_NAME                       (BoardLongName)
#define BOARD_SHORT_NAME                 (BoardName)
#define IAP_FIRMWARE_FILE                (iapFirmwareFile)


#define HAS_RTOSPLUSTCP_NETWORKING       0
#define HAS_WIFI_NETWORKING              1
#define HAS_MASS_STORAGE                 1
#define SUPPORT_TELNET                   0
#define HAS_SBC_INTERFACE                1
#define SUPPORT_MQTT                     1
#define SUPPORT_FTP                      1
#define SUPPORT_ACCELEROMETERS           1
#define HAS_WRITER_TASK                  1


// The physical capabilities of the machine
constexpr size_t NumDirectDrivers = 14;               // The maximum number of drives supported by the electronics
#if defined(SUPPORT_TMC22xx)
    constexpr size_t MaxSmartDrivers = NumDirectDrivers;            // The maximum number of smart drivers
    constexpr size_t NumTmcDriversSenseChannels = 1;
    #define TMC22xx_HAS_ENABLE_PINS         1
    #define TMC22xx_VARIABLE_NUM_DRIVERS    1
    #define TMC22xx_USE_SLAVEADDR           0
    #define TMC22xx_HAS_MUX                 0
    #define SUPPORT_TMC22xx                 1
    #define HAS_STALL_DETECT                1
    #define TMC22xx_DEFAULT_STEALTHCHOP     0
# if defined(SUPPORT_TMC51xx)
    #define SUPPORT_TMC51xx                 1
    #define TMC51xx_USES_USART              0
    #define TMC51xx_VARIABLE_NUM_DRIVERS    1

# endif
#elif defined(SUPPORT_TMC51xx)
    constexpr size_t MaxSmartDrivers = NumDirectDrivers;            // The maximum number of smart drivers
    constexpr size_t NumTmcDriversSenseChannels = 1;
    #define SUPPORT_TMC51xx                 1
    #define TMC51xx_USES_USART              0
    #define HAS_STALL_DETECT                1
    #define TMC51xx_VARIABLE_NUM_DRIVERS    1
#else
    constexpr size_t MaxSmartDrivers = 0;            // The maximum number of smart drivers
#endif


#include <Duet3Common.h>
#if STM32H7
constexpr size_t MaxPortsPerHeater = 2;

constexpr size_t MaxBedHeaters = 12;
constexpr size_t MaxChamberHeaters = 4;
constexpr int8_t DefaultBedHeater = 0;
constexpr int8_t DefaultE0Heater = 1;                // Index of the default first extruder heater, used only for the legacy status response

constexpr size_t NumThermistorInputs = 9;

constexpr size_t MinAxes = 3;                        // The minimum and default number of axes
constexpr size_t MaxAxes = 30;                       // The maximum number of movement axes in the machine, usually just X, Y and Z, <= DRIVES
constexpr size_t MaxDriversPerAxis = 8;              // The maximum number of stepper drivers assigned to one axis

constexpr size_t MaxExtruders = 20;                  // The maximum number of extruders

constexpr size_t MaxAxesPlusExtruders = 32;          // May be <= MaxAxes + MaxExtruders

constexpr size_t MaxHeatersPerTool = 20;
constexpr size_t MaxExtrudersPerTool = 12;

constexpr unsigned int MaxTriggers = 32;            // Must be <= 32 because we store a bitmap of pending triggers in a uint32_t
constexpr size_t MaxLedPorts = 32;                  // Max number of LED ports
#else
constexpr size_t MaxPortsPerHeater = 2;

constexpr size_t MaxBedHeaters = 4;
constexpr size_t MaxChamberHeaters = 4;
constexpr int8_t DefaultBedHeater = 0;
constexpr int8_t DefaultE0Heater = 1;                // Index of the default first extruder heater, used only for the legacy status response

constexpr size_t NumThermistorInputs = 9;

constexpr size_t MinAxes = 3;                        // The minimum and default number of axes
constexpr size_t MaxAxes = 15;                       // The maximum number of movement axes in the machine, usually just X, Y and Z, <= DRIVES
constexpr size_t MaxDriversPerAxis = 4;              // The maximum number of stepper drivers assigned to one axis

constexpr size_t MaxExtruders = 14;                  // The maximum number of extruders

constexpr size_t MaxAxesPlusExtruders = 15;

constexpr size_t MaxHeatersPerTool = 4;
constexpr size_t MaxExtrudersPerTool = 8;

constexpr unsigned int MaxTriggers = 16;            // Must be <= 32 because we store a bitmap of pending triggers in a uint32_t
constexpr size_t MaxLedPorts = 32;                  // Max number of LED ports
#endif

constexpr float MaxTmc5160Current = 6300.0;			// The maximum current we allow the TMC5160/5161 drivers to be set to

#if SUPPORT_CAN_EXPANSION
constexpr size_t MaxCanDrivers = 20;
constexpr size_t MaxCanBoards = 20;
constexpr unsigned int CanDeviceNumber = 0;			// we use FDCAN1 (which is device 0)
#endif

//Steppers
NamedEnum(DriverType, uint8_t,
	unknown,
    none,
    stepdir,
	tmcuartauto,
	tmc2208,
	tmc2209,
    tmc2660,
	tmcspiauto,
	tmc5160,
	tmc2240
);

// HAS_SMART_DRIVERS is defined in Pins.h, we duplicate it for the board files to use
//#define HAS_SMART_DRIVERS		(SUPPORT_TMC2660 || SUPPORT_TMC22xx || SUPPORT_TMC51xx)
extern Pin ENABLE_PINS[NumDirectDrivers];
extern Pin STEP_PINS[NumDirectDrivers];
extern Pin DIRECTION_PINS[NumDirectDrivers];
//#if HAS_SMART_DRIVERS
#if (SUPPORT_TMC2660 || SUPPORT_TMC22xx || SUPPORT_TMC51xx)
extern DriverType TMC_DRIVER_TYPE[NumDirectDrivers];
extern Pin TMC_PINS[NumDirectDrivers];
extern Pin DriverDiagPins[NumDirectDrivers];
extern size_t totalSmartDrivers;
extern size_t num5160SmartDrivers;
extern SSPChannel SmartDriversSpiChannel;
extern uint32_t SmartDriversSpiCsDelay;
#endif
#if SUPPORT_TMC22xx
    constexpr uint32_t DriversBaudRate = 50000;
#endif

extern uint32_t STEP_DRIVER_MASK; // Mask for parallel write to all steppers on port 2 (calculated in after loading board.txt)
extern bool hasStepPinsOnDifferentPorts;
extern bool hasDriverCurrentControl;
extern float digipotFactor;

constexpr uint32_t DefaultStandstillCurrentPercent = 100;

// Initial pin states
constexpr size_t MaxInitialPins = 8;
extern Pin PinsSetHigh[MaxInitialPins];
extern Pin PinsSetLow[MaxInitialPins];

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
extern float DefaultThermistorSeriesR;

constexpr size_t MaxSpiTempSensors = MaxInitialPins;
extern Pin SpiTempSensorCsPins[MaxSpiTempSensors];  // Digital pins the 31855s have their select lines tied to
extern SSPChannel TempSensorSSPChannel;

#if HAS_SBC_INTERFACE
    extern Pin SbcTfrReadyPinConfig;
    extern Pin SbcCsPinConfig;
    extern Pin SbcTfrReadyPin;
    extern Pin SbcCsPin;
    extern SSPChannel SbcSpiChannel;
    extern bool SbcLoadConfig;
    extern bool SbcMode;
#endif

#if SUPPORT_SPICAN
    extern Pin CanCsPin;
    extern SSPChannel CanSpiChannel;
    extern uint32_t CanSpiFrequency;
#endif
#if STM32H7
    extern Pin CanReadPin;
    extern Pin CanWritePin;
#endif

// Power control
extern Pin ATX_POWER_PIN;// Digital pin number that controls the ATX power on/off
extern bool ATX_POWER_INVERTED;
extern bool ATX_INITIAL_POWER_ON;
extern bool ATX_POWER_STATE;

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
extern Pin LcdA0Pin;
extern Pin LcdBeepPin;
extern Pin EncoderPinA;
extern Pin EncoderPinB;
extern Pin EncoderPinSw;
extern Pin PanelButtonPin;

extern Pin DiagPin;
extern bool DiagOnPolarity;
extern Pin ActLedPin;
extern bool ActOnPolarity;

constexpr size_t NumSPIPins = 3;
extern Pin SPIPins[NumSPIDevices][NumSPIPins]; //GPIO pins for softwareSPI (used with SharedSPI)

#define SERIAL_AUX_DEVICE   UART_Slot0
#define serialWiFi  UART_Slot1
//#define SERIAL_AUX2_DEVICE  UART_Slot2

constexpr size_t NumSerialChannels = 2;				// The number of serial IO channels (USB + AUX)
constexpr size_t NumberSerialPins = 2;
extern Pin AuxSerialRxTxPins[NumberSerialPins];

#if defined(SERIAL_AUX2_DEVICE)
    extern Pin Aux2SerialRxTxPins[NumberSerialPins];
#endif

#define SERIAL_MAIN_DEVICE  serialUSB  //USB


#if HAS_WIFI_NETWORKING
    NamedEnum(NetworkModuleType, uint8_t,
        espauto,
        none,
        esp8266,
        esp32,
        esp32eth
    );
    extern NetworkModuleType NetworkModule;
    extern const char* ModuleFiles[];
    extern Pin EspDataReadyPin;
    extern Pin SamTfrReadyPin;
    extern Pin EspResetPin;
    extern Pin EspEnablePin;
    extern Pin SamCsPin;
    extern Pin APIN_SerialWiFi_TXD;
    extern Pin APIN_SerialWiFi_RXD;
    extern Pin WifiSerialRxTxPins[NumberSerialPins];
    extern SSPChannel WiFiSpiChannel;
    extern uint32_t WiFiClockReg;

    extern Pin APIN_ESP_SPI_MOSI;
    extern Pin APIN_ESP_SPI_MISO;
    extern Pin APIN_ESP_SPI_SCK;
#endif

#if SUPPORT_LED_STRIPS
extern Pin NeopixelOutPin;
#endif

#if HAS_VOLTAGE_MONITOR
extern Pin PowerMonitorVinDetectPin;
constexpr float PowerMonitorVoltageRange = 11.0 * 3.3;						// We use an 11:1 voltage divider
extern uint32_t VInDummyReading;
#endif
extern Pin StepperPowerEnablePin;

#if SUPPORT_ACCELEROMETERS
extern SSPChannel AccelerometerSpiChannel;
#endif

//Timer 5 is used for Step Generation
#define STEP_TC             (TIM5)
#define STEP_TC_IRQN        TIM5_IRQn
#define STEP_TC_HANDLER     TIM5_IRQHandler

bool LookupPinName(const char *pn, LogicalPin& lpin, bool& hardwareInverted) noexcept;
const char *GetPinNames(LogicalPin lp) noexcept;

constexpr size_t MaxBoardNameLength = 32;
extern char BoardName[MaxBoardNameLength];
extern char BoardLongName[MaxBoardNameLength];
extern char iapFirmwareFile[MaxBoardNameLength*2];
extern size_t totalSmartDrivers;
extern size_t num5160SmartDrivers;

// Enum to represent allowed types of pin access
// We don't have a separate bit for servo, because Duet PWM-capable ports can be used for servos if they are on the Duet main board
enum class PinCapability: uint8_t
{
	// Individual capabilities
	none = 0u,
	read = 1u,				// digital read
	ain = 2u,				// analog read
	write = 4u,				// digital write
	pwm = 8u,				// PWM write
	npDma = 16u,			// Neopixel output using DMA e.g. using SPI MOSI

	// Combinations
	ainr = 1u|2u,
	rw = 1u|4u,
	wpwm = 4u|8u,
	rwpwm = 1u|4u|8u,
	ainrw = 1u|2u|4u,
	ainrwpwm = 1u|2u|4u|8u,
	npDmaW = 4u | 16u,
    all = 1u|2u|4u|8u|16u
};

constexpr inline PinCapability operator|(PinCapability a, PinCapability b) noexcept
{
	return (PinCapability)((uint8_t)a | (uint8_t)b);
}

constexpr inline PinCapability operator&(PinCapability a, PinCapability b) noexcept
{
	return (PinCapability)((uint8_t)a & (uint8_t)b);
}

//This needs to be const as its used in other places to create arrays
constexpr unsigned int NumNamedPins = P_END;

extern "C" void debugPrintf(const char* fmt, ...) noexcept __attribute__ ((format (printf, 1, 2)));

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
                    if(STEP_PINS[pos] != NoPin) digitalWrite(STEP_PINS[pos], 1); //set high
                }
                driverMap = driverMap >> 1;
                pos++;
            }
        }
        else
        {
            //pins all on port 2, parallel write
            //LPC_GPIO2->SET = driverMap;
            debugPrintf("All pins on same port not supported\n");
        }
    }
    
    // Set all step pins low
    // This needs to be as fast as possible, so we do a parallel write to the port(s).
    // We rely on only those port bits that are step pins being set in the STEP_DRIVER_MASK variable
    static inline void StepDriversLow(uint32_t driverMap) noexcept
    {
        if(hasStepPinsOnDifferentPorts == true )
        {
            //Using driver pos in bitmap to match position in STEP_PINS
            uint8_t pos=0;
            while (driverMap!=0 && pos < NumDirectDrivers)
            {
                if(driverMap & 0x01)
                {
                    if(STEP_PINS[pos] != NoPin) digitalWrite(STEP_PINS[pos], 0); //set low
                }
                driverMap = driverMap >> 1;
                pos++;
            }
        }
        else
        {
            //pins all on port 2, parallel write
            //LPC_GPIO2->CLR = STEP_DRIVER_MASK; //clear only pins that are 1 in the mask
            debugPrintf("All pins on same port not supported\n");
        }
    }
}
extern bool IsSTM32Firmware(const char *firmwareName, const uint32_t len) noexcept;
#endif
