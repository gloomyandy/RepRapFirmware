#include "RepRapFirmware.h"

//Configurable variables. Note for most items the default value is set in BoardConfig.cpp


//All I/Os default to input and are floating
Pin PinsSetHigh[MaxInitialPins];
Pin PinsSetLow[MaxInitialPins];

Pin TEMP_SENSE_PINS[NumThermistorInputs];
Pin SpiTempSensorCsPins[MaxSpiTempSensors]; // Used to deselect all devices at boot
SSPChannel TempSensorSSPChannel;
float DefaultThermistorSeriesR;

Pin ATX_POWER_PIN;                          // Pin to use to control external power
bool ATX_POWER_INVERTED;                    // Should the state of this pin be inverted
bool ATX_INITIAL_POWER_ON;                  // Should external power be on/off at startup
bool ATX_POWER_STATE = true;                // We may not have an actual pin so use this to track state

//SDCard pins and settings
Pin SdCardDetectPins[NumSdCards] = {NoPin, NoPin};
Pin SdSpiCSPins[NumSdCards];                // Internal, external. Note:: ("slot" 0 in CORE is configured to be LCP SSP1 to match default RRF behaviour)
uint32_t ExternalSDCardFrequency;           //default to 4MHz
SSPChannel ExternalSDCardSSPChannel;        // Off by default
uint32_t InternalSDCardFrequency;           //default to 25MHz


Pin LcdCSPin;                               //LCD Chip Select
Pin LcdA0Pin;                               //DataControl Pin (A0) if none used set to NoPin
Pin LcdBeepPin;
Pin EncoderPinA;
Pin EncoderPinB;
Pin EncoderPinSw;                           //click
Pin PanelButtonPin;                         //Extra button on Viki and RRD Panels (reset/back etc)
SSPChannel LcdSpiChannel;                   //Off by default

Pin DiagPin;
bool DiagOnPolarity = true;
Pin ActLedPin;
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
DriverType TMC_DRIVER_TYPE[NumDirectDrivers];

size_t totalSmartDrivers;
size_t num5160SmartDrivers;
SSPChannel SmartDriversSpiChannel;
uint32_t SmartDriversSpiCsDelay;
#endif

uint32_t STEP_DRIVER_MASK = 0;                          //SD: mask of the step pins on Port 2 used for writing to step pins in parallel
bool hasStepPinsOnDifferentPorts = false;               //for boards that don't have all step pins on same port
bool hasDriverCurrentControl = false;                   //Supports digipots to set stepper current
float digipotFactor = 0.0;                              //defualt factor for converting current to digipot value


Pin SPIPins[NumSPIDevices][NumSPIPins];                 //GPIO pins for hardware/software SPI (used with SharedSPI)


#if HAS_WIFI_NETWORKING
    NetworkModuleType NetworkModule;
    const char* ModuleFiles[] = {"", "", "WiFiModule_esp8266.bin", "WiFiModule_esp32.bin", "WiFiModule_esp32eth.bin"};
    Pin EspDataReadyPin;
    Pin SamTfrReadyPin;
    Pin EspResetPin;
    Pin EspEnablePin = NoPin;
    Pin SamCsPin;
    Pin APIN_SerialWiFi_TXD = NoPin;
    Pin APIN_SerialWiFi_RXD = NoPin;
    SSPChannel WiFiSpiChannel;
    uint32_t WiFiClockReg;
    Pin APIN_ESP_SPI_MOSI = NoPin;
    Pin APIN_ESP_SPI_MISO = NoPin;
    Pin APIN_ESP_SPI_SCK = NoPin;
    Pin WifiSerialRxTxPins[NumberSerialPins];
#endif
    
//Aux Serial

#if defined(SERIAL_AUX_DEVICE)
    Pin AuxSerialRxTxPins[NumberSerialPins]; //Default to UART0
#endif
#if defined(SERIAL_AUX2_DEVICE)
    Pin Aux2SerialRxTxPins[NumberSerialPins];
#endif

#if HAS_SBC_INTERFACE
    Pin SbcTfrReadyPinConfig;
    Pin SbcCsPinConfig;
    Pin SbcTfrReadyPin;
    Pin SbcCsPin;
    SSPChannel SbcSpiChannel;
    bool SbcLoadConfig;
    bool SbcMode;
#endif

#if SUPPORT_SPICAN
    Pin CanCsPin;
    SSPChannel CanSpiChannel;
    uint32_t CanSpiFrequency;
#endif

#if STM32H7
    Pin CanReadPin;
    Pin CanWritePin;
#endif

#if SUPPORT_LED_STRIPS
Pin NeopixelOutPin;
#endif

#if HAS_VOLTAGE_MONITOR
Pin PowerMonitorVinDetectPin;
uint32_t VInDummyReading;
#endif
Pin StepperPowerEnablePin;

#if SUPPORT_ACCELEROMETERS
SSPChannel AccelerometerSpiChannel;
#endif

// Check to see if the board shortname is for an STM32 based board or not
bool IsSTM32Firmware(const char *boardName, const uint32_t len) noexcept
{
    const char *endBoard = boardName + len;
    return (len >= 5 && (!strncmp("stm", boardName, 3) || !strncmp("_f4, ", endBoard - 3, 3) || !strncmp("_h723", endBoard - 5, 5) || !strncmp("_h743", endBoard - 5, 5)));
}
