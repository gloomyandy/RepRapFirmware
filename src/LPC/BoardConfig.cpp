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

#include "BoardConfig.h"
#include "RepRapFirmware.h"
#include "GCodes/GCodeResult.h"
#include "sd_mmc.h"
#include "SharedSPI.h"

#include "Platform.h"

#include "SoftwarePWM.h"

//Single entry for Board name
static const boardConfigEntry_t boardEntryConfig[]=
{
    {"lpc.board", &lpcBoardName, nullptr, cvStringType},
};

//All other board configs
static const boardConfigEntry_t boardConfigs[]=
{
    {"leds.diagnostic", &DiagPin, nullptr, cvPinType},
    {"lpc.internalSDCard.spiFrequencyHz", &InternalSDCardFrequency, nullptr, cvUint32Type},

    {"stepper.enablePins", ENABLE_PINS, &MaxTotalDrivers, cvPinType},
    {"stepper.stepPins", STEP_PINS, &MaxTotalDrivers, cvPinType},
    {"stepper.directionPins", DIRECTION_PINS, &MaxTotalDrivers, cvPinType},
    {"stepper.digipotFactor", &digipotFactor, nullptr, cvFloatType},

    {"heat.tempSensePins", TEMP_SENSE_PINS, &NumThermistorInputs, cvPinType},
    {"heat.spiTempSensorCSPins", SpiTempSensorCsPins, &MaxSpiTempSensors, cvPinType},
    
    {"atxPowerPin", &ATX_POWER_PIN, nullptr, cvPinType},
    {"atxPowerPinInverted", &ATX_POWER_INVERTED, nullptr, cvBoolType},
    
    {"externalSDCard.csPin", &SdSpiCSPins[1], nullptr, cvPinType},
    {"externalSDCard.cardDetectPin", &SdCardDetectPins[1], nullptr, cvPinType},
    {"lpc.externalSDCard.spiFrequencyHz", &ExternalSDCardFrequency, nullptr, cvUint32Type},
    {"lpc.externalSDCard.spiChannel", &ExternalSDCardSSPChannel, nullptr, cvUint8Type},

#if SUPPORT_12864_LCD
    {"lcd.lcdCSPin", &LcdCSPin, nullptr, cvPinType},
    {"lcd.lcdBeepPin", &LcdBeepPin, nullptr, cvPinType},
    {"lcd.encoderPinA", &EncoderPinA, nullptr, cvPinType},
    {"lcd.encoderPinB", &EncoderPinB, nullptr, cvPinType},
    {"lcd.encoderPinSw", &EncoderPinSw, nullptr, cvPinType},
    {"lcd.lcdDCPin", &LcdDCPin, nullptr, cvPinType},
    {"lcd.panelButtonPin", &PanelButtonPin, nullptr, cvPinType},
    {"lcd.spiChannel", &LcdSpiChannel, nullptr, cvUint8Type},
#endif
    
    {"lpc.softwareSPI.pins", SoftwareSPIPins, &NumSoftwareSPIPins, cvPinType}, //SCK, MISO, MOSI
    
#if defined(ESP8266WIFI)
    {"8266wifi.EspDataReadyPin", &EspDataReadyPin, nullptr, cvPinType},
    {"8266wifi.LpcTfrReadyPin", &SamTfrReadyPin, nullptr, cvPinType},
    {"8266wifi.EspResetPin", &EspResetPin, nullptr, cvPinType},
#endif
    
    {"lpc.adcEnablePreFilter", &ADCEnablePreFilter, nullptr, cvBoolType},
    {"lpc.adcPreFilterNumberSamples", &ADCPreFilterNumberSamples, nullptr, cvUint8Type},
    {"lpc.adcPreFilterSampleRate", &ADCPreFilterSampleRate, nullptr, cvUint32Type},

    
#if LPC_TMC_SOFT_UART
    {"stepper.TmcUartPins", TMC_UART_PINS, &MaxTotalDrivers, cvPinType},
#endif
};


static inline bool isSpaceOrTab(char c) noexcept
{
    return (c == ' ' || c == '\t');
}
    
BoardConfig::BoardConfig() noexcept
{
    
}


void BoardConfig::Init() noexcept
{

    GCodeResult rslt;
    String<100> reply;
    FileStore *configFile = nullptr;
    
    NVIC_SetPriority(DMA_IRQn, NvicPrioritySpi);

    //Mount the Internal SDCard
    do
    {
        rslt = MassStorage::Mount(0, reply.GetRef(), false);
    }
    while (rslt == GCodeResult::notFinished);
    
    if (rslt == GCodeResult::ok)
    {
        //Open File
        if (reprap.GetPlatform().SysFileExists("board.txt"))
        {
            configFile = reprap.GetPlatform().OpenFile(DEFAULT_SYS_DIR, "board.txt", OpenMode::read);
        }
        else
        {
            delay(3000);        // Wait a few seconds so users have a chance to see this
            reprap.GetPlatform().MessageF(UsbMessage, "Unable to read board configuration: %sboard.txt...\n",DEFAULT_SYS_DIR );
            return;
        }
    }
    else
    {
        // failed to mount card
        delay(3000);        // Wait a few seconds so users have a chance to see this
        reprap.GetPlatform().MessageF(UsbMessage, "%s\n", reply.c_str());
        return;
    }
    
    if(configFile != nullptr)
    {
        
        reprap.GetPlatform().MessageF(UsbMessage, "Loading config from %sboard.txt...\n", DEFAULT_SYS_DIR );

        
        //First find the board entry to load the correct PinTable for looking up Pin by name
        BoardConfig::GetConfigKeys(configFile, boardEntryConfig, (size_t) 1);
        if(!SetBoard(lpcBoardName)) // load the Correct PinTable for the defined Board (RRF3)
        {
            //Failed to find string in known boards array
            reprap.GetPlatform().MessageF(UsbMessage, "Unknown board: %s\n", lpcBoardName );
            SafeStrncpy(lpcBoardName, "generic", 8); //replace the string in lpcBoardName to "generic"
        }

        //Load all other config settings now that PinTable is loaded.
        configFile->Seek(0); //go back to beginning of config file
        BoardConfig::GetConfigKeys(configFile, boardConfigs, (size_t) ARRAY_SIZE(boardConfigs));
        configFile->Close();
        
        
        //Calculate STEP_DRIVER_MASK (used for parallel writes)
        STEP_DRIVER_MASK = 0;
        for(size_t i=0; i<MaxTotalDrivers; i++)
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
                    hasStepPinsOnDifferentPorts = true;
                    // will use a basic mapping for stepping instead of parallel port writes.....
                    reprap.GetPlatform().MessageF(UsbMessage, "Step pins are on different ports. Pins will not be written in parallel.\n" );
                }
            }
        }
        
        //Does board have build in current control via digipots?
        if(digipotFactor > 1)
        {
            hasDriverCurrentControl = true;
        }
        
        //Setup the Software SPI Pins
        sspi_setPinsForChannel(SWSPI0, SoftwareSPIPins[0], SoftwareSPIPins[1], SoftwareSPIPins[2]);

        //Internal SDCard SPI Frequency
        sd_mmc_reinit_slot(0, SdSpiCSPins[0], InternalSDCardFrequency);
        
        //Configure the External SDCard
        if(SdSpiCSPins[1] != NoPin)
        {
            setPullup(SdCardDetectPins[1], true);
            //set the SSP Channel for External SDCard
            if(ExternalSDCardSSPChannel == SSP0 || ExternalSDCardSSPChannel == SSP1 || ExternalSDCardSSPChannel == SWSPI0)
            {
                sd_mmc_setSSPChannel(1, ExternalSDCardSSPChannel); //must be called before reinit
            }
            //set the CSPin and the frequency for the External SDCard
            sd_mmc_reinit_slot(1, SdSpiCSPins[1], ExternalSDCardFrequency);
        }
        
        #if defined(ESP8266WIFI)
            if(SamCsPin != NoPin) pinMode(SamCsPin, OUTPUT_LOW);
            if(EspResetPin != NoPin) pinMode(EspResetPin, OUTPUT_LOW);
        #endif

        //Init pins for LCD
        //make sure to init ButtonPin as input incase user presses button
        if(PanelButtonPin != NoPin) pinMode(PanelButtonPin, INPUT); //unused
        if(LcdDCPin != NoPin) pinMode(LcdDCPin, OUTPUT_HIGH); //unused
        if(LcdBeepPin != NoPin) pinMode(LcdBeepPin, OUTPUT_LOW);
        // Set the 12864 display CS pin low to prevent it from receiving garbage due to other SPI traffic
        if(LcdCSPin != NoPin) pinMode(LcdCSPin, OUTPUT_LOW);

        //Init Diagnostcs Pin
        pinMode(DiagPin, OUTPUT_LOW);
        
        //Configure ADC pre filter
        ConfigureADCPreFilter(ADCEnablePreFilter, ADCPreFilterNumberSamples, ADCPreFilterSampleRate);
    }
}


//Convert a pin string into a RRF Pin
Pin BoardConfig::StringToPin(const char *strvalue) noexcept
{
    //check size.. should be 3chars or 4 chars i.e. 0.1, 2.25, 2nd char should always be .
    uint8_t len = strlen(strvalue);
    if((len == 3 || len == 4) && strvalue[1] == '.' )
    {
        const char *ptr = NULL;
        
        uint8_t port = SafeStrtol(strvalue, &ptr, 10);
        if(ptr > strvalue && port <= 4)
        {
            uint8_t pin = SafeStrtol(strvalue+2, &ptr, 10);
            
            if(ptr > strvalue+2 && pin < 32)
            {
                //Convert the Port and Pin to match the arrays in CoreLPC
                Pin lpcpin = (Pin) ( (port << 5) | pin);
                return lpcpin;
            }
        }
    }
    //debugPrintf("Invalid pin %s, defaulting to NoPin\n", strvalue);
    
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
        return PinTable[lp].pin; //lookup succeeded, return the Pin
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
                    reprap.GetPlatform().MessageF(mtype, "NoPin ");
                }
                else
                {
                    reprap.GetPlatform().MessageF(mtype, "%d.%d ", (pin >> 5), (pin & 0x1F) );
                }
            }
            break;
        case cvBoolType:
            reprap.GetPlatform().MessageF(mtype, "%s ", (*(bool *)(variable) == true)?"true":"false" );
            break;
        case cvFloatType:
            reprap.GetPlatform().MessageF(mtype, "%.2f ",  (double) *(float *)(variable) );
            break;
        case cvUint8Type:
            reprap.GetPlatform().MessageF(mtype, "%u ",  *(uint8_t *)(variable) );
            break;
        case cvUint16Type:
            reprap.GetPlatform().MessageF(mtype, "%d ",  *(uint16_t *)(variable) );
            break;
        case cvUint32Type:
            reprap.GetPlatform().MessageF(mtype, "%lu ",  *(uint32_t *)(variable) );
            break;
        case cvStringType:
            reprap.GetPlatform().MessageF(mtype, "%s ",  (char *)(variable) );
            break;
        default:{
            
        }
    }
}


//Information printed by M122 P200
void BoardConfig::Diagnostics(MessageType mtype) noexcept
{
    reprap.GetPlatform().MessageF(mtype, "== Configurable Board.txt Settings ==\n");

    //Print the board name
    boardConfigEntry_t board = boardEntryConfig[0];
    reprap.GetPlatform().MessageF(mtype, "%s = ", board.key );
    BoardConfig::PrintValue(mtype, board.type, board.variable);
    reprap.GetPlatform().MessageF(mtype, "\n");

    
    //Print rest of board configurations
    const size_t numConfigs = ARRAY_SIZE(boardConfigs);
    for(size_t i=0; i<numConfigs; i++)
    {
        boardConfigEntry_t next = boardConfigs[i];

        reprap.GetPlatform().MessageF(mtype, "%s = ", next.key );
        if(next.maxArrayEntries != nullptr)
        {
            reprap.GetPlatform().MessageF(mtype, "{ ");
            for(size_t p=0; p<*(next.maxArrayEntries); p++)
            {
                //TODO:: handle other values
                if(next.type == cvPinType){
                    BoardConfig::PrintValue(mtype, next.type, (void *)&((Pin *)(next.variable))[p]);
                }
            }
            reprap.GetPlatform().MessageF(mtype, "}\n");
        }
        else
        {
            BoardConfig::PrintValue(mtype, next.type, next.variable);
            reprap.GetPlatform().MessageF(mtype, "\n");

        }
    }

    reprap.GetPlatform().MessageF(mtype, "\n== Software PWM ==\n");
    for(uint8_t i=0; i<MaxNumberSoftwarePWMPins; i++)
    {
        SoftwarePWM *next = softwarePWMEntries[i];
        if(next != nullptr)
        {
            const Pin pin = next->GetPin();
            reprap.GetPlatform().MessageF(mtype, "Pin %d.%d @ %dHz\n", (pin >> 5), (pin & 0x1f), next->GetFrequency() );
        }
    };
    
    
    //Print Servo PWM Timer or HW PWM assignments
    reprap.GetPlatform().MessageF(mtype, "\n== Servo PWM ==\n");
    reprap.GetPlatform().MessageF(mtype, "Hardware PWM = %dHz ", HardwarePWMFrequency );
    PrintPinArray(mtype, UsedHardwarePWMChannel, NumPwmChannels);
    reprap.GetPlatform().MessageF(mtype, "Timer2 PWM = 50Hz ");
    PrintPinArray(mtype, Timer2PWMPins, MaxTimerEntries);
    
    
}

void BoardConfig::PrintPinArray(MessageType mtype, Pin arr[], uint16_t numEntries) noexcept
{
    reprap.GetPlatform().MessageF(mtype, "[ ");
    for(uint8_t i=0; i<numEntries; i++)
    {
        if(arr[i] != NoPin)
        {
            reprap.GetPlatform().MessageF(mtype, "%d.%d ", (arr[i]>>5), (arr[i] & 0x1f));
        }
        else
        {
            reprap.GetPlatform().MessageF(mtype, "NoPin ");
        }
        
    }
    reprap.GetPlatform().MessageF(mtype, "]\n");
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
                uint8_t val = SafeStrtoul(valuePtr, &ptr, 10);
                if(val < 0) val = 0;
                if(val > 0xFF) val = 0xFF;
                
                *(uint8_t *)(variable) = val;
            }
            break;
        case cvUint16Type:
            {
                const char *ptr = nullptr;
                uint16_t val = SafeStrtoul(valuePtr, &ptr, 10);
                if(val < 0) val = 0;
                if(val > 0xFFFF) val = 0xFFFF;
                
                *(uint16_t *)(variable) = val;
                    
            }
            break;
        case cvUint32Type:
            {
                const char *ptr = nullptr;
                *(uint32_t *)(variable) = SafeStrtoul(valuePtr, &ptr, 10);
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
            
        default:
            debugPrintf("Unhandled ValueType\n");
    }
}



bool BoardConfig::GetConfigKeys(FileStore *configFile, const boardConfigEntry_t *boardConfigEntryArray, const size_t numConfigs) noexcept
{

    size_t maxLineLength = 120;
    char line[maxLineLength];

    FilePosition fileSize = configFile->Length();
    size_t len = configFile->ReadLine(line, maxLineLength);
    while(len >= 0 && configFile->Position() != fileSize )
    {
        size_t pos = 0;
        while(pos < len && isSpaceOrTab(line[pos] && line[pos] != 0) == true) pos++; //eat leading whitespace

        if(pos < len){

            //check for comments
            if(line[pos] == '/' || line[pos] == '#')
            {
                //Comment - Skipping
            }
            else
            {
                const char* key = line + pos;
                while(pos < len && !isSpaceOrTab(line[pos]) && line[pos] != '=' && line[pos] != 0) pos++;
                line[pos] = 0;// null terminate the string (now contains the "key")

                pos++;

                //eat whitespace and = if needed
                while(pos < maxLineLength && line[pos] != 0 && (isSpaceOrTab(line[pos]) == true || line[pos] == '=') ) pos++; //skip spaces and =

                //debugPrintf("Key: %s", key);

                if(pos < len && line[pos] == '{')
                {
                    // { indicates the start of an array
                    
                    pos++; //skip the {

                    //Array of Values:
                    //TODO:: only Pin arrays are currently implemented
                    
                    //const size_t numConfigs = ARRAY_SIZE(boardConfigs);
                    for(size_t i=0; i<numConfigs; i++)
                    {
                        boardConfigEntry_t next = boardConfigEntryArray[i];
                        //Currently only handles Arrays of Pins
                        
                        
                        if(next.maxArrayEntries != nullptr /*&& next.type == cvPinType*/ && StringEqualsIgnoreCase(key, next.key))
                        {
                            //matched an entry in boardConfigEntryArray

                            //create a temp array to read into. Only copy the array entries into the final destination when we know the array is properly defined
                            const size_t maxArraySize = *next.maxArrayEntries;
                            
                            //Pin Array Type
                            Pin readArray[maxArraySize];

                            //eat whitespace
                            while(pos < maxLineLength && line[pos] != 0 && isSpaceOrTab(line[pos]) == true ) pos++;

                            bool searching = true;

                            size_t arrIdx = 0;

                            //search for values in Array
                            while( searching )
                            {
                                if(pos < maxLineLength)
                                {

                                    while(pos < maxLineLength && (isSpaceOrTab(line[pos]) == true)) pos++; // eat whitespace

                                    if(pos == maxLineLength)
                                    {
                                        debugPrintf("Got to end of line before end of array, line must be longer than maxLineLength");
                                        searching = false;
                                        break;
                                    }

                                    bool closedSuccessfully = false;
                                    //check brace isnt closed
                                    if(pos < maxLineLength && line[pos] == '}')
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

                                        //read until end condition - space,comma,}  or null / # ;
                                        while(pos < maxLineLength && line[pos] != 0 && !isSpaceOrTab(line[pos]) && line[pos] != ',' && line[pos] != '}' && line[pos] != '/' && line[pos] != '#' && line[pos] != ';')
                                        {
                                            pos++;
                                        }

                                        //see if we ended due to comment, ;, or null
                                        if(pos == maxLineLength || line[pos] == 0 || line[pos] == '/' || line[pos] == '#' || line[pos]==';')
                                        {
                                            debugPrintf("Error: Array ended without Closing Brace?\n");
                                            searching = false;
                                            break;
                                        }

                                        //check if there is a closing brace after value without any whitespace, before it gets overwritten with a null
                                        if(line[pos] == '}')
                                        {
                                            closedSuccessfully = true;
                                        }

                                        line[pos] = 0; // null terminate the string

                                        //debugPrintf("%s ", valuePtr);

                                        //Put into the Temp Array
                                        if(arrIdx >= 0 && arrIdx<maxArraySize)
                                        {
                                            readArray[arrIdx] = LookupPin(valuePtr);
                                            
                                            //TODO:: HANDLE OTHER VALUE TYPES??
                                            

                                        }
                                    }

                                    if(closedSuccessfully == true)
                                    {
                                        //Array Closed - Finished Searching
                                        if(arrIdx >= 0 && arrIdx < maxArraySize) //arrIndx will be -1 if closed before reading any values
                                        {
                                            //All values read successfully, copy temp array into Final destination
                                            //dest array may be larger, dont overrite the default values
                                            for(size_t i=0; i<(arrIdx+1); i++ )
                                            {
                                                ((Pin *)(next.variable))[i] = readArray[i];
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
                    }//end for

                }
                else
                {
                    //single value
                    if(pos < maxLineLength && line[pos] != 0)
                    {
                        //should be at first char of value now
                        char *valuePtr = line+pos;

                        //read until end condition - space, ;, comment, null,etc
                        while(pos < maxLineLength && line[pos] != 0 && !isSpaceOrTab(line[pos]) && line[pos] != ';' && line[pos] != '/') pos++;

                        //overrite the end condition with null....
                        line[pos] = 0; // null terminate the string (the "value")

                        //Find the entry in boardConfigEntryArray using the key
                        //const size_t numConfigs = ARRAY_SIZE(boardConfigs);
                        for(size_t i=0; i<numConfigs; i++)
                        {
                            boardConfigEntry_t next = boardConfigEntryArray[i];
                            //Single Value config entries have nullptr for maxArrayEntries
                            if(next.maxArrayEntries == nullptr && StringEqualsIgnoreCase(key, next.key))
                            {
                                //match
                                BoardConfig::SetValueFromString(next.type, next.variable, valuePtr);
                            }
                        }
                    }
                }
            }
        }
        else
        {
            //Empty Line - Nothing to do here
        }

        len = configFile->ReadLine(line, maxLineLength); //attempt to read the next line
    }
    return false;
}
