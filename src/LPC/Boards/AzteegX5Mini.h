#ifndef AZTEEGX5MINI_H
#define AZTEEGX5MINI_H

#include "../Pins_LPC.h"

// List of assignable pins and their mapping from names to MPU ports. This is indexed by logical pin number.
// The names must match user input that has been concerted to lowercase and had _ and - characters stripped out.
// Aliases are separate by the , character.
// If a pin name is prefixed by ! then this means the pin is hardware inverted. The same pin may have names for both the inverted and non-inverted cases,
// for example the inverted heater pins on the expansion connector are available as non-inverted servo pins on a DFueX.

//http://files.panucatt.com/datasheets/x5mini_wiring_v1_1.pdf
constexpr PinEntry PinTable_AzteegX5MiniV1_1[] =
{
    //LEDs
    {P1_18, PinCapability::wpwm, "led1"},
    {P1_19, PinCapability::wpwm, "led2"},
    {P1_20, PinCapability::wpwm, "led3"},
    {P1_21, PinCapability::wpwm, "led4"},
    {P4_28, PinCapability::wpwm, "play"},
    
    //Thermistors
    {P0_23, PinCapability::ainrw, "bendtemp,th0"},
    {P0_24, PinCapability::ainrw, "eotemp,th1"},

    //Endstops
    {P1_24, PinCapability::read, "xstop"},
    {P1_26, PinCapability::read, "ystop"},
    {P1_28, PinCapability::read, "zstop"},
    {P1_29, PinCapability::read, "e0stop,estop"},

    //Heaters and Fans
    {P2_5, PinCapability::wpwm, "bed,hend" },
    {P2_7, PinCapability::wpwm, "e0heat,hbed" },
    {P2_4, PinCapability::wpwm, "fan0,fan" },
        
    //EXP1 Pins
    {P1_30, PinCapability::rwpwm, "P1.30"},
    {P1_22, PinCapability::rwpwm, "P1.22"},
    {P0_26, PinCapability::rwpwm, "P0.26"},
    {P0_25, PinCapability::rwpwm, "P0.25"},
    {P0_27, PinCapability::rwpwm, "sda,P0.27"},
    {P4_29, PinCapability::rwpwm, "P4.29"},
    {P0_28, PinCapability::rwpwm, "scl,P0.28"},
    {P2_8,  PinCapability::rwpwm, "P2.8"},
    //3.3v
    //GND
    
    //EXP2 Pins
    {P1_31, PinCapability::rwpwm, "P1.31"},
    {P3_26, PinCapability::rwpwm, "P3.26"},
    {P2_11, PinCapability::rwpwm, "P2.11"},
    {P3_25, PinCapability::rwpwm, "P3.25"},
    {P1_23, PinCapability::rwpwm, "P1.23"},
    {P0_17, PinCapability::rwpwm, "P0.17"},
    {P0_16, PinCapability::rwpwm, "P0.16"},
    {P2_6,  PinCapability::rwpwm, "P2.6"},
    {P0_15, PinCapability::rwpwm, "P0.15"},
    {P0_18, PinCapability::rwpwm, "P0.18"},
    //GND
    //5V
};

constexpr BoardDefaults azteegX5Mini1_1Defaults = {
    {P0_10, P0_19, P0_21, P0_4, NoPin},   //enablePins
    {P2_1,  P2_2,  P2_3,  P2_0, NoPin},   //stepPins
    {P0_11, P0_20, P0_22, P0_5, NoPin},   //dirPins
#if LPC_TMC_SOFT_UART
    {NoPin, NoPin, NoPin, NoPin, NoPin},  //uartPins
    0,                                      // Smart drivers
#endif
    106.0,                         //digiPot Factor
};

constexpr PinEntry PinTable_AzteegX5MiniV2[] =
{
    //LEDs
    {P1_18, PinCapability::wpwm, "led1"},
    {P1_19, PinCapability::wpwm, "led2"},
    {P1_20, PinCapability::wpwm, "led3"},
    {P1_21, PinCapability::wpwm, "led4"},
    {P4_28, PinCapability::wpwm, "play"},
    
    //Thermistors
    {P0_23, PinCapability::ainrw, "bendtemp,th0"},
    {P0_24, PinCapability::ainrw, "eotemp,th1"},

    //Endstops
    {P1_24, PinCapability::read, "xstop"},
    {P1_26, PinCapability::read, "ystop"},
    {P1_28, PinCapability::read, "zstop"},
    {P1_29, PinCapability::read, "e0stop,estop"},

    //Heaters and Fans
    {P2_7, PinCapability::wpwm, "bed,hend" },
    {P2_5, PinCapability::wpwm, "e0heat,hbed" },
    {P1_25, PinCapability::wpwm, "fan0" },
    {P2_4, PinCapability::wpwm, "fan1" },
        
    //EXP1 Pins
    {P1_30, PinCapability::rwpwm, "P1.30"},
    {P1_22, PinCapability::rwpwm, "P1.22"},
    {P0_26, PinCapability::rwpwm, "P0.26"},
    {P0_25, PinCapability::rwpwm, "P0.25"},
    {P0_27, PinCapability::rwpwm, "sda,P0.27"},
    {P4_29, PinCapability::rwpwm, "P4.29"},
    {P0_28, PinCapability::rwpwm, "scl,P0.28"},
    {P2_8,  PinCapability::rwpwm, "P2.8"},
    //3.3v
    //GND
    
    //EXP2 Pins
    {P1_31, PinCapability::rwpwm, "P1.31"},
    {P3_26, PinCapability::rwpwm, "P3.26"},
    {P2_11, PinCapability::rwpwm, "P2.11"},
    {P3_25, PinCapability::rwpwm, "P3.25"},
    {P1_23, PinCapability::rwpwm, "P1.23"},
    {P0_17, PinCapability::rwpwm, "P0.17"},
    {P0_16, PinCapability::rwpwm, "P0.16"},
    {P2_6,  PinCapability::rwpwm, "P2.6"},
    {P0_15, PinCapability::rwpwm, "P0.15"},
    {P0_18, PinCapability::rwpwm, "P0.18"},
    //GND
    //5V
	
	//J7 Pins
    {P1_16, PinCapability::rwpwm, "P1.16"},
	{P1_17, PinCapability::rwpwm, "P1.17"},
	{P1_15, PinCapability::rwpwm, "P1.15"},
	{P0_14, PinCapability::rwpwm, "P0.14"},
	{P1_9, PinCapability::rwpwm, "P1.9"},
	{P1_10, PinCapability::rwpwm, "P1.10"},
	{P1_8, PinCapability::rwpwm, "P1.8"},
	{P1_4, PinCapability::rwpwm, "P1.4"},
	{P1_0, PinCapability::rwpwm, "P1.0"},
	{P1_1, PinCapability::rwpwm, "P1.1"},
    //Misc
    //{P4_12,  PinCapability::rwpwm, "Halt","P4.12"},


};

constexpr BoardDefaults azteegX5Mini2Defaults = {
    {P0_10, P0_19, P0_21, P0_4, NoPin},   //enablePins
    {P2_1,  P2_2,  P2_3,  P2_0, NoPin},   //stepPins
    {P0_11, P0_20, P0_22, P0_5, NoPin},   //dirPins
#if LPC_TMC_SOFT_UART
    {NoPin, NoPin, NoPin, NoPin, NoPin},  //uartPins
    0,                                      // Smart drivers
#endif
    106.0,                         //digiPot Factor
};

//http://files.panucatt.com/datasheets/x5mini_wiring_v3.pdf
constexpr PinEntry PinTable_AzteegX5MiniV3[] =
{
    //LEDs
    {P1_18, PinCapability::wpwm, "led1"},
    {P1_19, PinCapability::wpwm, "led2"},
    {P1_20, PinCapability::wpwm, "led3"},
    {P1_21, PinCapability::wpwm, "led4"},
    {P4_28, PinCapability::wpwm, "play"},
    
    //Thermistors
    {P0_23, PinCapability::ainrw, "bendtemp,th0"},
    {P0_24, PinCapability::ainrw, "eotemp,th1"},

    //Endstops
    {P1_24, PinCapability::read, "xstop"},
    {P1_26, PinCapability::read, "ystop"},
    {P1_28, PinCapability::read, "zstop"},
    {P1_29, PinCapability::read, "e0stop,estop"},

    //Heaters and Fans
    {P2_7, PinCapability::wpwm, "bed,hend" },
    {P2_5, PinCapability::wpwm, "e0heat,hbed" },
    {P1_25, PinCapability::wpwm, "fan0" },
    {P0_26, PinCapability::wpwm, "fan1" },
        
    //EXP1 Pins
    {P1_30, PinCapability::rwpwm, "P1.30"},
    {P1_22, PinCapability::rwpwm, "P1.22"},
    {P2_4, PinCapability::rwpwm, "P2.4"},
    {P0_25, PinCapability::rwpwm, "P0.25"},
    {P0_27, PinCapability::rwpwm, "sda,P0.27"},
    {P4_29, PinCapability::rwpwm, "P4.29"},
    {P0_28, PinCapability::rwpwm, "scl,P0.28"},
    {P2_8,  PinCapability::rwpwm, "P2.8"},
    //3.3v
    //GND
    
    //EXP2 Pins
    {P1_31, PinCapability::rwpwm, "P1.31"},
    {P3_26, PinCapability::rwpwm, "P3.26"},
    {P2_11, PinCapability::rwpwm, "P2.11"},
    {P3_25, PinCapability::rwpwm, "P3.25"},
    {P1_23, PinCapability::rwpwm, "P1.23"},
    {P0_17, PinCapability::rwpwm, "P0.17"},
    {P0_16, PinCapability::rwpwm, "P0.16"},
    {P2_6,  PinCapability::rwpwm, "P2.6"},
    {P0_15, PinCapability::rwpwm, "P0.15"},
    {P0_18, PinCapability::rwpwm, "P0.18"},
    //GND
    //5V
    
    //Misc
    //{P4_12,  PinCapability::rwpwm, "Halt","P4.12"},


};

constexpr BoardDefaults azteegX5Mini3Defaults = {
    {P0_10, P0_19, P0_21, P0_4, NoPin},   //enablePins
    {P2_1,  P2_2,  P2_3,  P2_0, NoPin},   //stepPins
    {P0_11, P0_20, P0_22, P0_5, NoPin},   //dirPins
#if LPC_TMC_SOFT_UART
    {NoPin, NoPin, NoPin, NoPin, NoPin},  //uartPins
    0,                                      // Smart drivers
#endif
    113.5,                         //digiPot Factor
};
#endif 