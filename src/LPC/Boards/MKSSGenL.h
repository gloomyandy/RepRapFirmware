#ifndef MKSSGenL_H
#define MKSSGenL_H

#include "../Pins_LPC.h"


// List of assignable pins and their mapping from names to MPU ports. This is indexed by logical pin number.
// The names must match user input that has been concerted to lowercase and had _ and - characters stripped out.
// Aliases are separate by the , character.
// If a pin name is prefixed by ! then this means the pin is hardware inverted. The same pin may have names for both the inverted and non-inverted cases,
// for example the inverted heater pins on the expansion connector are available as non-inverted servo pins on a DFueX.

constexpr PinEntry PinTable_MKSSGenL1_0[] =
{
    //Thermistors
    {P0_23, PinCapability::ainrw, "e0temp,th1"},
    {P0_24, PinCapability::ainrw, "bedtemp,TB"},
    {P0_25, PinCapability::ainrw, "e1temp,th2"},

    //Endstops (although they are labeled x- and x+ on the - and + get stripped out in RRF so we will use i.e. xmin/xmax
    {P1_24, PinCapability::rwpwm, "zstopmax,Z+"},
    {P1_25, PinCapability::rwpwm, "zstop,Z-"},
    {P1_26, PinCapability::rwpwm, "ystopmax,Y+"},
    {P1_27, PinCapability::rwpwm, "ystop,Y-"},
    {P1_28, PinCapability::rwpwm, "xstopmax,X+"},
    {P1_29, PinCapability::rwpwm, "xstop,X-"},

    //Heaters and Fans
    {P2_5, PinCapability::rwpwm, "bed"},
    {P2_7, PinCapability::rwpwm, "e0heat,e1"},
    {P2_6, PinCapability::rwpwm, "e1heat,e2"},
    {P2_4, PinCapability::rwpwm, "fan0,fan"},
	
	//servo
	{P1_23,  PinCapability::rwpwm, "servo0,P1.23" },
	{P2_0,  PinCapability::rwpwm, "servo1,P2.0" },
 
    //Spare pins (also as LEDs)
    {P1_21, PinCapability::rwpwm, "led1,P1.21"},
    {P1_20, PinCapability::rwpwm, "led2,P1.20"},
    {P1_19, PinCapability::rwpwm, "led3,P1.19"},
    {P1_18, PinCapability::rwpwm, "led4,P1.18"},
    
    //Exp1
    {P1_31, PinCapability::rwpwm, "P1.31"},
	{P1_30, PinCapability::rwpwm, "P1.30"},
    {P0_18, PinCapability::rwpwm, "P0.18"},
	{P0_16, PinCapability::rwpwm, "P0.16"},
    {P0_15, PinCapability::rwpwm, "P0.15"},    
    {P0_17, PinCapability::rwpwm, "P0.17"},
	{P1_0, PinCapability::rwpwm, "P1.0"},
	{P1_22, PinCapability::rwpwm, "P1.22"},
    
    //Exp2
    {P0_8,  PinCapability::rwpwm, "P0.8"},
	{P0_7,  PinCapability::rwpwm, "P0.7"},
    {P3_25, PinCapability::rwpwm, "P3.25"},
	{P0_28, PinCapability::rwpwm, "P0.28"},
    {P3_26, PinCapability::rwpwm, "P3.26"},
	{P0_9,  PinCapability::rwpwm, "P0.9"},
    {P0_27, PinCapability::rwpwm, "P0.27"}, 

	//TMC-SPI Pins
	{P4_28,  PinCapability::rwpwm, "P4.28"},
	{P0_4,  PinCapability::rwpwm, "P0.4"},
	{P0_5,  PinCapability::rwpwm, "P0.5"},

	//aux-1
	{P0_2,  PinCapability::rwpwm, "P0.2"},
	{P0_3,  PinCapability::rwpwm, "P0.3"},	
    
};

constexpr BoardDefaults mkssgenl1_0_Defaults = {
    {P2_1,  P2_8, P0_21, P2_12,  P0_10},   //enablePins
    {P2_2,  P0_19,  P0_22,  P2_13,   P0_1},    //stepPins
    {P2_3,  P0_20, P2_11, P0_11,  P0_0},   //dirPins
#if LPC_TMC_SOFT_UART
    {P1_4, P1_9, P1_14, P1_16, P4_29},    //uartPins
    5,                                      // Smart drivers
#endif
    0,                                 //digiPot Factor
};

constexpr PinEntry PinTable_MKSSGenL2_0[] =
{
    //Thermistors
    {P0_23, PinCapability::ainrw, "e0temp,th1"},
    {P0_24, PinCapability::ainrw, "bedtemp,TB"},
    {P0_25, PinCapability::ainrw, "e1temp,th2"},

    //Endstops (although they are labeled x- and x+ on the - and + get stripped out in RRF so we will use i.e. xmin/xmax
    {P1_24, PinCapability::rwpwm, "zstopmax,Z+"},
    {P1_25, PinCapability::rwpwm, "zstop,Z-"},
    {P1_26, PinCapability::rwpwm, "ystopmax,Y+"},
    {P1_27, PinCapability::rwpwm, "ystop,Y-"},
    {P1_28, PinCapability::rwpwm, "xstopmax,X+"},
    {P1_29, PinCapability::rwpwm, "xstop,X-"},

    //Heaters and Fans
    {P2_5, PinCapability::rwpwm, "bed"},
    {P2_7, PinCapability::rwpwm, "e0heat,e1"},
    {P2_6, PinCapability::rwpwm, "e1heat,e2"},
    {P2_4, PinCapability::rwpwm, "fan0,P2.4"},
	{P1_4, PinCapability::rwpwm, "fan1,P1.4"},
	
	//servo
	{P1_23,  PinCapability::rwpwm, "servo0,P1.23" },
	{P2_0,  PinCapability::rwpwm, "servo1,P2.0" },
 
    //Spare pins (also as LEDs)
    {P1_21, PinCapability::rwpwm, "led4,P1.21"},
    {P1_20, PinCapability::rwpwm, "led3,P1.20"},
    {P1_19, PinCapability::rwpwm, "led2,P1.19"},
    {P1_18, PinCapability::rwpwm, "led1,P1.18"},
    
    //Exp1
    {P1_31, PinCapability::rwpwm, "P1.31"},
	{P1_30, PinCapability::rwpwm, "P1.30"},
    {P0_18, PinCapability::rwpwm, "P0.18"},
	{P0_16, PinCapability::rwpwm, "P0.16"},
    {P0_15, PinCapability::rwpwm, "P0.15"},    
    {P0_17, PinCapability::rwpwm, "P0.17"},
	{P1_0, PinCapability::rwpwm, "P1.0"},
	{P1_22, PinCapability::rwpwm, "P1.22"},
    
    //Exp2
    {P0_8,  PinCapability::rwpwm, "P0.8"},
	{P0_7,  PinCapability::rwpwm, "P0.7"},
    {P3_25, PinCapability::rwpwm, "P3.25"},
	{P0_28, PinCapability::rwpwm, "P0.28"},
    {P3_26, PinCapability::rwpwm, "P3.26"},
	{P0_9,  PinCapability::rwpwm, "P0.9"},
    {P0_27, PinCapability::rwpwm, "P0.27"},   
	
	//TMC-SPI Pins
	{P1_16,  PinCapability::rwpwm, "P1.16"},
	{P0_4,  PinCapability::rwpwm, "P0.4"},
	{P0_5,  PinCapability::rwpwm, "P0.5"},
	
	//other
	{P0_26,  PinCapability::rwpwm, "P0.26"},
	
	//aux-1
	{P0_2,  PinCapability::rwpwm, "P0.2"},
	{P0_3,  PinCapability::rwpwm, "P0.3"},
	
	//wifi
	{P4_28,  PinCapability::rwpwm, "P4.28"},
	{P4_29,  PinCapability::rwpwm, "P4.29"},
    
};

constexpr BoardDefaults mkssgenl1_0_Defaults = {
    {P2_1,  P2_8, P0_21, P2_12,  P0_10},   //enablePins
    {P2_2,  P0_19,  P0_22,  P2_13,   P1_9},    //stepPins
    {P2_3,  P0_20, P2_11, P0_11,  P1_14},   //dirPins
#if LPC_TMC_SOFT_UART
    {P1_1, P1_8, P1_10, P1_15, P1_17},    //uartPins
    5,                                      // Smart drivers
#endif
    0,                                 //digiPot Factor
};

#endif