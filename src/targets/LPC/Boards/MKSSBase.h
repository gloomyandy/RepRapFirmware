#ifndef MKSSBASE_H
#define MKSSBASE_H

#include "../Pins_LPC.h"


// List of assignable pins and their mapping from names to MPU ports. This is indexed by logical pin number.
// The names must match user input that has been concerted to lowercase and had _ and - characters stripped out.
// Aliases are separate by the , character.
// If a pin name is prefixed by ! then this means the pin is hardware inverted. The same pin may have names for both the inverted and non-inverted cases,
// for example the inverted heater pins on the expansion connector are available as non-inverted servo pins on a DFueX.

constexpr PinEntry PinTable_MKSSbase1_3[] =
{
    //Thermistors
    {P0_23, PinCapability::ainrw, "bedtemp,th1"},
    {P0_24, PinCapability::ainrw, "e0temp,th2"},
    {P0_25, PinCapability::ainrw, "e1temp,th3"},
    {P0_26, PinCapability::ainrw, "th4"},

    //Endstops (although they are labeled x- and x+ on the - and + get stripped out in RRF so we will use i.e. xmin/xmax
    {P1_24, PinCapability::rwpwm, "xstop,xmin"},
    {P1_25, PinCapability::rwpwm, "xstopmax,xmax"},
    {P1_26, PinCapability::rwpwm, "ystop,ymin"},
    {P1_27, PinCapability::rwpwm, "ystopmax,ymax"},
    {P1_28, PinCapability::rwpwm, "zstop,zmin"},
    {P1_29, PinCapability::rwpwm, "zstopmax,zmax"},

    //Heaters and Fans
    {P2_5, PinCapability::rwpwm, "bed"},
    {P2_7, PinCapability::rwpwm, "e0heat,e1"},
    {P2_6, PinCapability::rwpwm, "e1heat,e2"},
    {P2_4, PinCapability::rwpwm, "fan0,fan"},

    //J7
    {P0_17, PinCapability::rwpwm, "P0.17"},
    {P0_16, PinCapability::rwpwm, "P0.16"},
    {P0_14, PinCapability::rwpwm, "P0.14"},
    
    //J8
    {P1_22, PinCapability::rwpwm, "P1.22"},
    {P1_23, PinCapability::rwpwm, "P1.23"},
    {P2_12, PinCapability::rwpwm, "P2.12"},
    {P2_11, PinCapability::rwpwm, "P2.11"},
    {P4_28, PinCapability::rwpwm, "P4.28"},
    
    //aux-1
	{P0_2,  PinCapability::rwpwm, "P0.2"},
	{P0_3,  PinCapability::rwpwm, "P0.3"},
    
    //Exp1
    {P1_31, PinCapability::rwpwm, "P1.31"},
    {P0_18, PinCapability::rwpwm, "P0.18"},
    {P0_14, PinCapability::rwpwm, "P0.14"},
    {P1_30, PinCapability::rwpwm, "P1.30"},
    
    //Exp2
    {P0_8,  PinCapability::rwpwm, "P0.8"},
    {P3_25, PinCapability::rwpwm, "P3.25"},
    {P3_26, PinCapability::rwpwm, "P3.26"},
    {P0_27, PinCapability::rwpwm, "P0.27"},
    {P0_7,  PinCapability::rwpwm, "P0.7"},
    {P0_28, PinCapability::rwpwm, "P0.28"},
    {P0_9,  PinCapability::rwpwm, "P0.9"},
    
};

constexpr BoardDefaults mkssbase1_3_Defaults = {
    5,
    {P0_4,  P0_10, P0_19, P0_21,  P4_29},   //enablePins
    {P2_0,  P2_1,  P2_2,  P2_3,   P2_8},    //stepPins
    {P0_5,  P0_11, P0_20, P0_22,  P2_13},   //dirPins
#if TMC_SOFT_UART
    {NoPin, NoPin, NoPin, NoPin, NoPin},    //uartPins
    0,                                      // Smart drivers
#endif
    113.33,                                 //digiPot Factor
};

#endif