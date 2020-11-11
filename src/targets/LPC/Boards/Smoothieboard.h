#ifndef SMOOTHIEBOARD_H
#define SMOOTHIEBOARD_H

#include "../Pins_LPC.h"

// List of assignable pins and their mapping from names to MPU ports. This is indexed by logical pin number.
// The names must match user input that has been concerted to lowercase and had _ and - characters stripped out.
// Aliases are separate by the , character.
// If a pin name is prefixed by ! then this means the pin is hardware inverted. The same pin may have names for both the inverted and non-inverted cases,
// for example the inverted heater pins on the expansion connector are available as non-inverted servo pins on a DFueX.

//https://raw.githubusercontent.com/Bouni/smoothieboard-graphics/master/smoothieboard-wiring.png
constexpr PinEntry PinTable_Smoothieboard[] =
{

    //Thermistors
    {P0_23, PinCapability::ainrw, "e0temp,t0"},
    {P0_24, PinCapability::ainrw, "bedtemp,t1"},
    {P0_25, PinCapability::ainrw, "e1temp,t2"},
    {P0_26, PinCapability::ainrw, "t3,P0.26"},

    //Endstops
    {P1_24, PinCapability::rwpwm, "xstop,xmin"},
    {P1_25, PinCapability::rwpwm, "xstopmax,xmax"},
    {P1_26, PinCapability::rwpwm, "ystop,ymin"},
    {P1_27, PinCapability::rwpwm, "ystopmax,ymax"},
    {P1_28, PinCapability::rwpwm, "zstop,zmin"},
    {P1_29, PinCapability::rwpwm, "zstopmax,zmax"},

    //Heaters and Fans (Big and Small Mosfets}
    {P1_23, PinCapability::rwpwm, "e1heat,q5"  }, //(Big Mosfet)
    {P2_5,  PinCapability::rwpwm, "bed,q6" },  //(Big Mosfet)
    {P2_7,  PinCapability::rwpwm, "e0heat,q7" },  //(Big Mosfet)
    {P1_22, PinCapability::rwpwm, "q4,P1.22" },  //(Small Mosfet)
    {P2_4,  PinCapability::rwpwm, "q8,P2.4" },  //(Small Mosfet)
    {P2_6,  PinCapability::rwpwm, "fan0,q9" },  //(Small Mosfet)

    //Spare pins (also as LEDs)
    {P1_21, PinCapability::rwpwm, "led1,P1.21"},
    {P1_20, PinCapability::rwpwm, "led2,P1.20"},
    {P1_19, PinCapability::rwpwm, "led3,P1.19"},
    {P1_18, PinCapability::rwpwm, "led4,P1.18"},

    //Spare pins (or used for LCD)
    {P1_22, PinCapability::rwpwm, "P1.22"},
    {P1_23, PinCapability::rwpwm, "P1.23"},
    {P1_31, PinCapability::rwpwm, "P1.31"},
    {P1_30, PinCapability::rwpwm, "P1.30"},
    {P3_25, PinCapability::rwpwm, "P3.25"},
    {P3_26, PinCapability::rwpwm, "P3.26"},
    {P2_11, PinCapability::rwpwm, "P2.11"},
	{P2_12, PinCapability::rwpwm, "P2.12"},

};

constexpr BoardDefaults smoothieBoardDefaults = {
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