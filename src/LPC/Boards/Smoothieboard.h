#ifndef SMOOTHIEBOARD_H
#define SMOOTHIEBOARD_H

#include "../PINS_LPC.h"


// List of assignable pins and their mapping from names to MPU ports. This is indexed by logical pin number.
// The names must match user input that has been concerted to lowercase and had _ and - characters stripped out.
// Aliases are separate by the , character.
// If a pin name is prefixed by ! then this means the pin is hardware inverted. The same pin may have names for both the inverted and non-inverted cases,
// for example the inverted heater pins on the expansion connector are available as non-inverted servo pins on a DFueX.



//https://raw.githubusercontent.com/Bouni/smoothieboard-graphics/master/smoothieboard-wiring.png
constexpr PinEntry PinTable_Smoothieboard[] =
{

    //Thermistors
    {P0_23, PinCapability::ainrw, "t1"},
    {P0_24, PinCapability::ainrw, "t2"},
    {P0_25, PinCapability::ainrw, "t3"},
    {P0_26, PinCapability::ainrw, "t4"},

    //Endstops
    {P1_24, PinCapability::rwpwm, "xmin"},
    {P1_25, PinCapability::rwpwm, "xmax"},
    {P1_26, PinCapability::rwpwm, "ymin"},
    {P1_27, PinCapability::rwpwm, "ymax"},
    {P1_28, PinCapability::rwpwm, "zmin"},
    {P1_29, PinCapability::rwpwm, "zmax"},

    //Heaters and Fans (Big and Small Mosfets}
    {P1_23, PinCapability::rwpwm, "q5"  }, //(Big Mosfet)
    {P2_5,  PinCapability::rwpwm, "q6" },  //(Big Mosfet)
    {P2_7,  PinCapability::rwpwm, "q7" },  //(Big Mosfet)
    {P1_22, PinCapability::rwpwm, "q4" },  //(Small Mosfet)
    {P2_4,  PinCapability::rwpwm, "q8" },  //(Small Mosfet)
    {P2_6,  PinCapability::rwpwm, "q9" },  //(Small Mosfet)

    //Spare pins (also as LEDs)
    {P1_21, PinCapability::rwpwm, "led1"},
    {P1_20, PinCapability::rwpwm, "led2"},
    {P1_19, PinCapability::rwpwm, "led3"},
    {P1_18, PinCapability::rwpwm, "led4"},

    //Spare pins (or used for LCD)
    {P1_22, PinCapability::rwpwm, "p1.22"},
    {P1_23, PinCapability::rwpwm, "p1.23"},
    {P1_31, PinCapability::rwpwm, "p1.31"},
    {P1_30, PinCapability::rwpwm, "p1.30"},
    {P3_25, PinCapability::rwpwm, "p3.25"},
    {P3_26, PinCapability::rwpwm, "p3.26"},
    {P2_11, PinCapability::rwpwm, "p2.11"},

};


constexpr BoardDefaults smoothieBoardDefaults = {
    {P0_4,  P0_10, P0_19, P0_21,  P4_29},   //enablePins
    {P2_0,  P2_1,  P2_2,  P2_3,   P2_8},    //stepPins
    {P0_5,  P0_11, P0_20, P0_22,  P2_13},   //dirPins
#if LPC_TMC_SOFT_UART
    {NoPin, NoPin, NoPin, NoPin, NoPin},    //uartPins
#endif
    true,                                   //currentControl
    113.33,                                 //digiPot Factor    
};

#endif
