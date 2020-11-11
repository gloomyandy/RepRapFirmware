#ifndef FLY_CDY_H
#define FLY_CDY_H

#include "../Pins_LPC.h"

// List of assignable pins and their mapping from names to MPU ports. This is indexed by logical pin number.
// The names must match user input that has been concerted to lowercase and had _ and - characters stripped out.
// Aliases are separate by the , character.
// If a pin name is prefixed by ! then this means the pin is hardware inverted. The same pin may have names for both the inverted and non-inverted cases,
// for example the inverted heater pins on the expansion connector are available as non-inverted servo pins on a DFueX.

constexpr PinEntry PinTable_FLY_CDY[] =
{
    //Thermistors
    {P0_23, PinCapability::ainrw, "bedtemp,tb"},
    {P0_26, PinCapability::ainrw, "e0temp,t0"},
    {P0_25, PinCapability::ainrw, "e1temp,t1"},
    {P0_24, PinCapability::ainrw, "e2temp,t2"},

    //Endstops
    {P1_29, PinCapability::rw, "xstop"},
    {P1_28, PinCapability::rw, "xstopmax"},
    {P1_27, PinCapability::rw, "ystop"},
    {P1_25, PinCapability::rw, "ystopmax"},
    {P1_22, PinCapability::rw, "zstop"},
    {P1_19, PinCapability::rw, "zstopmax"},

    //Heaters and Fans
    {P3_26,  PinCapability::wpwm, "bed" },
    {P3_25,  PinCapability::wpwm, "e0heat" },
    {P1_20,  PinCapability::wpwm, "e1heat" },
    {P1_23,  PinCapability::wpwm, "e2heat" },
    {P1_18,  PinCapability::wpwm, "fan0" },
    {P1_21,  PinCapability::wpwm, "fan1" },
    {P1_24,  PinCapability::wpwm, "fan2" },

    //Exp1
    {P2_7, PinCapability::rwpwm, "P2.7"}, 
    {P2_8, PinCapability::rwpwm, "P2.8"},
    {P0_22, PinCapability::rwpwm, "P0.22"}, 
    {P2_10, PinCapability::rwpwm, "P2.10"},
    {P1_19, PinCapability::rwpwm, "P1.19"},
    {P0_28, PinCapability::rwpwm, "P0.28"},
    {P1_30, PinCapability::rwpwm, "P1.30"},
    {P1_31, PinCapability::rwpwm, "P1.31"},

    //Exp2
    {P0_17, PinCapability::rwpwm, "P0.17"},
    {P0_15, PinCapability::rwpwm, "P0.15"},
    {P0_16, PinCapability::rwpwm, "P0.16"},
    {P0_18, PinCapability::rwpwm, "P0.18"},
    {P2_6, PinCapability::rwpwm,  "P2.6"},

    //Servos
    {P1_26,  PinCapability::rwpwm, "servo0,P1.26" },

	//Wifi
	{P0_1, PinCapability::rwpwm, "wifi1,P0.1"},
	{P0_0, PinCapability::rwpwm, "wifi2,P0.0"},

	//screen
	{P0_2,  PinCapability::rwpwm, "P0.2"},
	{P0_3,  PinCapability::rwpwm, "P0.3"},

	//TMC-SPI Pins
	{P0_19,  PinCapability::rwpwm, "P0_19"},  //SOFT MISO
	{P0_20,  PinCapability::rwpwm, "P0_20"},  //SOFT MOSI
	{P0_21,  PinCapability::rwpwm, "P0_21"},  //SCK

};

constexpr BoardDefaults FLY_CDY_Defaults = {
    {P1_0,  P1_8, P1_14, P1_17, P0_4,  P2_13, NoPin},     //enablePins
    {P2_0,  P2_1, P2_2,  P2_3,  P2_4,  P2_5,  NoPin},      //stepPins
    {P1_1,  P1_9, P1_15, P4_29, P2_11, P0_11, NoPin},      //dirPins
#if TMC_SOFT_UART
    {P1_4, P1_10, P1_16, P4_28, P2_12, P0_10, NoPin},    //uartPins
    6,                                      // Smart drivers
#endif
    0,                                      //digiPot Factor
};
#endif