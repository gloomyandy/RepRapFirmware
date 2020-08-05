#ifndef BIQU_SKR_H
#define BIQU_SKR_H

#include "../Pins_LPC.h"

// List of assignable pins and their mapping from names to MPU ports. This is indexed by logical pin number.
// The names must match user input that has been concerted to lowercase and had _ and - characters stripped out.
// Aliases are separate by the , character.
// If a pin name is prefixed by ! then this means the pin is hardware inverted. The same pin may have names for both the inverted and non-inverted cases,
// for example the inverted heater pins on the expansion connector are available as non-inverted servo pins on a DFueX.

constexpr PinEntry PinTable_BIQU_SKR_v1_1[] =
{
    //Thermistors
    {P0_23, PinCapability::ainrw, "bedtemp,tb"},
    {P0_24, PinCapability::ainrw, "e0temp,th0"},
    {P0_25, PinCapability::ainrw, "e1temp,th1"},

    //Endstops
    {P1_29, PinCapability::rw, "xstop,xmin"},
    {P1_28, PinCapability::rw, "xstopmax,xmax"},
    {P1_27, PinCapability::rw, "ystop,ymin"},
    {P1_26, PinCapability::rw, "ystopmax,ymax"},
    {P1_25, PinCapability::rw, "zstop,zmin"},
    {P1_24, PinCapability::rw, "zstopmax,zmax"},

    //Heaters and Fans (Big and Small Mosfets}
    {P2_5,  PinCapability::wpwm, "bed,hbed" },
    {P2_7,  PinCapability::wpwm, "e0heat,he0" },
    {P2_4,  PinCapability::wpwm, "e1heat,he1" },
    {P2_3,  PinCapability::wpwm, "fan0,fan" },

    //Exp1
    {P0_15, PinCapability::rwpwm, "P0.15"}, //SSP0 SCK
    {P0_16, PinCapability::rwpwm, "P0.16"},
    {P0_18, PinCapability::rwpwm, "P0.18"}, //SSP0 MOSI
    {P2_11, PinCapability::rwpwm, "P2.11"},
    {P1_30, PinCapability::rwpwm, "P1.30"},

    //Exp2
    {P1_31, PinCapability::rwpwm, "P1.31"},
    //0.18
    {P3_25, PinCapability::rwpwm, "P3.25"},
    {P1_23, PinCapability::rwpwm, "P1.23"},
    {P3_26, PinCapability::rwpwm, "P3.26"},
    {P0_17, PinCapability::rwpwm, "P0.17"}, //SSP0 MISO

    //LCD/SD/SPI header (most overlap with exp1/2)
    //0.15
    //0.18
    //0.16
    {P2_6,  PinCapability::rwpwm, "P2.6"},
    {P1_23, PinCapability::rwpwm, "P1.23"},
    //0.17
    //2.11
    {P3_25, PinCapability::rwpwm, "P3.25"},
    //1.31
    {P3_26, PinCapability::rwpwm, "P3.26"},
	
	//aux-1
	{P0_2,  PinCapability::rwpwm, "P0.2"},
	{P0_3,  PinCapability::rwpwm, "P0.3"},
};

constexpr BoardDefaults biquskr_1_1_Defaults = {
    {P4_28, P2_0, P0_19, P2_12, P0_10},     //enablePins
    {P0_4,  P2_1, P0_20, P0_11, P0_1},      //stepPins
    {P0_5,  P2_2, P0_21, P2_13, P0_0},      //dirPins
#if LPC_TMC_SOFT_UART
    {NoPin, NoPin, NoPin, NoPin, NoPin},    //uartPins
    0,                                      // Smart drivers
#endif
    0,                                      //digiPot Factor
};

// BIQU SKR version 1.3
constexpr PinEntry PinTable_BIQU_SKR_v1_3[] =
{
    //Thermistors
    {P0_23, PinCapability::ainrw, "bedtemp,tb"},
    {P0_24, PinCapability::ainrw, "e0temp,th0"},
    {P0_25, PinCapability::ainrw, "e1temp,th1"},
    
    //Endstops
    {P1_29, PinCapability::rwpwm, "xstop,x-"},
    {P1_28, PinCapability::rwpwm, "xstopmax,x+"},
    {P1_27, PinCapability::rwpwm, "ystop,y-"},
    {P1_26, PinCapability::rwpwm, "ystopmax,y+"},
    {P1_25, PinCapability::rwpwm, "zstop,z-"},
    {P1_24, PinCapability::rwpwm, "zstopmax,z+"},
    
    //Heaters and Fans (Big and Small Mosfets}
    {P2_5,  PinCapability::wpwm, "bed,hbed" },
    {P2_7,  PinCapability::wpwm, "e0heat,he0" },
    {P2_4,  PinCapability::wpwm, "e1heat,he1" },
    {P2_3,  PinCapability::wpwm, "fan0,fan" },
    
    //Servos
    {P2_0,  PinCapability::rwpwm, "servo,servo0" },
    
    //EXP1
    {P1_23, PinCapability::rwpwm, "P1.23"},
    {P1_22, PinCapability::rwpwm, "P1.22"},
    {P1_21, PinCapability::rwpwm, "P1.21"},
    {P1_20, PinCapability::rwpwm, "P1.20"},
    {P1_19, PinCapability::rwpwm, "P1.19"},
    {P1_18, PinCapability::rwpwm, "P1.18"},
    {P0_28, PinCapability::rwpwm, "P0.28"},
    {P1_30, PinCapability::rwpwm, "P1.30"},
    
    //EXP2
    {P1_31, PinCapability::rwpwm, "P1.31"},
    {P0_18, PinCapability::rw, "0.18"}, //SSP0 MOSI
    {P3_25, PinCapability::rwpwm, "P3.25"},
    {P0_16, PinCapability::rwpwm, "P0.16"},
    {P3_26, PinCapability::rwpwm, "P3.26"},
    {P0_15, PinCapability::rw, "0.15"}, //SSP0 SCK
    {P0_17, PinCapability::rw, "0.17"}, //SSP0 MISO
    
    //MONI-SD
    {P0_27, PinCapability::rwpwm, "data2,P0.27"},
    //following pins are the same as the internal sdcard
    //{P0_8,  PinCapability::rwpwm, "0.8"}, //SSP1 MISO
    //{P0_7,  PinCapability::rwpwm, "0.7"}, //SSP1 SCK
    //{P0_9,  PinCapability::rwpwm, "0.9"}, //SSP1 MOSI
    //{P0_6,  PinCapability::rwpwm, "0.6"}, //SEL (SD CS)

    //Other Headers on v1.3 Boards
    //TMC2208-UART
    //      Tx      Rx
    //X     4.29    1.17
    //Y     1.16    1.15
    //Z     1.14    1.10
    //E0    1.9     1.8
    //E1    1.4     1.1

    //TMC2130-SPI (Note SPI pins are not on a HW SPI channel)
    //XCS  1.17
    //YCS  1.15
    //ZCS  1.10
    //E0CS 1.8
    //E1CS 1.1

    //MISO 0.5
    //MOSI 4.28
    //SCK  0.4
	
	//aux-1
	{P0_2,  PinCapability::rwpwm, "P0.2"},
	{P0_3,  PinCapability::rwpwm, "P0.3"},
};

constexpr BoardDefaults biquskr_1_3_Defaults = {
    {P2_1, P2_8,  P0_21, P2_12,  P0_10},    //enablePins
    {P2_2, P0_19, P0_22, P2_13,  P0_1},     //stepPins
    {P2_6, P0_20, P2_11, P0_11,  P0_0},     //dirPins
#if LPC_TMC_SOFT_UART
    {P1_17, P1_15, P1_10, P1_8, P1_1},      //uartPins
    5,                                      // Smart drivers
#endif
    0                                       //digiPot Factor
};

// BIQU SKR version 1.4
constexpr PinEntry PinTable_BIQU_SKR_v1_4[] =
{
    //Thermistors
    {P0_23, PinCapability::ainrw, "e1temp,th1"},
    {P0_24, PinCapability::ainrw, "e0temp,th0"},
    {P0_25, PinCapability::ainrw, "bedtemp,tb"},

    //Endstops
    {P1_29, PinCapability::rwpwm, "xstop,x-stop"},
    {P1_28, PinCapability::rwpwm, "ystop,y-stop"},
    {P1_27, PinCapability::rwpwm, "zstop,z-stop"},
    {P1_26, PinCapability::rwpwm, "e0stop,e0det"},
    {P1_25, PinCapability::rwpwm, "e1stop,e1det"},
	{P1_0, PinCapability::rwpwm, "pwrdet,P1.0"},
	{P0_10, PinCapability::rwpwm, "probe"},
	
    //Heaters and Fans (Big and Small Mosfets}
    {P2_5,  PinCapability::wpwm, "bed,hbed" },
    {P2_7,  PinCapability::wpwm, "e0heat,he0" },
    {P2_4,  PinCapability::wpwm, "e1heat,he1" },
    {P2_3,  PinCapability::wpwm, "fan0,fan" },

    //Servos
    {P2_0,  PinCapability::rwpwm, "servo0" },
	
	//Neopixel
	{P1_24, PinCapability::rwpwm, "neopixel,P1.24"},

    //EXP1
    {P1_23, PinCapability::rwpwm, "P1.23"},
    {P1_22, PinCapability::rwpwm, "P1.22"},
    {P1_21, PinCapability::rwpwm, "P1.21"},
    {P1_20, PinCapability::rwpwm, "P1.20"},
    {P1_19, PinCapability::rwpwm, "P1.19"},
    {P1_18, PinCapability::rwpwm, "P1.18"},
    {P0_28, PinCapability::rwpwm, "P0.28"},
    {P1_30, PinCapability::rwpwm, "P1.30"},

    //EXP2
    {P1_31, PinCapability::rwpwm, "P1.31"},
    {P0_18, PinCapability::rw, "0.18"}, //SSP0 MOSI
    {P3_25, PinCapability::rwpwm, "P3.25"},
    {P0_16, PinCapability::rwpwm, "P0.16"},
    {P3_26, PinCapability::rwpwm, "P3.26"},
    {P0_15, PinCapability::rw, "0.15"}, //SSP0 SCK
    {P0_17, PinCapability::rw, "0.17"}, //SSP0 MISO

    //MONI-SD
    {P0_27, PinCapability::rwpwm, "data2,P0.27"},

	//Wifi
	{P4_28, PinCapability::rwpwm, "wifi1,P4.28"},
	{P4_29, PinCapability::rwpwm, "wifi2,P4.29"},
	
	//i2c
	{P0_0, PinCapability::rwpwm, "i2c1,P0.0"},
	{P0_1, PinCapability::rwpwm, "i2c2,P0.1"},
	
	//SPI
	{P0_26, PinCapability::rwpwm, "SPI1,P0.26"},
	{P0_7, PinCapability::rwpwm, "P0.7"},
	{P0_8, PinCapability::rwpwm, "P0.8"},
	{P0_9, PinCapability::rwpwm, "P0.9"},
	
	//tft
	{P0_2,  PinCapability::rwpwm, "P0.2"},
	{P0_3,  PinCapability::rwpwm, "P0.3"},
	
    //Other Headers on v1.4 Boards
    //TMC2208-UART
    //      Tx      Rx
    //X     4.29    1.17
    //Y     1.16    1.15
    //Z     1.14    1.10
    //E0    1.9     1.8
    //E1    1.4     1.1

    //TMC2130-UART (Note SPI pins are not on a HW SPI channel)
    //XCS  1.10
    //YCS  1.09
    //ZCS  1.08
    //E0CS 1.04
    //E1CS 1.01

    //MISO 0.5
    //MOSI 1.17
    //SCK  0.4
};

constexpr BoardDefaults biquskr_1_4_Defaults = {
    {P2_1, P2_8,  P0_21, P2_12,  P1_16},    //enablePins
    {P2_2, P0_19, P0_22, P2_13,  P1_15},    //stepPins
    {P2_6, P0_20, P2_11, P0_11,  P1_14},    //dirPins
#if LPC_TMC_SOFT_UART
    {P1_10, P1_9, P1_8, P1_4, P1_1},        //uartPins
    5,                                      // Smart drivers
#endif
    0                                       //digiPot Factor
};

#endif