#ifndef BIQU_SKR_H
#define BIQU_SKR_H

#include "../Pins_STM32.h"

// List of assignable pins and their mapping from names to MPU ports. This is indexed by logical pin number.
// The names must match user input that has been converted to lowercase and had _ and - characters stripped out.
// Aliases are separate by the , character.
// If a pin name is prefixed by ! then this means the pin is hardware inverted. The same pin may have names for both the inverted and non-inverted cases,
// for example the inverted heater pins on the expansion connector are available as non-inverted servo pins on a DFueX.

constexpr PinEntry PinTable_BIQU_SKR_PRO_v1_1[] =
{
    //Thermistors
    {PF_3, PinCapability::ainrw, "e0temp,t0"},
    {PF_4, PinCapability::ainrw, "e1temp,t1"},
    {PF_5, PinCapability::ainrw, "e2temp,t2"},
    {PF_6, PinCapability::ainrw, "bedtemp,t3"},

    //Endstops
    {PB_10, PinCapability::rwpwm, "xstop,x-stop"},
    {PE_12, PinCapability::rwpwm, "ystop,y-stop"},
    {PG_8, PinCapability::rwpwm, "zstop,z-stop"},
    {PE_15, PinCapability::rwpwm, "e0stop,e0det"},
    {PE_10, PinCapability::rwpwm, "e1stop,e1det"},
    {PG_5, PinCapability::rwpwm, "e2stop,e2det"},
	{PA_2, PinCapability::rwpwm, "probe"},
	
    //Heaters and Fans (Big and Small Mosfets}
    {PD_12,  PinCapability::wpwm, "bed,hbed" },
    {PB_1,  PinCapability::wpwm, "e0heat,he0" },
    {PD_14,  PinCapability::wpwm, "e1heat,he1" },
    {PB_0,  PinCapability::wpwm, "e2heat,he2" },
    {PC_8,  PinCapability::wpwm, "fan0,fan" },
    {PE_5,  PinCapability::wpwm, "fan1" },
    {PE_6,  PinCapability::wpwm, "fan2" },

    //Servos
    {PA_1,  PinCapability::rwpwm, "servo0" },
	
    //EXP1
    {PG_4, PinCapability::rwpwm, "PG4"},
    {PA_8, PinCapability::rwpwm, "PA8"},
    {PD_11, PinCapability::rwpwm, "PD11"},
    {PD_10, PinCapability::rwpwm, "PD10"},
    {PG_2, PinCapability::rwpwm, "PG2"},
    {PG_3, PinCapability::rwpwm, "PG3"},
    {PG_6, PinCapability::rwpwm, "PG6"},
    {PG_7, PinCapability::rwpwm, "PG7"},

    //EXP2
    {PB_14, PinCapability::rwpwm, "PB14"},
    {PB_13, PinCapability::rwpwm, "PB13"},
    {PG_10, PinCapability::rwpwm, "PG10"},
    {PB_12, PinCapability::rwpwm, "PB12"},
    {PF_11, PinCapability::rwpwm, "PF11"},
    {PB_15, PinCapability::rwpwm, "PB15"},
    {PF_12, PinCapability::rwpwm, "PF12"},
    {PF_13, PinCapability::rwpwm, "PF13"},

	//Extension 1
	{PC_9, PinCapability::rwpwm, "PC9"},
	{PF_9, PinCapability::rwpwm, "PF9"},
	{PC_4, PinCapability::rwpwm, "PC4"},
	{PG_11, PinCapability::rwpwm, "PG11"},
	{PG_14, PinCapability::rwpwm, "PG14"},
	{PC_1, PinCapability::rwpwm, "PC1"},
	{PF_8, PinCapability::rwpwm, "PF8"},
	{PF_10, PinCapability::rwpwm, "PF10"},
	{PC_5, PinCapability::rwpwm, "PC5"},
	{PG_13, PinCapability::rwpwm, "PG13"},
	{PD_3, PinCapability::rwpwm, "PD3"},
	{PF_7, PinCapability::rwpwm, "PF7"},

	//Extension 2
	{PD_0, PinCapability::rwpwm, "PD0"},
	{PD_2, PinCapability::rwpwm, "PD2"},
	{PD_0, PinCapability::rwpwm, "PD5"},
	{PE_0, PinCapability::rwpwm, "PE0"},
	{PE_2, PinCapability::rwpwm, "PE2"},
	{PE_4, PinCapability::rwpwm, "PE4"},

	//Wifi
	{PG_0, PinCapability::rwpwm, "wifi1,PG0"},
	{PG_1, PinCapability::rwpwm, "wifi2,PG1"},
	{PC_7, PinCapability::rwpwm, "wifi3,PC7"},
	{PC_6, PinCapability::rwpwm, "wifi4,PC6"},
	{PF_14, PinCapability::rwpwm, "wifi5,PF14"},
	{PF_15, PinCapability::rwpwm, "wifi6,PF15"},
	
	//SPI
	{PC_10, PinCapability::rwpwm, "PC10"},
	{PC_11, PinCapability::rwpwm, "PC11"},
	{PA_15, PinCapability::rwpwm, "X-CS,PA15"},
	{PC_12, PinCapability::rwpwm, "PC12"},
	{PB_9, PinCapability::rwpwm, "Z-CS,PB9"},
	{PB_8, PinCapability::rwpwm, "Y-CS,PB8"},
	{PB_3, PinCapability::rwpwm, "E0-CS,PB3"},
	{PG_15, PinCapability::rwpwm, "E1-CS,PG15"},
	{PG_12, PinCapability::rwpwm, "E2-CS,PG12"},
	
	//I2C
	{PB_7, PinCapability::rwpwm, "PB7"},
	{PB_6, PinCapability::rwpwm, "PB6"},
	
	//UART
	{PD_9, PinCapability::rwpwm, "PD9"},
	{PD_8, PinCapability::rwpwm, "PD8"},

};

constexpr BoardDefaults biquskr_pro_1_1_Defaults = {
	8,											// Number of drivers
    {PF_2, PD_7,  PC_0, PC_3,  PA_3, PF_0, PD_11, PG_7},   	//enablePins
    {PE_9, PE_11, PE_13, PE_14,  PD_15, PD_13, PG_3, PG_2},	//stepPins
    {PF_1, PE_8, PC_2, PA_0,  PE_7, PG_9, PD_10, PG_6},    	//dirPins
#if TMC_SOFT_UART
    {PC_13, PE_3, PE_1, PD_4, PD_1, PD_6, PF_11, PG_10},      //uartPins
    6,                                      	// Smart drivers
#endif
    0                                       	//digiPot Factor
};


constexpr PinEntry PinTable_BIQU_GTR_v1_0[] =
{
    //Thermistors
    {PC_1, PinCapability::ainrw, "e0temp,t0"},
    {PC_2, PinCapability::ainrw, "e1temp,t1"},
    {PC_3, PinCapability::ainrw, "e2temp,t2"},
    {PC_0, PinCapability::ainrw, "bedtemp,t3"},
	{PA_3, PinCapability::ainrw, "e3temp,Temp_M1"},
	{PF_9, PinCapability::ainrw, "e4temp,Temp_M2"},
	{PF_10, PinCapability::ainrw, "e5temp,Temp_M3"},
	{PF_7, PinCapability::ainrw, "e6temp,Temp_M4"},
	{PF_5, PinCapability::ainrw, "e7temp,Temp_M5"},

    //Endstops
    {PF_2, PinCapability::rwpwm, "xstop,x-stop"},
    {PC_13, PinCapability::rwpwm, "ystop,y-stop"},
    {PE_0, PinCapability::rwpwm, "zstop,z-stop"},
    {PG_14, PinCapability::rwpwm, "e0stop,e0det"},
    {PG_9, PinCapability::rwpwm, "e1stop,e1det"},
    {PD_3, PinCapability::rwpwm, "e2stop,e2det"},
	{PI_4, PinCapability::rwpwm, "e3stop,M1_Stop"},
	{PF_4, PinCapability::rwpwm, "e4stop,M2_Stop"},
	{PF_6, PinCapability::rwpwm, "e5stop,M3_Stop"},
	{PI_7, PinCapability::rwpwm, "e6stop,M4_Stop"},
	{PF_12, PinCapability::rwpwm, "e7stop,M5_Stop"},
	{PH_11, PinCapability::rwpwm, "probe"},
	{PI_11, PinCapability::rwpwm, "EI1,PI11"},
	{PH_6, PinCapability::rwpwm, "EI2,PH6"},
	
    //Heaters and Fans (Big and Small Mosfets}
    {PA_2,  PinCapability::wpwm, "bed,hbed" },
    {PB_1,  PinCapability::wpwm, "e0heat,heat0" },
    {PA_1,  PinCapability::wpwm, "e1heat,heat1" },
    {PB_0,  PinCapability::wpwm, "e2heat,heat2" },
	{PD_15,  PinCapability::wpwm, "e3heat,heat_M1" },
	{PD_13,  PinCapability::wpwm, "e4heat,heat_M2" },
	{PD_12,  PinCapability::wpwm, "e5heat,heat_M3" },
	{PE_13,  PinCapability::wpwm, "e6heat,heat_M4" },
	{PI_6,  PinCapability::wpwm, "e7heat,heat_M5" },
    {PE_5,  PinCapability::wpwm, "fan0" },
    {PE_6,  PinCapability::wpwm, "fan1" },
    {PC_8,  PinCapability::wpwm, "fan2" },
	{PI_5,  PinCapability::wpwm, "fan_M1" },
	{PE_9,  PinCapability::wpwm, "fan_M2" },
	{PE_11,  PinCapability::wpwm, "fan_M3" },
	{PC_9,  PinCapability::wpwm, "fan_M4" },
	{PE_14,  PinCapability::wpwm, "fan_M5" },

    //Servos
    {PB_11,  PinCapability::rwpwm, "servo0" },
	
    //EXP1
    {PC_11, PinCapability::rwpwm, "PC11"},
    {PA_15, PinCapability::rwpwm, "PA15"},
    {PC_10, PinCapability::rwpwm, "PC10"},
    {PA_8, PinCapability::rwpwm, "PA8"},
    {PG_8, PinCapability::rwpwm, "PG8"},
    {PG_7, PinCapability::rwpwm, "PG7"},
    {PG_6, PinCapability::rwpwm, "PG6"},
    {PG_5, PinCapability::rwpwm, "PG5"},

    //EXP2
    {PB_14, PinCapability::rwpwm, "PB14"},
    {PB_13, PinCapability::rwpwm, "PB13"},
    {PD_10, PinCapability::rwpwm, "PD10"},
    {PB_12, PinCapability::rwpwm, "PB12"},
    {PH_10, PinCapability::rwpwm, "PH10"},
    {PB_15, PinCapability::rwpwm, "PB15"},
    {PB_10, PinCapability::rwpwm, "PB10"},
	
	//Extension-0
	{PH_13, PinCapability::rwpwm, "PH13"},
	{PI_9, PinCapability::rwpwm, "PI9"},
	
	//Extension-1
	{PI_10, PinCapability::rwpwm, "PI10"},
	{PI_8, PinCapability::rwpwm, "PI8"},
	
	//Extension-2
	{PH_8, PinCapability::rwpwm, "PH8"},
	{PH_7, PinCapability::rwpwm, "PH7"},
	
	//Neopixel
	{PF_13, PinCapability::rwpwm, "Neopixel,PF13"},

	//Wifi
	{PF_11, PinCapability::rwpwm, "wifi1,PF11"},
	{PC_7, PinCapability::rwpwm, "wifi3,PC7"},
	{PC_6, PinCapability::rwpwm, "wifi4,PC6"},
	{PC_5, PinCapability::rwpwm, "wifi5,PC5"},
	
	//Raspberry Pi
	{PA_10, PinCapability::rwpwm, "PA10"},
	{PA_9, PinCapability::rwpwm, "PA9"},
	
	//SPI
	{PC_14, PinCapability::rwpwm, "X-CS,PC10"},
	{PE_1, PinCapability::rwpwm, "Y-CS,PE1"},
	{PB_5, PinCapability::rwpwm, "Z-CS,PB5"},
	{PG_10, PinCapability::rwpwm, "E0-CS,PG10"},
	{PD_4, PinCapability::rwpwm, "E1-CS,PD4"},
	{PC_12, PinCapability::rwpwm, "E2-CS,PC12"},
	{PB_6, PinCapability::rwpwm, "MISO,PB6"},
	{PG_15, PinCapability::rwpwm, "MOSI,PG15"},
	{PB_3, PinCapability::rwpwm, "SCK,PB3"},
	
	//Extra M5
	{PI_2, PinCapability::rwpwm, "KMOSI,PI2"},
	{PI_1, PinCapability::rwpwm, "KSCK,PI1"},
	{PH_2, PinCapability::rwpwm, "KCS,PH2"},
	{PF_13, PinCapability::rwpwm, "RGB_LED,PF13"},
};

constexpr BoardDefaults biqu_gtr_1_0_Defaults = {
	6+5,										// Number of drivers GTR + M5
	{PF_1, PE_4,  PB_9, PG_13,  PD_7, PD_2,PF_8,PG_2,PF_4,PE_8,PI_0},    //enablePins including M5
	{PC_15, PE_3, PB_8, PG_12,  PD_6, PD_1,PF_3,PD_14,PE_12,PG_0,PH_12},    //stepPins Including M5
	{PF_0, PE_2, PB_7, PG_11,  PD_5, PD_0,PG_3,PD_11,PE_10,PG_1,PH_15}, //dirPins Including M5
#if TMC_SOFT_UART
	{PC_14, PE_1, PB_5, PG_10, PD_4, PC_12,PG_4,PE_15,PE_7,PF_15,PH_14},  //uartpins including M5
    6,                                      // Smart drivers or 11 with M5
#endif
    0                                       //digiPot Factor
};

#endif