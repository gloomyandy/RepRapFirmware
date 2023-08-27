#ifndef FYSETC_H
#define FYSETC_H


// List of assignable pins and their mapping from names to MPU ports. This is indexed by logical pin number.
// The names must match user input that has been converted to lowercase and had _ and - characters stripped out.
// Aliases are separate by the , character.
// If a pin name is prefixed by ! then this means the pin is hardware inverted. The same pin may have names for both the inverted and non-inverted cases,
// for example the inverted heater pins on the expansion connector are available as non-inverted servo pins on a DFueX.
#if STM32F4
constexpr PinEntry PinTable_FYSETC_SPIDER[] =
{
    //Thermistors
    {PC_0, "e0temp,t0"},
    {PC_1, "e1temp,t1"},
    {PC_2, "e2temp,t2"},
    {PC_3, "bedtemp,t3"},

    //Endstops
    {PB_14,   "xstop,x-"},
    {PB_13,   "ystop,y-"},
    {PA_0,   "zstop,z-"},
    {PA_1,   "xstopmax,x+"},
    {PA_2,   "ystopmax,y+"},
    
    //Heaters and Fans (Big and Small Mosfets}
    {PB_4,    "bed,hbed" },
    {PB_15,    "e0heat,he0" },
    {PC_8,    "e1heat,he1" },
    {PB_3,    "e2heat,he2" },
    {PB_0,    "fan0,fan" },
    {PB_1,    "fan1" },
    {PB_2,    "fan2" },

    //Servos
    {PA_3,    "servo0,z+,zstopmax" },
	
    //EXP1
//    {PD_1,   "PD1"},
//    {PD_0,   "PD0"},
//    {PC_12,   "PC12"},
//    {PC_10,   "PC10"},
//    {PD_2,   "PD2"},
//    {PC_11,   "PC11"},
//    {PA_8,   "PA8"},
//    {PC_7,   "PC9"},

    //EXP2
//    {PA_6,   "PA6"},  // MISO
//    {PA_5,   "PA5"},  // SCK
//    {PC_6,   "PC6"},  // ENC_A
//    {PA_4,   "PA4"},  // CS
//    {PC_7,   "PC7"},  // ENC_B
//    {PA_7,   "PA7"},  // MOSI
//    {PB_10,  "PB10"},// SD_DET
	
  	//SPI
//    {PE_12,   "PE12"}, // SCK4
//    {PE_13,   "PE13"}, // MISO4
//    {PE_14,   "PE14"}, // MOSI4
//    {PE_7,    "X-CS,PE7"},
//    {PD_10,   "Z-CS,PD10"},
//    {PE_15,   "Y-CS,PE15"},
//    {PD_7,    "E0-CS,PD7"},
//    {PC_14,   "E1-CS,PC14"},
//    {PC_15,   "E2-CS,PC15"},
//    {PA_15,   "E3-CS,PA15"},
//    {PD_11,   "E4-CS,PD11"},
  	
  	//I2C
//    {PB_9,   "PB9"},  // SDA
//    {PB_8,   "PB8"},  // SCL
  	
  	//UART
//    {PA_9,   "PA9"},  // TX1
//    {PA_10,  "PA8"}, // RX1

  	// NEOPIXEL
//    {PD_3,   "PD3"},

    // RGB
//    {PB_6,   "PB6"},
//    {PB_5,   "PB5"},
//    {PB_7,   "PB7"},

};

constexpr BoardDefaults fysetc_spider_Defaults = {
    {0x8479e19e},                               // Signature
    SD_SPI1_B,                                  // SD Card access
    {   //CLK, MISO, MOSI
        {PA_5, PA_6, PA_7},                     //SPI0
        {NoPin, NoPin, NoPin},                  //SPI1
        {PC_10, PC_11, PC_12},                  //SPI2
        {PE_12, PE_13, PE_14},                  //SPI3
        {NoPin, NoPin, NoPin},                  //SPI4
        {NoPin, NoPin, NoPin},                  //SPI5
    },
	8,											// Number of drivers
    {PE_9,  PD_9,  PD_15, PD_4, PE_5,  PE_3,  PD_12, PE_1},   	//enablePins
    {PE_11, PD_8,  PD_14, PD_5, PE_6,  PE_2,  PE_8,  PC_5},	//stepPins
    {PE_10, PB_12, PD_13, PD_6, PC_13, PE_4,  PC_4,  PE_0},    	//dirPins
#if HAS_SMART_DRIVERS
    {PE_7,  PE_15, PD_10, PD_7, PC_14, PC_15, PA_15, PD_11},      //uartPins
    8,                                      	// Smart drivers
#endif
    0,                                       	//digiPot Factor
#if HAS_VOLTAGE_MONITOR
    NoPin,
#endif
    NoPin,
#if HAS_SBC_INTERFACE
    NoPin, NoPin, SSPNONE,
#endif
};
#endif
# if STM32F4 || STM32H723xx
constexpr PinEntry PinTable_FYSETC_SPIDER_KING407[] =
{
    //Thermistors
    {PC_1, "e0temp,t0"},
    {PF_9, "e1temp,t1"},
    {PC_3, "e2temp,t2"},
    {PC_2, "e3temp,t3"},
    {PC_0, "e4temp,t4"},
    {PF_10, "bedtemp,t5"},

    //Endstops
    {PC_5,   "xstop,x-"},
    {PC_4,   "ystop,y-"},
    {PB_6,   "zstop,z-"},
    {PB_5,   "xstopmax,x+"},
    {PF_13,  "ystopmax,y+"},
    {PF_14,  "zstopmax,z+"},
	{PA_0,   "probe"},
    
    //Heaters and Fans (Big and Small Mosfets}
    {PE_10,   "bed,hbed" },
    {PB_4,    "e0heat,he0" },
    {PB_0,    "e1heat,he1" },
    {PD_13,   "e2heat,he2" },
    {PC_8,    "e3heat,he3" },
    {PA_15,   "e4heat,he4" },
    {PE_8,    "fan0,fan" },
    {PE_9,    "fan1" },
    {PD_15,   "fan2" },
    {PD_12,   "fan3" },
    {PD_14,   "fan4" },
	{PB_1,    "tacho0" },
	{PB_7,    "tacho1" },

    //Servos
    {PA_1,    "servo0" },
	
    //EXP1
    {PE_7,   "LCDD7"},
    {PG_4,   "LCDD6"},
    {PC_11,   "LCDD5"},
    {PC_10,   "LCDD4"},
    {PD_0,   "LCDRS"},
    {PC_12,   "LCDEN"},
    {PA_8,   "BTNENC"},
    {PC_7,   "BEEP"},

    //EXP2
    {PA_6,   "LCDMISO"},  // MISO
    {PA_5,   "LCDSCK"},  // SCK
    {PC_6,   "BTNEN1"},  // ENC_A
    {PA_4,   "LCDSS"},  // CS
    {PC_7,   "BTNEN2"},  // ENC_B
    {PA_7,   "LCDMOSI"},  // MOSI
    {PB_10,  "LCDCD"},// SD_DET
	
  	//SPI
//    {PE_12,   "PE12"}, // SCK4
//    {PE_13,   "PE13"}, // MISO4
//    {PE_14,   "PE14"}, // MOSI4
//    {PD_2,    "X-CS,PD2"},
//    {PE_15,   "X2-CS,PE15"},
//    {PD_8,    "Y-CS,PD8"},
//    {PD_7,    "Z-CS,PD7"},
//    {PC_14,   "Z2-CS,PC14"},
//    {PC_15,   "E0-CS,PC15"},
//    {PG_3,    "E1-CS,PG3"},
//    {PD_9,    "E2-CS,PD9"},
//    {PF_5,    "E3-CS,PF5"},
//    {PG_11,   "E4-CS,PG11"},
  	
  	//I2C
//    {PF_0,   "PF1"},  // SDA
//    {PF_1,   "PF0"},  // SCL

  	// NEOPIXEL
    {PD_3,   "PD3"},
};

constexpr BoardDefaults fysetc_spider_king407_Defaults = {
    {0xb86f16db},                               // Signature
    SD_SPI1_B,                                  // SD Card access
    {   //CLK, MISO, MOSI
        {PA_5, PA_6, PA_7},                     //SPI0
        {PB_13, PB_14, PB_15},                  //SPI1
        {PC_10, PC_11, PC_12},                  //SPI2
        {PE_12, PE_13, PE_14},                  //SPI3
        {NoPin, NoPin, NoPin},                  //SPI4
        {NoPin, NoPin, NoPin},                  //SPI5
    },
	  10,											// Number of drivers
    {PE_11, PG_10, PG_15, PD_5, PE_6,  PE_2,  PG_9,  PB_2,  PF_2, PG_5,},   //enablePins
    {PG_7,  PD_11, PG_14, PD_4, PE_5,  PE_3,  PG_13, PE_1,  PF_4, PF_15,},  //stepPins
    {PG_6,  PD_10, PG_12, PD_6, PC_13, PE_4,  PG_8,  PE_0,  PF_3, PG_0},   	//dirPins
#if HAS_SMART_DRIVERS
    {PD_2, PE_15, PD_8,  PD_7, PC_14, PC_15, PG_3,  PD_9,  PF_5, PG_11},   //uartPins
    10,                                      	// Smart drivers
#endif
    0,                                       	//digiPot Factor
#if HAS_VOLTAGE_MONITOR
    NoPin,
#endif
    NoPin,
#if HAS_SBC_INTERFACE
    PB_11, PB_12, SSP2,
#endif
};
#endif
#endif
