#ifndef BIQU_SKR_H
#define BIQU_SKR_H


// List of assignable pins and their mapping from names to MPU ports. This is indexed by logical pin number.
// The names must match user input that has been converted to lowercase and had _ and - characters stripped out.
// Aliases are separate by the , character.
// If a pin name is prefixed by ! then this means the pin is hardware inverted. The same pin may have names for both the inverted and non-inverted cases,
// for example the inverted heater pins on the expansion connector are available as non-inverted servo pins on a DFueX.
#if STM32H7
# if STM32H743xx
constexpr PinEntry PinTable_BIQU_SKR_SE_BX_v2_0[] =
{
    //Thermistors
    {PH_4, "e0temp,th0"},
    {PA_3, "e1temp,th1"},
    {PH_5, "e2temp,thb"},

    //Endstops
    {PB_11, "xstop"},
    {PD_13, "xstopmax"},
    {PB_12, "ystop"},
    {PB_13, "ystopmax"},
    {PD_12, "zstop"},
    {PD_11, "zstopmax"},
	{PH_2, "probe"},
	
    //Heaters and Fans (Big and Small Mosfets}
    {PA_4,  "bed,hbed" },
    {PC_4,  "e0heat,he0" },
    {PC_5,  "e1heat,he1" },
    {PA_5,  "fan0,fan" },
    {PA_6,  "fan1" },
    {PA_7,  "fan2" },

    //Servos
    {PA_2,  "servo0" },
	

	//Extension 1
	{PC_9, "PC9"},
	{PF_9, "PF9"},
	{PC_4, "PC4"},
	{PG_11, "PG11"},
	{PG_14, "PG14"},
	{PC_1, "PC1"},
	{PF_8, "PF8"},
	{PF_10, "PF10"},
	{PC_5, "PC5"},
	{PG_13, "PG13"},
	{PD_3, "PD3"},
	{PF_7, "PF7"},

	//Extension 2
	{PD_0, "PD0"},
	{PD_2, "PD2"},
	{PD_0, "PD5"},
	{PE_0, "PE0"},
	{PE_2, "PE2"},
	{PE_4, "PE4"},

	//Wifi
	{PA_0, "wifitx,TX4"},
	{PA_1, "wifirx,RX4"},
    //TFT
	{PA_9, "tfttx,TX1"},
	{PA_10, "tftrx,RX1"},
};

constexpr BoardDefaults biqu_skr_se_bx_v2_0_Defaults = {
    {0xf1832a2},                 	            // Signatures
    SD_SPI3_B,                                  // SD Card access
    {   //CLK, MISO, MOSI
        {NoPin, NoPin, NoPin},                  //SPI0
        {NoPin, NoPin, NoPin},                  //SPI1
        {PC_10, PC_11, PC_12},                  //SPI2
        {NoPin, NoPin, NoPin},                  //SPI3
        {NoPin, NoPin, NoPin},                  //SPI4
        {NoPin, NoPin, NoPin},                  //SPI5
        {PE_2,  PE_5,  PE_6},                   //SPI6
        {NoPin, NoPin, NoPin},                  //SPI7
        {NoPin, NoPin, NoPin},                  //SPI8
    },
	5,											// Number of drivers
    {PG_14, PB_4, PG_9, PC_15, PD_2},        	//enablePins
    {PG_13, PB_3, PD_7, PC_14, PA_8},	        //stepPins
    {PG_12, PD_3, PD_6, PC_13, PC_9},    	    //dirPins
#if HAS_SMART_DRIVERS
    {PG_10, PD_4, PD_5, PI_8, PC_8},            //uartPins
    5,                                      	// Smart drivers
#endif
    0,                                       	//digiPot Factor
#if HAS_VOLTAGE_MONITOR
    NoPin,
#endif
    PI_11,
#if HAS_SBC_INTERFACE
    PH_12, PH_10, SSP4,
#endif
};
# endif
constexpr PinEntry PinTable_BTT_SKR_3[] =
{
    //Thermistors
    {PA_1, "bedtemp,tb"},
    {PA_2, "e0temp,th0"},
    {PA_3, "e1temp,th1"},

    //Endstops
    {PC_1, "xstop"},
    {PC_3, "ystop"},
    {PC_0, "zstop"},
    {PC_2, "e0stop,e0det"},
    {PA_0, "e1stop,e1det"},

    //Servos
    {PE_5,  "servo0" },

    //Probe
    {PC_13, "probe"},

    //Heaters and Fans (Big and Small Mosfets}
    {PD_7,  "bed,hbed" },
    {PB_3,  "e0heat,heat0" },
    {PB_4,  "e1heat,heat1" },
    {PB_7,  "fan0,fan" },
    {PB_6,  "fan1" },
    {PB_5,  "fan2" },

    //Neopixel
    {PE_6, "Neopixel"},

    //PSON
    {PE_4, "PSON"},

    //PWRDET
    {PC_15, "PWRDET"},

    //Status LED
    {PA_13, "LED,status"},
	
    //EXP1
    {PC_5, "BEEP"},
    {PB_0, "BTNENC"},
    {PB_1, "LCDEN"},
    {PE_8, "LCDRS"},
    {PE_9, "LCDD4"},
    {PE_10, "LCDD5"},
    {PE_11, "LCDD6"},
    {PE_12, "LCDD7"},

    //EXP2
    {PA_6, "LCDMISO"},
    {PA_5, "LCDSCK"},
    {PE_7, "BTNEN1"},
    {PA_4, "LCDSS"},
    {PB_2, "BTNEN2"},
    {PA_7, "LCDMOSI"},
    {PC_4, "LCDCD"},
};

constexpr BoardDefaults btt_skr_3_Defaults = {
# if STM32H743xx
    {0xaa36a0c4},                             // Signatures
# elif STM32H723xx
    {0x132ea777},                             // Signatures
# endif
    SD_SDIO,                                  // SD Card access
    {   //CLK, MISO, MOSI
        {PA_5, PA_6, PA_7},                   //SPI0 EXP2
        {PB_13, PB_14, PB_15},                //SPI1 WiFi
        {NoPin, NoPin, NoPin},                //SPI2
        {PE_14, PE_15, PE_13},                //SPI3 Drivers
        {PC_5, PB_1, PB_0},                   //SPI4 Accelerometer
        {PE_9, NoPin, PB_1},                  //SPI5 RepRapDiscount Display
        {NoPin, NoPin, NoPin},                //SPI6
        {NoPin, NoPin, NoPin},                //SPI7
        {NoPin, NoPin, NoPin},                //SPI8
    },
    5,                            // Number of drivers
    {PD_6, PD_1, PE_0, PC_7, PD_13}, // enablePins
    {PD_4, PA_15, PE_2, PD_15, PD_11},  // stepPins
    {PD_3, PA_8, PE_3, PD_14, PD_10},    // dirPins
#if HAS_SMART_DRIVERS
    {PD_5, PD_0, PE_1, PC_6, PD_12},    // uartpins
    5,                            // Smart drivers
#endif
    0,                            // digiPot Factor
#if HAS_VOLTAGE_MONITOR
    NoPin,
#endif
    NoPin,
#if HAS_SBC_INTERFACE
    NoPin, NoPin, SSPNONE,
#endif
};

# if STM32H723xx

constexpr PinEntry PinTable_BTT_OCTOPUSPRO_V1_1[] =
{
    //Thermistors
    {PF_3, "bedtemp,tb"},
    {PF_4, "e0temp,th0"},
    {PF_5, "e1temp,th1"},
    {PF_6, "e2temp,th2"},
    {PF_7, "e3temp,th3"},

    //Endstops
    {PG_6, "xstop"},
    {PG_9, "ystop"},
    {PG_10, "zstop"},
    {PG_11, "e0stop,e0det"},
    {PG_12, "e1stop,e1det"},
    {PG_13, "e2stop,e2det"},
    {PG_14, "e3stop,e3det"},
    {PG_15, "e4stop,e4det"},

    //Servos
    {PB_6,  "servo0" },

    //Probe
    {PB_7, "probe"},
    {PC_5, "probe1"},

    //Heaters and Fans (Big and Small Mosfets}
    {PA_1,  "bed,hbed" },
    {PA_0,  "e0heat,heat0" },
    {PA_3,  "e1heat,heat1" },
    {PB_0, "e2heat,heat2" },
    {PB_11, "e3heat,heat3" },
    {PA_8,  "fan0,fan" },
    {PE_5,  "fan1" },
    {PD_12, "fan2" },
    {PD_13, "fan3" },
    {PD_14, "fan4" },
    {PD_15, "fan5" },

    //EXP1
    {PE_15, "LCDD7"},
    {PE_13, "LCDD5"},
    {PE_10, "LCDRS"},
    {PE_7, "BTNENC"},
    {PE_8, "BEEP"},
    {PE_9, "LCDEN"},
    {PE_12, "LCDD4"},
    {PE_14, "LCDD6"},

    //EXP2
    {PC_15, "LCDCD"},
    {PB_2, "BTNEN2"},
    {PB_1, "BTNEN1"},
    {PA_6, "LCDMISO"},
    {PA_5, "LCDSCK"},
    {PA_4, "LCDSS"},
    {PA_7, "LCDMOSI"},

    //Neopixel
    {PB_10, "Neopixel"},

    //TFT
    {PA_9, "TX1,tfttx"},
    {PA_10, "RX1,tftrx"},
	{PD_6, "TX2"},
	{PD_5, "RX2"},

    //SPI
    {PC_4, "XCS"},
    {PD_11, "YCS"},
    {PC_6, "ZCS"},
    {PC_7, "E0CS"},
    {PF_2, "E1CS"},
    {PE_4, "E2CS"},
    {PE_1, "E3CS"},
    {PD_3, "E4CS"},
    {PA_6, "MISO"},
    {PA_7, "MOSI"},
    {PA_5, "SCK"},

    //I2C
    {PB_8, "SCL1"},
    {PB_9, "SDA1"},

    //PSON
    {PE_11, "PSON"},

    //PWRDET
    {PC_0, "PWRDET"},

    //Status LED
    {PA_13, "LED,status"},
};

constexpr BoardDefaults btt_octopuspro_Defaults = {
    {0x5450ab93},                               // Signatures
    SD_SDIO,                                    // SD Card access
    {   //CLK, MISO, MOSI
        {PA_5, PA_6, PA_7},                     //SPI0 Drivers + Screen + PT100
        {PB_13, PC_2, PC_3},                    //SPI1 WiFi
        {PB_3,  PB_4,  PB_5},                   //SPI2 SPI3 Header - J74
        {NoPin, NoPin, NoPin},                  //SPI3
        {PE_8, PE_9, PE_7},                     //SPI4 Accelerometer
        {PE_12, NoPin, PE_9},                   //SPI5 RepRapDiscount Display
    },
    8,                                          // Number of drivers
    {PF_14, PF_15, PG_5, PA_2, PG_2, PF_1, PD_4, PE_0}, // enablePins
    {PF_13, PG_0, PF_11, PG_4, PF_9, PC_13, PE_2, PE_6},// stepPins
    {PF_12, PG_1, PG_3, PC_1, PF_10, PF_0, PE_3,
#if STARTUP_DELAY
    NoPin
#else
    PA_14
#endif 
    },  // dirPins
#if HAS_SMART_DRIVERS
    {PC_4, PD_11, PC_6, PC_7, PF_2, PE_4, PE_1, PD_3},  // uartpins
    8,                                          // Smart drivers
#endif
    0,                                          // digiPot Factor
#if HAS_VOLTAGE_MONITOR
    NoPin,
#endif
    NoPin,
#if HAS_SBC_INTERFACE
    PD_10, PB_12, SSP2,
#endif
};


constexpr PinEntry PinTable_BTT_KRAKEN[] =
{
    //Thermistors
    {PB_0, "bedtemp"},
    {PB_1, "e0temp"},
    {PC_5, "e1temp"},
	{PC_4, "e2temp"},
	{PA_7, "e3temp"},

    //Endstops
    {PC_15, "xstop"},
    {PF_0, "ystop"},
    {PF_1, "zstop"},
    {PF_2, "e0stop"},
    {PF_3, "e1stop"},
	{PF_4, "e2stop"},
	{PF_10, "e3stop"},
	{PC_0, "e4stop"},

    //Servos
    {PE_9,  "servo0" },
	{PE_7,  "servo1" },

    //Probe
    {PG_1, "probe"},

    //Heaters and Fans (Big and Small Mosfets}
    {PF_5,  "bed" },
    {PF_6,  "e0heat" },
    {PF_7,  "e1heat" },
	{PF_9,  "e2heat" },
	{PF_8,  "e3heat" },
    {PA_0,  "fan0" },
    {PA_1,  "fan1" },
    {PA_2,  "fan2" },
	{PA_3,  "fan3" },
	{PA_4,  "fan4" },
	{PA_5,  "fan5" },
	{PA_6,  "fan6" },
	{PE_8,  "fan7" },
	{PC_1,  "fan6.tacho" },
	{PG_0,  "fan7.tacho" },

    //Neopixel
    {PF_12, "Neopixel"},
	{PF_11, "Neopixel1"},

    //PSON
    {PD_10, "PSON"},

    //IND-Det
    {PD_11, "INDDET"},
	
	//ADC
	{PC_2, "ADC1"},
	{PC_3, "ADC2"},
	{PB_10, "SCL2"},
	{PB_11, "SDA2"},
	
    //EXP1
    {PG_5, "BEEP"},
    {PG_4, "BTNENC"},
    {PG_3, "LCDEN"},
    {PG_2, "LCDRS"},
    {PD_15, "LCDD4"},
    {PD_14, "LCDD5"},
    {PD_13, "LCDD6"},
    {PD_12, "LCDD7"},

    //EXP2
    {PE_13, "LCDMISO"},
    {PE_12, "LCDSCK"},
    {PG_8, "BTNEN1"},
    {PE_11, "LCDSS"},
    {PG_7, "BTNEN2"},
    {PE_14, "LCDMOSI"},
    {PG_6, "LCDCD"},
};

constexpr BoardDefaults btt_kraken_Defaults = {
    {0x5450ab93, 0x26b1a20f},                             // Signatures
    SD_SPI2_A,                                  // SD Card access
    {   //CLK, MISO, MOSI
        {NoPin, NoPin, NoPin},                   //SPI0
        {PB_13, PB_14, PB_15},                //SPI1 SD Card
        {PC_10, PC_11, PC_12},                //SPI2 31865
        {PC_6, PC_7, PC_8},                //SPI3 Drivers
        {NoPin, NoPin, NoPin},                  //SPI4 31865
        {NoPin, NoPin, NoPin},                  //SPI5 RepRapDiscount Display
        {PE_12, PE_13, PE_14},                //SPI6 EXP2
        {NoPin, NoPin, NoPin},                //SPI7
        {NoPin, NoPin, NoPin},                //SPI8
    },
    8,                            // Number of drivers
    {PE_6, PE_3, PE_0, PB_7, PG_13, PG_12, PB_5, PG_14}, // enablePins
    {PC_14, PE_5, PE_2, PB_9, PG_9, PG_11, PB_4, PG_15},  // stepPins
    {PC_13, PE_4, PE_1, PB_8, PG_10, PD_7, PB_3, PB_6},    // dirPins
#if HAS_SMART_DRIVERS
    {PD_6, PD_5, PD_4, PD_3, PD_2, PA_15, PA_9, PA_10},    // uartpins
    8,                            // Smart drivers
#endif
    0,                            // digiPot Factor
#if HAS_VOLTAGE_MONITOR
    NoPin,
#endif
    NoPin,
#if HAS_SBC_INTERFACE
    NoPin, NoPin, SSPNONE,
#endif
};
# endif
#endif
#if STM32F4
constexpr PinEntry PinTable_BIQU_SKR_PRO_v1_1[] =
{
    //Thermistors
    {PF_3, "e0temp,t0"},
    {PF_4, "e1temp,t1"},
    {PF_5, "e2temp,t2"},
    {PF_6, "bedtemp,t3"},

    //Endstops
    {PB_10, "xstop"},
    {PE_12, "ystop"},
    {PG_8, "zstop"},
    {PE_15, "e0stop,e0det"},
    {PE_10, "e1stop,e1det"},
    {PG_5, "e2stop,e2det"},
    {PA_2, "probe"},

    //Heaters and Fans (Big and Small Mosfets}
    {PD_12,  "bed,hbed" },
    {PB_1,  "e0heat,he0" },
    {PD_14,  "e1heat,he1" },
    {PB_0,  "e2heat,he2" },
    {PC_8,  "fan0,fan" },
    {PE_5,  "fan1" },
    {PE_6,  "fan2" },

    //Servos
    {PA_1,  "servo0" },

    //EXP1
    {PG_4, "BEEP"},
    {PA_8, "BTNENC"},
    {PD_11, "LCDEN"},
    {PD_10, "LCDRS"},
    {PG_2, "LCDD4"},
    {PG_3, "LCDD5"},
    {PG_6, "LCDD6"},
    {PG_7, "LCDD7"},

    //EXP2
    {PB_14, "LCDMISO"},
    {PB_13, "LCDSCK"},
    {PG_10, "BTNEN1"},
    {PB_12, "LCDSS"},
    {PF_11, "BTNEN2"},
    {PB_15, "LCDMOSI"},
    {PF_12, "LCDCD"},
    {PF_13, "KILL"},

    //Wifi
    {PG_0, "wifi1"},
    {PG_1, "wifi2"},
    {PC_7, "wifi3"},
    {PC_6, "wifi4"},
    {PF_14, "wifi5"},
    {PF_15, "wifi6"},

    //SPI
    {PA_15, "XCS"},
    {PB_9, "ZCS"},
    {PB_8, "YCS"},
    {PB_3, "E0CS"},
    {PG_15, "E1CS"},
    {PG_12, "E2CS"},
	
	//Status LED
    {PA_13, "status"},

};

constexpr BoardDefaults biquskr_pro_1_1_Defaults = {
    {0x768a39d6, 0x50da391, 0xa79a1917},        // Signatures
    SD_SPI1_A,                                  // SD Card access
    {   //CLK, MISO, MOSI
        {PA_5, PA_6, PB_5},                     //SPI0 SD Card
        {PB_13, PB_14, PB_15},                  //SPI1 WiFi
        {PC_10, PC_11, PC_12},                  //SPI2 Drivers and Accelerometer
        {PG_14, PC_1, PF_7},                    //SPI3 Screen
        {NoPin, NoPin, NoPin},                  //SPI4
        {NoPin, NoPin, NoPin},                  //SPI5
    },
    6,                                          // Number of drivers
    {PF_2, PD_7,  PC_0, PC_3,  PA_3, PF_0},     //enablePins
    {PE_9, PE_11, PE_13, PE_14,  PD_15, PD_13}, //stepPins
    {PF_1, PE_8, PC_2, PA_0,  PE_7, PG_9},      //dirPins
#if HAS_SMART_DRIVERS
    {PC_13, PE_3, PE_1, PD_4, PD_1, PD_6},      //uartPins
    6,                                          // Smart drivers
#endif
    0,                                          //digiPot Factor
#if HAS_VOLTAGE_MONITOR
    NoPin,
#endif
    NoPin,
#if HAS_SBC_INTERFACE
    PF_12, PB_12, SSP2,
#endif
};


constexpr PinEntry PinTable_BIQU_GTR_v1_0[] =
{
    //Thermistors
    {PC_1, "e0temp,t0"},
    {PC_2, "e1temp,t1"},
    {PC_3, "e2temp,t2"},
    {PC_0, "bedtemp,t3"},
    {PA_3, "e3temp,TempM1"},
    {PF_9, "e4temp,TempM2"},
    {PF_10, "e5temp,TempM3"},
    {PF_7, "e6temp,TempM4"},
    {PF_5, "e7temp,TempM5"},

    //Endstops
    {PF_2, "xstop"},
    {PC_13, "ystop"},
    {PE_0, "zstop"},
    {PG_14, "e0stop,e0det"},
    {PG_9, "e1stop,e1det"},
    {PD_3, "e2stop,e2det"},
    {PI_4, "e3stop,M1Stop"},
    {PF_4, "e4stop,M2Stop"},
    {PF_6, "e5stop,M3Stop"},
    {PI_7, "e6stop,M4Stop"},
    {PF_12, "e7stop,M5Stop"},
    {PH_11, "probe"},
    {PI_11, "EI1"},
    {PH_6, "EI2"},

    //Heaters and Fans (Big and Small Mosfets}
    {PA_2,  "bed,hbed" },
    {PB_1,  "e0heat,heat0" },
    {PA_1,  "e1heat,heat1" },
    {PB_0,  "e2heat,heat2" },
    {PD_15,  "e3heat,heatM1" },
    {PD_13,  "e4heat,heatM2" },
    {PD_12,  "e5heat,heatM3" },
    {PE_13,  "e6heat,heatM4" },
    {PI_6,  "e7heat,heatM5" },
    {PE_5,  "fan0" },
    {PE_6,  "fan1" },
    {PC_8,  "fan2" },
    {PI_5,  "fanM1" },
    {PE_9,  "fanM2" },
    {PE_11,  "fanM3" },
    {PC_9,  "fanM4" },
    {PE_14,  "fanM5" },

    //Servos
    {PB_11,  "servo0" },

    //EXP1
    {PC_11, "BEEP"},
    {PA_15, "BTNENC"},
    {PC_10, "LCDEN"},
    {PA_8, "LCDRS"},
    {PG_8, "LCDD4"},
    {PG_7, "LCDD5"},
    {PG_6, "LCDD6"},
    {PG_5, "LCDD7"},

    //EXP2
    {PB_14, "LCDMISO"},
    {PB_13, "LCDSCK"},
    {PD_10, "BTNEN1"},
    {PB_12, "LCDSS"},
    {PH_10, "BTNEN2"},
    {PB_15, "LCDMOSI"},
    {PB_10, "LCDCD"},

    //Neopixel
    {PF_13, "Neopixel"},

    //Wifi
    {PF_11, "wifi1"},
    {PC_7, "wifi3"},
    {PC_6, "wifi4"},
    {PC_5, "wifi5"},

    //SPI
    {PC_14, "XCS"},
    {PE_1, "YCS"},
    {PB_5, "ZCS"},
    {PG_10, "E0CS"},
    {PD_4, "E1CS"},
    {PC_12, "E2CS"},
    {PB_6, "MISO"},
    {PG_15, "MOSI"},
    {PB_3, "SCK"},

    //Extra M5
    {PI_2, "KMOSI"},
    {PI_1, "KSCK"},
    {PH_2, "KCS"},
    {PF_13, "RGBLED"},
	//Status LED
    {PA_0, "status"},

};

constexpr BoardDefaults biqu_gtr_1_0_Defaults = {
    {0x94a2cc03},                               // Signatures
    SD_SPI1_B,                                  // SD Card access
    {   //CLK, MISO, MOSI
        {PA_5, PA_6, PA_7},                     //SPI0 SD Card
        {PB_13, PB_14, PB_15},                  //SPI1 WiFi
        {NoPin, NoPin, NoPin},                  //SPI2
        {PI_9, PB_11, PH_11},                   //SPI3 Screen
        {PI_1, PI_2, NoPin},                    //SPI4 Thermocouple
        {PB_3, PB_6, PG_15},                    //SPI5 Drivers
    },
    6+5,                                        // Number of drivers GTR + M5
    {PF_1, PE_4,  PB_9, PG_13,  PD_7, PD_2,PF_8,PG_2,PF_4,PE_8,PI_0},    //enablePins including M5
    {PC_15, PE_3, PB_8, PG_12,  PD_6, PD_1,PF_3,PD_14,PE_12,PG_0,PH_12},    //stepPins Including M5
    {PF_0, PE_2, PB_7, PG_11,  PD_5, PD_0,PG_3,PD_11,PE_10,PG_1,PH_15}, //dirPins Including M5
#if HAS_SMART_DRIVERS
    {PC_14, PE_1, PB_5, PG_10, PD_4, PC_12,PG_4,PE_15,PE_7,PF_15,PH_14},  //uartpins including M5
    6,                                      // Smart drivers or 11 with M5
#endif
    0,                                       //digiPot Factor
#if HAS_VOLTAGE_MONITOR
    NoPin,
#endif
    NoPin,
#if HAS_SBC_INTERFACE
    PB_10, PB_12, SSP2,
#endif
};

constexpr PinEntry PinTable_BTT_RRF_E3_v1_1[] =
{
    //Thermistors
    {PA_0, "e0temp,th0"},
    {PA_1, "bedtemp,tb"},
    {PA_2, "e1temp,th1"},
    {PA_3, "PT100,th2"},

    //Endstops
    {PC_0, "xstop"},
    {PC_1, "ystop"},
    {PC_2, "zstop"},
    {PC_3, "e0stop,e0det"},
    {PB_10, "e1stop"},
    {PB_11, "x2stop"},
    {PE_3, "IO"},

    //Servos
    {PB_0,  "servo0" },

    //Probe
    {PC_5, "probe"},

    //Heaters and Fans (Big and Small Mosfets}
    {PB_4,  "bed,hbed" },
    {PB_3,  "e0heat,heat0" },
    {PE_4,  "e1heat" },
    {PB_5,  "fan0,fan" },
    {PB_6,  "fan1" },
    {PE_5,  "fan2" },
    {PE_6,  "fan3" },

    //EXP1
    {PB_1, "LCDD6"},
    {PB_2, "LCDD4"},
    {PE_7, "LCDEN"},
    {PE_8, "BEEP"},
    {PE_9,"BTNENC"},
    {PE_10, "LCDD5"},
    {PE_11,"LCDD7"},

    //Neopixel
    {PB_7, "Neopixel"},

    //TFT
    {PA_9, "TX1,tfttx"},
    {PA_10, "RX1,tftrx"},

    //UART
    {PB_8, "SDA1"},
    {PB_9, "SCL1"},

    //PSON
    {PE_1, "PSON"},

    //PWRDET
    {PE_0, "PWRDET"},

    //Status LED
    {PE_2, "LED,status"},

};

constexpr BoardDefaults btt_rrf_e3_1_1_Defaults = {
    {0x94a2cc03, 0xb173b733},                   // Signatures
    SD_SDIO,                                    // SD Card access
    {   //CLK, MISO, MOSI
        {NoPin, NoPin, NoPin},                  //SPI0
        {PB_13, PB_14, PB_15},                  //SPI1 WiFi
        {NoPin, NoPin, NoPin},                  //SPI2
        {NoPin, NoPin, NoPin},                  //SPI3
        {PE_10, NoPin, PE_11},                  //SPI4 Ender Screen
        {PB_1, PB_2, PE_11},                    //SPI5 Accelerometer
    },
    6,                                          // Number of drivers
    {PD_7, PD_3, PD_14, PD_10, PC_13, PE_15},   // enablePins
    {PD_5, PD_0, PC_6, PD_12, PC_15, PE_13},    // stepPins
    {PD_4, PA_15, PC_7, PD_13, PA_8, PE_12},    // dirPins
#if HAS_SMART_DRIVERS
    {PD_6, PD_1, PD_15, PD_11, PC_14, PE_14},   // uartpins
    4,                                          // Smart drivers
#endif
    0,                                          // digiPot Factor
#if HAS_VOLTAGE_MONITOR
    NoPin,
#endif
    NoPin,
#if HAS_SBC_INTERFACE
    NoPin, NoPin, SSPNONE,
#endif
};

constexpr PinEntry PinTable_BTT_SKR_2[] =
{
    //Thermistors
    {PA_1, "bedtemp,tb"},
    {PA_2, "e0temp,th0"},
    {PA_3, "e1temp,th1"},

    //Endstops
    {PC_1, "xstop"},
    {PC_3, "ystop"},
    {PC_0, "zstop"},
    {PC_2, "e0stop,e0det"},
    {PA_0, "e1stop,e1det"},

    //Servos
    {PE_5,  "servo0" },

    //Probe
    {PE_4, "probe"},

    //Heaters and Fans (Big and Small Mosfets}
    {PD_7,  "bed,hbed" },
    {PB_3,  "e0heat,heat0" },
    {PB_4,  "e1heat,heat1" },
    {PB_7,  "fan0,fan" },
    {PB_6,  "fan1" },
    {PB_5,  "fan2" },

    //EXP1
    {PE_13, "LCDD7"},
    {PE_11, "LCDD5"},
    {PE_9, "LCDRS"},
    {PC_5, "BEEP"},
    {PB_0, "BTNENC"},
    {PE_10, "LCDD4"},
    {PE_12, "LCDD6"},
    {PB_1, "LCDEN"},

    //EXP2
    {PC_4, "LCDCD"},
    {PB_2, "BTNEN2"},
    {PE_7, "BTNEN1"},
    {PA_6, "LCDMISO"},
    {PA_5, "LCDSCK"},
    {PA_4, "LCDSS"},
    {PA_7, "LCDMOSI"},

    //Neopixel
    {PE_6, "Neopixel"},

    //TFT
    {PA_9, "TX1,tfttx"},
    {PA_10, "RX1,tftrx"},

    //SPI
    {PE_0, "XCS"},
    {PD_3, "YCS"},
    {PD_0, "ZCS"},
    {PC_6, "E0CS"},
    {PD_12, "E1CS"},
    {PA_14, "MISO"},
    {PE_14, "MOSI"},
    {PE_15, "SCK"},

    //I2C
    {PB_8, "SCL1"},
    {PB_9, "SDA1"},

    //PSON
    {PE_8, "PSON"},

    //PWRDET
    {PC_15, "PWRDET"},

    //Status LED
    {PA_13, "LED,status"},

    //Safe power
    {PC_13, "SP"},
};

constexpr BoardDefaults btt_skr_2_Defaults = {
    {0xb75b00a7, 0x35f4602c},               // Signatures
    SD_SDIO,                                // SD Card access
    {   //CLK, MISO, MOSI
        {PA_5, PA_6, PA_7},                 //SPI0 Screen
        {PB_13, PB_14, PB_15},              //SPI1 WiFi
        {NoPin, NoPin, NoPin},              //SPI2
        {PE_15, PA_14, PE_14},              //SPI3 Drivers
        {PC_5, PB_1, PB_0},                 //SPI4 Accelerometer
        {PE_10, NoPin, PB_1},               //SPI5 RepRapDiscount Display
    },
    5,                                      // Number of drivers
    {PE_3, PD_6, PD_1, PC_7, PD_13},        // enablePins
    {PE_2, PD_5, PA_15, PD_15, PD_11},      // stepPins
    {PE_1, PD_4, PA_8, PD_14, PD_10},       // dirPins
#if HAS_SMART_DRIVERS
    {PE_0, PD_3, PD_0, PC_6, PD_12},        // uartpins
    5,                                      // Smart drivers
#endif
    0,                                      // digiPot Factor
#if HAS_VOLTAGE_MONITOR
    NoPin,
#endif
    PC_13,
#if HAS_SBC_INTERFACE
    PB_11, PB_12, SSP2,
#endif
};

constexpr PinEntry PinTable_BTT_OCTOPUS[] =
{
    //Thermistors
    {PF_3, "bedtemp,tb"},
    {PF_4, "e0temp,th0"},
    {PF_5, "e1temp,th1"},
    {PF_6, "e2temp,th2"},
    {PF_7, "e3temp,th3"},

    //Endstops
    {PG_6, "xstop"},
    {PG_9, "ystop"},
    {PG_10, "zstop"},
    {PG_11, "e0stop,e0det"},
    {PG_12, "e1stop,e1det"},
    {PG_13, "e2stop,e2det"},
    {PG_14, "e3stop,e3det"},
    {PG_15, "e4stop,e4det"},

    //Servos
    {PB_6,  "servo0" },

    //Probe
    {PB_7, "probe"},

    //Heaters and Fans (Big and Small Mosfets}
    {PA_1,  "bed,hbed" },
    {PA_2,  "e0heat,heat0" },
    {PA_3,  "e1heat,heat1" },
    {PB_10, "e2heat,heat2" },
    {PB_11, "e3heat,heat3" },
    {PA_8,  "fan0,fan" },
    {PE_5,  "fan1" },
    {PD_12, "fan2" },
    {PD_13, "fan3" },
    {PD_14, "fan4" },
    {PD_15, "fan5" },

    //EXP1
    {PE_15, "LCDD7"},
    {PE_13, "LCDD5"},
    {PE_10, "LCDRS"},
    {PE_7, "BTNENC"},
    {PE_8, "BEEP"},
    {PE_9, "LCDEN"},
    {PE_12, "LCDD4"},
    {PE_14, "LCDD6"},

    //EXP2
    {PC_15, "LCDCD"},
    {PB_2, "BTNEN2"},
    {PB_1, "BTNEN1"},
    {PA_6, "LCDMISO"},
    {PA_5, "LCDSCK"},
    {PA_4, "LCDSS"},
    {PA_7, "LCDMOSI"},

    //Neopixel
    {PB_0, "Neopixel"},

    //TFT
    {PA_9, "TX1,tfttx"},
    {PA_10, "RX1,tftrx"},

    //SPI
    {PC_4, "XCS"},
    {PD_11, "YCS"},
    {PC_6, "ZCS"},
    {PC_7, "E0CS"},
    {PF_2, "E1CS"},
    {PE_4, "E2CS"},
    {PE_1, "E3CS"},
    {PD_3, "E4CS"},
    {PA_6, "MISO"},
    {PA_7, "MOSI"},
    {PA_5, "SCK"},

    //I2C
    {PB_8, "SCL1"},
    {PB_9, "SDA1"},

    //PSON
    {PE_11, "PSON"},

    //PWRDET
    {PC_0, "PWRDET"},

    //Status LED
    {PA_13, "LED,status"},
};

constexpr BoardDefaults btt_octopus_Defaults = {
    {0x5e29d842},                               // Signatures
    SD_SDIO,                                    // SD Card access
    {   //CLK, MISO, MOSI
        {PA_5, PA_6, PA_7},                     //SPI0 Drivers + Screen
        {PB_13, PC_2, PC_3},                    //SPI1 WiFi
        {PB_3,  PB_4,  PB_5},                   //SPI2 SPI3 Header - J74
        {NoPin, NoPin, NoPin},                  //SPI3
        {PE_8, PE_9, PE_7},                     //SPI4 Accelerometer
        {PE_12, NoPin, PE_9},                   //SPI5 RepRapDiscount Display
    },
    8,                                          // Number of drivers
    {PF_14, PF_15, PG_5, PA_0, PG_2, PF_1, PD_4, PE_0}, // enablePins
    {PF_13, PG_0, PF_11, PG_4, PF_9, PC_13, PE_2, PE_6},// stepPins
    {PF_12, PG_1, PG_3, PC_1, PF_10, PF_0, PE_3,
#if STARTUP_DELAY
    NoPin
#else
    PA_14
#endif 
    },	// dirPins
#if HAS_SMART_DRIVERS
    {PC_4, PD_11, PC_6, PC_7, PF_2, PE_4, PE_1, PD_3},  // uartpins
    8,                                          // Smart drivers
#endif
    0,                                          // digiPot Factor
#if HAS_VOLTAGE_MONITOR
    NoPin,
#endif
    NoPin,
#if HAS_SBC_INTERFACE
    PD_10, PB_12, SSP2,
#endif
};
constexpr PinEntry PinTable_BTT_OCTOPUSPRO[] =
{
    //Thermistors
    {PF_3, "bedtemp,tb"},
    {PF_4, "e0temp,th0"},
    {PF_5, "e1temp,th1"},
    {PF_6, "e2temp,th2"},
    {PF_7, "e3temp,th3"},

    //Endstops
    {PG_6, "xstop"},
    {PG_9, "ystop"},
    {PG_10, "zstop"},
    {PG_11, "e0stop,e0det"},
    {PG_12, "e1stop,e1det"},
    {PG_13, "e2stop,e2det"},
    {PG_14, "e3stop,e3det"},
    {PG_15, "e4stop,e4det"},

    //Servos
    {PB_6,  "servo0" },

    //Probe
    {PB_7, "probe"},
    {PC_5, "probe1"},

    //Heaters and Fans (Big and Small Mosfets}
    {PA_1,  "bed,hbed" },
    {PA_2,  "e0heat,heat0" },
    {PA_3,  "e1heat,heat1" },
    {PB_10, "e2heat,heat2" },
    {PB_11, "e3heat,heat3" },
    {PA_8,  "fan0,fan" },
    {PE_5,  "fan1" },
    {PD_12, "fan2" },
    {PD_13, "fan3" },
    {PD_14, "fan4" },
    {PD_15, "fan5" },

    //EXP1
    {PE_15, "LCDD7"},
    {PE_13, "LCDD5"},
    {PE_10, "LCDRS"},
    {PE_7, "BTNENC"},
    {PE_8, "BEEP"},
    {PE_9, "LCDEN"},
    {PE_12, "LCDD4"},
    {PE_14, "LCDD6"},

    //EXP2
    {PC_15, "LCDCD"},
    {PB_2, "BTNEN2"},
    {PB_1, "BTNEN1"},
    {PA_6, "LCDMISO"},
    {PA_5, "LCDSCK"},
    {PA_4, "LCDSS"},
    {PA_7, "LCDMOSI"},

    //Neopixel
    {PB_0, "Neopixel"},

    //TFT
    {PA_9, "TX1,tfttx"},
    {PA_10, "RX1,tftrx"},

    //SPI
    {PC_4, "XCS"},
    {PD_11, "YCS"},
    {PC_6, "ZCS"},
    {PC_7, "E0CS"},
    {PF_2, "E1CS"},
    {PE_4, "E2CS"},
    {PE_1, "E3CS"},
    {PD_3, "E4CS"},
    {PA_6, "MISO"},
    {PA_7, "MOSI"},
    {PA_5, "SCK"},

    //I2C
    {PB_8, "SCL1"},
    {PB_9, "SDA1"},

    //PSON
    {PE_11, "PSON"},

    //PWRDET
    {PC_0, "PWRDET"},

    //Status LED
    {PA_13, "LED,status"},
};

constexpr BoardDefaults btt_octopuspro_Defaults = {
    {0x5e29d842},                               // Signatures
    SD_SDIO,                                    // SD Card access
    {   //CLK, MISO, MOSI
        {PA_5, PA_6, PA_7},                     //SPI0 Drivers + Screen + PT100
        {PB_13, PC_2, PC_3},                    //SPI1 WiFi
        {PB_3,  PB_4,  PB_5},                   //SPI2 SPI3 Header - J74
        {NoPin, NoPin, NoPin},                  //SPI3
        {PE_8, PE_9, PE_7},                     //SPI4 Accelerometer
        {PE_12, NoPin, PE_9},                   //SPI5 RepRapDiscount Display
    },
    8,                                          // Number of drivers
    {PF_14, PF_15, PG_5, PA_0, PG_2, PF_1, PD_4, PE_0}, // enablePins
    {PF_13, PG_0, PF_11, PG_4, PF_9, PC_13, PE_2, PE_6},// stepPins
    {PF_12, PG_1, PG_3, PC_1, PF_10, PF_0, PE_3,
#if STARTUP_DELAY
    NoPin
#else
    PA_14
#endif 
    },  // dirPins
#if HAS_SMART_DRIVERS
    {PC_4, PD_11, PC_6, PC_7, PF_2, PE_4, PE_1, PD_3},  // uartpins
    8,                                          // Smart drivers
#endif
    0,                                          // digiPot Factor
#if HAS_VOLTAGE_MONITOR
    NoPin,
#endif
    NoPin,
#if HAS_SBC_INTERFACE
    PD_10, PB_12, SSP2,
#endif
};

constexpr PinEntry PinTable_BIQU_OCTOPUS_X7[] =
{
    //Thermistors
    {PF_3, "e0temp,th0"},
    {PA_0, "bedtemp,tb"},

    //Endstops
    {PF_2, "xstop"},
    {PC_15, "ystop"},
    {PC_13, "zstop"},
    {PC_14, "e0stop,e0det"},
    {PF_4, "probe,PS1"},
	{PG_7, "pro_sw,PS"},

    //Heaters and Fans (Big and Small Mosfets}
    {PD_12,  "bed" },
    {PB_9,  "e0heat" },
    {PB_8,  "fan0,fan" },
    {PB_7,  "fan1" },
    {PE_5,  "fan2" },
	{PC_6,  "fan3" },
	{PE_6,  "fan4" },

    //EXP1
    {PG_5, "BEEP"},
    {PG_4, "BTNENC"},
    {PG_3, "LCDEN"},
    {PG_2, "LCDRS"},
    {PD_15, "LCDD4"},
    {PD_14, "LCDD5"},
    {PD_13, "LCDD6"},
    {PD_11, "LCDD7"},

    //EXP2
    {PB_14, "LCDMISO"},
    {PB_13, "LCDSCK"},
    {PD_10, "BTNEN1"},
    {PB_12, "LCDSS"},
    {PD_9, "BTNEN2"},
    {PB_15, "LCDMOSI"},
    {PD_8, "LCDCD"},
	
	//DIAGNOSTIC LED
	{PC_7, "LED,status"},
	
	//DIAG PIN
	{PE_13, "diag4"},
	{PF_9, "diag5"},
	{PF_15, "diag6"},

	//I2C
    {PF_1, "SCL1"},
    {PF_0, "SDA1"},

};

constexpr BoardDefaults biquoctopus_x7_Defaults = {
    {0x5e29d842, },                    			// Signatures
    SD_SDIO,                                  // SD Card access
    {   //CLK, MISO, MOSI
        {PA_5, PA_6, PA_7},                     //SPI0 Drivers
        {PB_13, PB_14, PB_15},                  //SPI1 EXP2
        {PB_3, PB_4, PB_5},                     //SPI2 WiFi
        {NoPin, NoPin, NoPin},                  //SPI3
        {NoPin, NoPin, NoPin},                  //SPI4
        {NoPin, NoPin, NoPin},                  //SPI5
    },
    7,                                          // Number of drivers
    {PA_1, PC_1, PC_5, PE_10, PE_15, PF_12, PG_1},     //enablePins
    {PA_3, PC_2, PB_0, PE_11, PB_10, PF_13, PE_7}, //stepPins
    {PA_4, PC_3, PB_1, PE_12, PB_11, PF_14, PE_8},      //dirPins
#if HAS_SMART_DRIVERS
    {PA_2, PC_0, PC_4, PE_9, PE_14, PF_11, PG_0},      //uartPins
    7,                                          // Smart drivers
#endif
    0,                                          //digiPot Factor
#if HAS_VOLTAGE_MONITOR
    NoPin,
#endif
    NoPin,
#if HAS_SBC_INTERFACE
    NoPin, NoPin, SSPNONE,
#endif
};

#endif
#endif
