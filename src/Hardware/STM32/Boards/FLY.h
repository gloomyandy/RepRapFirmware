#ifndef FLY_H
#define FLY_H

#include "../Pins_STM32.h"
// List of assignable pins and their mapping from names to MPU ports. This is indexed by logical pin number.
// The names must match user input that has been converted to lowercase and had _ and - characters stripped out.
// Aliases are separate by the , character.
// If a pin name is prefixed by ! then this means the pin is hardware inverted. The same pin may have names for both the inverted and non-inverted cases,
// for example the inverted heater pins on the expansion connector are available as non-inverted servo pins on a DFueX.

#if STM32H7
constexpr PinEntry PinTable_FLY_SUPER5[] =
{
    //Thermistors
    {PC_4, "ADC0"},
    {PC_5, "ADC1"},
    {PB_1, "ADC2"},
	

    //Endstops
    {PB_7, "IO0"},
    {PC_7, "IO1"},
    {PC_6, "IO2"},
    {PC_13, "IO3"},
    {PC_14, "IO4"},
    {PC_15, "IO5"},
    {PB_2, "NEOPIXEL"},
 
    // Servo
    {PE_6,  "SERVO0" },

    // Probe
    {PC_2, "probe"},

    //Heaters and Fans (Big and Small Mosfets}
    {PD_12,  "heat0" },
    {PD_13,  "heat1" },
    {PB_0,  "bedout,bed" }, 
    {PA_0,  "fan0" },
    {PA_1,  "fan1" },
    {PA_2,  "fan2" },
    {PA_3,  "fan3" },

    
    //EXP1
    
    
    
    
    {PA_8, "BEEP"},    {PA_13, "BTNENC"},
    {PA_15, "LCDEN"}, {PA_14, "LCDRS"},
    {PD_0, "LCDD4"},  {PD_1, "LCDD5"},
    {PD_4, "LCDD6"},  {PD_3, "LCDD7"},
	//GND 

    //EXP2
    
    
    
    {PB_4, "LCDMISO"},{PB_3, "LCDSCK"},
    {PD_7, "BTNEN2"}, {PB_6, "LCDSS"},
    {PD_6, "BTNEN1"}, {PB_5, "LCDMOSI"},
    {PD_5, "LCDCD"},

};

constexpr BoardDefaults fly_super5_Defaults = {
# if STM32H743xx
    {0x32757276, 0x2fe7ccfb},                   // Signatures
# elif STM32H723xx
    {0x69632a35},                               // Signatures
# endif
    SD_SDIO,                                    // SD Card access
    {   //CLK, MISO, MOSI
        {PA_5, PA_6, PA_7},                     //SPI0
        {PB_13, PB_14, PB_15},                  //SPI1
        {PB_3, PB_4, PB_5},                     //SPI2
        {NoPin, NoPin, NoPin},                  //SPI3
        {NoPin, NoPin, NoPin},                  //SPI4
        {NoPin, NoPin, NoPin},                  //SPI5
        {NoPin, NoPin, NoPin},                  //SPI6
        {NoPin, NoPin, NoPin},                  //SPI7
        {NoPin, NoPin, NoPin},                  //SPI8
    },
    5,                                          // Number of drivers
    {PB_11, PE_13, PE_9, PC_1, PE_3, NoPin, NoPin, NoPin},    //enablePins
    {PE_14, PE_11, PE_7, PE_4, PE_1, NoPin, NoPin, NoPin},     //stepPins
    {PE_15, PE_12, PE_8, PE_5, PE_2, NoPin, NoPin, NoPin},      //dirPins
#if HAS_SMART_DRIVERS
    {PB_10, PE_10, PA_4, PC_0, PE_0, NoPin, NoPin, NoPin},    //uartPins      
     5,                                         // Smart drivers
#endif
    0,
#if HAS_VOLTAGE_MONITOR
    PC_3,
#endif
    NoPin,
#if HAS_SBC_INTERFACE
    NoPin, NoPin, SSPNONE,
#endif
};

constexpr PinEntry PinTable_FLY_SUPER8H7[] =
{
    //Thermistors
    {PF_4, "ADC0,out0"},
    {PF_5, "ADC1,out1"},
    {PF_9, "ADC2,pwmout2,pwm2,out2"},
    {PF_10, "ADC3,out3"},
    {PC_0, "ADC4,out4"},
    {PC_1, "ADC5,out5"},
 

    //Endstops
    {PG_12, "io0"},
    {PG_11, "io1"},
    {PG_10, "io2"},
    {PG_9, "io3"},
    {PD_7, "io4"},
    {PD_6, "io5"},
    {PA_8, "io6"},
    {PF_8, "in7,pwmout1,pwm1,out1"},
    {PF_3, "hvin"},
    // Servo
    {PC_6,  "pwmout0,pwm0,servo" },

   // Probe
    {PC_3, "probe"},

    //Heaters and Fans (Big and Small Mosfets}
    {PB_0,  "heat0" },
    {PB_1,  "heat1" },
    {PC_7,  "heat2" },
    {PF_7,  "heat3" },
    {PF_6,  "heat4" },
    {PE_5,  "bedout,bed" }, 
    {PA_0,  "fan0" },
    {PA_1,  "fan1" },
    {PA_2,  "fan2" },
    {PA_3,  "fan3" },
    {PA_15,  "fan4" },
    {PB_11,  "fan5" },
    {PB_10,  "fan6" },
    {PD_12,  "fan7" },
    {PD_14,  "fan8" },
    {PD_15,  "fan9" },
    
	// UART
	{PA_9, "TX1"},
	{PA_10, "RX1"},

    //EXP1
    {PG_13, "LCDD7"},
    {PC_13, "LCDD5"},
    {PG_8, "LCDRS"},
    {PE_13, "BTNENC"},
    {PE_12, "BEEP"},
    {PB_2, "LCDEN"},
    {PC_14, "LCDD4"},
    {PG_14, "LCDD6"},

    //EXP2
    {PG_15, "LCDCD"},
    {PB_6, "BTNEN2"},
    {PB_7, "BTNEN1"},
    {PA_6, "LCDMISO"},
    {PA_5, "LCDSCK"},
    {PA_4, "LCDSS"},
    {PA_7, "LCDMOSI"},


};

constexpr BoardDefaults fly_super8h7_Defaults = {
# if STM32H743xx
    {0x32757276},                               // Signatures
# elif STM32H723xx
    {0x69632a35},                               // Signatures
#endif
    SD_SDIO,                                    // SD Card access
    {   //CLK, MISO, MOSI
        {PA_5, PA_6, PA_7},                     //SPI0
        {PB_13, PB_14, PB_15},                  //SPI1
        {PB_3, PB_4, PB_5},                     //SPI2
        {PD_3, PD_4, PD_5},                     //SPI3
        {NoPin, NoPin, NoPin},                  //SPI4
        {NoPin, NoPin, NoPin},                  //SPI5
        {NoPin, NoPin, NoPin},                  //SPI6
        {NoPin, NoPin, NoPin},                  //SPI7
        {NoPin, NoPin, NoPin},                  //SPI8

    },
	8,											// Number of drivers
    {PF_11, PF_14, PG_1, PE_9, PF_2, PC_15, PG_4, PG_7},   	//enablePins
    {PE_2, PE_3, PE_4, PE_14, PE_15, PE_1, PE_0, PE_6},	    //stepPins
    {PC_5, PF_13, PG_0, PE_8, PE_11, PF_0, PG_3, PG_6},    	//dirPins
#if HAS_SMART_DRIVERS
    {PC_4, PF_12, PF_15, PE_7, PE_10, PF_1, PG_2, PG_5},      //uartPins      
	 8,                                       	// Smart drivers
#endif
    0,
#if HAS_VOLTAGE_MONITOR
    PC_2,
#endif
    NoPin,
#if HAS_SBC_INTERFACE
    NoPin, NoPin, SSPNONE,
#endif
};

#endif
#if STM32F4

constexpr PinEntry PinTable_FLY_F407ZG[] =
{
    //Thermistors
    {PA_0, "e0temp,t0"},
    {PC_1, "e1temp,t1"},
    {PC_0, "e2temp,t2"},
    {PF_10, "e3temp,t3"},
    {PF_5, "e4temp,t4"},
    {PF_4, "e5temp,t5"},
    {PF_3, "bedtemp,tb"},

    //Endstops
    {PC_3, "xmin,xstop"},
    {PC_2, "xmax,xstopmax"},
    {PF_2, "ymin,ystop"},
    {PF_1, "ymax,ystopmax"},
    {PF_0, "zmin,zstop"},
    {PC_15, "zmax,zstopmax"},
    {PC_14, "z3"},
    {PA_3, "dljc"},
    
    //Heaters and Fans (Big and Small Mosfets}
    {PE_2,  "bed,hbed" },
    {PF_7,  "e0heat,he0" },
    {PF_6,  "e1heat,he1" },
    {PE_6,  "e2heat,he2" },
    {PE_5,  "e3heat,he3" },
    {PE_4,  "e4heat,he4" },
    {PE_3,  "e5heat,he5" },
    
    {PF_8,  "fan0,fan" },
    {PF_9,  "fan1" },
    {PA_2,  "fan2" },
    {PA_1,  "fan3" },
    {PE_13,  "fan4" },
    {PB_11,  "fan5" },

    // Servo
    {PE_11,  "servo0" },

    //EXP1
    {PB_10, "BEEP"},
    {PE_14, "LCDEN"},
    {PE_10, "LCDD4"},
    {PE_8, "LCDD6"},
    {PE_15, "BTNENC"},
    {PE_12, "LCDRS"},
    {PE_9, "LCDD5"},
    {PE_7, "LCDD7"},

    //EXP2
    {PB_14, "LCDMISO"},
    {PB_13, "LCDSCK"},
    {PC_5, "BTNEN1"},
    {PC_4, "BTNEN2"},
    {PF_11, "LCDSS"},
    {PB_15, "LCDMOSI"},
    {PB_2, "LCDCD"},

    //SD
    {PC_13, "SDCD"},
    {PC_9, "SDD1"},
    {PC_8, "SDD0"},
    {PC_12, "SDSCK"},
    {PD_2, "SDCMD"},
    {PC_11, "SDD3"},
    {PC_10, "SDD2"},

    // UART
    {PA_9, "TX1"},
    {PA_10, "RX1"},
};

constexpr BoardDefaults fly_f407zg_Defaults = {
    {0x8a5f5551, 0xd0c680ae},                   // Signatures
    SD_SDIO,                                    // SD Card access
    {   //CLK, MISO, MOSI
        {PA_5, PA_6, PA_7},                     //SPI0
        {PB_13, PB_14, PB_15},                  //SPI1
        {NoPin, NoPin, NoPin},                  //SPI2
        {PE_10, NoPin, PE_14},                  //SPI3
        {NoPin, NoPin, NoPin},                  //SPI4
        {NoPin, NoPin, NoPin},                  //SPI5
    },
    9,                                          // Number of drivers
    {PE_1, PG_12,  PD_7, PD_4,  PD_0, PG_8, PG_5, PG_2, PD_9},      //enablePins
    {PB_9, PB_8, PA_8, PC_7,  PC_6, PD_15, PD_14, PD_13, PD_12},    //stepPins
    {PE_0, PG_11, PD_6, PD_3,  PA_15, PG_7, PG_4, PD_11, PD_8},     //dirPins
#if HAS_SMART_DRIVERS
    {PG_13, PG_10, PD_5, PD_1,
#if STARTUP_DELAY
    // Avoid clash with jtag pins
    NoPin,
#else
    PA_14,
#endif
     PG_6, PG_3, PD_10, PB_12},                 //uartPins
    9,                                          // Smart drivers
#endif
    0,                                          //digiPot Factor
#if HAS_VOLTAGE_MONITOR
    NoPin,
#endif
    NoPin,
#if HAS_SBC_INTERFACE
    PB_2, PF_11, SSP2,
#endif
};


constexpr PinEntry PinTable_FLY_E3[] =
{
    //Thermistors
    {PA_4, "e0temp,t0"},
    {PA_3, "bedtemp,tb"},

    //Endstops
    {PA_2, "xmin,xstop"},
    {PA_1, "ymin,ystop"},
    {PC_5, "zmin,zstop"},

    // Servo
    {PB_0,  "servo0" },

   // Porbe
    {PC_4, "probe"},
    
    //Heaters and Fans (Big and Small Mosfets}
    {PA_0,  "bed,hbed" },
    {PA_5,  "e0heat,he0" },
    {PA_7,  "fan0,fan" },
    {PA_6,  "fan1" },

    //EXP1
    {PE_12, "BEEP"},
    {PE_11, "BTNENC"},
    {PE_10, "LCDEN"},
    {PE_9, "LCDD4"},
    {PE_8, "LCDD5"},
    {PE_7, "LCDD6"},
    {PB_1, "LCDD7"},

    //SD
    {PA_15, "SDCD"},
    {PC_9, "SDD1"},
    {PC_8, "SDD0"},
    {PC_12, "SDSCK"},
    {PD_2, "SDCMD"},
    {PC_11, "SDD3"},
    {PC_10, "SDD2"},

    //FPC
    {PB_10, "FPC1"},
    {PB_11, "FPC2"},
    {PD_10, "FPC3"},
    {PD_11, "FPC4"},
    {PD_12, "FPC5"},
    {PD_13, "FPC6"},
    {PD_14, "FPC7"},
    {PD_15, "FPC8"},
    {PC_6, "FPC9"},
    {PD_0, "FPC10"},
    {PD_1, "FPC11"},

};

constexpr BoardDefaults fly_e3_Defaults = {
    {0xd0c680ae, 0xfd2146b0},                   // Signatures
    SD_SDIO,                                    // SD Card access
    {   //CLK, MISO, MOSI
        {PA_5, PA_6, PA_7},                     //SPI0
        {PB_13, PB_14, PB_15},                  //SPI1
        {NoPin, NoPin, NoPin},                  //SPI2
        {PC_2, PC_0, PC_1},                     //SPI3
        {PE_8, NoPin, PB_1},                    //SPI4
        {PD_6, PD_4, PD_5},                     //SPI5
    },
    4,                                          // Number of drivers
    {PC_15, PE_5, PE_1, PB_5},                  //enablePins
    {PC_13, PE_3, PB_9, PB_3},                  //stepPins
    {PC_14, PE_4, PE_0, PB_4},                  //dirPins
#if HAS_SMART_DRIVERS
    {PE_6, PE_2, PB_8, PD_7},                   //uartPins            
    4,                                          // Smart drivers
#endif
    0, 
#if HAS_VOLTAGE_MONITOR
    NoPin,
#endif
    NoPin,
#if HAS_SBC_INTERFACE
    NoPin, NoPin, SSPNONE,
#endif
};


constexpr PinEntry PinTable_FLY_CDYV2[] =
{
    //Thermistors
    {PA_3, "e0temp,t0"},
    {PC_4, "e1temp,t1"},
    {PC_5, "e2temp,t2"},
    {PB_1, "bedtemp,tb"},

    //Endstops
    {PC_7, "xmin,xstop"},
    {PC_6, "xmax,xstopmax"},
    {PD_11, "ymin,ystop"},
    {PD_10, "ymax,ystopmax"},
    {PB_10, "zmin,zstop"},
    {PB_11, "zmax,zstopmax"},

    // Servo
    {PE_6,  "servo0" },

    // Probe
    {PC_2, "probe"},
    
    //Heaters and Fans (Big and Small Mosfets}
    {PB_0,  "bed,hbed" },
    {PD_12,  "e0heat,he0" },
    {PD_13,  "e1heat,he1" },
    {PD_14,  "e2heat,he2" },
    {PA_0,  "fan0,fan" },
    {PA_1,  "fan1" },
    {PA_2,  "fan2" },

    //EXP1
    {PA_14, "LCDD7"},
    {PA_13, "LCDD6"},
    {PE_15, "LCDD4"},
    {PE_14, "LCDD5"},
    {PB_2, "LCDEN"},
    {PE_13, "LCDCD"},

    // UART
    {PA_9, "TX1,BTNENC"},
    {PA_10, "RX1,LCDRS"},

    // WIFI UART
    {PD_8, "BTNEN1"},
    {PD_9, "BTNEN2"},

    //spi 1  (SD,MAX311865,MAX6675)
    {PA_4, "SPI1SDCS,LCDSS"},
    {PA_5, "SPI1SCK,LCDSCK"},
    {PA_6, "SPI1MISO,LCDMISO"},
    {PA_7, "SPI1MOSI,LCDMOSI"},
    {PE_7, "SPI1CS1"},
    {PE_8, "SPI1CS2"},

    //spi 3  (MOT TMC2130 TMC5160)
    {PB_3, "MOTSCK"},
    {PB_4, "MOTMISO"},
    {PB_5, "MOTMOSI"},

    //Signal light   Neopixel
    {PD_15, "Neopixel,LED"},

    //Laser 
    {PB_9, "Laser"},
    
    //Power ADC
    {PC_3, "PowerADC"}
};

constexpr BoardDefaults fly_cdyv2_Defaults = {
    {0x8a5f5551, 0xd0c680ae},                   // Signatures
    SD_SDIO,                                    // SD Card access
    {   //CLK, MISO, MOSI
        {PA_5, PA_6, PA_7},                     //SPI0
        {PB_13, PB_14, PB_15},                  //SPI1
        {PB_3, PB_4, PB_5},                     //SPI2
        {PE_15, NoPin, PB_2},                   //SPI3
        {NoPin, NoPin, NoPin},                  //SPI4
        {NoPin, NoPin, NoPin},                  //SPI5
    },
    6,                                          // Number of drivers
    {PC_1, PC_14, PB_8, PD_7, PD_4, PD_0},      //enablePins
    {PE_5, PE_4, PE_3, PE_2, PE_1, PE_0},       //stepPins
    {PC_0, PC_13, PB_7, PD_6, PD_3, PA_15},     //dirPins
#if HAS_SMART_DRIVERS
    {PC_15, PA_8, PB_6, PD_5, PD_1, PE_9},      //uartPins      
     6,                                         // Smart drivers
#endif
    0,
#if HAS_VOLTAGE_MONITOR
    PC_3,
#endif
    NoPin,
#if HAS_SBC_INTERFACE
    NoPin, NoPin, SSPNONE,
#endif
};

constexpr PinEntry PinTable_FLY_E3_PRO[] =
{

    //Thermistors
    {PA_4, "e0temp,t0"},
    {PA_1, "e1temp,t1"},
    {PA_3, "bedtemp,tb"},
    {PC_1, "boardtemp,tboard"},
    //Endstops
    {PA_2, "xmin,xstop"},
    {PB_10, "ymin,ystop"},
    {PC_4, "zmin,zstop"},
    {PD_0, "e0min,e0stop"},
    {PD_1, "e1min,e1stop"},
    // Servo
    {PB_0,  "servo0" },

   // Porbe
    {PC_5, "probe"},
    
    //Heaters and Fans (Big and Small Mosfets}
    {PA_0,  "bed,hbed" },
    {PA_5,  "e0heat,he0" },
    {PC_6,  "e1heat,he1" },
    {PA_7,  "fan0,fan" },
    {PA_6,  "fan1" },
    {PB_6,  "fan2" },
    {PB_7,  "fan3" },

    //EXP1
    {PE_12, "BEEP"},
    {PE_11, "BTNENC"},
    {PE_10, "LCDEN"},
    {PE_9, "LCDD4"},
    {PE_8, "LCDD5"},
    {PE_7, "LCDD6"},
    {PB_1, "LCDD7"},

    //SD
    {PC_9, "SDD1"},
    {PC_8, "SDD0"},
    {PC_12, "SDSCK"},
    {PD_2, "SDCMD"},
    {PC_11, "SDD3"},
    {PC_10, "SDD2"},

    // UART
    {PA_9, "TX1"},
    {PA_10, "RX1"},

    //Signal light
     {PA_8, "LED"},

    //FPC
    {PB_11, "neopixel"},
    {PD_11, "PSON"},
    {PE_6, "laser"},
    {PC_3, "PowerADC"},
    {PD_10, "pwr, PWRDET"},

};

constexpr BoardDefaults fly_e3_pro_Defaults = {
    {0xd0c680ae},                   // Signatures
    SD_SDIO,                                    // SD Card access
    {   //CLK, MISO, MOSI
        {PA_5, PA_6, PA_7},                     //SPI0
        {PB_13, PB_14, PB_15},                  //SPI1
        {NoPin, NoPin, NoPin},                  //SPI2
        {PC_2, PC_0, PC_1},                     //SPI3
        {PE_8, NoPin, PB_1},                    //SPI4
        {PD_6, PD_4, PD_6},                     //SPI5
    },
    5,                                          // Number of drivers
    {PE_5, PE_1, PB_5, PD_6 ,PC_15},            //enablePins
    {PE_3, PB_9, PB_3, PD_4, PC_13},            //stepPins
    {PE_2, PB_8, PD_7, PD_3, PC_0 },            //dirPins
#if HAS_SMART_DRIVERS
    {PE_4, PE_0, PB_4, PD_5, PC_14},            // uartPins 
     5,                                         // Smart drivers
#endif
    0,  
#if HAS_VOLTAGE_MONITOR
    PC_3,
#endif
    NoPin,
#if HAS_SBC_INTERFACE
    NoPin, NoPin, SSPNONE,
#endif
};

constexpr PinEntry PinTable_FLY_SUPER8[] =
{
    //Thermistors
    {PF_4, "ADC0,out0"},
    {PF_5, "ADC1,out1"},
    {PF_9, "ADC2,pwmout2,pwm2,out2"},
    {PF_10, "ADC3,out3"},
    {PC_0, "ADC4,out4"},
    {PC_1, "ADC5,out5"},
 
    //Endstops
    {PG_12, "io0"},
    {PG_11, "io1"},
    {PG_10, "io2"},
    {PG_9, "io3"},
    {PD_7, "io4"},
    {PD_6, "io5"},
    {PA_8, "io6"},
    {PF_8, "in7,pwmout1,pwm1,out1"},
    {PF_3, "hvin"},
    // Servo
    {PC_6,  "pwmout0,pwm0,servo" },

    // Probe
    {PC_3, "probe"},

    //Heaters and Fans (Big and Small Mosfets}
    {PB_0,  "heat0" },
    {PB_1,  "heat1" },
    {PC_7,  "heat2" },
    {PF_7,  "heat3" },
    {PF_6,  "heat4" },
    {PE_5,  "bedout,bed" }, 
    {PA_0,  "fan0" },
    {PA_1,  "fan1" },
    {PA_2,  "fan2" },
    {PA_3,  "fan3" },
    {PA_15,  "fan4" },
    {PB_11,  "fan5" },
    {PB_10,  "fan6" },
    {PD_12,  "fan7" },
    {PD_14,  "fan8" },
    {PD_15,  "fan9" },
    
    //EXP1
    {PG_13, "LCDD7"},
    {PC_13, "LCDD5"},
    {PG_8, "LCDRS"},
    {PE_13, "BTNENC"},
    {PE_12, "BEEP"},
    {PB_2, "LCDEN"},
    {PC_14, "LCDD4"},
    {PG_14, "LCDD6"},

    //EXP2
    {PG_15, "LCDCD"},
    {PB_6, "BTNEN2"},
    {PB_7, "BTNEN1"},
    {PA_6, "LCDMISO"},
    {PA_5, "LCDSCK"},
    {PA_4, "LCDSS"},
    {PA_7, "LCDMOSI"},

};

constexpr BoardDefaults fly_super8_Defaults = {
    {0x8a5f5551, 0xd0c680ae},                   // Signatures
    SD_SDIO,                                    // SD Card access
    {   //CLK, MISO, MOSI
        {PA_5, PA_6, PA_7},                     //SPI0
        {PB_13, PB_14, PB_15},                  //SPI1
        {PB_3, PB_4, PB_5},                     //SPI2
        {PD_3, PD_4, PD_5},                     //SPI3
        {NoPin, NoPin, NoPin},                  //SPI4
        {NoPin, NoPin, NoPin},                  //SPI5
    },
    8,                                          // Number of drivers
    {PF_11, PF_14, PG_1, PE_9, PF_2, PC_15, PG_4, PG_7},    //enablePins
    {PE_2, PE_3, PE_4, PE_14, PE_15, PE_1, PE_0, PE_6},     //stepPins
    {PC_5, PF_13, PG_0, PE_8, PE_11, PF_0, PG_3, PG_6},     //dirPins
#if HAS_SMART_DRIVERS
    {PC_4, PF_12, PF_15, PE_7, PE_10, PF_1, PG_2, PG_5},    //uartPins      
     8,                                         // Smart drivers
#endif
    0,
#if HAS_VOLTAGE_MONITOR
    PC_2,
#endif
    NoPin,
#if HAS_SBC_INTERFACE
    NoPin, NoPin, SSPNONE,
#endif
};

constexpr PinEntry PinTable_FLY_E3_PROV3[] =
{

    //Thermistors
    {PA_4, "e0temp,t0"},
    {PA_1, "e1temp,t1"},
    {PA_3, "bedtemp,tb"},
    {PC_1, "boardtemp,tboard"},
    
    //Endstops
    {PA_2, "xmin,xstop"},
    {PB_10, "ymin,ystop"},
    {PC_4, "zmin,zstop"},
    {PD_0, "e0min,e0stop"},
    {PD_1, "e1min,e1stop"},
    
    // Servo
    {PB_0,  "servo0" },

    // Porbe
    {PC_5, "probe"},
    
    //Heaters and Fans (Big and Small Mosfets}
    {PA_0,  "bed,hbed" },
    {PA_5,  "e0heat,he0" },
    {PC_6,  "e1heat,he1" },
    {PA_7,  "fan0,fan" },
    {PA_6,  "fan1" },
    {PB_6,  "fan2" },
    {PB_7,  "fan3" },

    //EXP1
    {PE_11, "BEEP"},
    {PE_12, "BTNENC"},
    {PE_10, "LCDD5"},
    {PE_6, "LCDRS"},
    {PE_8, "LCDD7"},
    {PA_14, "LCDEN"},
    {PE_9, "LCDD4"},
    {PE_7, "LCDD6"},
    
    //EXP2
    {PA_13, "LCDCD"},
    {PB_4, "LCDMISO"},
    {PB_3, "LCDSCK"},
    {PB_2, "LCDSS"},
    {PB_5, "LCDMOSI"},

    //SD
    {PC_9, "SDD1"},
    {PC_8, "SDD0"},
    {PC_12, "SDSCK"},
    {PD_2, "SDCMD"},
    {PC_11, "SDD3"},
    {PC_10, "SDD2"},

    // UART
    {PA_9, "TX1,BTNEN2"},
    {PA_10, "RX1,BTNEN1"},

    //FPC
    {PB_11, "neopixel"},
    {PD_11, "PSON"},
    {PB_1, "laser"},
    {PD_10, "pwr, PWRDET"},

};

constexpr BoardDefaults fly_e3_prov3_Defaults = {
    {0xd0c680ae},                   // Signatures
    SD_SDIO,                                    // SD Card access
    {   //CLK, MISO, MOSI
        {PA_5, PA_6, PA_7},                     //SPI0
        {PB_13, PB_14, PB_15},                  //SPI1 Wifi module
        {PB_3, PB_4, PB_5},                     //SPI2 used for Fysetc dual wire display
        {PC_2, PC_0, PC_1},                     //SPI3
        {PE_10, NoPin, PE_8},                   //SPI4 used for single wire ender displays
        {NoPin, NoPin, NoPin},                  //SPI5
    },
    5,                                            // Number of drivers
    {PE_5, PE_1, PC_2, PD_6 ,PC_15},           //enablePins
    {PE_3, PB_9, PA_15, PD_4, PC_13},        //stepPins
    {PE_2, PB_8, PD_7, PD_3, PC_0 },        //dirPins
#if HAS_SMART_DRIVERS
    {PE_4, PE_0, PA_8, PD_5, PC_14},           
     5,                                      //uartPins                                              // Smart drivers
#endif
    0,  
#if HAS_VOLTAGE_MONITOR
    PC_3,
#endif
    NoPin,
#if HAS_SBC_INTERFACE
    NoPin, NoPin, SSPNONE,
#endif
};


constexpr PinEntry PinTable_FLY_GEMINI[] =
{
    //Thermistors
    {PC_0, "e0temp,t0"},
    {PC_2, "bedtemp,tb"},
 
    //Endstops
    {PA_3, "xstop"},
    {PB_1, "ystop"},
    {PB_10, "zstop"},

    // Servo
    {PB_0,  "servo0" },

    // Porbe
    {PA_1, "probe"},

    //Heaters and Fans (Big and Small Mosfets}
    {PA_2,  "bed,hbed" },
    {PA_0,  "e0heat,he0" },
    {PC_6,  "fan0,fan" },

    //SD
    {PC_9, "SDD1"},
    {PC_8, "SDD0"},
    {PC_12, "SDSCK"},
    {PD_2, "SDCMD"},
    {PC_11, "SDD3"},
    {PC_10, "SDD2"},

    // UART
    {PA_9, "TX1,LCDD4"},
    {PA_10, "RX1,LCDRS"},
    
    //EXP1
    {PA_8, "LCDD5"},
    {PA_4, "BTNENC"},
    {PA_13, "LCDEN"},
    
    //EXP2
    {PB_3, "LCDCD"},
    {PA_14, "BTNEN2"},
    {PA_15, "BTNEN1"},
    {PB_14, "LCDMISO"},
    {PB_13, "LCDSCK"},
    {PB_12, "LCDSS"},
    {PB_15, "LCDMOSI"},

};

constexpr BoardDefaults fly_gemini_Defaults = {
    {0xbfecc997},                       // Signatures
    SD_SDIO,                            // SD Card access
    {   //CLK, MISO, MOSI
        {PA_5, PA_6, PA_7},                     //SPI0
        {PB_13, PB_14, PB_15},                  //SPI1
        {NoPin, NoPin, NoPin},                  //SPI2
        {NoPin, NoPin, NoPin},                     //SPI3
        {NoPin, NoPin, NoPin},                    //SPI4
        {NoPin, NoPin, NoPin},                     //SPI5
    },
    4,                                            // Number of drivers
    {PB_2, PB_6, PB_5, PB_4},           //enablePins
    {PC_13, PC_14, PC_15, PC_3},        //stepPins
    {PC_1, PC_4, PC_5, PC_7},           //dirPins
#if HAS_SMART_DRIVERS
    {PB_11, PB_9, PB_8, PB_7},
     4,                                      //uartPins                                              // Smart drivers
#endif
    0,
#if HAS_VOLTAGE_MONITOR
    NoPin,
#endif
    NoPin,
#if HAS_SBC_INTERFACE
    PB_3, PB_12, SSP2,
#endif
};

constexpr PinEntry PinTable_FLY_GEMINI_V1_1[] =
{
    //Thermistors
    {PC_0, "e0temp,t0"},
    {PC_2, "bedtemp,tb"},
 
    //Endstops
    {PA_3, "xstop"},
    {PB_1, "ystop"},
    {PB_10, "zstop"},

    // Servo
    {PB_0,  "servo0" },

    // Porbe
    {PA_1, "probe"},

    //Heaters and Fans (Big and Small Mosfets}
    {PA_2,  "bed,hbed" },
    {PA_0,  "e0heat,he0" },
    {PC_6,  "fan0,fan" },
    {PC_7,  "fan1" },

    // UART
    {PA_9, "TX1,LCDD4"},
    {PA_10, "RX1,LCDRS"},

    //EXP1
    {PA_8, "LCDD5"},
    {PA_4, "BTNENC"},
    {PA_13, "LCDEN"},
    
    //EXP2
    {PB_3, "LCDCD"},
    {PA_14, "BTNEN2"},
    {PA_15, "BTNEN1"},
    {PB_14, "LCDMISO"},
    {PB_13, "LCDSCK"},
    {PB_12, "LCDSS"},
    {PB_15, "LCDMOSI"},

};

constexpr BoardDefaults fly_gemini_v1_1_Defaults = {
    {0x318f4fbe},                         // Signatures
    SD_SPI1_B,                            // SD Card access
    {   //CLK, MISO, MOSI
        {PA_5, PA_6, PA_7},                     //SPI0
        {PB_13, PB_14, PB_15},                  //SPI1
        {PB_3, PB_4, PB_5},                     //SPI2
        {NoPin, NoPin, NoPin},                  //SPI3
        {NoPin, NoPin, NoPin},                  //SPI4
        {NoPin, NoPin, NoPin},                  //SPI5
    },
    4,                                          // Number of drivers
    {PB_2, PD_2, PC_12, PC_11},                   //enablePins
    {PC_13, PC_14, PC_15, PC_3},                //stepPins
    {PC_1, PC_4, PC_5, PC_8},                   //dirPins
#if HAS_SMART_DRIVERS
    {PB_11, PB_9, PB_8, PB_7},
     4,                                         //uartPins                                              // Smart drivers
#endif
    0,
#if HAS_VOLTAGE_MONITOR
    NoPin,
#endif
    NoPin,
#if HAS_SBC_INTERFACE
    PB_3, PB_12, SSP2,
#endif
};

constexpr PinEntry PinTable_FLY_GEMINI_V2_0[] =
{
    //Thermistors
    {PC_0, "e0temp,t0"},
    {PC_2, "bedtemp,tb"},
 
    //Endstops
    {PA_3, "xstop"},
    {PB_1, "ystop"},
    {PB_10, "zstop"},

    // Servo
    {PB_0,  "servo0" },

    // Porbe
    {PA_1, "probe"},

    //Heaters and Fans (Big and Small Mosfets)
    {PA_2,  "bed,hbed" },
    {PA_0,  "e0heat,he0" },
    {PC_6,  "fan0,fan" },
    {PC_7,  "fan1" },

    // UART
    {PA_9, "TX1,LCDD4"},
    {PA_10, "RX1,LCDRS"},

    //EXP1
    {PA_8, "LCDD5"},
    {PB_6, "BTNENC"},
    {PA_13, "LCDEN"},
    {PA_9, "LCDD4"},

    
    //EXP2
    {PC_10, "LCDCD"},
    {PA_14, "BTNEN1"},
    {PA_15, "BTNEN2"},
    {PB_14, "LCDMISO"},
    {PB_13, "LCDSCK"},
    {PB_12, "LCDSS"},
    {PB_15, "LCDMOSI"},

};

constexpr BoardDefaults fly_gemini_v2_0_Defaults = {
    {0x318f4fbe},                         // Signatures
    SD_SPI1_B,                            // SD Card access
    {   //CLK, MISO, MOSI
        {PA_5, PA_6, PA_7},                     //SPI0
        {PB_13, PB_14, PB_15},                  //SPI1
        {PB_3, PB_4, PB_5},                     //SPI2
        {NoPin, NoPin, NoPin},                  //SPI3
        {NoPin, NoPin, NoPin},                  //SPI4
        {NoPin, NoPin, NoPin},                  //SPI5
    },
    4,                                          // Number of drivers
    {PB_2, PD_2, PC_12, PC_11},                   //enablePins
    {PC_13, PC_14, PC_15, PC_3},                //stepPins
    {PC_1, PC_4, PC_5, PC_8},                   //dirPins
#if HAS_SMART_DRIVERS
    {PB_11, PB_9, PB_8, PB_7},
     4,                                         //uartPins                                              // Smart drivers
#endif
    0,
#if HAS_VOLTAGE_MONITOR
    NoPin,
#endif
    NoPin,
#if HAS_SBC_INTERFACE
    PB_3, PB_12, SSP2,
#endif
};

constexpr PinEntry PinTable_FLY_E3_V2[] =
{

    //Thermistors
    {PC_4, "e0temp,t0"},
    {PC_5, "e1temp,t1"},
    {PB_1, "bedtemp,tb"},
 
    
    //Endstops
    {PE_7, "xmin,xstop"},
    {PE_8, "ymin,ystop"},
    {PE_9, "zmin,zstop"},

    
    // Servo
    {PE_6,  "servo0" },

    // Porbe
    {PC_2, "probe"},
    
    //Heaters and Fans (Big and Small Mosfets}
    {PB_0,  "bed,hbed"   },
    {PC_6,  "e0heat,he0" },
    {PC_7,  "e1heat,he1" },
    {PA_0,  "fan0,fan"   },
    {PA_1,  "fan1"      },
    {PA_2,  "fan2"      },
    {PA_3,  "fan3"      },

    //EXP1
    {PD_10,  "BEEP"  },    {PA_9,   "TX1,BTNENC" },
    {PA_8,   "LCDEN" },    {PA_10,  "RX1,LCDRS"  },
    {PE_14,  "LCDD5" },    {PE_15,  "LCDD4"      },
    {PA_14,  "LCDD6" },    {PA_13,  "LCDD7"      },
    
    
    //EXP2
    {PA_6,   "LCDMISO"},     {PA_5,  "LCDSCK"  },
    {PB_11,  "BTNEN2" },     {PA_4,  "LCDSS"   },
    {PB_10,  "BTNEN1" },     {PA_7,  "LCDMOSI" },
    {PE_13,  "TFCD,LCDCD"   },


    //SD
    {PC_9, "SDD1"   },
    {PC_8, "SDD0"   },
    {PC_12, "SDSCK" },
    {PD_2, "SDCMD"  },
    {PC_11, "SDD3"  },
    {PC_10, "SDD2"  },


};

constexpr BoardDefaults fly_e3_v2_Defaults = {
    {0xd0c680ae},                   // Signatures
    SD_SDIO,                                    // SD Card access
    {   //CLK, MISO, MOSI
        {PA_5, PA_6, PA_7},                     //SPI0 EXP SD
        {PB_13, PB_14, PB_15},                  //SPI1 Wifi module
        {PB_3, PB_4, PB_5},                     //SPI2 MOT
        {PD_12, PD_13, PD_11},                  //SPI3 MAX31855 MAX31865
        {PE_14, NoPin, PA_13},                  //SPI4 used for single wire ender displays
        {NoPin, NoPin, NoPin},                  //SPI5
    },
    5,                                            // Number of drivers
    {PC_1, PC_14, PE_3, PD_6 ,PD_3},           //enablePins
    {PE_5, PE_4, PE_1, PE_2, PE_0},        //stepPins
    {PC_0, PC_13, PB_7, PD_5, PD_1 },        //dirPins
#if HAS_SMART_DRIVERS
    {PC_15, PB_6, PD_7, PD_4, PD_0},           
     5,                                      //uartPins                                              // Smart drivers
#endif
    0,  
#if HAS_VOLTAGE_MONITOR
    PC_3,
#endif
    NoPin,
#if HAS_SBC_INTERFACE
    NoPin, NoPin, SSPNONE,
#endif
};

#endif
#endif