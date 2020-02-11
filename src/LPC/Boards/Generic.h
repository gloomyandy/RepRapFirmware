#ifndef GENERIC_H
#define GENERIC_H

#include "../PINS_LPC.h"


// List of assignable pins and their mapping from names to MPU ports. This is indexed by logical pin number.
// The names must match user input that has been concerted to lowercase and had _ and - characters stripped out.
// Aliases are separate by the , character.
// If a pin name is prefixed by ! then this means the pin is hardware inverted. The same pin may have names for both the inverted and non-inverted cases,
// for example the inverted heater pins on the expansion connector are available as non-inverted servo pins on a DFueX.

//assumes user are selecting correct pins, and correctly setup etc so we will set most pins as rwpwm (since any pin can do PWM on a timer as long as there are free slots available)
constexpr PinEntry PinTable_Generic[] =
{
    {P0_0,  PinCapability::rwpwm, "0.0"},
    {P0_1,  PinCapability::rwpwm, "0.1"},
    {P0_2,  PinCapability::ainrwpwm, "0.2"},
    {P0_3,  PinCapability::ainrwpwm, "0.3"},
    {P0_4,  PinCapability::rwpwm, "0.4"},
    {P0_5,  PinCapability::rwpwm, "0.5"},
    //{P0_6,  PinCapability::rwpwm, "0.6"}, //Internal SD_CS
    //{P0_7,  PinCapability::rwpwm, "0.7"}, //Internal SD_SCK
    //{P0_8,  PinCapability::rwpwm, "0.8"}, //Internal SD_MISO
    //{P0_9,  PinCapability::rwpwm, "0.9"}, //Internal SD_MOSI
    {P0_10, PinCapability::rwpwm, "0.10"},
    {P0_11, PinCapability::rwpwm, "0.11"},
    //{P0_12, PinCapability::rwpwm, "0.12"},  //N/A
    //{P0_13, PinCapability::rwpwm, "0.13"},  //N/A
    //{P0_14, PinCapability::rwpwm, "0.14"},  //N/A
    //{P0_15, PinCapability::rwpwm, "0.15"}, //SSP0_SCK
    {P0_16, PinCapability::rwpwm, "0.16"},
    //{P0_17, PinCapability::rwpwm, "0.17"}, //SSP0_MISO
    //{P0_18, PinCapability::rwpwm, "0.18"}, //SSP0_MOSI
    {P0_19, PinCapability::rwpwm, "0.19"},
    {P0_20, PinCapability::rwpwm, "0.20"},
    {P0_21, PinCapability::rwpwm, "0.21"},
    {P0_22, PinCapability::rwpwm, "0.22"},
    {P0_23, PinCapability::ainrwpwm, "0.23"},
    {P0_24, PinCapability::ainrwpwm, "0.24"},
    {P0_25, PinCapability::ainrwpwm, "0.25"},
    {P0_26, PinCapability::ainrwpwm, "0.26"},
    {P0_27, PinCapability::rwpwm, "0.27"},
    {P0_28, PinCapability::rwpwm, "0.28"},
    //{P0_29, PinCapability::rwpwm, "0.29"},    //USB D+
    //{P0_30, PinCapability::ainrwpwm, "0.30"}, //USB D-
    //{P0_31, PinCapability::ainrwpwm, "0.31"}, //N/A
    
    //Pins P1.[0,1,4,8,9,10,14,15,16,17] used by Ethernet
    {P1_0,  PinCapability::rwpwm, "1.0"},  //ENET_TXD0
    {P1_1,  PinCapability::rwpwm, "1.1"},  //ENET_TXD1
    {P1_2,  PinCapability::rwpwm, "1.2"},  //N/A
    {P1_3,  PinCapability::rwpwm, "1.3"},  //N/A
    {P1_4,  PinCapability::rwpwm, "1.4"},  //ENET_TX_EN
    {P1_5,  PinCapability::rwpwm, "1.5"},  //N/A
    {P1_6,  PinCapability::rwpwm, "1.6"},  //N/A
    {P1_7,  PinCapability::rwpwm, "1.7"},  //N/A
    {P1_8,  PinCapability::rwpwm, "1.8"},  //ENET_CRS
    {P1_9,  PinCapability::rwpwm, "1.9"},  //ENET_RXD0
    {P1_10, PinCapability::rwpwm, "1.10"},  //ENET_RXD1
    {P1_11, PinCapability::rwpwm, "1.11"},  //N/A
    {P1_12, PinCapability::rwpwm, "1.12"},  //N/A
    {P1_13, PinCapability::rwpwm, "1.13"},  //N/A
    {P1_14, PinCapability::rwpwm, "1.14"},  //ENET_RX_ER
    {P1_15, PinCapability::rwpwm, "1.15"},  //ENET_REF_CLK
    {P1_16, PinCapability::rwpwm, "1.16"},  //ENET_MDC
    {P1_17, PinCapability::rwpwm, "1.17"},  //ENET_MDIO
    {P1_18, PinCapability::rwpwm, "1.18"},
    {P1_19, PinCapability::rwpwm, "1.19"},
    {P1_20, PinCapability::rwpwm, "1.20"},
    {P1_21, PinCapability::rwpwm, "1.21"},
    {P1_22, PinCapability::rwpwm, "1.22"},
    {P1_23, PinCapability::rwpwm, "1.23"},
    {P1_24, PinCapability::rwpwm, "1.24"},
    {P1_25, PinCapability::rwpwm, "1.25"},
    {P1_26, PinCapability::rwpwm, "1.26"},
    {P1_27, PinCapability::rwpwm, "1.27"},
    {P1_28, PinCapability::rwpwm, "1.28"},
    {P1_29, PinCapability::rwpwm, "1.29"},
    {P1_30, PinCapability::rwpwm, "1.30"},
    {P1_31, PinCapability::rwpwm, "1.31"},
    {P2_0,  PinCapability::rwpwm, "2.0"},
    {P2_1,  PinCapability::rwpwm, "2.1"},
    {P2_2,  PinCapability::rwpwm, "2.2"},
    {P2_3,  PinCapability::rwpwm, "2.3"},
    {P2_4,  PinCapability::rwpwm, "2.4"},
    {P2_5,  PinCapability::rwpwm, "2.5"},
    {P2_6,  PinCapability::rwpwm, "2.6"},
    {P2_7,  PinCapability::rwpwm, "2.7"},
    {P2_8,  PinCapability::rwpwm, "2.8"},
    //{P2_9,  PinCapability::rwpwm, "2.9"}, //USB Connect
    {P2_10, PinCapability::rwpwm, "2.10"},
    {P2_11, PinCapability::rwpwm, "2.11"},
    {P2_12, PinCapability::rwpwm, "2.12"},
    {P2_13, PinCapability::rwpwm, "2.13"},
    //Pins 2.14-2.31 are N/A
    //{P2_14, PinCapability::rwpwm, "2.14"},
    //{P2_15, PinCapability::rwpwm, "2.15"},
    //{P2_16, PinCapability::rwpwm, "2.16"},
    //{P2_17, PinCapability::rwpwm, "2.17"},
    //{P2_18, PinCapability::rwpwm, "2.18"},
    //{P2_19, PinCapability::rwpwm, "2.19"},
    //{P2_20, PinCapability::rwpwm, "2.20"},
    //{P2_21, PinCapability::rwpwm, "2.21"},
    //{P2_22, PinCapability::rwpwm, "2.22"},
    //{P2_23, PinCapability::rwpwm, "2.23"},
    //{P2_24, PinCapability::rwpwm, "2.24"},
    //{P2_25, PinCapability::rwpwm, "2.25"},
    //{P2_26, PinCapability::rwpwm, "2.26"},
    //{P2_27, PinCapability::rwpwm, "2.27"},
    //{P2_28, PinCapability::rwpwm, "2.28"},
    //{P2_29, PinCapability::rwpwm, "2.29"},
    //{P2_30, PinCapability::rwpwm, "2.30"},
    //{P2_31, PinCapability::rwpwm, "2.31"},
    //Pins 3.0-3.24 are N/A
    //{P3_0,  PinCapability::rwpwm, "3.0"},
    //{P3_1,  PinCapability::rwpwm, "3.1"},
    //{P3_2,  PinCapability::rwpwm, "3.2"},
    //{P3_3,  PinCapability::rwpwm, "3.3"},
    //{P3_4,  PinCapability::rwpwm, "3.4"},
    //{P3_5,  PinCapability::rwpwm, "3.5"},
    //{P3_6,  PinCapability::rwpwm, "3.6"},
    //{P3_7,  PinCapability::rwpwm, "3.7"},
    //{P3_8,  PinCapability::rwpwm, "3.8"},
    //{P3_9,  PinCapability::rwpwm, "3.9"},
    //{P3_10, PinCapability::rwpwm, "3.10"},
    //{P3_11, PinCapability::rwpwm, "3.11"},
    //{P3_12, PinCapability::rwpwm, "3.12"},
    //{P3_13, PinCapability::rwpwm, "3.13"},
    //{P3_14, PinCapability::rwpwm, "3.14"},
    //{P3_15, PinCapability::rwpwm, "3.15"},
    //{P3_16, PinCapability::rwpwm, "3.16"},
    //{P3_17, PinCapability::rwpwm, "3.17"},
    //{P3_18, PinCapability::rwpwm, "3.18"},
    //{P3_19, PinCapability::rwpwm, "3.19"},
    //{P3_20, PinCapability::rwpwm, "3.20"},
    //{P3_21, PinCapability::rwpwm, "3.21"},
    //{P3_22, PinCapability::rwpwm, "3.22"},
    //{P3_23, PinCapability::rwpwm, "3.23"},
    //{P3_24, PinCapability::rwpwm, "3.24"},
    {P3_25, PinCapability::rwpwm, "3.25"},
    {P3_26, PinCapability::rwpwm, "3.26"},
    //pins 3.27-3.31 are N/A
    //{P3_27, PinCapability::rwpwm, "3.27"},
    //{P3_28, PinCapability::rwpwm, "3.28"},
    //{P3_29, PinCapability::rwpwm, "3.29"},
    //{P3_30, PinCapability::rwpwm, "3.30"},
    //{P3_31, PinCapability::rwpwm, "3.31"},
    //Pins 4.0-4.27 are N/A
    //{P4_0,  PinCapability::rwpwm, "4.0"},
    //{P4_1,  PinCapability::rwpwm, "4.1"},
    //{P4_2,  PinCapability::rwpwm, "4.2"},
    //{P4_3,  PinCapability::rwpwm, "4.3"},
    //{P4_4,  PinCapability::rwpwm, "4.4"},
    //{P4_5,  PinCapability::rwpwm, "4.5"},
    //{P4_6,  PinCapability::rwpwm, "4.6"},
    //{P4_7,  PinCapability::rwpwm, "4.7"},
    //{P4_8,  PinCapability::rwpwm, "4.8"},
    //{P4_9,  PinCapability::rwpwm, "4.9"},
    //{P4_10, PinCapability::rwpwm, "4.10"},
    //{P4_11, PinCapability::rwpwm, "4.11"},
    //{P4_12, PinCapability::rwpwm, "4.12"},
    //{P4_13, PinCapability::rwpwm, "4.13"},
    //{P4_14, PinCapability::rwpwm, "4.14"},
    //{P4_15, PinCapability::rwpwm, "4.15"},
    //{P4_16, PinCapability::rwpwm, "4.16"},
    //{P4_17, PinCapability::rwpwm, "4.17"},
    //{P4_18, PinCapability::rwpwm, "4.18"},
    //{P4_19, PinCapability::rwpwm, "4.19"},
    //{P4_20, PinCapability::rwpwm, "4.20"},
    //{P4_21, PinCapability::rwpwm, "4.21"},
    //{P4_22, PinCapability::rwpwm, "4.22"},
    //{P4_23, PinCapability::rwpwm, "4.23"},
    //{P4_24, PinCapability::rwpwm, "4.24"},
    //{P4_25, PinCapability::rwpwm, "4.25"},
    //{P4_26, PinCapability::rwpwm, "4.26"},
    //{P4_27, PinCapability::rwpwm, "4.27"},
    {P4_28, PinCapability::rwpwm, "4.28"},
    {P4_29, PinCapability::rwpwm, "4.29"},
    //Pins 4.30-4.31 are N/A
    //{P4_30, PinCapability::rwpwm, "4.30"},
    //{P4_31, PinCapability::rwpwm, "4.31"},
    
    
};


//NoPin for Generic, Actual pins must be specified in /sys/Board.txt 
constexpr BoardDefaults genericDefaults = {
    {NoPin, NoPin, NoPin, NoPin, NoPin},    //enablePins
    {NoPin, NoPin, NoPin, NoPin, NoPin},    //stepPins
    {NoPin, NoPin, NoPin, NoPin, NoPin},    //dirPins
#if LPC_TMC_SOFT_UART
    {NoPin, NoPin, NoPin, NoPin, NoPin},    //uartPins
#endif
    0    ,                                  //digiPot Factor    
};

#endif
