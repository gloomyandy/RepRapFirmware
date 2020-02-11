#ifndef REARM_H
#define REARM_H

#include "../PINS_LPC.h"


// List of assignable pins and their mapping from names to MPU ports. This is indexed by logical pin number.
// The names must match user input that has been concerted to lowercase and had _ and - characters stripped out.
// Aliases are separate by the , character.
// If a pin name is prefixed by ! then this means the pin is hardware inverted. The same pin may have names for both the inverted and non-inverted cases,
// for example the inverted heater pins on the expansion connector are available as non-inverted servo pins on a DFueX.


//NOTE:: In general RAMPS silk/official pinout is used as the labels since most of the connections are on the RAMPS shield
constexpr PinEntry PinTable_Rearm[] =
{

    //Thermistors
    {P0_23, PinCapability::ainrw, "t0,a13"},        //Ext0 Therm
    {P0_24, PinCapability::ainrw, "t1,a14"},        //Bed Therm
    {P0_25, PinCapability::ainrw, "t2,a15"},        //Ext1 Therm
    
    //Endstops (although they are labeled x- and x+ on the - and + get stripped out in RRF so we will use i.e. xmin/xmax
    {P1_24, PinCapability::rwpwm, "xmin,d3"},
    {P1_25, PinCapability::rwpwm, "xmax,d2"},
    {P1_26, PinCapability::rwpwm, "ymin,d14"},
    {P1_27, PinCapability::rwpwm, "ymax,d15"},
    {P1_29, PinCapability::rwpwm, "zmin,d18"},
    {P1_28, PinCapability::rwpwm, "zmax,d19"},

    //Heaters and Fans (Mosfets)
    {P2_7, PinCapability::rwpwm, "d8"  },            //HB Mosfet
    {P2_4, PinCapability::rwpwm, "d9" },             //HE2 Mosfet
    {P2_5, PinCapability::rwpwm, "d10" },            //HE1 Mosfet

    //Servos (only max of 3 servos can be configured_
    {P1_20, PinCapability::rwpwm, "servo0,d11"},     //PWM1[2]
    {P1_21, PinCapability::rwpwm, "servo1,d6"},      //PWM1[3]
    {P1_19, PinCapability::rwpwm, "servo2,d5"},
    {P1_18, PinCapability::rwpwm, "servo3,d4"},      //PWM1[1]
    
    //Ramps  AUX1 Pins
    //P0_2 D1 (TXD0)  (Used by AUX Serial)
    //P0_3 D0 (RXD0)  (Used by Aux Serial)
    {P0_27, PinCapability::rwpwm, "d57"},
    {P0_28, PinCapability::rwpwm, "d58"},

    //Ramps Aux2 Pins
    {P0_26,  PinCapability::ainrw, "a9,d63"},       //(ADC3, DAC)
    //D40,     (NC)
    //D42,     (NC)
    //A11/D65, (NC)
    {P2_6, PinCapability::rwpwm, "d59"},               //a5 (overlaps J3)
    //A10/D64, (NC)
    //D44,     (NC)
    //A12/D66  (NC)
    
    //Ramps Aux3 Pins
    //{P0_15, PinCapability::rw, "sck,d52"},
    //{P0_17, PinCapability::rw, "miso,d50"},
    {P1_23, PinCapability::rwpwm, "d53"},              //(overlaps J3)
    //{P0_18, PinCapability::rw, "mosi,d51"},         //(overlaps Aux4)
    {P1_31, PinCapability::rwpwm, "d49"},
    
    //Ramps Aux4 Pins
    //D32, (NC)
    //D47, (NC)
    //D45, (NC)
    //D43, (NC)
    {P1_22,PinCapability::rwpwm, "d41"},
    //D39, (NC)
    {P1_30, PinCapability::rwpwm, "d37"},
    {P2_11, PinCapability::rwpwm, "d35"},
    {P3_25, PinCapability::rwpwm, "d33"},
    {P3_26, PinCapability::rwpwm, "d31"},
    //D29, (NC)
    //D27, (NC)
    //D25, (NC)
    //{P0_15, PinCapability::rw, "d23"},              //(SCK)  (Overlaps Aux3)
    //{P0_18, PinCapability::rw, "d17"},              //(MOSI) (overlaps Aux3)
    {P0_16, PinCapability::rwpwm, "d16"},              //(CS)
    
    //Ramps I2C Header
    {P0_0, PinCapability::rwpwm, "sca,d20,20"},        //labelled on board as 20
    {P0_1, PinCapability::rwpwm, "scl,d21,21"},        //labelled on board as 21
    
    
    //ReArm Headers
    //J3
    //P0_15, SCLK (overlaps Aux3/4)
    //P0_16, (Overlaps  Aux4)
    {P1_23, PinCapability::rwpwm,"d53"},               //PWM1[4]
    {P2_11, PinCapability::rwpwm,"d35"},
    {P1_31, PinCapability::rwpwm,"d49"},
    //P0_18, MOSI (overlaps Aux4/Aux3)
    {P2_6,  PinCapability::rwpwm,"d59"},               //(overlaps Aux2)
    //P0_17, MISO (overlaps Aux3)
    {P3_25, PinCapability::rwpwm,"d33"},               //PWM1[2]
    {P3_26, PinCapability::rwpwm,"d31"},               //PWM1[3]
    
    //J5
    {P1_22, PinCapability::rwpwm,"d41"},
    {P1_30, PinCapability::rwpwm,"d37"},
    //P1_21, d6     PWM1[3] //overlaps with Servos
    //P0_26, a9/d63 //Overlaps Aux2
    

    //Misc
    {P2_12, PinCapability::rwpwm,"pson"}, //PS-ON
    {P4_28, PinCapability::rwpwm,"play"}, //Play LED

};


constexpr BoardDefaults rearmDefaults = {
    {P0_10, P0_19, P0_21, P0_4, P4_29},     //enablePins
    {P2_1,  P2_2,  P2_3,  P2_0, P2_8},      //stepPins
    {P0_11, P0_20, P0_22, P0_5, P2_13},     //dirPins
#if LPC_TMC_SOFT_UART
    {NoPin, NoPin, NoPin, NoPin, NoPin},    //uartPins
#endif
    0,                                      //digiPot Factor
};



#endif
