/*
 * TMCSoftUart.cpp
 * Software UART implementations for TMC22XX chips for the LPC17XX MCU.
 * 
 * Note: The DMA version of this code is based upon an original implementation from Artem-B:
 * https://github.com/Artem-B/TMC2208Stepper/tree/soft-uart
 * However this version differs considerably to avoid problems with the original implementation
 * and in particular to allow it to run without disabling interrupts during operation.
 * 
 *  Created on: 29 Jan 2020
 *      Author: gloomyandy
 */

#include "RepRapFirmware.h"

#if SUPPORT_TMC22xx
#include "TMCSoftUART.h"
#include "TMC22xx.h"
#include "RepRap.h"
#include "Movement/Move.h"
#include "Movement/StepTimer.h"
#include "Hardware/Cache.h"

#include "DMA.h"





// Soft UART implementation
#if LPC_TMC_SOFT_UART
#define DMA_UART
#ifdef DMA_UART

#define SU_READ_OVERSAMPLE 4
#define SU_WRITE_OVERSAMPLE 4

static constexpr uint32_t SU_BAUD_RATE = DriversBaudRate;
//static constexpr uint32_t SU_READ_OVERSAMPLE = READ_OVERSAMPLE;
//static constexpr uint32_t SU_WRITE_OVERSAMPLE = WRITE_OVERSAMPLE;
static constexpr uint32_t SU_MAX_BYTES = 8;
static constexpr uint32_t SU_GAP_BYTES = 1;
static constexpr uint32_t SU_WRITE_BUFFER = 2;
static constexpr uint32_t SU_FRAME_LENGTH = 10;

// Select sample rates, ensure that one is always an exact multiple of the other
#if (READ_OVERSAMPLE >= WRITE_OVERSAMPLE)
    static const uint32_t SU_READ_PERIOD = (getPclk(PCLK_TIMER1)/(SU_BAUD_RATE*SU_READ_OVERSAMPLE)) - 1;
    static const uint32_t SU_WRITE_PERIOD = (getPclk(PCLK_TIMER1)/(SU_BAUD_RATE*SU_READ_OVERSAMPLE))*(SU_READ_OVERSAMPLE/SU_WRITE_OVERSAMPLE) - 1;
#else
    static const uint32_t SU_READ_PERIOD = (getPclk(PCLK_TIMER1)/(SU_BAUD_RATE*SU_WRITE_OVERSAMPLE))*(SU_WRITE_OVERSAMPLE/SU_READ_OVERSAMPLE) - 1;
    static const uint32_t SU_WRITE_PERIOD = (getPclk(PCLK_TIMER1)/(SU_BAUD_RATE*SU_WRITE_OVERSAMPLE)) - 1;
#endif

enum class SUStates
{
	idle,
    writesync,
	writing,
	delay,
    readsync1,
    readsync2,
	reading,
	complete,
	error
};
static volatile SUStates SUState = SUStates::idle;

static Pin SUPin;
static volatile uint32_t *SUPinSetPtr;
static volatile uint32_t *SUPinClrPtr;
static volatile uint8_t *SUPinReadPtr;
static __attribute__ ((used,section("AHBSRAM0"))) uint32_t SUPinBit;
static volatile uint8_t *SUWritePtr;
static uint32_t SUWriteCnt;
static uint32_t SUReadCnt;
static volatile uint8_t *SUReadPtr;
static DMA_TransferDescriptor_t *SUDescFirst;
static DMA_TransferDescriptor_t *SUDescCurrent;
static DMA_TransferDescriptor_t SUDescRead;

__attribute__ ((used,section("AHBSRAM0"))) uint8_t SUDmaBits[((SU_MAX_BYTES + SU_GAP_BYTES)*SU_FRAME_LENGTH)*SU_READ_OVERSAMPLE];
// The read sample buffer is also used to hold the DMA descriptors used for write operations (to save memory), make sure it is big enough
static_assert(SU_WRITE_BUFFER*SU_FRAME_LENGTH*sizeof(DMA_TransferDescriptor_t) <= sizeof(SUDmaBits), "DMABits buffer is not large enough for write operations");

static void SetupPins()
{
    auto pin = (LPC_GPIO_T*)(LPC_GPIO0_BASE + ((SUPin) & ~0x1f));
    SUPinSetPtr = &(pin->SET);
    SUPinClrPtr = &(pin->CLR);
    SUPinBit = 1 << (SUPin & 0x1f);
    SUPinReadPtr = (uint8_t *)&(pin->PIN);
    SUPinReadPtr += (SUPin & 0x1f)/8;
}


static void PrepareDescriptors()
{
    SUDescFirst = SUDescCurrent = (DMA_TransferDescriptor_t *)SUDmaBits;
    // create descriptors for three bytes worth of output 8 bata bits plus 2 start/stop bits 30 descriptors
    // Each descriptor will write a single bit oversampled times
    for(uint32_t i = 0; i < SU_WRITE_BUFFER*SU_FRAME_LENGTH; i++)
    {
        SUDescCurrent->src = (uint32_t) &SUPinBit;
        SUDescCurrent->dst = (uint32_t) SUPinClrPtr;
        SUDescCurrent->lli = (uint32_t) (SUDescCurrent+1);
        SUDescCurrent->ctrl = (SU_WRITE_OVERSAMPLE) |
                            GPDMA_DMACCxControl_SBSize(GPDMA_BSIZE_1) |
                            GPDMA_DMACCxControl_DBSize(GPDMA_BSIZE_1) |
                            GPDMA_DMACCxControl_SWidth(GPDMA_WIDTH_WORD) |
                            GPDMA_DMACCxControl_DWidth(GPDMA_WIDTH_WORD);
        SUDescCurrent++;
    }
    // link the last element back to the first
    (SUDescCurrent - 1)->lli = (uint32_t) (SUDescFirst);
    for(uint32_t i = 0; i < SU_WRITE_BUFFER; i++)
    {
       // generate an interrupt at the last bit of each byte
       SUDescFirst[i*SU_FRAME_LENGTH + SU_FRAME_LENGTH - 1].ctrl |= GPDMA_DMACCxControl_I;
    }
   
    // Setup the read descriptor
    SUDescRead.src = (uint32_t) SUPinReadPtr;
    SUDescRead.dst = (uint32_t) SUDmaBits;
    SUDescRead.lli = (uint32_t) 0;
    SUDescRead.ctrl = GPDMA_DMACCxControl_TransferSize(sizeof(SUDmaBits)) |
                     GPDMA_DMACCxControl_SBSize(GPDMA_BSIZE_1) |
                     GPDMA_DMACCxControl_DBSize(GPDMA_BSIZE_1) |
                     GPDMA_DMACCxControl_SWidth(GPDMA_WIDTH_BYTE) |
                     GPDMA_DMACCxControl_DWidth(GPDMA_WIDTH_BYTE) |
                     GPDMA_DMACCxControl_I |
                     GPDMA_DMACCxControl_DI;
    SUDescCurrent = SUDescFirst;
}

static void WriteByte(uint8_t val)
{
    // update the DMA descriptors to write the specified value
    // start bit
    (SUDescCurrent++)->dst = (uint32_t) SUPinClrPtr;
    // data bits
    for(int i = 0; i < 8; i++)
    {
        if (val & 1)
            (SUDescCurrent++)->dst = (uint32_t) SUPinSetPtr;
        else
            (SUDescCurrent++)->dst = (uint32_t) SUPinClrPtr;
        val >>= 1;
    }
    // stop bit
    (SUDescCurrent)->dst = (uint32_t) SUPinSetPtr;
    SUDescCurrent = (DMA_TransferDescriptor_t *)(SUDescCurrent->lli);
}

static uint32_t DecodeBytes(uint8_t *outp, uint32_t outlen)
{
    // Take an in memory oversampled capture of the output from the TMC22XX device, down sample
    // it and extract the original data bytes.
    uint32_t inlen = (outlen + SU_GAP_BYTES)*SU_FRAME_LENGTH*SU_READ_OVERSAMPLE;
    uint8_t mask = 1u << ((SUPin & 0x1f) % 8);
    uint8_t data;
    uint32_t outcnt = 0;
    uint32_t outbit = 0;
    int lastedge = 0;
    int midpoint = (SU_READ_OVERSAMPLE + 1)/2;
    bool lastbit = (SUDmaBits[lastedge + midpoint] & mask) != 0;
    for(uint32_t i = 0; i < inlen; i++)
    {
        bool thisbit = (SUDmaBits[lastedge + midpoint] & mask) != 0;
        // process the current bit
        if (outbit == 0)
        {
            // start bit must be zero
            if (!thisbit)
            {
                data = 0;
                outbit = 1;
            } 
        } 
        else if (++outbit < SU_FRAME_LENGTH)
        {
            // data bits
            data >>= 1;
            if (thisbit)
                data |= 0x80;
        }
        else if (thisbit)
        {
            // got valid stop bit, data byte complete
            *outp++ = data;
            if (++outcnt >= outlen)
                break;
            outbit = 0;
        }
        else
            // invalid stop bit, give up
            break; 

        if (thisbit != lastbit) 
        {
            // Bit flipped. Resync last_edge.
            lastedge += midpoint;
            while(((SUDmaBits[lastedge] & mask) != 0) == thisbit)
                lastedge--;
            lastedge++;
        }
        // Advance to the (expected) edge of the next bit.
        lastedge += SU_READ_OVERSAMPLE;
        lastbit = thisbit;       
    }
    return outcnt;
}

static void DmaInterrupt()
{
    switch(SUState)
    {
    case SUStates::idle:
        break;
    case SUStates::writesync:
        // delay writing new data until last bit of next to last byte
        SUState = SUStates::writing;
        break;
    case SUStates::writing:
        // We have now wirtten the last bit of a byte, top up the buffer if we have more data to write
        WriteByte(*SUWritePtr++);
        if (--SUWriteCnt == 0)
            SUState = SUStates::delay;
        break;
    case SUStates::delay:
        // last write is in progress add sync delays
        // First delay is to allow the final bit to complete before we switch the pin to read
        SUDescCurrent->src = (uint32_t) &SUPinBit;
        SUDescCurrent->dst = (uint32_t) &SUPinBit;
        SUDescCurrent->ctrl = (SU_WRITE_OVERSAMPLE*2) |
                            GPDMA_DMACCxControl_SBSize(GPDMA_BSIZE_1) |
                            GPDMA_DMACCxControl_DBSize(GPDMA_BSIZE_1) |
                            GPDMA_DMACCxControl_SWidth(GPDMA_WIDTH_WORD) |
                            GPDMA_DMACCxControl_DWidth(GPDMA_WIDTH_WORD) |
                            GPDMA_DMACCxControl_I;;
        SUDescCurrent = (DMA_TransferDescriptor_t *)(SUDescCurrent->lli);
        // 2nd delay is to position the start of the read operation to be before the TMC starts to send
        // data. Note that by the time it executes we may have modified the clock speed
        SUDescCurrent->src = (uint32_t) &SUPinBit;
        SUDescCurrent->dst = (uint32_t) &SUPinBit;
        SUDescCurrent->lli = (uint32_t) (&SUDescRead);
        SUDescCurrent->ctrl = (SU_READ_OVERSAMPLE*4) |
                            GPDMA_DMACCxControl_SBSize(GPDMA_BSIZE_1) |
                            GPDMA_DMACCxControl_DBSize(GPDMA_BSIZE_1) |
                            GPDMA_DMACCxControl_SWidth(GPDMA_WIDTH_WORD) |
                            GPDMA_DMACCxControl_DWidth(GPDMA_WIDTH_WORD);
        SUState = SUStates::readsync1;
        break;
    case SUStates::readsync1:
        SUState = SUStates::readsync2;
        break;
    case SUStates::readsync2:
        // sync is in progress switch pin direction
		pinMode(SUPin, INPUT_PULLUP);
        LPC_TIMER1->MR[0] = SU_READ_PERIOD;
        LPC_TIMER1->TC  = 0x00;  // Reset the Timer Count to 0
        SUState = SUStates::reading;
        break;
    case SUStates::reading:
        // Last bit of read operation captured
        SUState = SUStates::complete;
        pinMode(SUPin, OUTPUT_HIGH);
        break;
    case SUStates::complete:
    case SUStates::error:
        break;
    }
}

static void DmaStart()
{
    // Prepare the DMA hardware for the write/read operation
    LPC_SYSCTL->DMAREQSEL |= (1 << (GPDMA_CONN_MAT1_0 - 16));

    const uint8_t channelNumber = DMAGetChannelNumber(DMA_TIMER_MAT1_0);
    GPDMA_CH_T *pDMAch = (GPDMA_CH_T *) &(LPC_GPDMA->CH[channelNumber]);

    pDMAch->CONFIG = GPDMA_DMACCxConfig_H;                        //halt the DMA channel - Clears DMA FIFO
    
    LPC_GPDMA->INTTCCLEAR = (((1UL << (channelNumber)) & 0xFF));    //clear terminal count interrupt requests for Rx and Tx Channels
    LPC_GPDMA->INTERRCLR = (((1UL << (channelNumber)) & 0xFF));     //clear the error interrupt request

    // Enable DMA channels
    LPC_GPDMA->CONFIG = GPDMA_DMACConfig_E;
    while (!(LPC_GPDMA->CONFIG & GPDMA_DMACConfig_E)) {}

    // Setup initial write transfer we always start with the pin high
    pDMAch->SRCADDR  = (uint32_t)&SUPinBit;
    pDMAch->DESTADDR = (uint32_t)SUPinSetPtr;
    pDMAch->CONTROL  = (SU_WRITE_OVERSAMPLE)
                            | GPDMA_DMACCxControl_SBSize(GPDMA_BSIZE_1)     //Source burst size set to 1
                            | GPDMA_DMACCxControl_DBSize(GPDMA_BSIZE_1)     //Destination burst size set to 1
                            | GPDMA_DMACCxControl_SWidth(GPDMA_WIDTH_WORD)  //Source Tranfser width set to 1 byte
                            | GPDMA_DMACCxControl_DWidth(GPDMA_WIDTH_WORD); //Destination Transfer width set to 1 byte

    pDMAch->LLI = (uint32_t)SUDescFirst;

    pDMAch->CONFIG = // GPDMA_DMACCxConfig_E   
                        0                                                          //Enable
                            | GPDMA_DMACCxConfig_SrcPeripheral((GPDMA_CONN_MAT1_0 - 8))                       
                            | GPDMA_DMACCxConfig_TransferType(GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA)
                            | GPDMA_DMACCxConfig_IE                         //Interrupt Error Mask
                            | GPDMA_DMACCxConfig_ITC;                       //Terminal count interrupt mask
                                  


    SUState = SUStates::writing;
    LPC_TIMER1->MR[0] = SU_WRITE_PERIOD;
    LPC_TIMER1->TC  = 0x00;  // Reset the Timer Count to 0

	Chip_GPDMA_ChannelCmd(LPC_GPDMA, channelNumber, ENABLE);
}


void LPCSoftUARTAbort() noexcept
{
	SUState = SUStates::idle;
	if (SUPin != NoPin)
    	pinMode(SUPin, OUTPUT_HIGH);
}

void LPCSoftUARTStartTransfer(uint8_t driver, volatile uint8_t *WritePtr, uint32_t WriteCnt, volatile uint8_t *ReadPtr, uint32_t ReadCnt) noexcept
{
	SUPin = TMC_UART_PINS[driver];
	if (SUPin != NoPin)
	{
        SetupPins();
        PrepareDescriptors();
        SUWritePtr = WritePtr;
        SUWriteCnt = WriteCnt;
        // Pre buffer the write data
        for(uint32_t i = 0; i < SU_WRITE_BUFFER; i++)
        {
            WriteByte(*SUWritePtr++);
            SUWriteCnt--;
        }
        SUReadPtr = ReadPtr;
        SUReadCnt = ReadCnt;
		pinMode(SUPin, OUTPUT_HIGH);
        DmaStart();
	}
}

bool LPCSoftUARTCheckComplete() noexcept
{
	if (SUState == SUStates::complete)
	{
        pinMode(SUPin, OUTPUT_HIGH);
		SUState = SUStates::idle;
        int cnt = DecodeBytes((uint8_t *)SUReadPtr, SUReadCnt);
        if (cnt <= 0)
            // data error
            return false;
		// I/O complete
        return true;
	}
    return false;
}

void LPCSoftUARTInit() noexcept
{
	InitialiseDMA();
    AttachDMAChannelInterruptHandler(DmaInterrupt, DMA_TIMER_MAT1_0);
    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_TIMER1); //enable power and clocking
    LPC_TIMER1->PR   =  0;
    LPC_TIMER1->MR[0] = SU_WRITE_PERIOD;
    LPC_TIMER1->TC  = 0x00;  // Reset the Timer Count to 0
    LPC_TIMER1->MCR = ((1<<SBIT_MR0R));     // Int on MR0 match and Reset Timer on MR0 match
    LPC_TIMER1->TCR  = (1 <<SBIT_CNTEN); //Start Timer

	for(size_t i = 0; i < NumDirectDrivers; i++)
		if (TMC_UART_PINS[i] != NoPin)
			pinMode(TMC_UART_PINS[i], OUTPUT_HIGH);

	SUPin = NoPin;
	SUState = SUStates::idle;
}

void LPCSoftUARTShutdown() noexcept
{
	LPCSoftUARTAbort();
}

#else

constexpr uint32_t SU_BAUD_RATE = DriversBaudRate;
enum class SUStates
{
	idle,
	writeSync,
	writing,
	readSync,
	reading,
	complete,
	error
};
static uint32_t SUPeriod;
static volatile uint32_t SUWriteCnt = 0;
static volatile uint32_t SUReadCnt = 0;
static volatile uint8_t *SUWritePtr;
static volatile uint8_t *SUReadPtr;
static volatile int SUBitCnt;
static volatile uint32_t SUData;
static volatile Pin SUPin;
static LPC_GPIO_T *SUPort;
static uint8_t SUPinNo;
static uint32_t SUPinBit;
static volatile int SUOffset;
static volatile int SUBaseOffset;
static volatile SUStates SUState = SUStates::idle;




extern "C" void RIT_IRQHandler() noexcept __attribute__ ((hot));

void RIT_IRQHandler() noexcept
{
	Chip_RIT_ClearInt(LPC_RITIMER);
	switch(SUState)
	{
	case SUStates::idle:
		break;
	case SUStates::writeSync:
		{
			// First four bits of a write are used to establish the baud rate for the connection.
			// We record the actual time that the bits are written and use this to adjust the timing of
			// the rest of the write and read operations. This reduces framing errors.
			// First write the data
            if (SUData & 0x1)
                SUPort->SET = SUPinBit;
            else
                SUPort->CLR = SUPinBit;
			int offset = Chip_RIT_GetCounter(LPC_RITIMER);
			SUData >>= 1;
			if (++SUBitCnt == 1)
				// record time offset of first bit
				SUBaseOffset = offset;
			else if (SUBitCnt == 5)
			{
				// caclculate per bit timing offset for sync sequence 
				SUOffset = (offset - SUBaseOffset)/4;
				// adjust timing for rest of write operation
				Chip_RIT_SetCOMPVAL(LPC_RITIMER, SUPeriod + SUOffset);
				// Sync complete
				SUState = SUStates::writing;	
			}
			break;
		}

	case SUStates::writing:
    	if (++SUBitCnt <= 10 ) 
		{
			// send data 
            if (SUData & 0x1)
                SUPort->SET = SUPinBit;
            else
                SUPort->CLR = SUPinBit;
			SUData >>= 1;
    	}
    	else if (--SUWriteCnt > 0)
		{
			// send start bit of next byte
            SUPort->CLR = SUPinBit;
			// get data and add stop bit (but not start bit as we have already handled that)
			SUData = 0x100 | *SUWritePtr++;
			// Start bit has already been sent
			SUBitCnt = 1;
		}
		else
		{
			// end of output, switch to input mode
			util_UpdateBit(SUPort->DIR, SUPinNo, 0);
			// TMC22XX will start sending data 8 bits after the write completes
			Chip_RIT_SetCOMPVAL(LPC_RITIMER, (SUPeriod + SUOffset)*8);
			SUState = SUStates::readSync;
		}
		break;
	case SUStates::readSync:
		{
			uint8_t inbit = util_IsBitSet(SUPort->PIN, SUPinNo);
			// Do we have the start bit?
			if (!inbit) 
			{
				// got start bit, set timing for reading bits
				Chip_RIT_SetCOMPVAL(LPC_RITIMER, SUPeriod + SUOffset);
				SUBitCnt = 1;
				SUData = 0;
				SUState = SUStates::reading;
			}
			else
				// no start bit, give up
				SUState = SUStates::error;
			break;
		}
	case SUStates::reading:
		if (++SUBitCnt <= 9)
		{
			// data bits, read data
			uint8_t inbit = util_IsBitSet(SUPort->PIN, SUPinNo);
      		SUData >>= 1;
      		if (inbit)
        		SUData |= 0x80;
		}
		else
		{
			// Stop bit read, store data byte in buffer
			*SUReadPtr++ =  static_cast<uint8_t>(SUData);
			if (--SUReadCnt <= 0)
				SUState = SUStates::complete;
			else
				SUState = SUStates::readSync;		
		}
		break;
	case SUStates::complete:
	case SUStates::error:		
		Chip_RIT_Disable(LPC_RITIMER);
		break;			
	}
}	

void LPCSoftUARTAbort() noexcept
{
	Chip_RIT_Disable(LPC_RITIMER);
	SUReadCnt = 0;
	SUWriteCnt = 0;
	SUState = SUStates::idle;
	if (SUPin != NoPin)
		pinMode(SUPin, OUTPUT_HIGH);
}

void LPCSoftUARTStartTransfer(uint8_t driver, volatile uint8_t *WritePtr, uint32_t WriteCnt, volatile uint8_t *ReadPtr, uint32_t ReadCnt) noexcept
{
	LPCSoftUARTAbort();
	SUPin = TMC_UART_PINS[driver];
	if (SUPin != NoPin)
	{
		// got valid Pin so set things up for transfer
		LPC_RITIMER->COUNTER = 0;
		Chip_RIT_SetCOMPVAL(LPC_RITIMER, SUPeriod);
		SUWritePtr = WritePtr;
		SUReadPtr = ReadPtr;
		// Add start and stop bits to first byte
		SUData = (static_cast<uint32_t>(*SUWritePtr) << 1) | 0x200;
		SUWritePtr++;
		SUBitCnt = 0;
		SUOffset = 0;
		pinMode(SUPin, OUTPUT_HIGH);
		// use direct access to keep interupt handler as fast as possible
		SUPort = (LPC_GPIO_T*)(LPC_GPIO0_BASE + ((SUPin) & ~0x1f));
		SUPinNo = SUPin & 0x1f;
        SUPinBit = 1 << SUPinNo;
		SUWriteCnt = WriteCnt;
		SUReadCnt = ReadCnt;
		SUState = SUStates::writeSync;
		Chip_RIT_Enable(LPC_RITIMER);
	}
}


bool LPCSoftUARTCheckComplete() noexcept
{
	if (SUState == SUStates::complete)
	{
		SUState = SUStates::idle;
		// I/O complete
        return true;
	}
    return false;
}

void LPCSoftUARTInit() noexcept
{
	Chip_RIT_Init(LPC_RITIMER);
	SUPeriod = Chip_Clock_GetPeripheralClockRate(SYSCTL_PCLK_RIT)/(SU_BAUD_RATE);
	Chip_RIT_SetCOMPVAL(LPC_RITIMER, SUPeriod);

	Chip_RIT_EnableCTRL(LPC_RITIMER, RIT_CTRL_ENCLR);

	for(size_t i = 0; i < NumDirectDrivers; i++)
		if (TMC_UART_PINS[i] != NoPin)
			pinMode(TMC_UART_PINS[i], OUTPUT_HIGH);

	SUPin = NoPin;
	SUState = SUStates::idle;
	NVIC_EnableIRQ(RITIMER_IRQn);
}


void LPCSoftUARTShutdown() noexcept
{
	LPCSoftUARTAbort();
	NVIC_DisableIRQ(RITIMER_IRQn);
}
#endif
#endif
#endif
