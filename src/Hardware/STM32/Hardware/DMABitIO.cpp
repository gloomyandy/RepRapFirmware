/*
 * DMABitIO.cpp
 * Permform bit streming I/O operations to a GPIO pin via DMA:
 * * Software UART implementations for TMC22XX.
 * * Neopixel bit I/O
 * 
 * Note: The DMA version of this code is based upon an original implementation from Artem-B:
 * https://github.com/Artem-B/TMC2208Stepper/tree/soft-uart
 * However this version differs considerably to avoid problems with the original implementation
 * and in particular to allow it to run without disabling interrupts during operation.
 * 
 * This version re-worked for STM32 DMA controllers. Added Neopixel support
 * 
 *  Created on: 1 Aug 2020
 *      Author: gloomyandy
 */

#include "RepRapFirmware.h"

#if SUPPORT_TMC22xx || SUPPORT_DMA_NEOPIXEL
#include "DMABitIO.h"
#include "TMC22xx.h"
#include "Platform/RepRap.h"
#include "HardwareTimer.h"
#include "Movement/Move.h"
#include "Movement/StepTimer.h"
#include "Cache.h"
#include "AppNotifyIndices.h"

#define SU_OVERSAMPLE 4

static constexpr uint32_t SU_BAUD_RATE = DriversBaudRate;
static constexpr uint32_t SU_MAX_BYTES = 8;
static constexpr uint32_t SU_GAP_BYTES = 1;
static constexpr uint32_t SU_GAP_BITS = 4;
static constexpr uint32_t SU_FRAME_LENGTH = 10;
static constexpr uint32_t SU_MAX_RETRY = 0;
static constexpr uint32_t SU_BUFFER_SZ = ((SU_MAX_BYTES+SU_GAP_BYTES)*SU_FRAME_LENGTH)*SU_OVERSAMPLE;
enum SUStates
{
    idle,
    writing,
    reading,
    neowrite1,
    neowrite2,
    neowrite3,
    complete,
    error
};
static volatile SUStates SUState = SUStates::idle;
static HardwareTimer SUTimer(TIM1);
static Pin SUPin;
static volatile uint32_t *SUPinSetClrPtr;
static volatile uint32_t *SUPinReadPtr;
static uint32_t SUPinBit;
static uint32_t SUSetBit;
static uint32_t SUClrBit;
static uint32_t SUWriteCnt;
static uint8_t* SUWritePtr;
static uint32_t SUReadCnt;
static uint8_t* SUReadPtr;
static uint32_t SUBitCnt;
DMA_HandleTypeDef SUDma;
static uint32_t SUPeriod;
static TaskHandle SUWaitingTask;
static Mutex SUMutex;

__nocache uint32_t SUDmaBits[SU_BUFFER_SZ];

static constexpr uint32_t NEO_OVERSAMPLE = 4;
static constexpr uint32_t NEO_WORDS_PER_BYTE = (8*NEO_OVERSAMPLE);
// We use a double buffer, work out how big it can be
static constexpr uint32_t NEO_BYTES_PER_BUFF = (SU_BUFFER_SZ/2)/NEO_WORDS_PER_BYTE;
static constexpr uint32_t NEO_BASE_TIME = 400/(1);

static uint8_t *NEOWritePtr = nullptr;
static uint32_t NEOWriteCnt = 0;
static uint32_t NEOOneBits[NEO_OVERSAMPLE];
static uint32_t NEOZeroBits[NEO_OVERSAMPLE];



static void SetupPins()
{
    GPIO_TypeDef *port = get_GPIO_Port(STM_PORT(SUPin));
    uint32_t pinNo = STM_PIN(SUPin);
    SUPinSetClrPtr = &(port->BSRR);
    SUPinBit = 1 << pinNo;
    SUSetBit = SUPinBit;
    SUClrBit = SUPinBit << 16;
    SUPinReadPtr = &(port->IDR);
}

static void WriteByte(uint8_t val)
{
    // place one bye of data into the DMA buffer
    // Start bit
    SUDmaBits[SUBitCnt++] = SUClrBit;
    // data bits
    for(int i = 0; i < 8; i++)
    {
        if (val & 1)
            SUDmaBits[SUBitCnt++] = SUSetBit;
        else
            SUDmaBits[SUBitCnt++] = SUClrBit;
        val >>= 1;
    }
    // stop bit
    SUDmaBits[SUBitCnt++] = SUSetBit;
}

static uint32_t DecodeBytes(uint8_t *outp, uint32_t outlen)
{
    // Take an in memory oversampled capture of the output from the TMC22XX device, down sample
    // it and extract the original data bytes.
    uint32_t inlen = (outlen + SU_GAP_BYTES)*SU_FRAME_LENGTH*SU_OVERSAMPLE;
    uint32_t mask = SUPinBit;
    uint8_t data;
    uint32_t outcnt = 0;
    uint32_t outbit = 0;
    int lastedge = 0;
    int midpoint = (SU_OVERSAMPLE + 1)/2;
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
        lastedge += SU_OVERSAMPLE;
        lastbit = thisbit;       
    }
    return outcnt;
}

static void NeoWriteByte(uint8_t val) noexcept
{
    for(int i = 0; i < 8; i++)
    {
        if (val & 0x80)
        {
            for(uint32_t i = 0; i < NEO_OVERSAMPLE; i++)
                SUDmaBits[SUBitCnt++] = NEOOneBits[i];
        }
        else
        {
            for(uint32_t i = 0; i < NEO_OVERSAMPLE; i++)
                SUDmaBits[SUBitCnt++] = NEOZeroBits[i];
        }
        val <<= 1;
    }
}

static void NeoWriteZeroByte() noexcept
{
    for(int i = 0; i < 8; i++)
    {
        for(uint32_t i = 0; i < NEO_OVERSAMPLE; i++)
            SUDmaBits[SUBitCnt++] = SUClrBit;
    }
}

static bool NeoAddToBuffer(uint32_t cnt)
{
    while (cnt-- > 0)
    {
        if (NEOWriteCnt > 0)
        {
            NeoWriteByte(*NEOWritePtr++);
            NEOWriteCnt--;
        }
        else
            NeoWriteZeroByte();
    }
    // Flip buffer back to start if needed
    if (SUBitCnt > SU_BUFFER_SZ/2) SUBitCnt = 0;
    return NEOWriteCnt == 0;
}

static void DmaInterrupt(DMA_HandleTypeDef *_hdma)
{
    switch(SUState)
    {
    case SUStates::idle:
        break;
    case SUStates::writing:
        // we get called at the end of writing
        SUTimer.pause();
        pinMode(SUPin, INPUT_PULLUP);
        SUDma.Init.Direction = DMA_PERIPH_TO_MEMORY;
        HAL_DMA_Start_IT(&SUDma, (uint32_t)SUPinReadPtr, (uint32_t)SUDmaBits, sizeof(SUDmaBits)/sizeof(uint32_t));
    	SUTimer.setOverflow(SUPeriod, TICK_FORMAT);
        SUTimer.setCount(0, TICK_FORMAT);
        SUState = SUStates::reading;
        SUTimer.resume();
        break;
    case SUStates::reading:
        // Last bit of read operation captured
        SUTimer.pause();
        SUState = SUStates::complete;
        pinMode(SUPin, OUTPUT_HIGH);
        // fall through
    case SUStates::complete:
    case SUStates::error:
        TaskBase::GiveFromISR(SUWaitingTask, NotifyIndices::BitIO);
        break;
    case SUStates::neowrite1:
    case SUStates::neowrite2:
        if (NeoAddToBuffer(NEO_BYTES_PER_BUFF)) SUState = (SUStates)((int)SUState+1);
        break;
    case SUStates::neowrite3:
        SUState = SUStates::complete;
        SUTimer.pause();
        HAL_DMA_Abort_IT(&SUDma);
        TaskBase::GiveFromISR(SUWaitingTask, NotifyIndices::BitIO);
        break;
    }
}

static void DmaStart()
{
    uint8_t *p = SUWritePtr;
    SUBitCnt = 0;
    // The timing of the first stop bit is critical as it sets the timing of the remaining bits
    // to ensure this is as accurate as possible we insert a number of dummy high bits before
    // the first start bit.
    for(uint32_t i = 0; i < SU_GAP_BITS; i++)
        SUDmaBits[SUBitCnt++] = SUSetBit;
    // Pre buffer the write data
    for(uint32_t i = 0; i < SUWriteCnt; i++)
    {
        WriteByte(*p++);
    }
    // The TMC driver will wait for 8 bits of time before sending the reply. We pad the output
    // data by half of this time.
    for(uint32_t i = 0; i < SU_GAP_BITS; i++)
        SUDmaBits[SUBitCnt++] = SUSetBit;
    pinMode(SUPin, OUTPUT_HIGH);
    SUDma.Init.Direction = DMA_MEMORY_TO_PERIPH;
    SUDma.Init.Mode = DMA_NORMAL;
    HAL_DMA_RegisterCallback(&SUDma, HAL_DMA_XFER_HALFCPLT_CB_ID, nullptr);
    HAL_DMA_Start_IT(&SUDma, (uint32_t)SUDmaBits, (uint32_t)SUPinSetClrPtr, SUBitCnt);
    SUTimer.setOverflow(SUPeriod*SU_OVERSAMPLE, TICK_FORMAT);
    SUTimer.setCount(0, TICK_FORMAT);
    SUState = SUStates::writing;
    SUTimer.resume();
}


bool TMCSoftUARTTransfer(Pin pin, volatile uint8_t *WritePtr, uint32_t WriteCnt, volatile uint8_t *ReadPtr, uint32_t ReadCnt, uint32_t timeout) noexcept
{
    bool ret = false;
    if (pin != NoPin)
    {
        MutexLocker lock(SUMutex, timeout);
        if (!lock.IsAcquired())
        {
            debugPrintf("TMC UART failed to acquire mutex\n");
            return ret;
        }

        SUPin = pin;
        SetupPins();
        SUWritePtr = (uint8_t *)WritePtr;
        SUWriteCnt = WriteCnt;
        SUReadPtr = (uint8_t *)ReadPtr;
        SUReadCnt = ReadCnt;
        SUWaitingTask = TaskBase::GetCallerTaskHandle();
        DmaStart();
        while (SUState != SUStates::complete)
        {
            if (!TaskBase::TakeIndexed(NotifyIndices::BitIO, timeout))
            {
                debugPrintf("TMC UART timeout\n");
                SUTimer.pause();
                HAL_DMA_Abort_IT(&SUDma);
                break;
            }
        }
        SUWaitingTask = 0;
        if (SUState == SUStates::complete)
        {
            uint32_t cnt = DecodeBytes(SUReadPtr, SUReadCnt);
            if (cnt == SUReadCnt)
                ret = true;
        }
        pinMode(SUPin, OUTPUT_HIGH);
        SUState = SUStates::idle;
	}
    return ret;
}

void DMABitIOInit() noexcept
{
    //debugPrintf("BIOInit....\n");
    SUMutex.Create("BITIO");
    uint32_t period = SUTimer.getTimerClkFreq()/(SU_BAUD_RATE*SU_OVERSAMPLE);
    //debugPrintf("SU base freq %d setting period %d\n", static_cast<int>(SUTimer.getTimerClkFreq()), static_cast<int>(period));
    SUPeriod = period;
    SUTimer.setOverflow(period*SU_OVERSAMPLE, TICK_FORMAT);
    SUTimer.setCount(0, TICK_FORMAT);
    __HAL_TIM_ENABLE_DMA(SUTimer.getHandle(), TIM_DMA_UPDATE);
    __HAL_RCC_DMA2_CLK_ENABLE();    
    SUDma.Instance                 = DMA2_Stream5;
#if STM32H7
    SUDma.Init.Request             = DMA_REQUEST_TIM1_UP;
#else
    SUDma.Init.Channel             = DMA_CHANNEL_6;
#endif
    SUDma.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    SUDma.Init.PeriphInc           = DMA_PINC_DISABLE;
    SUDma.Init.MemInc              = DMA_MINC_ENABLE;
    SUDma.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    SUDma.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    SUDma.Init.Mode                = DMA_NORMAL;
    SUDma.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
    SUDma.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;         
    SUDma.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    SUDma.Init.MemBurst            = DMA_MBURST_SINGLE;
    SUDma.Init.PeriphBurst         = DMA_PBURST_SINGLE;
    HAL_DMA_Init(&SUDma);
    HAL_DMA_RegisterCallback(&SUDma, HAL_DMA_XFER_CPLT_CB_ID, DmaInterrupt);
    SUPin = NoPin;
    SUState = SUStates::idle;
    SUWaitingTask = 0;
    NVIC_EnableIRQ(DMA2_Stream5_IRQn);
}

void DMABitIOShutdown() noexcept
{
    SUTimer.pause();
    NVIC_DisableIRQ(DMA2_Stream5_IRQn);
}

static void NeoSetupBitTiming(uint32_t *bits, uint32_t threshold)
{
    if (threshold > NEO_OVERSAMPLE)
    {
        debugPrintf("Neopixel timing too high %d > %d\n", (int)threshold, (int)NEO_OVERSAMPLE);
        threshold = NEO_OVERSAMPLE;
    }
    //debugPrintf("Set timing threshold %d\n", threshold);
    for(uint32_t i = 0; i < threshold; i++)
        bits[i] = SUSetBit;
    for(uint32_t i = threshold; i < NEO_OVERSAMPLE; i++)
        bits[i] = SUClrBit;
}

bool NeopixelDMAWrite(Pin pin, uint32_t freq, uint8_t *bits, uint32_t cnt, uint32_t zeroTime, uint32_t oneTime, uint32_t timeout) noexcept
{
    MutexLocker lock(SUMutex, timeout);
    if (!lock.IsAcquired())
    {
        debugPrintf("Neopixel DMA failed to acquire mutex\n");
        return false;
    }
    // Setup timer frequency
    uint32_t period = SUTimer.getTimerClkFreq()/(freq);
    //debugPrintf("SU base freq %d setting period %d\n", static_cast<int>(SUTimer.getTimerClkFreq()), static_cast<int>(period));
    //debugPrintf("SuState %d\n", SUState);
    SUPin = pin;
    pinMode(SUPin, OUTPUT_LOW);
    pin_speed(SUPin, GPIO_SPEED_FREQ_LOW);
    SetupPins();
    NeoSetupBitTiming(NEOZeroBits, (zeroTime+NEO_BASE_TIME-1)/NEO_BASE_TIME);
    NeoSetupBitTiming(NEOOneBits, (oneTime+NEO_BASE_TIME-1)/NEO_BASE_TIME);
    SUBitCnt = 0;
    NEOWritePtr = bits;
    NEOWriteCnt = cnt;
    // Write dummy bytes to avoid odd longer first pulse
    NeoWriteZeroByte();
    SUState = SUStates::neowrite1;
    if (NeoAddToBuffer(NEO_BYTES_PER_BUFF - 1)) SUState = (SUStates)((int)SUState+1);
    //debugPrintf("SUBitCnt %d\n", SUBitCnt);
    if (NeoAddToBuffer(NEO_BYTES_PER_BUFF)) SUState = (SUStates)((int)SUState+1);    
    //debugPrintf("SUBitCnt %d total buffer size (words) %d state %d\n", SUBitCnt, NEO_BYTES_PER_BUFF*NEO_WORDS_PER_BYTE*2, SUState);
    SUDma.Init.Direction = DMA_MEMORY_TO_PERIPH;
    SUDma.Init.Mode = DMA_CIRCULAR;
    HAL_DMA_RegisterCallback(&SUDma, HAL_DMA_XFER_HALFCPLT_CB_ID, DmaInterrupt);
    SUWaitingTask = TaskBase::GetCallerTaskHandle();
    HAL_DMA_Start_IT(&SUDma, (uint32_t)SUDmaBits, (uint32_t)SUPinSetClrPtr, NEO_BYTES_PER_BUFF*NEO_WORDS_PER_BYTE*2);
    SUTimer.setOverflow(period, TICK_FORMAT);
    SUTimer.setCount(0, TICK_FORMAT);
    SUTimer.resume();
    while(SUState != SUStates::complete)
    {
        if (!TaskBase::TakeIndexed(NotifyIndices::BitIO, timeout))
        {
            debugPrintf("DMABitIO timeout\n");
            SUTimer.pause();
            HAL_DMA_Abort_IT(&SUDma);
            break;
        }
    }
    SUWaitingTask = 0;
    SUState = SUStates::idle;
    return true;
}   

extern "C" void DMA2_Stream5_IRQHandler()
{
    HAL_DMA_IRQHandler(&SUDma);    
}

#endif
