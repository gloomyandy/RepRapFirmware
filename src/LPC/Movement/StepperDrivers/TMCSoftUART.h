



#ifndef TMCSOFTUART_H_
#define TMCSOFTUART_H_

void LPCSoftUARTAbort() noexcept;
void LPCSoftUARTStartTransfer(uint8_t driver, volatile uint8_t *WritePtr, uint32_t WriteCnt, volatile uint8_t *ReadPtr, uint32_t ReadCnt) noexcept;
bool LPCSoftUARTCheckComplete() noexcept;
void LPCSoftUARTInit() noexcept;
void LPCSoftUARTShutdown() noexcept;
#endif