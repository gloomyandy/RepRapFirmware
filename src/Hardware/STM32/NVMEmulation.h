#ifndef NVMEMULATION_H_
#define NVMEMULATION_H_

#include "Core.h"


void NVMEmulationRead(void *data, uint32_t dataLength) noexcept;
bool NVMEmulationErase() noexcept;
bool NVMEmulationWrite(const void *data, uint32_t dataLength) noexcept;



#endif
