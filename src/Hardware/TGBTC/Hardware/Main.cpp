#include <CoreIO.h>
#include <RepRapFirmware.h>
#include <ResetCause.h>
// Program initialisation
void AppInit() noexcept
{
	// Some bootloaders leave UASRT3 enabled, make sure it does not cause problems
	HAL_NVIC_DisableIRQ(USART3_IRQn);
	InitResetCause();
}

// End