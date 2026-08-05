#include "Devices.h"
#include <RepRapFirmware.h>
#include <AnalogIn.h>
#include <AnalogOut.h>
#if CORE_USES_TINYUSB
# include <TinyUsbInterface.h>
# include <Platform/TaskPriorities.h>
#endif

#if CORE_USES_TINYUSB

constexpr size_t UsbDeviceTaskStackWords = 200;
static Task<UsbDeviceTaskStackWords> usbDeviceTask;

#endif

SerialCDC serialUSB;
#ifdef SERIAL_USB2_DEVICE
SerialCDC serialUSB2(1);
#endif


// Device initialisation
void DeviceInit() noexcept
{
	LegacyAnalogIn::AnalogInInit();
	AnalogOut::Init();
#if CORE_USES_TINYUSB
	CoreUsbInit(NvicPriorityUSB);
	usbDeviceTask.Create(CoreUsbDeviceTask, "USBD", nullptr, TaskPriority::UsbPriority);
#endif

}

void StopAnalogTask() noexcept
{
}

void StopUsbTask() noexcept
{
#if CORE_USES_TINYUSB
	usbDeviceTask.TerminateAndUnlink();
#endif
}

