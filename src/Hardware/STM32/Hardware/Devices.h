#ifndef SRC_STM32_HARDWARE_DEVICES_H_
#define SRC_STM32_HARDWARE_DEVICES_H_

#include <AsyncSerial.h>
#define SUPPORT_USB		1		// needed by SerialCDC.h
#include <SerialCDC.h>
extern SerialCDC serialUSB;
#ifdef SERIAL_USB2_DEVICE
extern SerialCDC serialUSB2;
#endif
void DeviceInit() noexcept;
void StopAnalogTask() noexcept;
void StopUsbTask() noexcept;

#endif /* SRC_STM32_HARDWARE_DEVICES_H_ */
