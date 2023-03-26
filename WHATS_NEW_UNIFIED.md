Version 3.5beta2+_106
=====================
* Apply cold extrusion fix
* Note this version is held on aseparate RRF branch v3.5-cold-extrude-fix

Version 3.5beta2+_105
=====================
* Fixed crash when using laser mode
* Fixed H723 based boards not showing input voltage correctly
* Fixed display not beeping

Version 3.5beta2+_102
=====================
* Duet3d 3.5beta2
* Enable MQTT support (requires ESP firmware >= V2.1b3)
* STM32F4: Add support for MCP2518FD SP CAN-FD interfaces
* STM32F4: Improved SPI1 support (allow interrupt operation)
* Increase open files to 20
* Ensure ESP32 modules are fully disabled by M552 s-1


Version 3.5beta1_102
====================
* Duet3D 3.5beta1
* Removed LPC17xx support from source

Version 3.4.5+_104
==================
* Fixed crash when using laser mode
* Fixed H723 based boards not showing input voltage correctly

Version 3.4.5+_103
==================
* Updated pins Fly E3 V2

Version 3.4.5+_102
==================
* Duet3d 3.4.5+ fixes
* Added Fly Super8 pro H723 and Fly Super5 H723
* Updated pins Fly Gemini V2 and Fly E3 V2

Version 3.4.5_102
=================
* Duet3d V3.4.5 fixes
* Added biquoctopus_x7/troodon_v2

Version 3.4.5_101
=================
* Duet3d V3.4.5
* Added new SKR pro signature

Version 3.4.4_104
=================
* Add pins for FLY_E3_V2

Version 3.4.4_103
=================
* Add support for STM32H723 and updated SKR3
* Fix occasional WiFi disconnect with large uploads

Version 3.4.4_102
=================
* Fix bug. Using hardware driven Neopixels can interrupt TMC UART interface
* Switch compiler to GNU Arm Embedded Toolchain 10.3-2021.10 to bring into line with Duet3D

Version 3.4.4_101
=================
* Duet3d 3.4.4 changes
* Fix bug when trying to shutdown TMC22xx drivers if watchdog expires

Version 3.4.2_101
=================
* Duet3d 3.4.2 changes
* Fix bug with returning string name of pins
* Updated Fysetc Spider King pins

Version 3.4.2rc2_102
====================
* Duet3d 3.4-extruder-only-movement-fix
* STM32F4 Fix SDIO error during firmware update
* Added "biquskr_3_ez"
* Added configuration options "pins.SetHigh" and "pins.SetLow"

Version 3.4.2rc2_101
====================
* Duet3d 3.4.2rc2

Version 3.4.2rc1+_105
=====================
* Fix H7 bootloader to allow it to work with klipper
* Stop stack being trashed during some software reset calls
* Fix reporting of board reset reason


Version 3.4.2rc1+_104
=====================
* Duet3d 3.4.2rc1+ as of 28/07/2022
* Allow firmware update over CAN-FD for boards in expansion mode
* Adjust firmware filenames to allow CAN-FD updates
* Enable FTP
* Add Fysetc Spider King
* Report RAM addresses in M122 P200


Version 3.4.2rc1+_101
=====================
* Duet3d 3.4.2rc1+ as of 17/07/2022
* Fix timeouts with single TMC22xx driver
* Improved UART error detection for TMC22xx drivers
* Add new configuration options:
  leds.diagnosticOn : boolean, sets polarity of diag LED
  leds.activity : Pin, sets pin for CAN activity LED
  leds.activityOn : boolean, sets polarity of CAN activity LED
* Added support for Reset and EmergencyReset when in CAN expansion board mode


Version 3.4.2beta1_101
=================
* Duet3d 3.4.2beta1 as of 29/6/2022
* Changed M569.8 to M569.9 to avoid clash
* Added Fly super8_pro
* PWM hardware now only allocated first time it is needed (output != 0 && output != 1)
* STM32H7: Software PWM not working on H7 builds (either full on or off)
* STM32H7: M569.8/M569.9 not recognised

Version 3.4.1_102
=================
* Fixed bug when freeing attached I/O pin

Version 3.4.1_101
==================
* Duet3D 3.4.1
* Fix pins for Fly e3_prov3
* Firmware filenames are now mcu specific (firmware.bin -> firmware-stm32h7.bin/firmware-stm32f4.bin)
* A firmware update (via M997) now performs an emergency stop (shutdown heaters/additional boards)
* Reset WiFi SPI device after setting new clock speed (avoids spurious error report)

Version 3.4.1RC2_101
====================
* Duet3D 3.4.1RC2
* Allow default thermistor series resistor value to be configured (heat.thermistorSeriesResistor)
* Allow default esp WiFi clock registor to be configured (8266wifi.clockReg)
* Don't reset accumulated pixel state unless pixel string changes
* Cleanup pin names to allow usage of "_" and "-" as in Duet3D version
* STM32F4: Allow pin D.13 to use hardware PWM

Version 3.4.1RC102
==================
* Duet3d 3.4.1RC1
* Updated build system to support multiple STM32 builds
* STM32H7 support
* STM32H7 Bootloader
* SBC RAM based IAP
* FD-CAN support
* Hardware (DMA) Neopixel support
* Support multiple Neopixel strings (via enhanced M950 commnd)
* DHT22 Support
* New USB CDC driver (Removes ST code)
* Allow configuration of sense resistor and max driver current (M569.8)
* Experimental diskless SBC mode (board.txt on SBC)
* Added Fly Super5
* Added Fly Gemini V2 board
* Added BTT SKR 3
* Removed old board.txt keys
* Increased various system limits
* Fixed BTT Octopus naming


Version 3.4.1RC1_2
==================
* Switched rsense and max current configuration to M569.8

Version 3.4.1RC1_1
==================
* Duet3d 3.4.1RC1
* Added Fly Gemini V2 board
* Increased various system limits
* Fixed BTT Octopus naming
* Allow configuration of sense resistor and max driver current

Version 3.4.0_2
===============
* Updated Gemini V1.1 pins

Version 3.4.0_1
===============
* Duet3d 3.4.0
* Updated pins files

Version 3.4rc2_1
================
* Duet3d 3.4rc2
* Retry SDIO initialization if switch to wide mode fails

Version 3.4rc1_1
================
* Duet 3D 3.4rc1
* Board fixes from v3.3.0_16

Version 3.4beta7+7_1
====================
* Duet 3D 3.4beta7+7

Version 3.4beta7_3
==================
* Fix number of drivers in gemini config
* Fix bug pin A.0 set to input mode during board init

Version 3.4beta7_2
==================
* Report supported boards as part of M122 p200 output
* Include chaanges from 3.3.0_14

Version 3.4beta7_1
==================
* Duet3d 3.4beta7

Version 3.4beta6_2
==================
* Increase heaters per tool to 4
* include changes from 3.3.0_12

Version 3.4.0beta6_1
====================
* Duet3d 3.3beta6
* Add changes from 3.3.0_11
* Include CCMRam usage in M122 output


Version 3.4.0beta5_1
====================
* Duet3D 3.3beta5
* Minor changes to M122 p200 format

Version 3.4.0beta4_1
====================
* Duet3D 3.4beta4
* SBC Increase stack space to allow more complex expressions

Version 3.4.0beta3+1_2
======================
* Duet3D 3.3beta3+1 changes
* pickup up changes from 3.3.0_8 and 3.3.0_9

Version 3.4beta2_2
==================
* Improve SBC robustness when connection is dropped and reconnects
* Pick up changes from 3.3.0_7

Version 3.3.0_16
================
* STM32F4 Fix Fly E3 Pro V2 Pins
* STM32F4 Added updated BTT SKR V2 signature
* STM32F4 Improved error handling for SDIO SD card interface
* STM32F4 Initial support for Fly Gemini V1.1

Version 3.3.0_15
================
* STM32F4 Fix number of drivers in Fly Gemini config

Version 3.3.0_14
================
* STM32F4 Update Fly Gemini board signature


Version 3.3.0_13
================
* STM32F4 Fix problems with using SDIO based SD cards in SBC mode


Version 3.3.0_12
================
* STM32F4 Add support for Fly Gemini
* STM32F4 Report usage of SPI device when channel has not been defined (avoid board reset)

Version 3.3.0_11
================
* STM32F4 Allow multiple names for same board configuration
* STM32F4 Added support for Fly CDYv3
* STM32F4 Added support for Fly E3 Pro v2
* STM32F4 Added support for BTT Octopus Pro v1 (STM32429 MCU)


Verison 3.3.0_10
===============
* LPC Fix bug that can leave software PWM running when it should be off

Version 3.3.0_9
===============
* Remove "_" from pin name definitions
* Fix problem with spaces in board.txt pin arrays
* Ignore "_" in pin names used in board.txt
* Output RAW ADC readings in M308 output when heat debug is enabled

Version 3.3.0_8
===============
* STM32F4 update Fly Super8 pins
* STM32F4 update Spider pins
* STM32F4 update Fly 407ZG pins
* Filter WiFi module debug output to remove garbage characters

Version 3.3.0_7
===============
* STM32F4 Fix occasional bad memory access when using software PWM

Version 3.4beta2_2
==================
* Duet3d V3.4beta2 with input shaping
* Disable support for LPC build
* STM32F4: Improved writer task interface (handle multiple buffers/tasks)
* STM32F4: Fix print sometimes stalls
* STM32F4: Fix deleting sys files when mass storage is not enabled, but linux is

Version 3.3.0_6
===============
* STM32F4: Fix occassional deadlock when writing accelerometer data to file
* STM32F4: Update BTT RRF E3 board definitions
* STM32F4: Fix crash when TMC5160 SPI channel not configured
* Allow use of named objects when using SBC build
* STM32F4: Initial support for Fly Super8
* STM32F4: Initial support for BTT Octopus (STM32429 MCU)

Version 3.3.0_5
===============
* Fix board sometimes resets after WiFi firmware upload
* New firmware filenames

Version 3.3.0_4
===============
* STM32F4: Add support for esp32 WiFi modules
* STM32F4: Fix Fly E3 SPI5 defaults
* STM32F4: Improve TMC5160 detection
* STM32F4: Retry TMC5160 SPI writes on error
* STM32F4: Fix spurious UART interrupts at boot
* Add new debug module for driver outputs
* Improve software PWM rounding at higher frequencies
* Improve revovery from SD card errors



Version 3.3_3
=============
* Allow setting of Neopixel timings with M150 X2 Tt0:t1:tc:tr
* Add support for RGBW Neopixels
* STM32F4: Fix SDIO timeout problem
* STM32F4: Enable support for 2 probes
* LPC: Enabled support for object tracking names
* STM32F4: Fix release of wrong external interrupt pin


Version 3.3_2
=============
* Fix problem with Spider UART pin


Version 3.3_1
=============
* Duet3d Version 3.3
* STM32F4: Fix pin usage when using non default SPI device for WiFi
* STM32F4: Initial support for Fysetc Spider (with STM32F407 mcu)
* STM32F4: Remove duplicate set of board definitions





Version 3.3-RC3_2
=================
* STM32F4: Reduced interupt overhead for Step and Software PWM timers
* STM32F4: Adjusted interrupt priority to reduce jitter on software PWM output
* STM32F4: Removed pin definitions for BTT-EXP-MOT and SKR Pro



Version 3.3-RC3_1
=================
* Duet3D version 3.3-RC3
* Fix firmware crash when using M574 with invalid S param
* STM32F4: Include board name in M122 output
* STM32F4: Fix null ptr reference when no Aux device
* STM32F4: Fix ensure Step Timer is running before first use
* STM32F4: Allow operation without bootloader
* LPC: Fix drivers that used i2c current setting not set if the build had TMC drivers enabled



Version 3.3-RC2+3_1
===================
* Duet3D version 3.3-RC2+3
* STM32F4: Add support for external SD cards
* LPC: Fix for board not booting with zero smart drivers


Version 3.3-RC2+1_1
===================
* WiFi Access point fix from Duet3D


Version 3.3-RC2_1
=================
* RRF V3.3-RC2
* STM32F4: New build stats
* STM32F4: Adjust GPIO timing and SPI pullups 


Version 3.3-RC1_1
==================
* RRF V3.3-RC1
* LPC: Adjust stack sizes
* Adjust parse stack to allow more complex operations (SBC)
* STM32F4: Make WiFi Firmware update I/O buffer available to DMA
* STM32F4: Disable BTT "anti-reverse protection" feature
* STM32F4: Fixed incorrect LCDCS pin for BTT SKR V2
* STM32F4: Allow configuration of WiFi/SBC SPI channels



version 3.3-beta3_2
===================
* Fix Neopixel configuration check
* STM32F4: Add checks for change on interrupt pin being unused
* STM32F4: Ensure step timer is initialised before first use
* STM32F4: Make reverse driver checks more robust
* STM32F4: Allow RRF to use any available UART/Pin for WiFi/Aux

Version 3.3-beta3_1
===================
* RRF V3.3-Beta3
* STM32F4 Enable accelerometer support
* STM32F4 New board.txt entry: accelerometer.spiChannel

Version 3.3-beta1_6
===================
* STM32F4: Add pins for BTT RRF E3 IDX board
* STM32F4: Increase number of extruders per tool to 4
* STM32F4: Enable object tracking
* STM32F4: Add driver checking
* STM32F4: Initial support for BTT SKR V2

Version 3.3-beta1_5
===================
STM32F4: Reduce SD card buffer size back to values used in Version 3.3-beta1_3

Version 3.3-beta1_4
===================
* STM32F4: Added additional signature for SKR pro boards
* STM32F4: Minor SDIO improvements and bug fixes
* STM32F4: Added board definition for FLY E3-Pro
* STM32F4: Allow VIn value to settle before first reading
* STM32F4: Fix for boards that ground MAX31855 T- pin (avoid short to ground error)
* STM32F4: Increase number of heat sensor SPI CS pins to 4
* STM32F4: Changed  BTT RRF E3 board name to be "biquskr_rrf_e3_1.1"

Version 3.3-beta1_3
===================
* Duet3D V3.3-beta1 support
* Switched mcu interface to CoreN2G
* Switch PinTable to 1:1 mapping. Allows all pins to be accessed using pin notation
* Updated board definitions for Fly/BTT
* STM32F4: Support for TMC5160 drivers
* STM32F4: Support for mixed TMC22xx/TMC5160
* STM32F4: Allow alternate pins for hardware SPI devices (via SPI0.Pins, SPI1.Pins, SPI2.Pins)
* STM32F4: Allow use of SPI3
* STM32F4: Add support for voltage monitor
* STM32F4: Added "generic" board config
* STM32F4: Allow use of SPI3 for SD card access
* STM32F4: Do not make potentially damaging changes with unrecognised hardware (during attempts to mount the SD card)


Version 3.2_7
=============
* Fix Fly CDY pin definition
* Allow use of SPI<n>.pins to configure SPI device pins in board.txt
* Preliminary support for the BTT RRF E3 (board name "biquskr_rrf_e3_1.0")




Version 3.2_6
=============
* Fix Fly CDYV2 board definitions
* Fix BTT E3-Turbo board definitions
* Add support for E3-Turbo driver sleep mode
* Allow use of "+" "-" "^" when defining pins for output high, output low, input pullup
* Fix bug with board.txt files that contain whitespace before key name
* Add version information to M122 P200 output



Version 3.2_4
=============
* Added aliases for config names that contain "lpc"
    "board" = "lpc.board"
    "8266wifi.TfrReadyPin" = "8266wifi.lpcTfrReadyPin"
    "sbc.TfrReadyPin" = "sbc.lpcTfrReadyPin"
* M122 P200 reports all available pin names
* Names defined with mixed case now work correctly
* Add support for MAX6675
* USB serial no longer waits for DTR (to match Duet)
* LPC Increase max gcode line length to 256
* LPC Increase number of open files to 6
* STM32F4 add support for LCD displays (ST7567 and ST7920)
* STM32F4 increse number of software SPI devices from 1 to 3
* STM32F4 improved software SPI performance
* STM32F4 add support for SPI modes 1, 2, 3 to Software SPI device
* STM32F4 improved UART driver
* STM32F4 report SDIO errors
* STM32F4 fix hardware PWM bug, PWM not disabled correctly
* STM32F4 Correct mcu temperature for VRef changes
* Add preliminary support for Fly CDYV2 board




Version 3.2_1
===============
* RRF V3.2 (See the RRF docs for details)
* STM32F4 use hardware for CRC32 calculations
* Fix board reset when attempting to update and empty ESP8266 device
* Fix SDIO based card capacity calculation
* Fix WiFi RX/TX pin assignments


Version 3.2-RC2
===============
* RRF V3.2-RC1 (See the RRF docs for details)
* LPC increased SBC code buffer size tp 2048 bytes



Version 3.2-RC2_2
===============
* RRF V3.2-RC2 (See the RRF docs for details)
* Support for SDIO based SD card readers
* Reduced memory usage for permanently allocated objects
* Dynamic allocation of DDA objects via M595 (Frees more memory on LPC, may need fine tuning)
* Add support for Fly 407 (fly_f407zg) and E3 (fly_e3) boards
* New LPC SBC Settings: "SpiTransferMode": 3, "SpiBufferSize": 3072, "MaxCodeBufferSize": 200, "MaxMessageLength": 2560
* Removed undescore from pin names

Version 3.2-beta4.1_1
=====================
* RRF v3.2-beta4.1 (See the RRF docs for details)
* LPC reduce memory usage of SBC version (to allow for beta4 features)
* LPC add support for BTT SKR E3 Turbo (name biquskr_e3t)
* STM Allow use of non standard CS pins for ESP8266 and SBC interfaces (new board.txt settings sbc.csPin and 8266wifi.csPin)
* STM Improved ADC accuracy via oversampling
* STM Fix heater off by one bug
* Experimental board identification by bootloader hash



Version 3.2-beta3.2_1
===================
* RRF v3.2-beta3.2 (See the RRF docs for details)
* LPC reduce number of probe points (to 16) to avoid firmware reset
* LPC Increased exception handler stack size (to avoid resets)
* LPC fix PWM interrupt priority
* TMC22xx detect and ignore missing drivers
* TMC22xx fix problem were settings are sometimes lost at startup
* TMC22xx change default operating mode to stealthchop to match Duet and doc
* Fix for watchdog timeout when creating a hiccup step
* Set inital ATX power state with atx.initialPowerOn (default is true).
* Enable/Disable TMC drivers based on ATX power (via M80/M81)
* Add support for NeoPixel RGB LEDs via m150 (new board.txt entry led.neopixelPin)
* LPC Add support for FLY-CDY board.




Version 3.2-beta2_2
===================
* RRF v3.2-beta2 (See the RRF docs for details)
* STM32 initial support for M5 expansion board.
* STM32 incresed number of drivers/fans etc.
* LPC increase number of motors per axis to 3.
* LPC adjust memory usage to allow for new SBC interface.
* Initial support for driver expansion board (SKR 1.3/1.4/Pro)
* Fix for SanDisk Sd card compatibility problem.


New Unified LPC/STM32F4 version
==================================
As of V3.2-beta1_2 both LPC and STM32F4 based boards are supported via mcu specific core components and a set of common sources. 

Version 3.2-beta1_2
===================
This is the first release using the new unified build system. It updates RRF to match the Duet3D 3.2-beta1 release.
* RRF v3.2-beta1 (see the RRF docs for details).
* LPC version now supports up to 7 stepper drivers.
* Increased number axis (for idex etc.).
* Updated FreeRTOS and DuetWiFiSocketServer





Summary of LPC specific changes
===============================================

Please note
============
The sample board.txt files which are here: https://github.com/gloomyandy/RepRapFirmware/tree/v3.01-dev-lpc/LPC/ExampleBoardConfig are out of date and may contain invalid settings. In addition the current software is not very good at detecting errors in these files and providing feedback to the user. Hopefully this will be improved soon, but for now the best way to check if the settings are correct is to issue an M122 p200 and check that the configuration matches your board. The current best source for the available settings is the configuration source: https://github.com/gloomyandy/RepRapFirmware/blob/v3.01-dev-lpc/src/LPC/BoardConfig.cpp#L33

Version 3.1.1-14
=============
This version contains only two changes...
* Fix for a bug that can result in changes to the PWM output used for Fans/Heaters not being made if the frequency of the output is changed at the same time. Often this willoccur the first time that the speed is set.
* The Ethernet build no longer has support for LCD displays enabled due to memory limitations.

Version 3.1.1-12
=============
This version contains a number of fixes and minor improvements following user testing.
* Detect ADC problems and attempt to fix them. In some situations the ADC does not seem to start operation correctly after reboot. This results in very high (2000 degrees plus) temperature readings. This change attempts to detect this and restart the ADC.
* UART 3 support is now enabled by default. This allows the WiFi UART interface and PanelDue to be used at the same time on some hardware (BTT SKR V1.4).
* Fix firmware reset caused by division by zero when setting the PWM frequency to zero.
* Added Azteeg X5 mini v2 and MKS SGen L (thanks to Jay_S).

Version 3.1.1-9
=============
This version contains changes to allow the updating of ESP8266 WiFi firmware via the serial port interface. Note that to use this change may require hardware changes to any adaptor boards you are using. Also includes updates to the USB serial port implementation to reduce memory and improve performance.


Version 3.1.1-8
=============
This version contains one major new feature, support for TMC2209 devices which includes stall detection and sensorless homing. However it also includes a number of other changes (some to support the TMC2209 feature, others to fix bugs or improve other features), these are detailed below:
* New board.txt setting stepper.TmcDiagPins = {<pin 0>, <pin 2>, ...} This setting is used to provide details of the pins used for the TMC2209 DIAG output. This must be provided for the drivers which use stall detection/sensorless homing. By Default no pins are defined.
* SerialUSB Writeable now returns the number of bytes that can be written. This will hopefully fix some of the watchdog timeouts caused by the firmware attempting to write data when no program was consuming the output. There may still be problems with debug output.
* New software PWM implementation. This new implementation has a lower overhead then the original version and is more robust to situations in which the timer is set to times that have already passed. 
* Fix watchdog timeout caused by PanelDue or other serial devices. This was caused by by the PanelDue output generating framing errors that were not cleared when enabling the UART.
* DMA completion interrupts. These have been reworked to allow devices to use completion interrupts that are called at a different interrupt level to that used by the main DMA device. This in turn allows us to use a higher priority interrupt for the DMA devices itself while still allowing the use of DMA with devices that make use of RTOS scheduling.
* Various build system changes. The new TMC2209 driver is now located in the LPC specific part of the tree, the build scripts have been updated for this and to preserve the map files for debug use.
* Flash accelerator configuration. Previous builds used settings that are in theory invalid for the LPC1769 chip.
* Zero allocated memory. Allocated memory was previously left uninitialised, this could cause unpredictable results in some situations.
* Adjusted interrupt priorities. This allows the TMC2209 driver to run the serial communications (via the software UART) at a higher baud rate, which allows changes to stall detection, motor current etc. to propagate to the drivers more rapidly.
* Increased GCode buffer size from 101 to 201 bytes to match Duet settings.
* Various debug enhancements, allow better stack capture/display, pwm debug display, USBSerial display.

Version 3.1.1
=============
* Updated to include bugfixes from DC42
* Fixed problem with default MAC address for Ethernet builds that resulted in some routers not providing an IP address.
* Added board configuration for Azteeg X5 Mini V3 (Thanks to Jay_s)

Version 3.1.0
============
Updated to the 3.1.0 release of RRF from DC42


Version 3.01 RC12+
=============
* New features
    * Added configuration option stepper.numSmartDrivers = <n> to allow the number of smart drivers (TMC22XX) to be limited. The drivers must be contiguous and start at unit 0. So if you have a BTT SKR V1.4 board with say 3 TMC2208s and 1 other driver, the 2208s must be in slots 0, 1, 2 and the remainiong driver in slot 3 or 4. You can use RRF to assign any of those slots to an axis/extruder.
    * Added configuration option SSP0.pins = {SCK, MISO, MOSI, CS} to allow using the alternate hardware pins on port 1 to be used for the SSP0 pins SCK can be 0.15/1.20, MISO 0.17/1.23, MOSI 0.18/1.24 and CS 0.16/1.21. This options allows a modified SBase board to be used with a standard WiFi firmware build.
* Known issues
    * Running with just a single TMC22XX driver will result in a large number of read timeouts.


Version 3.01 RC12
=============
* Updated to RC12 from DC42
* Added capability to update the LPC firmware via DWC when operating in SBC mode

Version 3.01 RC6
==============
* Updated with latest RC6 from DC42. 
* Various SBC communication parameters adjusted to bring them into line with the Duet.
* Network task stack size increased to avoid stack overflow resets

Version 3.01 RC5
==============
* Updated with latest RC5 from DC42. 


Version 3.01 RC4
==============
* Added initial experimental support for SBC interface

* Added configurable UART class to allow selecting UART from board config by using the RX/TX pins for AUX serial and WIFI Serial.
    * AUX serial pins will default to the UART0 pins as it has done in previous versions. Setting Aux serial to NoPin, NoPin can save a bit of memory as the buffers won't be created.
    * New board.txt entries:
        * 8266wifi.serialRxTxPins - array of pins {RX, TX} (Requires WIFI to be compiled into binary)
        * serial.aux.rxTxPins - array of pins {RX, TX}
        * serial.aux2.rxTxPins - array of pins {RX, TX}  (Requires AUX2 to be compiled into binary)

* Added board.txt entry "heat.spiTempSensorChannel" to select the spi channel for the SPI Temperature sensors. Defaults to SSP0.

### Changed board entries
For naming consistancy, the following board.txt entries have been changed:
| New  | Previous   |
|---|---|
| atx.powerPin  | atxPowerPin  |
| atx.powerPinInverted  | atxPowerPinInverted  |
| sdCard.internal.spiFrequencyHz | lpc.internalSDCard.spiFrequencyHz   |
| sdCard.external.csPin  | externalSDCard.csPin  |
| sdCard.external.cardDetectPin  | externalSDCard.cardDetectPin  |
| sdCard.external.spiFrequencyHz  | lpc.externalSDCard.spiFrequencyHz  |
| sdCard.external.spiChannel  | lpc.externalSDCard.spiChannel  |
| softwareSPI.pins  | lpc.softwareSPI.pins  |
| 8266wifi.espDataReadyPin  | 8266wifi.EspDataReadyPin  |
| 8266wifi.lpcTfrReadyPin  | 8266wifi.LpcTfrReadyPin  |
| 8266wifi.espResetPin| 8266wifi.EspResetPin |
| 8266wifi.serialRxTxPins  | 8266wifi.SerialRxTxPins  |
| sbc.lpcTfrReadyPin  | linuxTfrReadyPin  |
| adc.prefilter.enable  | lpc.adcEnablePreFilter  |
| adc.preFilter.numberSamples  | lpc.adcPreFilterNumberSamples |
| adc.preFilter.sampleRate | lpc.adcPreFilterSampleRate |

  


Version 3.01 RC3
================
* Added new build option TMC22XX set to true to provide support for TMC22XX drivers via the UART interface.
* Added new build option ESP8266WIFI_SERIAL (available when usinf ESP8266 WiFi) define this to use enable the UART interface to the ESP8266 board. Note that on SKR based boards only a single UART device is easily available and so to allow the use of panel due and other serial based control screens this option should not be enabled.
* New pin names. The pin names used for the various boards has been updated to provided a consistant naming policy across different boards.


Version 3.01 Beta2
=================

* Removed board.txt entry: stepper.hasDriverCurrentControl. Now relies on stepper.digipotFactor being set to enable current control.

Version 3.01 beta1
=================

* Added new board.txt entry "lpc.adcEnablePreFilter" to enable/disable the ADC prefilter. [Experimental]


Version 3.0 beta13
=================

* Implemented DMA. SharedSPI updated to use DMA for SSP and also blocks while waiting to complete (or timeout) so RTOS can process other tasks while watiting
* Removed lpc.uartPanelDueMode as M575 supports new raw mode from 3.0b13.
* Added new board.txt entries for ESP8266Wifi:
    * 8266wifi.EspDataReadyPin - this pin needs to be on Port 0 or Port 2 as it uses external interrupts
    * 8266wifi.LpcTfrReadyPin
    * 8266wifi.EspResetPin



Version 3.0 beta3
=================


### Board Config (board.txt)

* Removed Fan Pins, Heater Pins, Tacho pins and Endstops configuration options as they are now configurable by GCode (See GCode M950) and needs to be set in config.g (See https://duet3d.dozuki.com/Wiki/RepRapFirmware_3_overview).
* Removed SpecialPins as it's no longer required in RRF3. M950 is now used to create a "gpio port number", and that gpio port number is used in M42 and M280.
* Removed lpc.externalInterruptPins config. It is now done automatially when attempting to attach an interrupt to a pin on port 0 or port 2 (i.e. for Fan Tacho pin or configuring a filament detector).
* Removed lpc.slowPWM.pins, lpc.fastPWM.pins and lpc.servoPins. They are now assigned automatically when using M950.
* PWM for heaters and fans are now generated using a free running timer. This allows creating different PWM with frequencies instead of being limited to 2 options.  
* Added new entry: lpc.board. Currently supported values are: smoothieboard, rearm,  mkssbase_1.3, azsmzmini, biquskr_1.1, biquskr_1.3, azteegx5mini_1.1 and generic.  For example: lpc.board = smoothieboard;
    * Currently does not support control of drivers via UART/SPI on Biqu SKR 1.3. 
    * When defining a board, the names on the silk screen (or official pinout if there is no names on the silk) can be used in M950. LPC style of port.pin (i.e. 1.23) can also be used if desired, however.
    * For ReArm, the silk on the RAMPs shield is used and additionally support from the official pinout that uses arduino mega naming can be used, i.e. D8, etc.
    * For the above boards (except generic), some defaults are included in the firmware for stepper pins (en/step/dir), current control and will be used as the default. They can be overriden by defining those variables in board.txt.
    * The generic board has no defaults. 
* Added new atxPowerPinInverted entry which is set to true or false.  Set to true if the PSON needs to be inverted. Default is False.
* Added new lpc.externalSDCard.spiChannel to select which hardware SSP pins are used for SDCard. Must be 0 to select SSP0 or 1 to select SSP1. Default is SSP1.


Version 2.04RC1
=================

### Board Config (Board.txt)
* Software SPI added. New pin arrary option:  lpc.softwareSPI.pins
    * Assigns GPIO pins to operate as SoftwareSPI.
    * Default Value: lpc.softwareSPI.pins = {NoPin, NoPin, NoPin} ; //Set to GPIO pins to use as SCK, MISO, MOSI
* New SPI Channel options added for ExternalSDCard and LCD. Channel 0 = SSP0, 1 = SSP1 and 2 = SoftwareSPI
    * lpc.externalSDCard.spiChannel option added to select SPI Channel external SDCard uses.
    * lcd.spiChannel option added to select SPI Channel LCD uses (typically 0 or 2). Default



Version 2.03
=================

### SDCard

* Updated functions for SDCard and SharedSPI to improve performance now using ChaN 2018 routines. Includes 16bit block transfers and takes advantage of the LPC SSP 8 frame FIFOs. 
* Added detection for SDCards that support HighSpeed mode which enables the card interface SPI speeds up to 50MHz.
* Added check to ensure that if user enters a higher SPI frequency than the card can support to only use the max card speed.
* If users select a slower SPI speed than the card can handle, that speed will instead be used (i.e. to have slower SPI speeds over long cables for external SDCard slot if desired).
* Implemented SDCard read/write retries as RRF does.

### Networking

* Fixed a bug causing a buffer to not be freed after a read socket error.
* Added mutexes to the callbacks which are called from a different task (IPTask).
* Upgraded FreeRTOS+TCP to V2.0.11
* Added extra FreeRTOS priorty and changed priories so the EMac task has a higher priority than IPTask.   
* Removed the f_expand to preallocate space as it takes longer than the default network time out when uploading large files.
