#---RepRapFirmware---
RRF_SRC_BASE  = $(REPRAPFIRMWARE_DIR)/src

RRF_SRC_DIRS = FilamentMonitors GCodes GCodes/GCodeBuffer Heating 
RRF_SRC_DIRS += Movement Movement/BedProbing Movement/Kinematics Movement/HeightControl
RRF_SRC_DIRS += Storage Libraries/sha1 Comms Platform PrintMonitor Accelerometers
RRF_SRC_DIRS += Heating/Sensors Fans ObjectModel Endstops Hardware Hardware/Spi Tools
RRF_SRC_DIRS += Display Display/Lcd Display/Lcd/Fonts Display/Lcd/ST7567 Display/Lcd/ST7920 GPIO bossa
RRF_SRC_DIRS += CAN ClosedLoop InputMonitors LedStrips
#STM RRF Addons
RRF_SRC_DIRS += Hardware/STM32 Hardware/STM32/Libraries/Fatfs
RRF_SRC_DIRS += Hardware/STM32/Hardware

#networking support?
ifeq ($(NETWORK), ETHERNET)
	$(info Ethernet is not supported on STM32H7:)
else ifeq ($(NETWORK), COMBINED) 
	RRF_SRC_DIRS += Networking Networking/ESP8266WiFi Hardware/STM32/Networking/ESP8266WiFi
	RRF_SRC_DIRS += Networking/MQTT Networking/MQTT/MQTT_C/src
	RRF_SRC_DIRS += Sbc
else ifeq ($(NETWORK), WIFI) 
	RRF_SRC_DIRS += Networking Networking/ESP8266WiFi Hardware/STM32/Networking/ESP8266WiFi
	RRF_SRC_DIRS += Networking/MQTT Networking/MQTT/MQTT_C/src
else ifeq ($(NETWORK), SBC)
	RRF_SRC_DIRS += Sbc Networking
#	RRF_SRC_DIRS += targets/common/NoNetwork
else
#	RRF_SRC_DIRS += targets/common/NoNetwork
endif

ifeq ($(TMC22XX), true)
	RRF_SRC_DIRS += Hardware/STM32/Movement/StepperDrivers
else ifeq ($(TMC51XX), true)
	RRF_SRC_DIRS += Hardware/STM32/Movement/StepperDrivers
endif

#Find the c and cpp source files
RRF_SRC = $(RRF_SRC_BASE) $(addprefix $(RRF_SRC_BASE)/, $(RRF_SRC_DIRS))
RRF_OBJ_SRC_C	   += $(foreach src, $(RRF_SRC), $(wildcard $(src)/*.c) ) 
RRF_OBJ_SRC_CXX   += $(foreach src, $(RRF_SRC), $(wildcard $(src)/*.cpp) )
ifeq ($(TMC22XX), true)
	RRF_OBJ_SRC_CXX += $(RRF_SRC_BASE)/Movement/StepperDrivers/DriverMode.cpp
else ifeq ($(TMC51XX), true)
	RRF_OBJ_SRC_CXX += $(RRF_SRC_BASE)/Movement/StepperDrivers/DriverMode.cpp
endif
RRF_OBJ_SRC_CXX += $(RRF_SRC_BASE)/Movement/StepperDrivers/DriverData.cpp

# use Duet versions of FatFs files
RRF_OBJ_SRC_C += $(RRF_SRC_BASE)/Libraries/Fatfs/ff.c
RRF_OBJ_SRC_C += $(RRF_SRC_BASE)/Libraries/Fatfs/ffunicode.c

RRF_OBJS = $(patsubst %.c,$(BUILD_DIR)/%.o,$(RRF_OBJ_SRC_C)) $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(RRF_OBJ_SRC_CXX))


RRF_INCLUDES = $(addprefix -I, $(RRF_SRC))
RRF_INCLUDES += -I$(RRF_SRC_BASE)/Libraries/

RRF_INCLUDES += -I$(RRF_SRC_BASE)/Libraries/Fatfs/

#If building ESP8266 WIFI we only need to add the include from DuetWifiSocketServer as it has a file needed to compile RRF 
ifeq ($(NETWORK), WIFI)
	RRF_INCLUDES += -IDuetWiFiSocketServer/src/include -I$(RRF_SRC_BASE)/Networking/MQTT/MQTT_C/include
else ifeq ($(NETWORK), COMBINED)
	RRF_INCLUDES += -IDuetWiFiSocketServer/src/include -I$(RRF_SRC_BASE)/Networking/MQTT/MQTT_C/include
endif
RRF_INCLUDES += -IIAP/src
#end RRF

#Libc and libcpp in RRF
RRFLIBC_SRC_DIRS = libcpp libc
RRFLIBC_SRC = $(addprefix $(RRF_SRC_BASE)/, $(RRFLIBC_SRC_DIRS))
RRFLIBC_OBJ_SRC_C	  += $(foreach src, $(RRFLIBC_SRC), $(wildcard $(src)/*.c) ) 
RRFLIBC_OBJ_SRC_CXX   += $(foreach src, $(RRFLIBC_SRC), $(wildcard $(src)/*.cpp) )
RRFLIBC_OBJ_SRC_CC    += $(foreach src, $(RRFLIBC_SRC), $(wildcard $(src)/*.cc) )

RRFLIBC_INCLUDES = $(addprefix -I, $(RRFLIBC_SRC))

RRFLIBC_OBJS = $(patsubst %.c,$(BUILD_DIR)/%.o,$(RRFLIBC_OBJ_SRC_C)) $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(RRFLIBC_OBJ_SRC_CXX))
RRFLIBC_OBJS += $(patsubst %.cc,$(BUILD_DIR)/%.o,$(RRFLIBC_OBJ_SRC_CC))


#


