PROCESSOR = STM32H7

VARIANT ?= STM32H743
BUILD_DIR ?= ./build
FREERTOS_DIR ?= ./FreeRTOS
REPRAPFIRMWARE_DIR ?= ./RepRapFirmware
RRFLIBRARIES_DIR ?= ./RRFLibraries
CORESTM_DIR ?= ./CoreN2G
CANLIB_DIR ?= CANlib

BUILD ?= Debug

NETWORK ?= WIFI

TMC22XX ?= true
TMC51XX ?= true

GCC_PATH ?=

STARTUP_DELAY ?= -DSTARTUP_DELAY=10000

#Comment out to show compilation commands (verbose)
V=@

$(info Building RepRapFirmware for STM32H7 based boards:)
$(info  - Variant: $(VARIANT))

ifeq ($(VARIANT),STM32H723)
	BASE_OUTPUT_NAME=firmware-stm32h723
else
	BASE_OUTPUT_NAME=firmware-stm32h743
endif


## Cross-compilation commands 
CC      = $(GCC_PATH)arm-none-eabi-gcc
CXX     = $(GCC_PATH)arm-none-eabi-g++
LD      = $(GCC_PATH)arm-none-eabi-gcc
AR      = $(GCC_PATH)arm-none-eabi-ar
AS      = $(GCC_PATH)arm-none-eabi-as
OBJCOPY = $(GCC_PATH)arm-none-eabi-objcopy
OBJDUMP = $(GCC_PATH)arm-none-eabi-objdump
SIZE    = $(GCC_PATH)arm-none-eabi-size

MKDIR = mkdir -p

$(info  - MAKE_DIR: $(MAKE_DIR))
include $(MAKE_DIR)/Core.mk
include $(MAKE_DIR)/FreeRTOS.mk
include $(MAKE_DIR)/RRFLibraries.mk
include $(MAKE_DIR)/CANLib.mk
include $(MAKE_DIR)/RepRapFirmware.mk

ifeq ($(BUILD),Debug)
	DEBUG_FLAGS = -Os -g -DSTM32_DEBUG $(STARTUP_DELAY)
        $(info - Build: Debug) 
else
	DEBUG_FLAGS = -Os
        $(info - Build: Release)
endif
	

#Linker script to avoid built in Bootloader
LINKER_SCRIPT_BASE = $(REPRAPFIRMWARE_DIR)/src/Hardware/STM32/variants/$(VARIANT)/ldscript



#Path to the linker Script
LINKER_SCRIPT  = $(LINKER_SCRIPT_BASE).ld
$(info  - Linker Script used: $(LINKER_SCRIPT))


#Flags common for Core in c and c++
FLAGS  = -D__$(PROCESSOR)__ -D_XOPEN_SOURCE -DSTM32 -DSTM32H7 -D$(VARIANT)xx -DSTM32H7xx

# UART configuration
FLAGS  += -DHAVE_HWSERIAL1 -DHAVE_HWSERIAL2 -DHAVE_HWSERIAL3 -DHAVE_HWSERIAL4 -DHAVE_HWSERIAL5 -DHAVE_HWSERIAL6 -DUSB_CDC_USE_DTR
FLAGS  += -DUSE_UART0=1 -DUSE_UART1=1 -DUSE_UART2=1 -DSUPPORT_USB=1
# SPI configuration
FLAGS  += -DUSE_SSP1=1 -DUSE_SSP2=1 -DUSE_SSP3=1 -DUSE_SSP4=1 -DUSE_SSP5=1 -DUSE_SSP6=1 -DUSE_SWSPI=1
# SDIO configuration
FLAGS  += -DUSE_SDIO=1
# General configuration
FLAGS += -DRTOS -DFREERTOS_USED -DUSBCON -DUSBD_USE_CDC -DUSB_PRODUCT=\"RepRapFirmware\" -DVECT_TAB_OFFSET=0x20000
#FLAGS += -DRTOS -DFREERTOS_USED -DUSBCON -DUSBD_USE_CDC -DUSB_PRODUCT=\"RepRapFirmware\" -DVECT_TAB_OFFSET=0x00000
FLAGS += -DHAL_RRF=1 -DFORCE_ESTACK=1 -DSUPPORT_CAN=1
FLAGS +=  -Wall -c -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mfp16-format=ieee -mcpu=cortex-m7 -mthumb -fno-math-errno -mno-unaligned-access -ffunction-sections -fdata-sections
FLAGS += -nostdlib -Wdouble-promotion -fsingle-precision-constant -fstack-usage
#FLAGS += -Wfloat-equal
#FLAGS += -Wundef
FLAGS += $(DEBUG_FLAGS)
FLAGS += -MMD -MP 

ifeq ($(NETWORK), WIFI)
        $(info  - Networking: WIFI + SBC) 
        FLAGS += -DESP8266WIFI -DMQTT_USE_CUSTOM_SOCKET_HANDLE -DMQTTC_PAL_FILE="Networking/MQTT/mqtt_pal.h"
		OUTPUT_NAME =$(BASE_OUTPUT_NAME)-wifi
else ifeq ($(NETWORK), COMBINED)
        $(info  - Networking: ESP WIFI) 
        FLAGS += -DCOMBINEDFW -DMQTT_USE_CUSTOM_SOCKET_HANDLE -DMQTTC_PAL_FILE="Networking/MQTT/mqtt_pal.h"
		OUTPUT_NAME =$(BASE_OUTPUT_NAME)
else ifeq ($(NETWORK), SBC)
        $(info  - SBC Interface Enabled)
        FLAGS += -DLPC_SBC
		OUTPUT_NAME =$(BASE_OUTPUT_NAME)-sbc
else
        $(info  - Networking: None)
		OUTPUT_NAME =$(BASE_OUTPUT_NAME)
endif


ifeq ($(TMC22XX), true)
        $(info  - Smart Drivers: TMC22XX)
        FLAGS += -DSUPPORT_TMC22xx
endif
ifeq ($(TMC51XX), true)
        $(info  - Smart Drivers: TMC51XX)
        FLAGS += -DSUPPORT_TMC51xx
endif
$(info - Output name: $(OUTPUT_NAME))

CFLAGS   = $(FLAGS) -std=gnu11 -fgnu89-inline -Dnoexcept=
CXXFLAGS = $(FLAGS) -std=gnu++17 -fno-threadsafe-statics -fexceptions -fno-rtti -Wno-register
CXXFLAGS_RRFL = $(FLAGS) -std=gnu++17 -fno-threadsafe-statics -fno-exceptions -fno-rtti -Wno-register
CXXFLAGS_CORE = $(FLAGS) -std=gnu++17 -fno-threadsafe-statics -fno-exceptions -fno-rtti -Wno-register
CXXFLAGS_CANLIB = $(FLAGS) -std=gnu++17 -fno-threadsafe-statics -fno-exceptions -fno-rtti -Wno-register


#all Includes (RRF + Core)
INCLUDES = $(CORE_INCLUDES) $(RRFLIBRARIES_INCLUDES) $(RRF_INCLUDES) $(RRFLIBC_INCLUDES) $(CANLIB_INCLUDES)


DEPS = $(CORE_OBJS:.o=.d)
DEPS += $(RRF_OBJS:.o=.d)
DEPS += $(RRFLIBC_OBJS:.o=.d)
DEPS += $(RRFLIBRARIES_OBJS:.o=.d)
DEPS += $(CANLIB_OBJS:.o=.d)

default: all

all: firmware

-include $(DEPS)

firmware:  $(BUILD_DIR)/$(OUTPUT_NAME).elf

coreSTM: $(BUILD_DIR)/core.a

$(BUILD_DIR)/libSTMCore.a: $(CORE_OBJS)
	$(V)$(AR) rcs $@ $(CORE_OBJS)
	@echo "\nBuilt STMCore\n"
	
$(BUILD_DIR)/libRRFLibraries.a: $(RRFLIBRARIES_OBJS)
	$(V)$(AR) rcs $@ $(RRFLIBRARIES_OBJS)
	@echo "\nBuilt RRF Libraries\n"

#$(BUILD_DIR)/$(OUTPUT_NAME).elf: $(BUILD_DIR)/src/CoreMain.o $(BUILD_DIR)/libSTMCore.a $(BUILD_DIR)/libRRFLibraries.a $(RRFLIBC_OBJS)
#	$(V)$(LD) -L$(BUILD_DIR)/ -L$(CORE_DIR)/variants/BIGTREE_SKR_PRO_1v1/ -L$(CORE_DIR)/CMSIS/CMSIS/DSP/Lib/GCC --specs=nano.specs -Os -Wl,--warn-section-align -Wl,--fatal-warnings -fmerge-all-constants -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mcpu=cortex-m4 -mthumb -T$(LINKER_SCRIPT) -Wl,-Map,$(BUILD_DIR)/$(OUTPUT_NAME).map -o $(BUILD_DIR)/$(OUTPUT_NAME).elf -Wl,--cref -Wl,--check-sections -Wl,--gc-sections,--relax -Wl,--entry=Reset_Handler -Wl,--unresolved-symbols=report-all -Wl,--warn-common -Wl,--warn-section-align -Wl,--warn-unresolved-symbols -Wl,--defsym=LD_MAX_SIZE=1048576 -Wl,--defsym=LD_MAX_DATA_SIZE=196608 -Wl,--defsym=LD_FLASH_OFFSET=0x0 -Wl,--start-group $(BUILD_DIR)/src/CoreMain.o -lSTMCore -lRRFLibraries -larm_cortexM4l_math -lc -lm -lgcc -lstdc++ -Wl,--end-group

$(BUILD_DIR)/$(OUTPUT_NAME).elf: $(BUILD_DIR)/libSTMCore.a $(BUILD_DIR)/libRRFLibraries.a $(CANLIB_OBJS) $(RRFLIBC_OBJS) $(RRF_OBJS)
	@echo "\nCreating $(OUTPUT_NAME).bin"
	$(V)$(MKDIR) $(dir $@)
	$(V)$(LD) -L$(BUILD_DIR)/ -L$(CORE_DIR)/src/STM32F4/variants/$(VARIANT)/ -L$(CORE_DIR)/src/STM32/system/CMSIS/CMSIS/DSP/Lib/GCC --specs=nosys.specs -Xlinker -z -Xlinker muldefs -Os -Wl,--fatal-warnings -fmerge-all-constants -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mfp16-format=ieee -mcpu=cortex-m7 -mthumb -fno-math-errno -T$(LINKER_SCRIPT) -Wl,-Map,$(BUILD_DIR)/$(OUTPUT_NAME).map -o $(BUILD_DIR)/$(OUTPUT_NAME).elf -Wl,--cref -Wl,--check-sections -Wl,--gc-sections,--relax -Wl,--entry=Reset_Handler -Wl,--unresolved-symbols=report-all -Wl,--warn-common -Wl,--warn-unresolved-symbols -Wl,--no-warn-rwx-segment -Wl,--defsym=LD_MAX_SIZE=2097152 -Wl,--defsym=LD_MAX_DATA_SIZE=524288 -Wl,--defsym=LD_FLASH_OFFSET=0x0 -Wl,--start-group $(RRFLIBC_OBJS) -lSTMCore $(RRF_OBJS) $(CANLIB_OBJS) -lRRFLibraries -larm_cortexM7l_math -lc -lm -lgcc -lstdc++ -Wl,--end-group
	$(V)$(OBJCOPY) --strip-unneeded -O binary $(BUILD_DIR)/$(OUTPUT_NAME).elf $(BUILD_DIR)/$(OUTPUT_NAME).bin
	$(V)$(SIZE) $(BUILD_DIR)/$(OUTPUT_NAME).elf
	-@$(MAKE_DIR)/staticMemStats.sh $(BUILD_DIR)/$(OUTPUT_NAME)
	
$(BUILD_DIR)/%.o: %.c
	@echo "[$<]"
	$(V)$(MKDIR) $(dir $@)
	$(V)$(CC)  $(CFLAGS) $(DEFINES) $(INCLUDES) -MMD -MP -MM -MF $(patsubst %.o,%.d,$@) $<
	$(V)$(CC)  $(CFLAGS) $(DEFINES) $(INCLUDES) -MMD -MP -o $@ $<

$(BUILD_DIR)/$(RRFLIBRARIES_DIR)/%.o : $(RRFLIBRARIES_DIR)/%.cpp
	@echo "RRFL [$<]"
	$(V)$(MKDIR) $(dir $@)
	$(V)$(CXX) $(CXXFLAGS_RRFL) $(DEFINES) $(INCLUDES) -MMD -MP -MM -MF $(patsubst %.o,%.d,$@) $<
	$(V)$(CXX) $(CXXFLAGS_RRFL) $(DEFINES) $(INCLUDES) -MMD -MP -o $@ $<

$(BUILD_DIR)/$(CORESTM_DIR)/%.o : $(CORESTM_DIR)/%.cpp
	@echo "CORE [$<]"
	$(V)$(MKDIR) $(dir $@)
	$(V)$(CXX) $(CXXFLAGS_CORE) $(DEFINES) $(INCLUDES) -MMD -MP -MM -MF $(patsubst %.o,%.d,$@) $<
	$(V)$(CXX) $(CXXFLAGS_CORE) $(DEFINES) $(INCLUDES) -MMD -MP -o $@ $<

$(BUILD_DIR)/$(CANLIB_DIR)/%.o : $(CANLIB_DIR)/%.cpp
	@echo "CANLIB [$<]"
	$(V)$(MKDIR) $(dir $@)
	$(V)$(CXX) $(CXXFLAGS_CANLIB) $(DEFINES) $(INCLUDES) -MMD -MP -MM -MF $(patsubst %.o,%.d,$@) $<
	$(V)$(CXX) $(CXXFLAGS_CANLIB) $(DEFINES) $(INCLUDES) -MMD -MP -o $@ $<

$(BUILD_DIR)/%.o: %.cpp
	@echo "[$<]"
	$(V)$(MKDIR) $(dir $@)
	$(V)$(CXX) $(CXXFLAGS) $(DEFINES) $(INCLUDES) -MMD -MP -MM -MF $(patsubst %.o,%.d,$@) $<
	$(V)$(CXX) $(CXXFLAGS) $(DEFINES) $(INCLUDES) -MMD -MP -o $@ $<

$(BUILD_DIR)/%.o: %.cc
	@echo "[$<]"
	$(V)$(MKDIR) $(dir $@)
	$(V)$(CXX) $(CXXFLAGS) $(DEFINES) $(INCLUDES) -MMD -MP -MM -MF $(patsubst %.o,%.d,$@) $<
	$(V)$(CXX) $(CXXFLAGS) $(DEFINES) $(INCLUDES) -MMD -MP -o $@ $<

$(BUILD_DIR)/$(CORESTM_DIR)/%.o: $(CORESTM_DIR)/%.S
	@echo "[$<]"
	$(V)$(MKDIR) $(dir $@)
	$(V)$(CXX) -x assembler-with-cpp $(CXXFLAGS) $(DEFINES) $(INCLUDES) -MMD -MP -MM -MF $(patsubst %.o,%.d,$@) $<
	$(V)$(CXX) -x assembler-with-cpp $(CXXFLAGS) $(DEFINES) $(INCLUDES) -MMD -MP -o $@ $<

$(BUILD_DIR)/%.o: %.S
	@echo "[$<]"
	$(V)$(MKDIR) $(dir $@)
	$(V)$(CXX) -x assembler-with-cpp $(CXXFLAGS) $(DEFINES) $(INCLUDES) -MMD -MP -MM -MF $(patsubst %.o,%.d,$@) $<
	$(V)$(CXX) -x assembler-with-cpp $(CXXFLAGS) $(DEFINES) $(INCLUDES) -MMD -MP -o $@ $<

cleanrrf:
	-rm -f $(RRF_OBJS)  $(BUILD_DIR)/libRRFLibraries.a
	
cleancore:
	-rm -f $(CORE_OBJS) $(BUILD_DIR)/libLPCCore.a

cleanrrflibraries:
	-rm -f $(RRFLIBRARIES_OBJS) $(BUILD_DIR)/libRRFLibraries.a

clean: distclean

distclean:
	-rm -rf $(BUILD_DIR)/ 

upload:
	ST-LINK_CLI.exe -c SWD Hrst UR -P "$(BUILD_DIR)/$(OUTPUT_NAME).bin" 0x8020000 -ClrBP -NoPrompt -Rst -Run

.PHONY: all firmware clean distclean $(BUILD_DIR)/$(OUTPUT_NAME).elf
