CORE_DIR = $(CORESTM_DIR)


#Core
CORE_SRC_DIRS  = src src/STM32
CORE_SRC_DIRS += src/STM32/system src/STM32/system/STM32H7xx 
CORE_SRC_DIRS += src/STM32/system/Drivers/CMSIS/Device/ST/STM32H7xx/Include src/STM32/system/Drivers/CMSIS/Device/ST/STM32H7xx/Source
CORE_SRC_DIRS += src/STM32/system/Drivers/STM32H7xx_HAL_Driver/Inc src/STM32/system/Drivers/STM32H7xx_HAL_Driver/Src
CORE_SRC_DIRS += src/STM32/system/CMSIS/CMSIS/Core/Include src/STM32/system/Drivers/CMSIS/Device/ST/STM32H7xx/Source/Templates/gcc
CORE_SRC_DIRS += src/STM32/variants/$(VARIANT) src/STM32/ConfigurableUART src/STM32/SharedSPI src/STM32/PWM src/STM32/SDIO src/STM32/CRC32
ifneq ($(USETINYUSB), true)
    CORE_SRC_DIRS += src/STM32/usb2
endif

CORE_SRC = $(CORE_DIR) $(addprefix $(CORE_DIR)/, $(CORE_SRC_DIRS))
CORE_INCLUDES = $(addprefix -I, $(CORE_SRC))

#Find all c and c++ files for Core
CORE_OBJ_SRC_C    += $(foreach src, $(CORE_SRC), $(wildcard $(src)/*.c))
CORE_OBJ_SRC_CXX   += $(foreach src, $(CORE_SRC), $(wildcard $(src)/*.cpp))
CORE_OBJS = $(patsubst %.c,$(BUILD_DIR)/%.o,$(CORE_OBJ_SRC_C)) $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(CORE_OBJ_SRC_CXX))
CORE_OBJS += $(BUILD_DIR)/CoreN2G/src/STM32/startup_stm32yyxx.o
