

#Core
TINYUSB_SRC_DIRS  = src src/tinyusb/src src/tinyusb/src/class src/tinyusb/src/common src/tinyusb/src/device 
TINYUSB_SRC_DIRS += src/tinyusb/src/osal src/tinyusb/src/host src/tinyusb/src/portable
TINYUSB_SRC_DIRS += src/tinyusb/src/class/cdc src/tinyusb/src/portable/synopsys/dwc2

TINYUSB_SRC = $(TINYUSB_DIR) $(addprefix $(TINYUSB_DIR)/, $(TINYUSB_SRC_DIRS))
TINYUSB_INCLUDES = $(addprefix -I, $(TINYUSB_SRC))

#Find all c and c++ files for Core
TINYUSB_OBJ_SRC_C    += $(foreach src, $(TINYUSB_SRC), $(wildcard $(src)/*.c))
TINYUSB_OBJ_SRC_CXX   += $(foreach src, $(TINYUSB_SRC), $(wildcard $(src)/*.cpp))
TINYUSB_OBJS = $(patsubst %.c,$(BUILD_DIR)/%.o,$(TINYUSB_OBJ_SRC_C)) $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(TINYUSB_OBJ_SRC_CXX))
