##########################################################################################################################
# File automatically-generated by tool: [projectgenerator] version: [3.15.2] date: [Tue May 24 09:43:32 CST 2022] 
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version·
# ------------------------------------------------
# 注意！注意！注意！
# 文件夹名字中绝不可以有空格！！！
mkfile_path := $(dir $(abspath $(MAKEFILE_LIST)))  #获取当前正在执行的makefile的绝对路径的目录
cur_makefile_path := $(notdir $(patsubst %/,%,$(mkfile_path)))  #获取当前正在执行的makefile的绝对目录
current_dir := $(notdir $(shell pwd))
$(warning $(mkfile_path))
$(warning $(cur_makefile_path))
$(warning $(current_dir))
######################################
# target
######################################
PRE_TARGET = $(strip $(cur_makefile_path))

ifeq ("$(PRE_TARGET)" , "$(current_dir)")
 $(warning =======================>> equal <<=======================)
endif

ifneq ("$(PRE_TARGET)" , "$(current_dir)")
 $(error =======================>> file name error <<=======================)
endif

TARGET = $(PRE_TARGET)
# ROBOT = None_robot
ROBOT = Hero
######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -O1 -g


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build
BUILD_TEMP_DIR = temp
######################################
# source
######################################
# C sources
C_SOURCES =  \
Src/main.c \
Src/gpio.c \
Src/freertos.c \
Src/adc.c \
Src/can.c \
Src/dma.c \
Src/i2c.c \
Src/spi.c \
Src/tim.c \
Src/usart.c \
Src/usb_device.c \
Src/usbd_conf.c \
Src/usbd_desc.c \
Src/usbd_cdc_if.c \
Src/stm32f4xx_it.c \
Src/stm32f4xx_hal_msp.c \
Src/stm32f4xx_hal_timebase_tim.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c \
Src/system_stm32f4xx.c \
Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
Middlewares/Third_Party/FreeRTOS/Source/list.c \
Middlewares/Third_Party/FreeRTOS/Source/queue.c \
Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c \
Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
Middlewares/Third_Party/FreeRTOS/Source/timers.c \
Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c \
Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \
Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c \
Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c \
$(wildcard ./bsp/*.c) \
$(wildcard ./components/devices/*.c) \
$(wildcard ./components/controller/*.c) \
$(wildcard ./components/algorithm/*.c) \
$(wildcard ./Thread/*.c) \
$(wildcard ./application/*.c) \
$(wildcard ./application/$(ROBOT)/bsp/*.c) \
$(wildcard ./application/$(ROBOT)/Mode/*.c) \
$(wildcard ./application/$(ROBOT)/Thread/*.c) \
$(wildcard ./application/$(ROBOT)/Thread/ConfigTask/*.c) \
$(wildcard ./application/$(ROBOT)/Variables/*.c) \
$(wildcard ./application/$(ROBOT)/Devices/*.c)

# ASM sources
ASM_SOURCES =  \
startup_stm32f407xx.s


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DSTM32F407xx \
-D__VFP_FP__ \
-DARM_MATH_CM4 \
-D__FPU_USED=1U \
-D__FPU_PRESENT=1U \
-DARM_MATH_MATRIX_CHECK \
-DARM_MATH_ROUNDING


# AS includes
AS_INCLUDES =  \
-IInc

# C includes
C_INCLUDES =  \
-IInc \
-IDrivers/STM32F4xx_HAL_Driver/Inc \
-IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy \
-IMiddlewares/Third_Party/FreeRTOS/Source/include \
-IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS \
-IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \
-IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc \
-IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc \
-IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
-IDrivers/CMSIS/Include \
-Ibsp \
-Icomponents/algorithm \
-Icomponents/controller \
-Icomponents/devices \
-Icomponents/algorithm/Include \
-IThread \
-Iapplication \
-Iapplication/$(ROBOT) \
-Iapplication/$(ROBOT)/bsp \
-Iapplication/$(ROBOT)/Mode \
-Iapplication/$(ROBOT)/Thread \
-Iapplication/$(ROBOT)/Thread/ConfigTask \
-Iapplication/$(ROBOT)/Variables \
-Iapplication/$(ROBOT)/Devices

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g3 -gdwarf-2 -fvar-tracking
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F407IGHx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys -l:libAHRS.a -l:libarm_cortexM4lf_math.a 

LIBDIR = \
-Llib

LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections
# printf float
#LDFLAGS += -lc -lrdimon -u _printf_float

############################################################################################################################################################
# C++
############################################################################################################################################################
USE_CXX = 0
# C++ sources
CXX_SOURCES = \
$(wildcard ./Thread/*.cpp)

# C++ includes
CXX_INCLUDES = 

ifdef GCC_PATH
CXX = $(GCC_PATH)/$(PREFIX)g++
ASXX = $(GCC_PATH)/$(PREFIX)g++ -x assembler-with-cpp
else
CXX = $(PREFIX)g++
ASXX = $(PREFIX)g++ -x assembler-with-cpp
endif
#######################################
# C++FLAGS
#######################################
CXXFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(CXX_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -std=c++11 

ifeq ($(DEBUG), 1)
CXXFLAGS += -g -gdwarf-2
endif
# Generate dependency information
CXXFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"
############################################################################################################################################################
# C++ LDFLAGS
LDFLAGSXX = $(MCU) -specs=nano.specs -specs=nosys.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections  -lstdc++
############################################################################################################################################################

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of C objects
OBJECTS = $(addprefix $(BUILD_DIR)/$(BUILD_TEMP_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/$(BUILD_TEMP_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))
# list of C++ objects
ifeq ($(USE_CXX), 1)
OBJECTSXX = $(addprefix $(BUILD_DIR)/,$(notdir $(CXX_SOURCES:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CXX_SOURCES)))
endif

$(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR)
	$(CXX) -c $(CXXFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

$(BUILD_DIR)/$(BUILD_TEMP_DIR)/%.o: %.c  | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(BUILD_TEMP_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/$(BUILD_TEMP_DIR)/%.o: %.s  | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

ifeq ($(USE_CXX), 0)
$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) | $(BUILD_DIR)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@
endif
ifeq ($(USE_CXX), 1)
$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) $(OBJECTSXX) Makefile | $(BUILD_DIR)
	$(CC) $(OBJECTS) $(OBJECTSXX) $(LDFLAGSXX) -o $@
	$(SZ) $@
endif

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@
	mkdir $@/$(BUILD_TEMP_DIR)

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)
clean-bash:
	rm -rf  $(BUILD_DIR)
clean-cmd:
	-del $(BUILD_DIR)
remake:
	rm -rf $(BUILD_DIR)
	make all
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
