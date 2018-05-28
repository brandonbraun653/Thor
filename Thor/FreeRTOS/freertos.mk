####TODO: 
# 1. Make checks for the correct floating point support

# GNU Make Standard Library: https://gmsl.sourceforge.io/ 
include /usr/bin/gmsl
include /usr/bin/colors

ifndef PROJECT_BUILD_ROOT
$(error Please define the build directory in "PROJECT_BUILD_ROOT")
endif

# Figure out the file directory so we can locate build outputs correctly
THOR_ROOT        ?= $(dir $(filter %thor.mk, $(MAKEFILE_LIST)))
RTOS_ROOT        := $(dir $(filter %freertos.mk, $(MAKEFILE_LIST)))
RTOS_BUILD_ROOT  := $(PROJECT_BUILD_ROOT)
RTOS_DBG_DIR     := $(addprefix $(RTOS_BUILD_ROOT), debug/freertos/)
RTOS_RLS_DIR     := $(addprefix $(RTOS_BUILD_ROOT), release/freertos/)

STM32_DEVICE        ?= $(strip $(call substr, $(STM32_PROCESSOR),1,10))xx
STM32_DEVICE_FAMILY ?= $(strip $(call substr, $(STM32_DEVICE),1,8))

###########################################################
# Directories 
###########################################################
$(shell mkdir -p $(RTOS_DBG_DIR))
$(shell mkdir -p $(RTOS_RLS_DIR))

RTOS_INC_DIRS = $(RTOS_ROOT)include/
RTOS_SRC_DIRS = $(RTOS_ROOT) $(RTOS_ROOT)portable/Common/

# Add directories for the STM32F7 series
ifeq ($(filter %f7, $(STM32_DEVICE_FAMILY)),stm32f7)
RTOS_INC_DIRS += $(addprefix $(RTOS_ROOT), portable/GCC/ARM_CM7/r0p1/)
RTOS_SRC_DIRS += $(addprefix $(RTOS_ROOT), portable/GCC/ARM_CM7/r0p1/)
endif

# Add directories for the STM32F4 series
ifeq ($(filter %f4, $(STM32_DEVICE_FAMILY)), stm32f4)
RTOS_INC_DIRS += $(addprefix $(RTOS_ROOT), portable/GCC/ARM_CM4F/)
RTOS_SRC_DIRS += $(addprefix $(RTOS_ROOT), portable/GCC/ARM_CM4F/)
endif


###########################################################
# Files
###########################################################
RTOS_INC_FILES  = $(foreach dir, $(RTOS_INC_DIRS), $(wildcard $(dir)*.h))
RTOS_SRC_FILES  = $(foreach dir, $(RTOS_SRC_DIRS), $(wildcard $(dir)*.c))

# Specify the kind of memory allocation strategy to be used. See here for info: https://www.freertos.org/a00111.html 
RTOS_SRC_FILES += $(RTOS_ROOT)portable/MemMang/heap_4.c

RTOS_OBJ_FILES := $(patsubst %.c, %.o, $(RTOS_SRC_FILES))

# Let Make know where all the .c files are at
vpath %.c $(sort $(dir $(RTOS_SRC_FILES)))

###########################################################
# Build vars
###########################################################
# Generates the expected .o files that should be built
RTOS_OBJECTS_DBG := $(sort $(addprefix $(RTOS_DBG_DIR), $(notdir $(RTOS_OBJ_FILES))))
RTOS_OBJECTS_RLS := $(sort $(addprefix $(RTOS_RLS_DIR), $(notdir $(RTOS_OBJ_FILES))))

# Basic compiler & linker options common to all builds
RTOS_INCLUDES := $(addprefix -I, $(RTOS_INC_DIRS))
RTOS_CFLAGS    = -c -fno-common -fmessage-length=0 -Wall -fno-exceptions -ffunction-sections -fdata-sections
RTOS_CDEFS     = 

# On debug target, add these parameters 
rtos_debug: RTOS_CFLAGS  += -ggdb -O0
rtos_debug: RTOS_CDEFS   += -DDEBUG=1

# On release target, add these parameters 
rtos_release: RTOS_CFLAGS  += -O3
rtos_release: RTOS_CDEFS   += -DNDEBUG=1 -DRELEASE=1

###########################################################
# Recipes
###########################################################
.PHONY: rtos_all rtos_release rtos_debug rtos_clean rtos_test
rtos_all: rtos_release rtos_debug

rtos_release: $(RTOS_OBJECTS_RLS)

rtos_debug: $(RTOS_OBJECTS_DBG)

rtos_clean:
	@rm -f $(RTOS_DBG_DIR)*
	@rm -f $(RTOS_RLS_DIR)*
	$(call colorecho, $(GREEN), FreeRTOS Build Files Cleaned)

rtos_test:
	@echo $(sort $(dir $(RTOS_SRC_FILES)))
	@echo 

#------------------------------------------
# Primary build recipes, triggered off of $(RTOS_OBJECTS_xxx)
#------------------------------------------
$(RTOS_DBG_DIR)%.o: %.c $(addprefix $(RTOS_ROOT), include/FreeRTOSConfig.h)
	$(call colorecho, $(CYAN), Compiling $(notdir $<) into $@)
	@$(CC) $(RTOS_CFLAGS) $(USER_CFLAGS) $(RTOS_CDEFS) -std=gnu11 $(RTOS_INCLUDES) $< -o $@ 

$(RTOS_RLS_DIR)%.o: %.c $(addprefix $(RTOS_ROOT), include/FreeRTOSConfig.h)
	$(call colorecho, $(CYAN), Compiling $(notdir $<) into $@)
	@$(CC) $(RTOS_CFLAGS) $(USER_CFLAGS) $(RTOS_CDEFS) -std=gnu11 $(RTOS_INCLUDES) $< -o $@ 