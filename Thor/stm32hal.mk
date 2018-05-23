####TODO: 
# 1. Make checks for the correct floating point support
# 2. 


# GNU Make Standard Library: https://gmsl.sourceforge.io/ 
include /usr/bin/gmsl
include /usr/bin/colors

ifndef PROJECT_BUILD_ROOT
$(error Please define the build directory in "PROJECT_BUILD_ROOT")
endif

# Figure out the file directory so we can locate build outputs correctly
THOR_ROOT         ?= $(dir $(filter %thor.mk, $(MAKEFILE_LIST)))
STM32_BUILD_ROOT  := $(PROJECT_BUILD_ROOT)
STM32_HAL_DBG_DIR := $(addprefix $(STM32_BUILD_ROOT), debug/hal/)
STM32_HAL_RLS_DIR := $(addprefix $(STM32_BUILD_ROOT), release/hal/)

# Figure out all the details of which device we are using. This configures the
# HAL library properly.
STM32_DEVICE            = $(strip $(call substr, $(STM32_PROCESSOR),1,10))xx
STM32_DEVICE_UC         = $(call uc, $(strip $(call substr, $(STM32_PROCESSOR),1,10)))xx
STM32_DEVICE_FAMILY    := $(strip $(call substr, $(STM32_DEVICE),1,8))
STM32_DEVICE_FAMILY_UC := $(call uc, $(strip $(STM32_DEVICE_FAMILY)))

###########################################################
# Directories 
###########################################################
$(shell mkdir -p $(STM32_HAL_DBG_DIR))
$(shell mkdir -p $(STM32_HAL_RLS_DIR))

##### HAL #####
ifneq ($(shell test -d $(STM32_HAL_ROOT)/Drivers && echo -n yes),yes)
$(error Did you specify the correct HAL directory? Could not find "$(STM32_HAL_ROOT)/Drivers")
endif 

# Configuration files for what peripherals are to be enabled 
STM32_DEVICE_CONF_DIR := $(addprefix $(THOR_ROOT), /Thor/device/conf/)

# Chip register and data structure definitions for multiple device families
STM32_DEVICE_SYS_DIR  := $(addprefix $(THOR_ROOT), /Thor/device/sys/)

# Startup initialization settings
STM32_DEVICE_STARTUP_DIR := $(addprefix $(THOR_ROOT), /Thor/device/startup/)

HAL_DIR      := $(filter %xx_HAL_Driver, $(wildcard $(STM32_HAL_ROOT)/Drivers/*))
HAL_INC_DIRS := $(HAL_DIR)/Inc/ $(HAL_DIR)/Inc/Legacy/ $(STM32_DEVICE_CONF_DIR)
HAL_SRC_DIRS := $(HAL_DIR)/Src/ $(HAL_DIR)/Src/Legacy/

##### CMSIS ##### 
CMSIS_DIR := $(filter %CMSIS, $(wildcard $(STM32_HAL_ROOT)/Drivers/*))

# Gets the correct CMSIS directory based on device family name (i.e. STM32Fx)
CMSIS_DEVICE_INC_DIR := $(CMSIS_DIR)/Device/ST/$(call uc, $(STM32_DEVICE_FAMILY))xx/Include/
ifneq ($(shell test -d $(CMSIS_DEVICE_INC_DIR) && echo -n yes),yes)
$(error Did you specify the correct STM32 device? Could not find "$(CMSIS_DEVICE_INC_DIR)")
endif 

CMSIS_INC_DIRS := $(CMSIS_DEVICE_INC_DIR) $(CMSIS_DIR)/Include/
CMSIS_SRC_DIRS := $(STM32_DEVICE_SYS_DIR)

##### Generic STM32 #####
STM32_INC_DIRS := $(HAL_INC_DIRS) $(CMSIS_INC_DIRS)
STM32_SRC_DIRS := $(HAL_SRC_DIRS) $(CMSIS_SRC_DIRS)

###########################################################
# Files
###########################################################
# Header files that are chip family and sub-family specific and contain most of the critical definitions.
STM32_DEVICE_INC_FILES := $(STM32_DEVICE_CONF_DIR)$(STM32_DEVICE_FAMILY)xx_hal_conf.h $(CMSIS_DEVICE_INC_DIR)$(STM32_DEVICE).h

STM32_INC_FILES := $(foreach dir, $(STM32_INC_DIRS), $(wildcard $(dir)*.h))
STM32_SRC_FILES  = $(foreach dir, $(STM32_SRC_DIRS), $(wildcard $(dir)*.c))

# Device specific startup file
STM32_SRC_FILES += $(STM32_DEVICE_STARTUP_DIR)$(STM32_DEVICE_FAMILY_UC)/startup_$(STM32_DEVICE).c
STM32_OBJ_FILES := $(patsubst %.c, %.o, $(STM32_SRC_FILES))


###########################################################
# Build vars
###########################################################
# Generates the expected .o files that should be built
STM32_OBJECTS_DBG := $(sort $(addprefix $(STM32_HAL_DBG_DIR), $(notdir $(STM32_OBJ_FILES))))
STM32_OBJECTS_RLS := $(sort $(addprefix $(STM32_HAL_RLS_DIR), $(notdir $(STM32_OBJ_FILES))))

# Basic compiler & linker options common to all builds
STM32_INCLUDES := $(addprefix -I, $(STM32_INC_DIRS))
STM32_CFLAGS    = -c -fno-common -fmessage-length=0 -Wall -fno-exceptions -ffunction-sections -fdata-sections
STM32_CDEFS     = -DUSE_FULL_LL_DRIVER -D$(STM32_DEVICE_UC)

# On debug target, add these parameters 
hal_debug: STM32_CFLAGS  += -ggdb -O0
hal_debug: STM32_CDEFS   += -DDEBUG=1 -DDEBUG_DEFAULT_INTERRUPT_HANDLERS

# On release target, add these parameters 
hal_release: STM32_CFLAGS  += -O3
hal_release: STM32_CDEFS   += -DNDEBUG=1 -DRELEASE=1

# Let Make know where all the .c files are
vpath %.c $(sort $(dir $(STM32_SRC_FILES)))


###########################################################
# Recipes
###########################################################
.PHONY: hal_all hal_release hal_debug hal_clean 
hal_all: hal_release hal_debug

hal_release: $(STM32_OBJECTS_RLS)

hal_debug: $(STM32_OBJECTS_DBG)

hal_clean:
	@rm -f $(STM32_HAL_DBG_DIR)*
	@rm -f $(STM32_HAL_RLS_DIR)*
	$(call colorecho, $(GREEN), STM32 HAL Build Files Cleaned)

hal_test:
	@echo $(STM32_DEVICE_INC_FILES)

#------------------------------------------
# Primary build recipes, triggered off of $(STM32_OBJECTS_xxx)
#------------------------------------------
$(STM32_HAL_RLS_DIR)%.o: %.c $(STM32_DEVICE_INC_FILES)
	$(call colorecho, $(CYAN), Compiling $(notdir $<) into $@)
	@$(CC) $(STM32_CFLAGS) $(USER_CFLAGS) $(STM32_CDEFS) -std=gnu11 $(STM32_INCLUDES) $< -o $@

$(STM32_HAL_DBG_DIR)%.o: %.c $(STM32_DEVICE_INC_FILES)
	$(call colorecho, $(CYAN), Compiling $(notdir $<) into $@)
	@$(CC) $(STM32_CFLAGS) $(USER_CFLAGS) $(STM32_CDEFS) -std=gnu11 $(STM32_INCLUDES) $< -o $@