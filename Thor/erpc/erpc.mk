# GNU Make Standard Library: https://gmsl.sourceforge.io/ 
include /usr/bin/gmsl
include /usr/bin/colors


ifndef PROJECT_BUILD_ROOT
$(error Please define the build directory in "PROJECT_BUILD_ROOT")
endif

# Figure out the file directory so we can locate build outputs correctly
THOR_ROOT        ?= $(dir $(filter %thor.mk, $(MAKEFILE_LIST)))
RTOS_ROOT 		 ?= $(dir $(filter %freertos.mk, $(MAKEFILE_LIST)))
ERPC_ROOT        := $(dir $(filter %erpc.mk, $(MAKEFILE_LIST)))
ERPC_BUILD_ROOT  := $(PROJECT_BUILD_ROOT)
ERPC_DBG_DIR     := $(addprefix $(ERPC_BUILD_ROOT), debug/erpc/)
ERPC_RLS_DIR     := $(addprefix $(ERPC_BUILD_ROOT), release/erpc/)

STM32_DEVICE        ?= $(strip $(call substr, $(STM32_PROCESSOR),1,10))xx
STM32_DEVICE_FAMILY ?= $(strip $(call substr, $(STM32_DEVICE),1,8))

###########################################################
# Directories 
###########################################################
$(shell mkdir -p $(ERPC_DBG_DIR))
$(shell mkdir -p $(ERPC_RLS_DIR))

# eRPC
ERPC_SRC_DIRS  = $(addprefix $(ERPC_ROOT), bin/ config/ infra/ port/ setup/ transports/)
ERPC_INC_DIRS  = $(addprefix $(ERPC_ROOT), bin/ config/ infra/ port/ setup/ transports/)


ERPC_INC_DIRS += $(THOR_ROOT) $(RTOS_INC_DIRS) $(STM32_INC_DIRS) $(EIGEN_ROOT) $(BOOST_ROOT)

###########################################################
# Files
###########################################################
#ERPC_INC_FILES = $(foreach dir, $(ERPC_INC_DIRS), $(wildcard $(dir)*.h))
#ERPC_INC_FILES += $(addprefix $(THOR_ROOT), /include/thor.hpp /include/definitions.hpp /include/serial.hpp)
#ERPC_INC_FILES += $(addprefix $(THOR_ROOT), /FreeRTOS/include/FreeRTOS.h /FreeRTOS/include/semphr.h)

ERPC_SRC_FILES  = $(foreach dir, $(ERPC_SRC_DIRS), $(wildcard $(dir)*.cpp))
ERPC_OBJ_FILES := $(patsubst %.cpp, %.o, $(ERPC_SRC_FILES))


###########################################################
# Build vars
###########################################################
# Generates the expected .o files that should be built
ERPC_OBJECTS_DBG := $(sort $(addprefix $(ERPC_DBG_DIR), $(notdir $(ERPC_OBJ_FILES))))
ERPC_OBJECTS_RLS := $(sort $(addprefix $(ERPC_RLS_DIR), $(notdir $(ERPC_OBJ_FILES))))

# Basic compiler and linker options common to all builds
ERPC_INCLUDES := $(addprefix -I, $(ERPC_INC_DIRS))
ERPC_CXXFLAGS  = -c -fno-common -fmessage-length=0 -Wall -fexceptions -ffunction-sections -fdata-sections
ERPC_CXXDEFS   = -DUSE_FULL_LL_DRIVER -D$(STM32_DEVICE_UC) -DUSING_FREERTOS -DUSING_ERPC -DEIGEN_INITIALIZE_MATRICES_BY_ZERO

# On debug target, add these parameters
erpc_debug: ERPC_CXXFLAGS += -ggdb -O0
erpc_debug: ERPC_CXXDEFS  += -DDEBUG=1 -DDEBUG_DEFAULT_INTERRUPT_HANDLERS

# On release target, add these parameters
erpc_release: ERPC_CXXFLAGS += -O3
erpc_release: ERPC_CXXDEFS  += -DNDEBUG=1 -DRELEASE=1 -DEIGEN_NO_DEBUG


ERPC_SRC_DIRS := $(strip $(sort $(dir $(ERPC_SRC_FILES))))
VPATH = $(ERPC_SRC_DIRS)

###########################################################
# Recipes
###########################################################
.PHONY: erpc_all erpc_release erpc_debug erpc_clean

erpc_all: erpc_release erpc_debug

erpc_release: $(ERPC_OBJECTS_RLS)

erpc_debug: $(ERPC_OBJECTS_DBG)

erpc_clean:
	@rm -f $(ERPC_DBG_DIR)*
	@rm -f $(ERPC_RLS_DIR)*
	$(call colorecho, $(GREEN), eRPC Build Files Cleaned)

erpc_test:
	@echo $(STM32_INC_DIRS)


#------------------------------------------
# Primary build recipes, triggered off of $(RTOS_OBJECTS_xxx)
#------------------------------------------
$(ERPC_DBG_DIR)%.o: %.cpp 
	$(call colorecho, $(CYAN), Compiling $(notdir $<) into $@)
	@$(CPP) $(ERPC_CXXFLAGS) $(USER_CXXFLAGS) $(ERPC_CXXDEFS) -std=gnu++11 $(ERPC_INCLUDES) $< -o $@

$(ERPC_RLS_DIR)%.o: %.cpp
	$(call colorecho, $(CYAN), Compiling $(notdir $<) into $@)
	@$(CPP) $(ERPC_CXXFLAGS) $(USER_CXXFLAGS) $(ERPC_CXXDEFS) -std=gnu++11 $(ERPC_INCLUDES) $< -o $@