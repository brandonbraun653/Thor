
# GNU Make Standard Library: https://gmsl.sourceforge.io/ 
include /usr/bin/gmsl
include /usr/bin/colors

# Define some larger architectural design setup choices 
USE_FREERTOS ?= TRUE
USE_CHIMERA  ?= FALSE
USE_ERPC     ?= TRUE


# Pull in the dependencies in this order:
# 	1) STM32 Hal
# 	2) FreeRTOS
#	3) eRPC (depends on 1 & 2)
include $(addprefix $(THOR_ROOT), /Thor/stm32hal.mk)
include $(addprefix $(THOR_ROOT), /Thor/FreeRTOS/freertos.mk)
include $(addprefix $(THOR_ROOT), /Thor/erpc/erpc.mk)

# Make sure that the necessary dependencies exist!
ifndef PROJECT_BUILD_ROOT
$(error Please define the build directory in "PROJECT_BUILD_ROOT")
endif

ifndef EIGEN_ROOT
$(error Place Eigen path in "EIGEN_ROOT" before including thor.mk)
else ifneq ($(shell test -d $(EIGEN_ROOT)/Eigen && echo -n yes),yes)
$(error Eigen directory is invalid! Could not find "$(EIGEN_ROOT)/Eigen")
endif

ifndef BOOST_ROOT
$(error Place Boost path in "BOOST_ROOT" before including thor.mk)
else ifneq ($(shell test -d $(BOOST_ROOT)/boost && echo -n yes),yes)
$(error Boost directory is invalid! Could not find "$(BOOST_ROOT)/boost")
endif 

# Figure out the file directory so we can locate build outputs correctly
THOR_BUILD_ROOT  := $(PROJECT_BUILD_ROOT)
THOR_DBG_DIR     := $(addprefix $(THOR_BUILD_ROOT), debug/thor/)
THOR_RLS_DIR     := $(addprefix $(THOR_BUILD_ROOT), release/thor/)


###########################################################
# Directories 
###########################################################
$(shell mkdir -p $(THOR_DBG_DIR))
$(shell mkdir -p $(THOR_RLS_DIR))

EIGEN_DIR      = $(addprefix $(EIGEN_ROOT), /Eigen/)
BOOST_DIR      = $(addprefix $(BOOST_ROOT), /boost/)
THOR_INC_DIRS := $(THOR_ROOT) $(STM32_INC_DIRS) $(EIGEN_ROOT) $(BOOST_ROOT)

# These includes come from freertos.mk
ifeq ($(USE_FREERTOS), TRUE)
THOR_INC_DIRS += $(RTOS_INC_DIRS)
endif

###########################################################
# Files
###########################################################
EIGEN_HEADERS := $(shell find $(EIGEN_DIR) -type f -name '*.h' -o -name '*.hpp')
BOOST_HEADERS := $(shell find $(BOOST_DIR) -type f -name '*.h' -o -name '*.hpp')

THOR_INC_FILES := $(wildcard $(THOR_ROOT)/Thor/include/*.hpp)
THOR_SRC_FILES := $(wildcard $(THOR_ROOT)/Thor/source/*.cpp)
THOR_OBJ_FILES := $(patsubst %.cpp, %.o, $(THOR_SRC_FILES))

# Let Make know where all the .cpp files are
#vpath %.cpp $(sort $(dir $(THOR_SRC_FILES)))/
THOR_SRC_DIRS := $(strip $(sort $(dir $(THOR_SRC_FILES))))

###########################################################
# Build vars
###########################################################
# Generates the expected .o files that should be built
THOR_OBJECTS_DBG := $(sort $(addprefix $(THOR_DBG_DIR), $(notdir $(THOR_OBJ_FILES))))
THOR_OBJECTS_RLS := $(sort $(addprefix $(THOR_RLS_DIR), $(notdir $(THOR_OBJ_FILES))))

# Basic compiler & linker options common to all builds
THOR_INCLUDES    := $(addprefix -I, $(THOR_INC_DIRS))
THOR_CXXFLAGS     = -c -fno-common -fmessage-length=0 -Wall -fno-rtti -fexceptions -ffunction-sections -fdata-sections
THOR_CXXDEFS      = -DUSE_FULL_LL_DRIVER -DEIGEN_INITIALIZE_MATRICES_BY_ZERO -D$(STM32_DEVICE_UC) -D$(STM32_DEVICE_FAMILY_UC) 

# On debug target, add these parameters 
thor_debug: THOR_CXXFLAGS  += -ggdb -O0
thor_debug: THOR_CXXDEFS   += -DDEBUG=1 -DDEBUG_DEFAULT_INTERRUPT_HANDLERS 

# On release target, add these parameters 
thor_release: THOR_CXXFLAGS  += -O3
thor_release: THOR_CXXDEFS   += -DNDEBUG=1 -DRELEASE=1 -DEIGEN_NO_DEBUG


# Conditionally update build vars
ifeq ($(USE_CHIMERA), TRUE)
THOR_CXXDEFS += -DUSING_CHIMERA
endif 

#---------------------------------------
# Update the dependencies to be built in this order
#	1) HAL
# 	2) FreeRTOS (if needed)
# 	3) eRPC (if using 2)
# 
THOR_RELEASE_DEPS = hal_release 
THOR_DEBUG_DEPS   = hal_debug
THOR_CLEAN_DEPS   = hal_clean

ifeq ($(USE_FREERTOS), TRUE)
THOR_RELEASE_DEPS += rtos_release erpc_release
THOR_DEBUG_DEPS   += rtos_debug	erpc_debug
THOR_CLEAN_DEPS   += rtos_clean erpc_clean
THOR_CXXDEFS      += -DUSE_FREERTOS -DUSING_FREERTOS
endif

ifeq ($(USE_ERPC), TRUE)
THOR_RELEASE_DEPS += erpc_release
THOR_DEBUG_DEPS   += erpc_debug
THOR_CLEAN_DEPS   += erpc_clean
THOR_CXXDEFS      += -DUSING_ERPC
endif 

###########################################################
# Recipes
###########################################################
.PHONY: thor_all thor_release thor_debug thor_clean
thor_all: thor_release thor_debug

thor_release: $(THOR_RELEASE_DEPS) $(THOR_OBJECTS_RLS)

thor_debug: $(THOR_DEBUG_DEPS) $(THOR_OBJECTS_DBG)

thor_clean: $(THOR_CLEAN_DEPS)
	@rm -f $(THOR_DBG_DIR)*
	@rm -f $(THOR_RLS_DIR)*
	$(call colorecho, $(GREEN), Thor Build Files Cleaned)

thor_test:
	@echo $(sort $(dir $(THOR_SRC_FILES)))
	@echo $(THOR_SRC_DIRS)

#------------------------------------------
# Primary build recipes, triggered off of $(THOR_OBJECTS_xxx)
#------------------------------------------
#$(EIGEN_HEADERS) $(BOOST_HEADERS)
$(THOR_DBG_DIR)%.o: $(THOR_SRC_DIRS)%.cpp
	$(call colorecho, $(CYAN), Compiling $(notdir $<) into $@)
	@$(CPP) $(THOR_CXXFLAGS) $(USER_CXXFLAGS) $(THOR_CXXDEFS) -std=gnu++11 $(THOR_INCLUDES) $< -o $@ 

$(THOR_RLS_DIR)%.o: $(THOR_SRC_DIRS)%.cpp
	$(call colorecho, $(CYAN), Compiling $(notdir $<) into $@)
	@$(CPP) $(THOR_CXXFLAGS) $(USER_CXXFLAGS) $(THOR_CXXDEFS) -std=gnu++11 $(THOR_INCLUDES) $< -o $@ 