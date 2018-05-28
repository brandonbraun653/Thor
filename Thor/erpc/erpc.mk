# GNU Make Standard Library: https://gmsl.sourceforge.io/ 
include /usr/bin/gmsl
include /usr/bin/colors

ifndef PROJECT_BUILD_ROOT
$(error Please define the build directory in "PROJECT_BUILD_ROOT")
endif

# Figure out the file directory so we can locate build outputs correctly
THOR_ROOT        ?= $(dir $(filter %thor.mk, $(MAKEFILE_LIST)))
ERPC_ROOT        := $(dir $(filter %erpc.mk, $(MAKEFILE_LIST)))
ERPC_BUILD_ROOT  := $(PROJECT_BUILD_ROOT)
ERPC_DBG_DIR     := $(addprefix $(ERPC_BUILD_ROOT), debug/erpc/)
ERPC_RLS_DIR     := $(addprefix $(ERPC_BUILD_ROOT), release/erpc/)


###########################################################
# Directories 
###########################################################
$(shell mkdir -p $(ERPC_DBG_DIR))
$(shell mkdir -p $(ERPC_RLS_DIR))

ERPC_INC_DIRS = $(addprefix $(ERPC_ROOT), bin/ config/ infra/ port/ setup/ transports/)
ERPC_SRC_DIRS = $(ERPC_INC_DIRS)


###########################################################
# Files
###########################################################
ERPC_INC_FILES = $(foreach dir, $(ERPC_INC_DIRS), $(wildcard $(dir)*.h))
ERPC_SRC_FILES = $(foreach dir, $(ERPC_INC_DIRS), $(wildcard $(dir)*.cpp))

ERPC_INC_FILES += $(addprefix $(THOR_ROOT), include/thor.hpp include/definitions.hpp include/serial.hpp)
ERPC_INC_FILES += $(addprefix $(THOR_ROOT), FreeRTOS/include/FreeRTOS.h FreeRTOS/include/semphr.h)

ERPC_OBJ_FILES := $(patsubst %.cpp, %.o, $(ERPC_SRC_FILES))

###########################################################
# Build vars
###########################################################


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
	@echo $(ERPC_OBJ_FILES)


#------------------------------------------
# Primary build recipes, triggered off of $(RTOS_OBJECTS_xxx)
#------------------------------------------

# I think I need to link against heap_4.o

$(ERPC_DBG_DIR)%.o:
	@echo hello

$(ERPC_RLS_DIR)%.o:
	@echo hello