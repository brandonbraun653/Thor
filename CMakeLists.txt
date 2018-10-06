cmake_minimum_required(VERSION 3.12.2)

# TEMP VARS FOR TESTING
set(STM32_CHIP "stm32f446xx")
# TEMP VARS FOR TESTING

# --------------------------------------
# Properties for this file
# --------------------------------------
set(THOR_VERSION "1.0")
set(PRJ_ROOT "${CMAKE_CURRENT_LIST_DIR}")

# --------------------------------------
# Pull the STM32 toolchain and some needed functions
# --------------------------------------
include("${PRJ_ROOT}/cmake/thor.cmake")
include("${PRJ_ROOT}/stm32-cmake/cmake/toolchain.cmake")
include("${PRJ_ROOT}/stm32-cmake/cmake/gcc_stm32.cmake")

# --------------------------------------
# Define the installation directory based on the STM32 Family
# --------------------------------------
set(CMAKE_MODULE_PATH 
    "C:/CMakeModules"
    "C:/CMakeModules/stm32hal_f4/1.21.0"
)

if(NOT INSTALL_CMAKE_DIR)
    if(WIN32)
        set(INSTALL_CMAKE_DIR "C:/CMakeModules/thor/${THOR_VERSION}/STM32${STM32_FAMILY}")
    elseif(UNIX)
        set(INSTALL_CMAKE_DIR "/home/$ENV{USER}/CMakeModules/thor/${THOR_VERSION}/STM32${STM32_FAMILY}")
    else()
        message(WARNING "No install directory is set! You will be able to build locally but not install.")
    endif()
endif()

# --------------------------------------
# Give a hint to the find_package module on where boost will lie
# This would be a directory that contains a folder named something like "boost_1_68_0"
# --------------------------------------
if("$ENV{BOOST_ROOT}" STREQUAL "")
    message(FATAL_ERROR "Could not find BOOST_ROOT environment variable!")
else()
    set(BOOST_ROOT $ENV{BOOST_ROOT})
endif()

# --------------------------------------
# Find several packages needed
# --------------------------------------
find_package(Boost REQUIRED)
find_package("STM32${STM32_FAMILY}_HAL" REQUIRED)
find_package(FreeRTOS 10.0.0 REQUIRED COMPONENTS "STM32${STM32_FAMILY}")

# --------------------------------------
# Define directories. These are relative to this file on purpose 
# so that the install command can prepend the INSTALL_PREFIX to it. 
# --------------------------------------
set(THOR_REL_INC_DIR "Thor/Thor/include")
set(THOR_REL_SRC_DIR "Thor/Thor/source")

# --------------------------------------
# Grab all the needed .cpp files. These are used for the compiler to build 
# all the supported static libraries.
# --------------------------------------
set(THOR_SRC_FILES 
    "${PRJ_ROOT}/${THOR_REL_SRC_DIR}/clock.cpp"
    "${PRJ_ROOT}/${THOR_REL_SRC_DIR}/core.cpp"
    "${PRJ_ROOT}/${THOR_REL_SRC_DIR}/defaults.cpp"
    "${PRJ_ROOT}/${THOR_REL_SRC_DIR}/definitions.cpp"
    "${PRJ_ROOT}/${THOR_REL_SRC_DIR}/exceptions.cpp"
    "${PRJ_ROOT}/${THOR_REL_SRC_DIR}/exti.cpp"
    "${PRJ_ROOT}/${THOR_REL_SRC_DIR}/flags.cpp"
    "${PRJ_ROOT}/${THOR_REL_SRC_DIR}/gdb.cpp"
    "${PRJ_ROOT}/${THOR_REL_SRC_DIR}/gpio.cpp"
    "${PRJ_ROOT}/${THOR_REL_SRC_DIR}/i2c.cpp"
    "${PRJ_ROOT}/${THOR_REL_SRC_DIR}/interrupt.cpp"
    "${PRJ_ROOT}/${THOR_REL_SRC_DIR}/mem.cpp"
    "${PRJ_ROOT}/${THOR_REL_SRC_DIR}/nucleo.cpp"
    "${PRJ_ROOT}/${THOR_REL_SRC_DIR}/print.cpp"
    "${PRJ_ROOT}/${THOR_REL_SRC_DIR}/pwm.cpp"
    "${PRJ_ROOT}/${THOR_REL_SRC_DIR}/ringbuffer.cpp"
    "${PRJ_ROOT}/${THOR_REL_SRC_DIR}/rpc.cpp"
    "${PRJ_ROOT}/${THOR_REL_SRC_DIR}/serial.cpp"
    "${PRJ_ROOT}/${THOR_REL_SRC_DIR}/spi.cpp"
    "${PRJ_ROOT}/${THOR_REL_SRC_DIR}/thor.cpp"
    "${PRJ_ROOT}/${THOR_REL_SRC_DIR}/threads.cpp"
    "${PRJ_ROOT}/${THOR_REL_SRC_DIR}/timer.cpp"
    "${PRJ_ROOT}/${THOR_REL_SRC_DIR}/uart.cpp"
    "${PRJ_ROOT}/${THOR_REL_SRC_DIR}/usart.cpp"
)

# --------------------------------------
# Grab all the needed .hpp files. This is for the purpose of installing all the 
# targets into the INSTALL_CMAKE_DIR and need to be relative paths.
# --------------------------------------
set(THOR_INC_FILES
    "${THOR_REL_INC_DIR}/clock.hpp"
    "${THOR_REL_INC_DIR}/config.hpp"
    "${THOR_REL_INC_DIR}/core.hpp"
    "${THOR_REL_INC_DIR}/defaults.hpp"
    "${THOR_REL_INC_DIR}/definitions.hpp"
    "${THOR_REL_INC_DIR}/exceptions.hpp"
    "${THOR_REL_INC_DIR}/exti.hpp"
    "${THOR_REL_INC_DIR}/flags.hpp"
    "${THOR_REL_INC_DIR}/gdb.hpp"
    "${THOR_REL_INC_DIR}/gpio.hpp"
    "${THOR_REL_INC_DIR}/headers.hpp"
    "${THOR_REL_INC_DIR}/i2c.hpp"
    "${THOR_REL_INC_DIR}/interrupt.hpp"
    "${THOR_REL_INC_DIR}/macro.hpp"
    "${THOR_REL_INC_DIR}/mem.hpp"
    "${THOR_REL_INC_DIR}/nucleo.hpp"
    "${THOR_REL_INC_DIR}/preprocessor.hpp"
    "${THOR_REL_INC_DIR}/print.hpp"
    "${THOR_REL_INC_DIR}/pwm.hpp"
    "${THOR_REL_INC_DIR}/ringbuffer.hpp"
    "${THOR_REL_INC_DIR}/rpc.hpp"
    "${THOR_REL_INC_DIR}/serial.hpp"
    "${THOR_REL_INC_DIR}/spi.hpp"
    "${THOR_REL_INC_DIR}/system.hpp"
    "${THOR_REL_INC_DIR}/thor.hpp"
    "${THOR_REL_INC_DIR}/threads.hpp"
    "${THOR_REL_INC_DIR}/timer.hpp"
    "${THOR_REL_INC_DIR}/types.hpp"
    "${THOR_REL_INC_DIR}/uart.hpp"
    "${THOR_REL_INC_DIR}/usart.hpp"
    "${THOR_REL_INC_DIR}/utilities.hpp"
)

# --------------------------------------
# Thor Resources [Depends on Boost, FreeRTOS, STM32HAL]
# --------------------------------------
# This is for local builds
set(THOR_BUILD_INC_DIRS 
    "${PRJ_ROOT}/Thor"
    "${Boost_INCLUDE_DIR}"
)

# These are for the install command
set(THOR_INSTALL_INC_DIRS
    "${INSTALL_CMAKE_DIR}/Thor"
    "${Boost_INCLUDE_DIR}"
)

# --------------------------------------
# Thor Project
# --------------------------------------
project(THOR VERSION ${THOR_VERSION} LANGUAGES CXX)
add_library(${PROJECT_NAME} ${THOR_SRC_FILES})

if(USING_CHIMERA)
    set_property(TARGET ${PROJECT_NAME} PROPERTY FEATURE_CHIMERA "ON")
endif()

# Include Directories
target_include_directories(${PROJECT_NAME} PRIVATE ${THOR_BUILD_INC_DIRS})
target_include_directories(${PROJECT_NAME} INTERFACE
    "$<BUILD_INTERFACE:${THOR_BUILD_INC_DIRS}>"
    "$<INSTALL_INTERFACE:${THOR_INSTALL_INC_DIRS}>"
)

# Compiler Definitions
SET_THOR_COMPILER_DEFINITIONS(${PROJECT_NAME})

# Compiler Options
#SET_THOR_COMPILER_OPTIONS(${PROJECT_NAME})
target_compile_options(${PROJECT_NAME} PUBLIC -fexceptions)
target_compile_options(${PROJECT_NAME} PUBLIC $<$<CONFIG:DEBUG>:-ggdb -Og>)
target_compile_options(${PROJECT_NAME} PUBLIC $<$<CONFIG:RELEASE>:-O3>)
target_compile_options(${PROJECT_NAME} PRIVATE --std=gnu++11)

# Linked Libraries
target_link_libraries(${PROJECT_NAME} stm32f446xx FreeRTOS_STM32F4)

# get_target_property(testvar ${PROJECT_NAME} COMPILE_DEFINITIONS)
# message(STATUS "compile defs: ${testvar}")

