cmake_minimum_required(VERSION 3.12.2)

set(THOR_ROOT "${CMAKE_CURRENT_LIST_DIR}")

# --------------------------------------
# Identify the location of the cmake tools directory
# --------------------------------------
set(STM32_CMAKE_TOOLS_DIR "${THOR_ROOT}/cmake" CACHE STRING "Thor: STM32 location for cmake tools")


# Assign the Thor toolchain file if one isn't passed to the command line
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${STM32_CMAKE_TOOLS_DIR}/toolchain.cmake")
endif()


# --------------------------------------
# STM32 Resources
# --------------------------------------
include("${STM32_CMAKE_TOOLS_DIR}/gcc_stm32.cmake")

# Using definitions from gcc_stm32.cmake, find, validate, and import the specific device family
# cmake file
include(${STM32_CMAKE_DIR}/gcc_stm32${STM32_FAMILY_LOWER}.cmake)
STM32_GET_CHIP_TYPE(${STM32_CHIP} STM32_CHIP_TYPExx)

set(STM32_HAL_DIR "${THOR_ROOT}/Thor/lib/stm32hal_${STM32_FAMILY_LOWER}")
set(STM32_HAL_INC_DIRS "STM32${STM32_FAMILY}_HAL_INC_DIRS")

message(STATUS "Stuff: ${STM32F4_HAL_INC_DIRS}")

# --------------------------------------
# FreeRTOS Resources
# --------------------------------------
set(FREERTOS_DIR "${THOR_ROOT}/Thor/lib/FreeRTOS")

# Try and find the Boost library 

# if(NOT $ENV{BOOST_ROOT})
#     message(FATAL_ERROR "Could not find BOOST_ROOT environment variable!")
# endif()

# set(BOOST_ROOT $ENV{BOOST_ROOT})

# find_package(BOOST 1.65.1)


## Define Directories




set(THOR_INC_DIR "${THOR_ROOT}/Thor/src/include")
set(THOR_SRC_DIR "${THOR_ROOT}/Thor/src/source")

file(GLOB THOR_SRC_FILES "${THOR_SRC_DIR}/*.cpp")


project(thor)
add_library(${PROJECT_NAME} main.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC ${STM32_HAL_INC_DIRS})

target_compile_definitions(${PROJECT_NAME} PUBLIC -DSTM32F446xx -DUSE_FULL_LL_DRIVER)

target_link_libraries(${PROJECT_NAME} $<$<CONFIG:DEBUG>: 
    "${STM32_HAL_DIR}/project/makefile/libstm32${STM32_FAMILY_LOWER}_hal_dbg.a"
    "${FREERTOS_DIR}/build/libFreeRTOS_dbg.a"
>)
target_link_libraries(${PROJECT_NAME} $<$<CONFIG:RELEASE>: 
    "${STM32_HAL_DIR}/project/makefile/libstm32${STM32_FAMILY_LOWER}_hal_rel.a"
    "${FREERTOS_DIR}/build/libFreeRTOS_rel.a"
>)
