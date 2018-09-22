cmake_minimum_required(VERSION 3.12.2)
set_property(GLOBAL PROPERTY USING_THOR "TRUE")

set(THOR_ROOT "${CMAKE_CURRENT_LIST_DIR}")

# --------------------------------------
# Identify the location of the cmake tools directory
# --------------------------------------
set_property(GLOBAL PROPERTY STM32_CMAKE_TOOLS_DIR "${THOR_ROOT}/cmake")
set(STM32_CMAKE_TOOLS_DIR "${THOR_ROOT}/cmake")


# Assign the Thor toolchain file if one isn't passed to the command line
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${STM32_CMAKE_TOOLS_DIR}/toolchain.cmake")
endif()

# --------------------------------------
# STM32 Resources
# --------------------------------------
include("${STM32_CMAKE_TOOLS_DIR}/gcc_stm32.cmake")
STM32_VALIDATE_DEVICE()

# Find and include the correct HAL CMakeLists.txt file
string(TOLOWER ${STM32_FAMILY} STM32_FAMILY_LOWER)
include("${THOR_ROOT}/Thor/lib/stm32hal_${STM32_FAMILY_LOWER}/CMakeLists.txt")

# Get a few properties set in the above file
get_property(STM32_HAL_INC_DIRS GLOBAL PROPERTY STM32_HAL_INC_DIRS)
get_property(STM32_COMPILER_OPTIONS GLOBAL PROPERTY STM32_COMPILER_OPTIONS)
get_property(STM32_HAL_DEVICE_DEFINITION GLOBAL PROPERTY STM32_HAL_DEVICE_DEFINITION)

# --------------------------------------
# FreeRTOS Resources
# --------------------------------------
include("${THOR_ROOT}/Thor/lib/FreeRTOS/CMakeLists.txt")

# --------------------------------------
# Boost Resources
# --------------------------------------
# Give a hint to the find_package module on where boost will lie
# This would be a directory that contains a folder named something like "boost_1_68_0"
if("$ENV{BOOST_ROOT}" STREQUAL "")
    message(FATAL_ERROR "Could not find BOOST_ROOT environment variable!")
else()
    set(BOOST_ROOT $ENV{BOOST_ROOT})    
endif()

find_package(BOOST)

# --------------------------------------
# Thor Resources [Depends on Boost, FreeRTOS, STM32HAL]
# --------------------------------------
# Add build specific definitions
get_property(STM32_HAL_COMPILER_DEFINITIONS GLOBAL PROPERTY STM32_HAL_COMPILER_DEFINITIONS)

set(THOR_COMPILER_DEFINITIONS
    -DUSING_FREERTOS 
    -DUSE_FREERTOS 
    -DEIGEN_INITIALIZE_MATRICES_BY_ZERO
    ${STM32_HAL_COMPILER_DEFINITIONS}
)
set(THOR_COMPILER_DEFINITIONS ${THOR_COMPILER_DEFINITIONS} $<$<CONFIG:DEBUG>: -DDEBUG=1 -DDEBUG_DEFAULT_INTERRUPT_HANDLERS>)
set(THOR_COMPILER_DEFINITIONS ${THOR_COMPILER_DEFINITIONS} $<$<CONFIG:RELEASE>: -DNDEBUG=1 -DRELEASE=1 -DEIGEN_NO_DEBUG>)
set_property(GLOBAL PROPERTY THOR_COMPILER_DEFINITIONS "${THOR_COMPILER_DEFINITIONS}")

# Compiler Options
set(THOR_COMPILER_OPTIONS ${STM32_COMPILER_OPTIONS} --std=gnu++11 -fexceptions)
set_property(GLOBAL PROPERTY THOR_COMPILER_OPTIONS "${THOR_COMPILER_OPTIONS}")

# Gather all the include directories and make them public
set(THOR_INC_DIRS 
    "${THOR_ROOT}/Thor" 
    ${STM32_HAL_INC_DIRS} 
    ${FREERTOS_INC_DIRS} 
    ${Boost_INCLUDE_DIR}
)
set_property(GLOBAL PROPERTY THOR_INC_DIRS "${THOR_INC_DIRS}")

# Assign the source files
set(THOR_SRC_FILES 
    "${THOR_ROOT}/Thor/Thor/source/clock.cpp"
    "${THOR_ROOT}/Thor/Thor/source/core.cpp"
    "${THOR_ROOT}/Thor/Thor/source/defaults.cpp"
    "${THOR_ROOT}/Thor/Thor/source/definitions.cpp"
    "${THOR_ROOT}/Thor/Thor/source/exceptions.cpp"
    "${THOR_ROOT}/Thor/Thor/source/exti.cpp"
    "${THOR_ROOT}/Thor/Thor/source/flags.cpp"
    "${THOR_ROOT}/Thor/Thor/source/gdb.cpp"
    "${THOR_ROOT}/Thor/Thor/source/gpio.cpp"
    "${THOR_ROOT}/Thor/Thor/source/i2c.cpp"
    "${THOR_ROOT}/Thor/Thor/source/interrupt.cpp"
    "${THOR_ROOT}/Thor/Thor/source/mem.cpp"
    "${THOR_ROOT}/Thor/Thor/source/nucleo.cpp"
    "${THOR_ROOT}/Thor/Thor/source/print.cpp"
    "${THOR_ROOT}/Thor/Thor/source/pwm.cpp"
    "${THOR_ROOT}/Thor/Thor/source/ringbuffer.cpp"
    "${THOR_ROOT}/Thor/Thor/source/rpc.cpp"
    "${THOR_ROOT}/Thor/Thor/source/serial.cpp"
    "${THOR_ROOT}/Thor/Thor/source/spi.cpp"
    "${THOR_ROOT}/Thor/Thor/source/thor.cpp"
    "${THOR_ROOT}/Thor/Thor/source/threads.cpp"
    "${THOR_ROOT}/Thor/Thor/source/timer.cpp"
    "${THOR_ROOT}/Thor/Thor/source/uart.cpp"
    "${THOR_ROOT}/Thor/Thor/source/usart.cpp"
)

# --------------------------------------
# eRPC Resources [Depends on Thor]
# --------------------------------------
include("${THOR_ROOT}/Thor/lib/erpc/CMakeLists.txt")

# --------------------------------------
# Thor Project
# --------------------------------------
project(thor)
add_library(${PROJECT_NAME} ${THOR_SRC_FILES})

# Force the link language to C++
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

# Include Directories
target_include_directories(${PROJECT_NAME} PUBLIC ${THOR_INC_DIRS})

# Compiler Definitions 
target_compile_definitions(${PROJECT_NAME} PUBLIC ${THOR_COMPILER_DEFINITIONS})

# Compiler Options
target_compile_options(${PROJECT_NAME} PRIVATE ${THOR_COMPILER_OPTIONS})
target_compile_options(${PROJECT_NAME} PRIVATE $<$<CONFIG:DEBUG>:-ggdb -Og>)
target_compile_options(${PROJECT_NAME} PRIVATE $<$<CONFIG:RELEASE>:-O3>)


target_link_libraries(${PROJECT_NAME} $<$<CONFIG:DEBUG>: 
    "libstm32${STM32_FAMILY_LOWER}_hal_dbg.a"
    "libFreeRTOS_dbg.a"
    "liberpc_server_dbg.a"
>)
target_link_libraries(${PROJECT_NAME} $<$<CONFIG:RELEASE>: 
    "libstm32${STM32_FAMILY_LOWER}_hal_rel.a"
    "libFreeRTOS_rel.a"
    "liberpc_server_rel.a"
>)
