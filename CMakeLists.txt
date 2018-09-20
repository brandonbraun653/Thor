cmake_minimum_required(VERSION 3.12.2)

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

# Define a few variables used in the HAL CMakeLists.txt file
set_property(GLOBAL PROPERTY STM32_DEVICE_FAMILY "${STM32_FAMILY}")
set_property(GLOBAL PROPERTY STM32_DEVICE_ID "${STM32_CHIP}")

# Find and include the correct HAL CMakeLists.txt file
set(STM32_HAL_ROOT "${THOR_ROOT}/Thor/lib/stm32hal_${STM32_FAMILY_LOWER}")
include("${STM32_HAL_ROOT}/CMakeLists.txt")

# Add the generic include directories
set(STM32_HAL_INC_DIRS "${STM32${STM32_FAMILY}_HAL_INC_DIRS}")

# --------------------------------------
# FreeRTOS Resources
# --------------------------------------
set(FREERTOS_ROOT "${THOR_ROOT}/Thor/lib/FreeRTOS")
include("${FREERTOS_ROOT}/CMakeLists.txt")

# --------------------------------------
# Boost Resources
# --------------------------------------
# Give a hint to the find_package module on where boost will lie
if("$ENV{BOOST_ROOT}" STREQUAL "")
    message(FATAL_ERROR "Could not find BOOST_ROOT environment variable!")
else()
    set(BOOST_ROOT $ENV{BOOST_ROOT})    
endif()

if(NOT BOOST_VER)
    set(BOOST_VER "1.65.1")
    message(STATUS "Boost Version not specified. Defaulting to 1.65.1")
endif()

find_package(BOOST "1.65" EXACT)

## Define Directories
set(THOR_INC_DIR "${THOR_ROOT}/Thor")
set(THOR_SRC_DIR "${THOR_ROOT}/Thor/Thor/source")

file(GLOB THOR_SRC_FILES "${THOR_SRC_DIR}/*.cpp")

project(thor)
add_library(${PROJECT_NAME} ${THOR_SRC_FILES})

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

target_include_directories(${PROJECT_NAME} PUBLIC ${THOR_INC_DIR} ${STM32_HAL_INC_DIRS} ${FREERTOS_INC_DIRS} ${Boost_INCLUDE_DIR})


get_property(STM32_COMPILER_OPTIONS GLOBAL PROPERTY STM32_COMPILER_OPTIONS)
target_compile_options(${PROJECT_NAME} PRIVATE ${STM32_COMPILER_OPTIONS} --std=gnu++11 -fexceptions)
target_compile_options(${PROJECT_NAME} PRIVATE $<$<CONFIG:DEBUG>:-ggdb -Og>)
target_compile_options(${PROJECT_NAME} PRIVATE $<$<CONFIG:RELEASE>:-O3>)



set(THOR_COMPILE_DEFINITIONS
    -DUSING_FREERTOS 
    -DUSE_FREERTOS 
    -DSTM32F446xx 
    -DUSE_FULL_LL_DRIVER
    -DEIGEN_INITIALIZE_MATRICES_BY_ZERO
)

# Add target/build specific definitions
target_compile_definitions(${PROJECT_NAME} PUBLIC $<$<CONFIG:DEBUG>: ${THOR_COMPILE_DEFINITIONS} -DDEBUG=1 -DDEBUG_DEFAULT_INTERRUPT_HANDLERS>)
target_compile_definitions(${PROJECT_NAME} PUBLIC $<$<CONFIG:RELEASE>: ${THOR_COMPILE_DEFINITIONS} -DNDEBUG=1 -DRELEASE=1 -DEIGEN_NO_DEBUG>)

target_link_libraries(${PROJECT_NAME} $<$<CONFIG:DEBUG>: 
    "libstm32${STM32_FAMILY_LOWER}_hal_dbg.a"
    "libFreeRTOS_dbg.a"
>)
target_link_libraries(${PROJECT_NAME} $<$<CONFIG:RELEASE>: 
    "libstm32${STM32_FAMILY_LOWER}_hal_rel.a"
    "libFreeRTOS_rel.a"
>)
