cmake_minimum_required(VERSION 3.12.2)

# Assign the Thor toolchain file if one isn't passed to the command line
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "cmake/toolchain.cmake")
endif()

# TODO: Eventually derive these from the parent project
#------------------------------------------------------------------
set(STM32_TARGET_FAMILY_LOWER "stm32f4")
set(STM32_TARGET_FAMILY_UPPER "STM32F4")
set(STM32_TARGET_DEVICE_LOWER "stm32f446xx")
set(STM32_TARGET_DEVICE_UPPER "STM32F446xx")
#------------------------------------------------------------------

## Define Directories
set(STM32_HAL_DIR "${CMAKE_CURRENT_LIST_DIR}/Thor/lib/stm32hal_f4")



# add_subdirectory(${STM32_HAL_DIR})
include(${STM32_HAL_DIR}/CMakeLists.txt)

project(thor)
add_library(${PROJECT_NAME} main.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC ${STM32F4_HAL_INC_DIRS})

target_compile_definitions(${PROJECT_NAME} PUBLIC -DSTM32F446xx -DUSE_FULL_LL_DRIVER)

target_link_libraries(${PROJECT_NAME} $<$<CONFIG:DEBUG>: ${STM32_HAL_DIR}/project/makefile/libstm32f4_hal_dbg.a>)
target_link_libraries(${PROJECT_NAME} $<$<CONFIG:RELEASE>: ${STM32_HAL_DIR}/project/makefile/libstm32f4_hal_rel.a>)
