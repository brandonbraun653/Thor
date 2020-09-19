# ====================================================
# LLD Projects Always Included
# ====================================================
add_subdirectory("common")
add_subdirectory("interface")

# ====================================================
# Select the appropriate Low Level Driver
# ====================================================
if(Toolchain::HAS_GCC_ARM_NONE_EABI)
  if(${THOR_FAMILY} MATCHES "^L4$")
    add_subdirectory("stm32l4x")
  else()
    message(FATAL_ERROR "No low level driver was found for Thor Family: ${THOR_FAMILY}!")
  endif()
else()
  # If not using the arm-none-eabi compiler, then the LLD is provided by either
  # the MOCK or SIM drivers at the interface layer.
endif()

# ====================================================
# Common headers needed by low level drivers
# ====================================================
set(EXT_INCLUDES lld_common_inc)
add_library(${EXT_INCLUDES} INTERFACE)

target_link_libraries(${EXT_INCLUDES} INTERFACE type_safe_inc)

if(Toolchain::REQUIRES_FREERTOS_THREADS)
  target_link_libraries(${EXT_INCLUDES} INTERFACE
    freertos_inc
    freertos_port
    freertos_cfg
  )
endif()
export(TARGETS ${EXT_INCLUDES} FILE "${PROJECT_BINARY_DIR}/Thor/lld/${EXT_INCLUDES}.cmake")