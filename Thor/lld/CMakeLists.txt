# ====================================================
# LLD Projects Always Included
# ====================================================
add_subdirectory("common")
add_subdirectory("interface")

# ====================================================
# Select the appropriate Low Level Driver
# ====================================================
if(${THOR_FAMILY} MATCHES "^L4$")
  add_subdirectory("stm32l4x")
elseif(${THOR_FAMILY} MATCHES "^MOCK$")
  message("Matched mock family")
else()
  message(FATAL_ERROR "No low level driver was found for Thor Family: ${THOR_FAMILY}!")
endif()

# ====================================================
# Various exports
# ====================================================

# LLD Common Include External Dependencies
add_library(lld_ext_inc_dep INTERFACE)
target_link_libraries(lld_ext_inc_dep INTERFACE
  lld_common_inc_dep

  freertos_inc freertos_port freertos_cfg
)

if(FREERTOS)
  target_link_libraries(lld_ext_inc_dep INTERFACE
    freertos_inc
    freertos_port
    freertos_cfg
  )
endif()
export(TARGETS lld_ext_inc_dep FILE "${PROJECT_BINARY_DIR}/Thor/lld/ext-inc-dep.cmake")