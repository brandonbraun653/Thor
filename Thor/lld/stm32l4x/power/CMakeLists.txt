# ====================================================
# Common
# ====================================================
set(LINK_LIBS
  thor_inc          # Thor public headers
  chimera_inc       # Chimera public headers
  aurora_inc        # Aurora public headers
  Boost::boost      # Boost headers
  lld_common_inc   # 3rd Party headers
  prj_device_target # Compiler options for target device
)

# ====================================================
# Variant Library
# ====================================================
set(VARIANT thor_lld_stm32l4_power_variant)
if(${THOR_CHIP} MATCHES "^L432$")
  add_library(${VARIANT} STATIC variant/hw_power_register_stm32l432kc.cpp)
  target_link_libraries(${VARIANT} PRIVATE ${LINK_LIBS})
  export(TARGETS ${VARIANT} FILE "${PROJECT_BINARY_DIR}/Thor/lld/${VARIANT}.cmake")
else()
  message(FATAL_ERROR "Unsupported STM32L4 driver variant for: power")
endif()

# ====================================================
# Driver Library (+ Variant)
# ====================================================
set(CORE thor_lld_stm32l4_power)
add_library(${CORE} STATIC
  hw_power_driver_stm32l4.cpp
  hw_power_mapping.cpp
)
target_link_libraries(${CORE} PRIVATE ${LINK_LIBS} ${VARIANT})
export(TARGETS ${CORE} FILE "${PROJECT_BINARY_DIR}/Thor/lld/${CORE}.cmake")
