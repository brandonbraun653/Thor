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
set(VARIANT thor_lld_stm32l4_spi_variant)
if(${THOR_CHIP} MATCHES "^L432$")
  add_library(${VARIANT} STATIC variant/hw_spi_register_stm32l432kc.cpp)
  target_link_libraries(${VARIANT} PRIVATE ${LINK_LIBS})
  export(TARGETS ${VARIANT} FILE "${PROJECT_BINARY_DIR}/Thor/lld/${VARIANT}.cmake")
else()
  message(FATAL_ERROR "Unsupported STM32L4 driver variant for: spi")
endif()

# ====================================================
# Driver Library (+ Variant)
# ====================================================
set(CORE thor_lld_stm32l4_spi)
add_library(${CORE} STATIC
  hw_spi_driver.cpp
  hw_spi_data.cpp
)
target_link_libraries(${CORE} PRIVATE ${LINK_LIBS} ${VARIANT})
export(TARGETS ${CORE} FILE "${PROJECT_BINARY_DIR}/Thor/lld/${CORE}.cmake")
