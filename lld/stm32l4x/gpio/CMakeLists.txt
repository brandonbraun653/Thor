include("${COMMON_TOOL_ROOT}/cmake/utility/embedded.cmake")

if(${THOR_CHIP} MATCHES "^L432$")
  set(VARIANT_SRC variant/hw_gpio_register_stm32l432kc.cpp)
else()
  message(FATAL_ERROR "Unsupported STM32L4 driver variant for: gpio")
endif()

gen_static_lib_variants(
  TARGET
    thor_lld_stm32l4_gpio
  SOURCES
    ../../interface/gpio/gpio_common_driver.cpp
    hw_gpio_data.cpp
    ${VARIANT_SRC}
  PRV_LIBRARIES
    aurora_intf_inc
    chimera_intf_inc
    thor_intf_inc
    Boost::boost
    lld_common_inc
  EXPORT_DIR
    "${PROJECT_BINARY_DIR}/Thor/lld"
)
